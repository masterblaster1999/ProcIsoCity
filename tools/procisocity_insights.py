#!/usr/bin/env python3
"""
procisocity_insights.py

A zero-dependency (stdlib-only) analysis tool for ProcIsoCity "dossier" exports.

It ingests:
  - summary.json        (optional)
  - ticks.csv           (optional)
  - tile_metrics.csv    (recommended)

…and emits:
  - insights.json       (machine-readable summary)
  - insights.md         (human-readable report)

It is designed to be schema-tolerant: it discovers metric columns by name heuristics and
falls back gracefully when certain fields are missing.

Example:
  python tools/procisocity_insights.py dossier_out --out insights.json --md insights.md

Why this exists:
  The project already exports rich artifacts (tile_metrics, tick traces, etc.). This tool
  adds *analytics* that are useful for regression, seed mining, and "city science":
    - inequality (Gini) of land value / traffic / goods / risk
    - spatial autocorrelation (Moran's I) for land value (are values clustered?)
    - hot spot tiles for selected layers
    - correlations between metrics (e.g., land value vs flood risk)
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

# -----------------------------
# Utilities
# -----------------------------

def _is_number(s: str) -> bool:
    try:
        float(s)
        return True
    except Exception:
        return False


def _to_float(s: str) -> Optional[float]:
    s = s.strip()
    if s == "" or s.lower() in {"nan", "null", "none"}:
        return None
    try:
        return float(s)
    except Exception:
        return None


def _to_int(s: str) -> Optional[int]:
    s = s.strip()
    if s == "" or s.lower() in {"nan", "null", "none"}:
        return None
    try:
        return int(float(s))
    except Exception:
        return None


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def gini(values: Sequence[float]) -> Optional[float]:
    """
    Compute Gini coefficient in [0,1] for non-negative values.
    Returns None if insufficient or all zeros.

    Uses a stable O(n log n) approach: sort + cumulative sums.
    """
    if not values:
        return None
    xs = [v for v in values if v is not None and math.isfinite(v)]
    if not xs:
        return None
    # Gini is typically defined for non-negative measures. If negatives exist,
    # shift by min to make them non-negative.
    mn = min(xs)
    if mn < 0.0:
        xs = [v - mn for v in xs]
    xs.sort()
    n = len(xs)
    total = sum(xs)
    if total <= 0.0:
        return 0.0
    # G = (2*sum_i i*x_i)/(n*sum_x) - (n+1)/n, with i starting at 1
    cum = 0.0
    for i, x in enumerate(xs, start=1):
        cum += i * x
    g = (2.0 * cum) / (n * total) - (n + 1.0) / n
    return clamp(g, 0.0, 1.0)


def pearson_corr(a: Sequence[float], b: Sequence[float]) -> Optional[float]:
    """
    Pearson correlation between two sequences, skipping non-finite pairs.
    Returns None if < 3 valid pairs or zero variance.
    """
    pairs: List[Tuple[float, float]] = []
    for x, y in zip(a, b):
        if x is None or y is None:
            continue
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        pairs.append((x, y))
    if len(pairs) < 3:
        return None
    xs = [p[0] for p in pairs]
    ys = [p[1] for p in pairs]
    mx = statistics.fmean(xs)
    my = statistics.fmean(ys)
    sx2 = sum((x - mx) ** 2 for x in xs)
    sy2 = sum((y - my) ** 2 for y in ys)
    if sx2 <= 0.0 or sy2 <= 0.0:
        return None
    cov = sum((x - mx) * (y - my) for x, y in pairs)
    return cov / math.sqrt(sx2 * sy2)


def sparkline(values: Sequence[float], width: int = 48) -> str:
    """
    A tiny unicode sparkline, useful for quick glance in markdown.
    """
    blocks = "▁▂▃▄▅▆▇█"
    xs = [v for v in values if v is not None and math.isfinite(v)]
    if not xs:
        return ""
    mn = min(xs)
    mx = max(xs)
    if mx <= mn:
        return blocks[0] * min(len(values), width)
    # downsample to width
    if len(values) > width:
        stride = len(values) / width
        sampled = []
        for i in range(width):
            j = int(i * stride)
            sampled.append(values[j])
    else:
        sampled = list(values)
    out = []
    for v in sampled:
        if v is None or not math.isfinite(v):
            out.append(" ")
            continue
        t = (v - mn) / (mx - mn)
        idx = int(clamp(t, 0.0, 0.999999) * len(blocks))
        out.append(blocks[idx])
    return "".join(out)


@dataclass
class MetricStats:
    count: int
    mean: float
    stdev: float
    min: float
    p10: float
    p50: float
    p90: float
    p99: float
    max: float

    def to_json(self) -> Dict[str, Any]:
        return {
            "count": self.count,
            "mean": self.mean,
            "stdev": self.stdev,
            "min": self.min,
            "p10": self.p10,
            "p50": self.p50,
            "p90": self.p90,
            "p99": self.p99,
            "max": self.max,
        }


def compute_stats(values: Sequence[float]) -> Optional[MetricStats]:
    xs = [v for v in values if v is not None and math.isfinite(v)]
    if not xs:
        return None
    xs.sort()
    n = len(xs)
    mean = statistics.fmean(xs)
    stdev = statistics.pstdev(xs) if n >= 2 else 0.0
    def pct(p: float) -> float:
        if n == 1:
            return xs[0]
        k = (n - 1) * p
        i = int(math.floor(k))
        j = min(i + 1, n - 1)
        t = k - i
        return xs[i] * (1.0 - t) + xs[j] * t
    return MetricStats(
        count=n,
        mean=mean,
        stdev=stdev,
        min=xs[0],
        p10=pct(0.10),
        p50=pct(0.50),
        p90=pct(0.90),
        p99=pct(0.99),
        max=xs[-1],
    )


# -----------------------------
# Dossier ingestion
# -----------------------------

def load_summary_json(path: Path) -> Optional[Dict[str, Any]]:
    if not path.exists():
        return None
    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def load_ticks_csv(path: Path) -> Optional[List[Dict[str, str]]]:
    if not path.exists():
        return None
    rows: List[Dict[str, str]] = []
    try:
        with path.open("r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            for r in reader:
                rows.append(dict(r))
        return rows
    except Exception:
        return None


def _choose_column(headers: Sequence[str], candidates: Sequence[str]) -> Optional[str]:
    """
    Pick the first header that matches a candidate (case-insensitive), with some normalization.
    """
    hs = {h.lower(): h for h in headers}
    # direct
    for c in candidates:
        if c.lower() in hs:
            return hs[c.lower()]
    # normalized: remove underscores/spaces
    def norm(s: str) -> str:
        return "".join(ch for ch in s.lower() if ch.isalnum())
    hnorm = {norm(h): h for h in headers}
    for c in candidates:
        cn = norm(c)
        if cn in hnorm:
            return hnorm[cn]
    return None


def _discover_metric_columns(headers: Sequence[str]) -> Dict[str, str]:
    """
    Heuristic discovery for common dossier metrics.
    """
    # Canonical metric -> list of possible column names
    desired = {
        "land_value": ["landvalue", "land_value", "landValue", "land_value_norm", "landvalue_norm"],
        "traffic": ["traffic", "traffic_flow", "commute_traffic", "traffic_util", "congestion"],
        "goods_fill": ["goods_fill", "goodsfill", "goods", "goods_pressure", "goods_util"],
        "flood_depth": ["flood_depth", "flooddepth", "sea_depth", "flood"],
        "ponding_depth": ["ponding_depth", "pondingdepth", "pond_depth", "ponding"],
        "services": ["services", "service", "service_sat", "services_sat", "satisfaction_services"],
        "happiness": ["happiness", "happy"],
        "population": ["population", "pop", "residents"],
        "jobs": ["jobs", "employment"],
    }
    out: Dict[str, str] = {}
    for metric, cands in desired.items():
        col = _choose_column(headers, cands)
        if col:
            out[metric] = col
    return out


def load_tile_metrics_csv(path: Path) -> Optional[Tuple[List[Dict[str, str]], Dict[str, str], str, str]]:
    """
    Returns:
      rows, metric_map, x_col, y_col
    """
    if not path.exists():
        return None
    try:
        with path.open("r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            headers = reader.fieldnames or []
            x_col = _choose_column(headers, ["x", "tile_x", "ix", "col"])
            y_col = _choose_column(headers, ["y", "tile_y", "iy", "row"])
            if not x_col or not y_col:
                # fallback: first two integer-ish columns
                # We'll scan first row to decide.
                first = next(reader, None)
                if first is None:
                    return None
                # rebuild reader by reopening file
                f.seek(0)
                reader = csv.DictReader(f)
                headers = reader.fieldnames or []
                intish = []
                for h in headers:
                    v = first.get(h, "")
                    if _to_int(v) is not None:
                        intish.append(h)
                if len(intish) >= 2:
                    x_col, y_col = intish[0], intish[1]
                else:
                    # give up
                    return None
            metric_map = _discover_metric_columns(headers)
            rows = [dict(r) for r in reader]
        return rows, metric_map, x_col, y_col
    except Exception:
        return None


# -----------------------------
# Spatial analytics (Moran's I)
# -----------------------------

def morans_i_grid(values: List[Optional[float]], w: int, h: int) -> Optional[float]:
    """
    Moran's I for a grid stored row-major in `values`, using 4-neighbor contiguity.
    Missing cells (None / non-finite) are skipped.

    Returns None if insufficient data or degenerate variance.
    """
    if w <= 0 or h <= 0:
        return None
    # collect valid indices
    idxs = [i for i, v in enumerate(values) if v is not None and math.isfinite(v)]
    n = len(idxs)
    if n < 3:
        return None
    mean = statistics.fmean(values[i] for i in idxs)  # type: ignore[arg-type]
    # denominator
    denom = 0.0
    for i in idxs:
        v = values[i]
        assert v is not None
        dv = v - mean
        denom += dv * dv
    if denom <= 0.0:
        return None

    num = 0.0
    s0 = 0.0

    def at(x: int, y: int) -> Optional[float]:
        v = values[y * w + x]
        if v is None or not math.isfinite(v):
            return None
        return v

    # Iterate undirected neighbor edges once (right + down), count as symmetric (x2).
    for y in range(h):
        for x in range(w):
            v = at(x, y)
            if v is None:
                continue
            dv = v - mean
            if x + 1 < w:
                vr = at(x + 1, y)
                if vr is not None:
                    dvr = vr - mean
                    num += 2.0 * dv * dvr
                    s0 += 2.0
            if y + 1 < h:
                vd = at(x, y + 1)
                if vd is not None:
                    dvd = vd - mean
                    num += 2.0 * dv * dvd
                    s0 += 2.0

    if s0 <= 0.0:
        return None
    return (n / s0) * (num / denom)


# -----------------------------
# Main report generation
# -----------------------------

def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Compute analytics from a ProcIsoCity dossier folder (stdlib-only).")
    ap.add_argument("dossier_dir", type=str, help="Path to a dossier folder (contains summary.json, ticks.csv, tile_metrics.csv).")
    ap.add_argument("--summary", type=str, default="", help="Override path to summary.json")
    ap.add_argument("--ticks", type=str, default="", help="Override path to ticks.csv")
    ap.add_argument("--tiles", type=str, default="", help="Override path to tile_metrics.csv")
    ap.add_argument("--out", type=str, default="insights.json", help="Output JSON path (default: insights.json)")
    ap.add_argument("--md", type=str, default="insights.md", help="Output Markdown path (default: insights.md)")
    ap.add_argument("--topk", type=int, default=20, help="Top-K hotspots to report per metric (default: 20)")
    args = ap.parse_args(argv)

    dossier = Path(args.dossier_dir)
    if not dossier.exists():
        raise SystemExit(f"dossier_dir does not exist: {dossier}")

    summary_path = Path(args.summary) if args.summary else dossier / "summary.json"
    ticks_path = Path(args.ticks) if args.ticks else dossier / "ticks.csv"
    tiles_path = Path(args.tiles) if args.tiles else dossier / "tile_metrics.csv"

    summary = load_summary_json(summary_path)
    ticks = load_ticks_csv(ticks_path)

    tile_pack = load_tile_metrics_csv(tiles_path)
    if tile_pack is None:
        raise SystemExit(f"Could not read tile_metrics.csv at: {tiles_path}")
    tile_rows, metric_map, x_col, y_col = tile_pack

    # Determine grid bounds
    xs: List[int] = []
    ys: List[int] = []
    for r in tile_rows:
        xi = _to_int(r.get(x_col, ""))
        yi = _to_int(r.get(y_col, ""))
        if xi is None or yi is None:
            continue
        xs.append(xi)
        ys.append(yi)
    if not xs or not ys:
        raise SystemExit("tile_metrics.csv did not contain usable x/y coordinates.")
    w = max(xs) + 1
    h = max(ys) + 1

    # Pull metric arrays
    # For each discovered metric, gather values in the row order. Also keep a grid for land_value.
    metric_values: Dict[str, List[Optional[float]]] = {m: [] for m in metric_map.keys()}
    coords: List[Tuple[int, int]] = []

    for r in tile_rows:
        xi = _to_int(r.get(x_col, ""))
        yi = _to_int(r.get(y_col, ""))
        if xi is None or yi is None:
            # keep alignment but mark coord invalid
            coords.append((-1, -1))
        else:
            coords.append((xi, yi))
        for m, col in metric_map.items():
            metric_values[m].append(_to_float(r.get(col, "")))

    # Compute stats + gini for each metric
    stats_json: Dict[str, Any] = {}
    for m, vals in metric_values.items():
        xs_f = [v for v in vals if v is not None and math.isfinite(v)]
        st = compute_stats(xs_f)
        stats_json[m] = {
            "column": metric_map[m],
            "stats": st.to_json() if st else None,
            "gini": gini([v for v in xs_f if v is not None]),  # type: ignore[arg-type]
        }

    # Hotspots
    def topk(vals: List[Optional[float]], k: int) -> List[Dict[str, Any]]:
        # simple O(n log n) sort (n is manageable), filtering invalid coords
        items: List[Tuple[float, int, int]] = []
        for (x, y), v in zip(coords, vals):
            if x < 0 or y < 0 or v is None or not math.isfinite(v):
                continue
            items.append((v, x, y))
        items.sort(reverse=True, key=lambda t: t[0])
        out = []
        for v, x, y in items[: max(0, k)]:
            out.append({"x": x, "y": y, "value": v})
        return out

    hotspots: Dict[str, Any] = {}
    for m, vals in metric_values.items():
        hotspots[m] = topk(vals, args.topk)

    # Moran's I for land value if available
    moran = None
    if "land_value" in metric_map:
        # build grid in row-major; default None
        grid: List[Optional[float]] = [None] * (w * h)
        land_vals = metric_values["land_value"]
        for (x, y), v in zip(coords, land_vals):
            if x < 0 or y < 0:
                continue
            grid[y * w + x] = v if (v is not None and math.isfinite(v)) else None
        moran = morans_i_grid(grid, w, h)

    # Correlations (pairwise among key metrics)
    corr: Dict[str, Any] = {}
    key_metrics = [m for m in ("land_value", "traffic", "goods_fill", "flood_depth", "ponding_depth", "services") if m in metric_map]
    for i in range(len(key_metrics)):
        for j in range(i + 1, len(key_metrics)):
            a = key_metrics[i]
            b = key_metrics[j]
            c = pearson_corr(
                [v if v is not None else None for v in metric_values[a]],  # type: ignore[list-item]
                [v if v is not None else None for v in metric_values[b]],  # type: ignore[list-item]
            )
            corr[f"{a}__vs__{b}"] = c

    # Ticks summary
    tick_summary: Dict[str, Any] = {}
    if ticks:
        # discover day column
        headers = list(ticks[0].keys()) if ticks else []
        day_col = _choose_column(headers, ["day", "tick", "t"])
        tick_summary["rows"] = len(ticks)
        if day_col:
            tick_summary["day_col"] = day_col
        # choose a few common series to sparkline
        series_candidates = {
            "population": ["population", "pop", "residents"],
            "jobs": ["jobs"],
            "money": ["money", "cash", "funds"],
            "happiness": ["happiness", "happy"],
        }
        series_out: Dict[str, Any] = {}
        for name, cands in series_candidates.items():
            col = _choose_column(headers, cands)
            if not col:
                continue
            vals = [_to_float(r.get(col, "")) for r in ticks]
            series_out[name] = {
                "column": col,
                "spark": sparkline([v if v is not None else float("nan") for v in vals]),
                "start": vals[0] if vals else None,
                "end": vals[-1] if vals else None,
                "min": min([v for v in vals if v is not None], default=None),
                "max": max([v for v in vals if v is not None], default=None),
            }
        tick_summary["series"] = series_out

    # Final JSON payload
    payload: Dict[str, Any] = {
        "dossier_dir": str(dossier),
        "inputs": {
            "summary_json": str(summary_path) if summary_path.exists() else None,
            "ticks_csv": str(ticks_path) if ticks_path.exists() else None,
            "tile_metrics_csv": str(tiles_path),
        },
        "grid": {"width": w, "height": h, "tiles": len(tile_rows)},
        "metrics": stats_json,
        "hotspots": hotspots,
        "spatial": {"morans_i_land_value_4nbr": moran},
        "correlations": corr,
        "ticks": tick_summary,
        "summary": summary,
    }

    out_json = Path(args.out)
    out_md = Path(args.md)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_md.parent.mkdir(parents=True, exist_ok=True)

    with out_json.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, sort_keys=False)

    # Markdown report
    md_lines: List[str] = []
    md_lines.append("# ProcIsoCity Insights\n")
    md_lines.append(f"- Dossier: `{dossier}`\n")
    md_lines.append(f"- Grid: **{w} x {h}**  (tiles: {len(tile_rows)})\n")
    if moran is not None:
        md_lines.append(f"- Moran's I (land value, 4-neighbor): **{moran:.4f}**  _(>0 clustered, ~0 random, <0 dispersed)_\n")
    md_lines.append("\n## Metric summaries\n")
    for m in key_metrics:
        info = stats_json.get(m, {})
        st = info.get("stats")
        g = info.get("gini")
        md_lines.append(f"### `{m}`  (column `{info.get('column')}`)\n")
        if st:
            md_lines.append(f"- count: {st['count']}\n")
            md_lines.append(f"- mean: {st['mean']:.4f}, stdev: {st['stdev']:.4f}\n")
            md_lines.append(f"- p10/p50/p90/p99: {st['p10']:.4f} / {st['p50']:.4f} / {st['p90']:.4f} / {st['p99']:.4f}\n")
            md_lines.append(f"- min/max: {st['min']:.4f} / {st['max']:.4f}\n")
        if g is not None:
            md_lines.append(f"- Gini: **{g:.4f}**\n")
        hs = hotspots.get(m, [])
        if hs:
            md_lines.append(f"- Top {min(args.topk, len(hs))} hotspots (x, y, value):\n")
            md_lines.append("  - " + "\n  - ".join([f"({p['x']}, {p['y']}): {p['value']:.4f}" for p in hs[: min(10, len(hs))]]) + "\n")
        md_lines.append("\n")

    if corr:
        md_lines.append("## Correlations (Pearson)\n")
        for k, v in sorted(corr.items(), key=lambda kv: (kv[1] is None, -(kv[1] or 0.0))):
            if v is None:
                continue
            md_lines.append(f"- `{k}`: **{v:.4f}**\n")
        md_lines.append("\n")

    if tick_summary.get("series"):
        md_lines.append("## Time series (ticks.csv)\n")
        for name, s in tick_summary["series"].items():
            md_lines.append(f"### `{name}` (column `{s['column']}`)\n")
            md_lines.append(f"`{s['spark']}`\n\n")
            md_lines.append(f"- start: {s['start']}\n- end: {s['end']}\n- min: {s['min']}\n- max: {s['max']}\n\n")

    md_lines.append("## Notes\n")
    md_lines.append("- This tool is schema-tolerant: if a metric column is not found, it is skipped.\n")
    md_lines.append("- If you rename columns in exporter code, consider updating the heuristics in `tools/procisocity_insights.py`.\n")

    with out_md.open("w", encoding="utf-8") as f:
        f.write("\n".join(md_lines))

    print(f"Wrote: {out_json}")
    print(f"Wrote: {out_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
