#!/usr/bin/env python3
"""procisocity_compare.py

Stdlib-only regression comparison tool for ProcIsoCity dossier exports.

Given two dossier directories (A and B), this tool attempts to load:
  - summary.json
  - ticks.csv
  - tile_metrics.csv

…and produces:
  - compare.json (machine-readable)
  - compare.md   (human-readable)

The tool is intentionally schema-tolerant:
  - It discovers numeric columns by parsing values.
  - It compares only the intersection of columns present in both inputs.
  - If an artifact is missing, that section is skipped.

Examples:

  # Minimal (writes compare.json/compare.md next to current working dir)
  python tools/procisocity_compare.py dossier_A dossier_B

  # Custom outputs + labels
  python tools/procisocity_compare.py dossier_A dossier_B \
    --label-a baseline --label-b candidate \
    --out dossier_B/compare.json --md dossier_B/compare.md

  # Inspect top per-tile deltas for selected metrics
  python tools/procisocity_compare.py dossier_A dossier_B \
    --metric land_value --metric livability --top 25

Notes:
  - For per-tile comparisons, both dossiers must include tile_metrics.csv and share the
    same tile grid (same width/height, or at least the same set of (x,y) keys).
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
# Basic parsing helpers
# -----------------------------


def _to_float(s: str) -> Optional[float]:
    if s is None:
        return None
    s = s.strip()
    if s == "" or s.lower() in {"nan", "null", "none"}:
        return None
    try:
        v = float(s)
    except Exception:
        return None
    if not math.isfinite(v):
        return None
    return v


def _to_int(s: str) -> Optional[int]:
    v = _to_float(s)
    if v is None:
        return None
    try:
        return int(v)
    except Exception:
        return None


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def gini(values: Sequence[float]) -> Optional[float]:
    """Compute a basic Gini coefficient in [0,1] for non-negative values."""
    xs = [v for v in values if v is not None and math.isfinite(v)]
    if not xs:
        return None

    mn = min(xs)
    if mn < 0.0:
        xs = [v - mn for v in xs]

    xs.sort()
    n = len(xs)
    total = sum(xs)
    if total <= 0.0:
        return 0.0

    cum = 0.0
    for i, x in enumerate(xs, start=1):
        cum += i * x

    g = (2.0 * cum) / (n * total) - (n + 1.0) / n
    return clamp(g, 0.0, 1.0)


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


def compute_stats(values: Sequence[Optional[float]]) -> Optional[MetricStats]:
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


def rmse(a: Sequence[Optional[float]], b: Sequence[Optional[float]]) -> Optional[float]:
    acc = 0.0
    n = 0
    for x, y in zip(a, b):
        if x is None or y is None:
            continue
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        d = x - y
        acc += d * d
        n += 1
    if n == 0:
        return None
    return math.sqrt(acc / n)


def read_csv_rows(path: Path) -> Tuple[List[str], List[Dict[str, str]]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        header = list(reader.fieldnames or [])
        rows: List[Dict[str, str]] = []
        for row in reader:
            rows.append({k: (v if v is not None else "") for k, v in row.items()})
        return header, rows


def discover_numeric_columns(rows: Sequence[Dict[str, str]], *, min_valid_frac: float = 0.75) -> List[str]:
    """Discover numeric columns by parsing floats.

    A column is considered numeric if at least min_valid_frac of non-empty values are parseable floats.
    """
    if not rows:
        return []

    cols = list(rows[0].keys())
    numeric: List[str] = []
    for c in cols:
        valid = 0
        seen = 0
        for r in rows:
            s = r.get(c, "")
            if s is None:
                continue
            s2 = s.strip()
            if s2 == "":
                continue
            seen += 1
            if _to_float(s2) is not None:
                valid += 1
        if seen == 0:
            continue
        if (valid / seen) >= min_valid_frac:
            numeric.append(c)
    return numeric


def load_json(path: Path) -> Optional[Dict[str, Any]]:
    try:
        with path.open("r", encoding="utf-8") as f:
            v = json.load(f)
        if isinstance(v, dict):
            return v
        return None
    except Exception:
        return None


def fmt_float(x: Optional[float], *, digits: int = 4) -> str:
    if x is None:
        return ""
    if not math.isfinite(x):
        return ""
    # Avoid -0.0000
    if abs(x) < 10 ** (-(digits + 1)):
        x = 0.0
    return f"{x:.{digits}f}"


def fmt_int(x: Optional[int]) -> str:
    return "" if x is None else str(x)


def safe_rel_delta(a: Optional[float], b: Optional[float]) -> Optional[float]:
    if a is None or b is None:
        return None
    if not (math.isfinite(a) and math.isfinite(b)):
        return None
    denom = abs(a)
    if denom < 1e-9:
        return None
    return (b - a) / denom


# -----------------------------
# Comparison logic
# -----------------------------


def compare_summary(a: Optional[Dict[str, Any]], b: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    out: Dict[str, Any] = {
        "present_a": a is not None,
        "present_b": b is not None,
        "same_seed": None,
        "same_size": None,
        "seed_a": None,
        "seed_b": None,
        "width_a": None,
        "width_b": None,
        "height_a": None,
        "height_b": None,
        "hash_a": None,
        "hash_b": None,
        "tool_a": None,
        "tool_b": None,
    }

    if a:
        out["seed_a"] = a.get("seed")
        out["width_a"] = a.get("width")
        out["height_a"] = a.get("height")
        out["hash_a"] = a.get("hash")
        out["tool_a"] = a.get("tool")
    if b:
        out["seed_b"] = b.get("seed")
        out["width_b"] = b.get("width")
        out["height_b"] = b.get("height")
        out["hash_b"] = b.get("hash")
        out["tool_b"] = b.get("tool")

    if a and b:
        out["same_seed"] = (out["seed_a"] == out["seed_b"]) and (out["seed_a"] is not None)
        out["same_size"] = (out["width_a"] == out["width_b"]) and (out["height_a"] == out["height_b"]) and (
            out["width_a"] is not None
        )

    return out


def compare_ticks(rows_a: Sequence[Dict[str, str]], rows_b: Sequence[Dict[str, str]]) -> Dict[str, Any]:
    out: Dict[str, Any] = {
        "present_a": bool(rows_a),
        "present_b": bool(rows_b),
        "common_days": 0,
        "day_min": None,
        "day_max": None,
        "metrics": [],
    }

    if not rows_a or not rows_b:
        return out

    # Identify day column.
    day_key = "day" if "day" in rows_a[0] and "day" in rows_b[0] else None
    if day_key is None:
        return out

    # Map day -> row.
    map_a: Dict[int, Dict[str, str]] = {}
    map_b: Dict[int, Dict[str, str]] = {}
    for r in rows_a:
        d = _to_int(r.get(day_key, ""))
        if d is None:
            continue
        map_a[d] = r
    for r in rows_b:
        d = _to_int(r.get(day_key, ""))
        if d is None:
            continue
        map_b[d] = r

    days = sorted(set(map_a.keys()).intersection(map_b.keys()))
    if not days:
        return out

    out["common_days"] = len(days)
    out["day_min"] = days[0]
    out["day_max"] = days[-1]

    # Numeric columns intersection (excluding day).
    num_a = set(discover_numeric_columns(rows_a))
    num_b = set(discover_numeric_columns(rows_b))
    cols = sorted((num_a & num_b) - {day_key})

    metrics: List[Dict[str, Any]] = []
    for c in cols:
        series_a: List[Optional[float]] = []
        series_b: List[Optional[float]] = []
        max_abs = 0.0
        max_abs_day: Optional[int] = None
        max_abs_delta: Optional[float] = None

        for d in days:
            va = _to_float(map_a[d].get(c, ""))
            vb = _to_float(map_b[d].get(c, ""))
            series_a.append(va)
            series_b.append(vb)
            if va is None or vb is None:
                continue
            delta = vb - va
            ad = abs(delta)
            if (max_abs_day is None) or (ad > max_abs):
                max_abs = ad
                max_abs_day = d
                max_abs_delta = delta

        # last day values
        a_last = _to_float(map_a[days[-1]].get(c, ""))
        b_last = _to_float(map_b[days[-1]].get(c, ""))

        # means
        a_stats = compute_stats(series_a)
        b_stats = compute_stats(series_b)

        metrics.append(
            {
                "metric": c,
                "a_last": a_last,
                "b_last": b_last,
                "delta_last": None if (a_last is None or b_last is None) else (b_last - a_last),
                "a_mean": None if a_stats is None else a_stats.mean,
                "b_mean": None if b_stats is None else b_stats.mean,
                "delta_mean": None
                if (a_stats is None or b_stats is None)
                else (b_stats.mean - a_stats.mean),
                "rmse": rmse(series_a, series_b),
                "max_abs_day": max_abs_day,
                "max_abs_delta": max_abs_delta,
            }
        )

    # Sort by absolute delta_last (descending) as a handy default.
    def sort_key(m: Dict[str, Any]) -> float:
        v = m.get("delta_last")
        if v is None:
            return -1.0
        return abs(float(v))

    metrics.sort(key=sort_key, reverse=True)
    out["metrics"] = metrics
    return out


def _infer_tile_key_cols(header: Sequence[str]) -> Tuple[Optional[str], Optional[str]]:
    # Dossier uses x,y. Allow a few variants.
    candidates_x = ["x", "tile_x", "tx"]
    candidates_y = ["y", "tile_y", "ty"]
    xcol = next((c for c in candidates_x if c in header), None)
    ycol = next((c for c in candidates_y if c in header), None)
    return xcol, ycol


def compare_tile_metrics(
    header_a: Sequence[str],
    rows_a: Sequence[Dict[str, str]],
    header_b: Sequence[str],
    rows_b: Sequence[Dict[str, str]],
    *,
    focus_metrics: Sequence[str],
    top_tiles: int,
) -> Dict[str, Any]:
    out: Dict[str, Any] = {
        "present_a": bool(rows_a),
        "present_b": bool(rows_b),
        "metrics": [],
        "tile_deltas": {},
        "note": None,
    }

    if not rows_a or not rows_b:
        return out

    x_a, y_a = _infer_tile_key_cols(header_a)
    x_b, y_b = _infer_tile_key_cols(header_b)
    if x_a is None or y_a is None or x_b is None or y_b is None:
        out["note"] = "Could not infer tile key columns (x/y)"
        return out

    num_a = set(discover_numeric_columns(rows_a, min_valid_frac=0.90))
    num_b = set(discover_numeric_columns(rows_b, min_valid_frac=0.90))

    # Exclude structural / categorical fields.
    #
    # We keep most numeric columns (including masks and counts) since they are often useful
    # regression signals. Only remove the coordinates and categorical identifiers that are
    # not meaningfully comparable as continuous metrics.
    exclude = {
        x_a,
        y_a,
        x_b,
        y_b,
        "level",
        "district",
        "variation",
    }
    cols = sorted((num_a & num_b) - exclude)

    metrics: List[Dict[str, Any]] = []
    for c in cols:
        series_a = [_to_float(r.get(c, "")) for r in rows_a]
        series_b = [_to_float(r.get(c, "")) for r in rows_b]
        st_a = compute_stats(series_a)
        st_b = compute_stats(series_b)
        if st_a is None or st_b is None:
            continue
        metrics.append(
            {
                "metric": c,
                "a": st_a.to_json(),
                "b": st_b.to_json(),
                "delta_mean": st_b.mean - st_a.mean,
                "delta_p50": st_b.p50 - st_a.p50,
                "delta_p90": st_b.p90 - st_a.p90,
                "delta_p99": st_b.p99 - st_a.p99,
                "gini_a": gini([v for v in series_a if v is not None]),
                "gini_b": gini([v for v in series_b if v is not None]),
            }
        )

    # Sort by absolute delta_mean.
    metrics.sort(key=lambda m: abs(float(m.get("delta_mean", 0.0))), reverse=True)
    out["metrics"] = metrics

    # Optional per-tile deltas for selected metrics.
    focus = [m for m in focus_metrics if m]
    if focus:
        # Index both by (x,y). For dossier exports, rows are already in row-major order,
        # but we avoid relying on that.
        idx_a: Dict[Tuple[int, int], Dict[str, str]] = {}
        idx_b: Dict[Tuple[int, int], Dict[str, str]] = {}
        for r in rows_a:
            x = _to_int(r.get(x_a, ""))
            y = _to_int(r.get(y_a, ""))
            if x is None or y is None:
                continue
            idx_a[(x, y)] = r
        for r in rows_b:
            x = _to_int(r.get(x_b, ""))
            y = _to_int(r.get(y_b, ""))
            if x is None or y is None:
                continue
            idx_b[(x, y)] = r

        keys = idx_a.keys() & idx_b.keys()
        if not keys:
            out["note"] = "No overlapping (x,y) tiles between dossiers"
            return out

        tile_deltas: Dict[str, Any] = {}
        for metric in focus:
            if metric not in cols:
                tile_deltas[metric] = {"note": "metric not present in both dossiers"}
                continue

            best: List[Tuple[float, int, int, float, float]] = []
            # Keep top-N by abs(delta)
            for (x, y) in keys:
                va = _to_float(idx_a[(x, y)].get(metric, ""))
                vb = _to_float(idx_b[(x, y)].get(metric, ""))
                if va is None or vb is None:
                    continue
                d = vb - va
                ad = abs(d)
                best.append((ad, x, y, va, vb))

            best.sort(key=lambda t: t[0], reverse=True)
            best = best[: max(0, int(top_tiles))]

            tile_deltas[metric] = {
                "tiles": [
                    {
                        "x": x,
                        "y": y,
                        "a": va,
                        "b": vb,
                        "delta": vb - va,
                    }
                    for (_, x, y, va, vb) in best
                ]
            }

        out["tile_deltas"] = tile_deltas

    return out


def render_markdown(
    label_a: str,
    label_b: str,
    summary_cmp: Dict[str, Any],
    ticks_cmp: Dict[str, Any],
    tiles_cmp: Dict[str, Any],
    *,
    top_metrics: int,
    top_tiles: int,
    focus_metrics: Sequence[str],
) -> str:
    lines: List[str] = []

    lines.append(f"# ProcIsoCity dossier comparison: {label_a} vs {label_b}")
    lines.append("")

    # ---- Summary ----
    lines.append("## Summary")
    if not (summary_cmp.get("present_a") and summary_cmp.get("present_b")):
        lines.append("- summary.json missing for one or both dossiers")
    else:
        seed_a = summary_cmp.get("seed_a")
        seed_b = summary_cmp.get("seed_b")
        w_a = summary_cmp.get("width_a")
        h_a = summary_cmp.get("height_a")
        w_b = summary_cmp.get("width_b")
        h_b = summary_cmp.get("height_b")
        hash_a = summary_cmp.get("hash_a")
        hash_b = summary_cmp.get("hash_b")

        lines.append(f"- seed: {seed_a} vs {seed_b}")
        lines.append(f"- size: {w_a}x{h_a} vs {w_b}x{h_b}")
        if hash_a is not None or hash_b is not None:
            lines.append(f"- hash: `{hash_a}` vs `{hash_b}`")
        if summary_cmp.get("same_seed") is False:
            lines.append("- ⚠️ Seeds differ")
        if summary_cmp.get("same_size") is False:
            lines.append("- ⚠️ Map sizes differ")

    lines.append("")

    # ---- Ticks ----
    lines.append("## Tick series (ticks.csv)")
    if not (ticks_cmp.get("present_a") and ticks_cmp.get("present_b")):
        lines.append("- ticks.csv missing for one or both dossiers")
    else:
        lines.append(
            f"- compared days: {ticks_cmp.get('day_min')} .. {ticks_cmp.get('day_max')} (n={ticks_cmp.get('common_days')})"
        )

        metrics: List[Dict[str, Any]] = list(ticks_cmp.get("metrics") or [])
        if not metrics:
            lines.append("- no comparable numeric metrics found")
        else:
            lines.append("")
            lines.append(f"Top {min(top_metrics, len(metrics))} metrics by |delta_last| (last common day):")
            lines.append("")
            lines.append("| metric | a_last | b_last | delta_last | delta_mean | rmse | max_abs_day | max_abs_delta |")
            lines.append("|---|---:|---:|---:|---:|---:|---:|---:|")
            for m in metrics[: max(0, int(top_metrics))]:
                lines.append(
                    "| {metric} | {a_last} | {b_last} | {d_last} | {d_mean} | {rmse} | {mad} | {madelta} |".format(
                        metric=m.get("metric", ""),
                        a_last=fmt_float(m.get("a_last")),
                        b_last=fmt_float(m.get("b_last")),
                        d_last=fmt_float(m.get("delta_last")),
                        d_mean=fmt_float(m.get("delta_mean")),
                        rmse=fmt_float(m.get("rmse")),
                        mad=fmt_int(m.get("max_abs_day")),
                        madelta=fmt_float(m.get("max_abs_delta")),
                    )
                )

    lines.append("")

    # ---- Tile metrics ----
    lines.append("## Tile metrics (tile_metrics.csv)")
    if not (tiles_cmp.get("present_a") and tiles_cmp.get("present_b")):
        lines.append("- tile_metrics.csv missing for one or both dossiers")
    else:
        note = tiles_cmp.get("note")
        if note:
            lines.append(f"- note: {note}")

        metrics = list(tiles_cmp.get("metrics") or [])
        if not metrics:
            lines.append("- no comparable numeric metrics found")
        else:
            lines.append("")
            lines.append("Top metrics by |delta_mean|:")
            lines.append("")
            lines.append("| metric | delta_mean | delta_p50 | delta_p90 | delta_p99 | gini_a | gini_b |")
            lines.append("|---|---:|---:|---:|---:|---:|---:|")
            for m in metrics[: max(0, int(top_metrics))]:
                lines.append(
                    "| {metric} | {dm} | {dp50} | {dp90} | {dp99} | {ga} | {gb} |".format(
                        metric=m.get("metric", ""),
                        dm=fmt_float(m.get("delta_mean")),
                        dp50=fmt_float(m.get("delta_p50")),
                        dp90=fmt_float(m.get("delta_p90")),
                        dp99=fmt_float(m.get("delta_p99")),
                        ga=fmt_float(m.get("gini_a")),
                        gb=fmt_float(m.get("gini_b")),
                    )
                )

        # Per-tile deltas
        if focus_metrics:
            lines.append("")
            lines.append("### Top per-tile deltas")
            lines.append("")
            tile_deltas = tiles_cmp.get("tile_deltas") or {}
            for metric in focus_metrics:
                lines.append(f"#### {metric}")
                entry = tile_deltas.get(metric)
                if not entry:
                    lines.append("- (no data)")
                    lines.append("")
                    continue
                if "note" in entry:
                    lines.append(f"- {entry['note']}")
                    lines.append("")
                    continue

                tiles = entry.get("tiles") or []
                if not tiles:
                    lines.append("- (no comparable tiles)")
                    lines.append("")
                    continue

                lines.append(f"Top {min(top_tiles, len(tiles))} tiles by |delta|:")
                lines.append("")
                lines.append("| x | y | a | b | delta |")
                lines.append("|---:|---:|---:|---:|---:|")
                for t in tiles[: max(0, int(top_tiles))]:
                    lines.append(
                        "| {x} | {y} | {a} | {b} | {d} |".format(
                            x=t.get("x", ""),
                            y=t.get("y", ""),
                            a=fmt_float(t.get("a")),
                            b=fmt_float(t.get("b")),
                            d=fmt_float(t.get("delta")),
                        )
                    )
                lines.append("")

    lines.append("")
    lines.append("---")
    lines.append("Generated by `tools/procisocity_compare.py` (stdlib-only).")

    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="Compare two ProcIsoCity dossier exports (stdlib-only).")
    ap.add_argument("a", type=str, help="Path to dossier directory A (baseline)")
    ap.add_argument("b", type=str, help="Path to dossier directory B (candidate)")
    ap.add_argument("--label-a", type=str, default="A", help="Label for dossier A")
    ap.add_argument("--label-b", type=str, default="B", help="Label for dossier B")
    ap.add_argument("--out", type=str, default="compare.json", help="Output JSON path")
    ap.add_argument("--md", type=str, default="compare.md", help="Output Markdown path")
    ap.add_argument(
        "--top",
        type=int,
        default=20,
        help="How many top metrics/tiles to include in reports (default: 20)",
    )
    ap.add_argument(
        "--metric",
        action="append",
        default=[],
        help="Metric to compute top per-tile deltas for (repeatable). Requires tile_metrics.csv in both dossiers.",
    )
    args = ap.parse_args()

    dir_a = Path(args.a)
    dir_b = Path(args.b)

    # Resolve to absolute for the JSON report.
    dir_a_abs = dir_a.resolve() if dir_a.exists() else dir_a
    dir_b_abs = dir_b.resolve() if dir_b.exists() else dir_b

    # Inputs
    sum_a = load_json(dir_a / "summary.json")
    sum_b = load_json(dir_b / "summary.json")

    header_ticks_a: List[str] = []
    header_ticks_b: List[str] = []
    rows_ticks_a: List[Dict[str, str]] = []
    rows_ticks_b: List[Dict[str, str]] = []

    try:
        if (dir_a / "ticks.csv").exists():
            header_ticks_a, rows_ticks_a = read_csv_rows(dir_a / "ticks.csv")
        if (dir_b / "ticks.csv").exists():
            header_ticks_b, rows_ticks_b = read_csv_rows(dir_b / "ticks.csv")
    except Exception:
        # Keep missing/parse errors as empty.
        header_ticks_a, rows_ticks_a = [], []
        header_ticks_b, rows_ticks_b = [], []

    header_tiles_a: List[str] = []
    header_tiles_b: List[str] = []
    rows_tiles_a: List[Dict[str, str]] = []
    rows_tiles_b: List[Dict[str, str]] = []

    try:
        if (dir_a / "tile_metrics.csv").exists():
            header_tiles_a, rows_tiles_a = read_csv_rows(dir_a / "tile_metrics.csv")
        if (dir_b / "tile_metrics.csv").exists():
            header_tiles_b, rows_tiles_b = read_csv_rows(dir_b / "tile_metrics.csv")
    except Exception:
        header_tiles_a, rows_tiles_a = [], []
        header_tiles_b, rows_tiles_b = [], []

    # Comparisons
    summary_cmp = compare_summary(sum_a, sum_b)
    ticks_cmp = compare_ticks(rows_ticks_a, rows_ticks_b)
    tiles_cmp = compare_tile_metrics(
        header_tiles_a,
        rows_tiles_a,
        header_tiles_b,
        rows_tiles_b,
        focus_metrics=list(args.metric or []),
        top_tiles=max(1, int(args.top)),
    )

    # Output JSON
    out_obj: Dict[str, Any] = {
        "tool": "procisocity_compare",
        "dossier_a": str(dir_a_abs),
        "dossier_b": str(dir_b_abs),
        "label_a": args.label_a,
        "label_b": args.label_b,
        "summary": summary_cmp,
        "ticks": ticks_cmp,
        "tiles": tiles_cmp,
    }

    out_path = Path(args.out)
    md_path = Path(args.md)

    try:
        os.makedirs(out_path.parent, exist_ok=True)
    except Exception:
        pass

    with out_path.open("w", encoding="utf-8") as f:
        json.dump(out_obj, f, indent=2, sort_keys=False)
        f.write("\n")

    # Output Markdown
    try:
        os.makedirs(md_path.parent, exist_ok=True)
    except Exception:
        pass

    md_text = render_markdown(
        args.label_a,
        args.label_b,
        summary_cmp,
        ticks_cmp,
        tiles_cmp,
        top_metrics=max(1, int(args.top)),
        top_tiles=max(1, int(args.top)),
        focus_metrics=list(args.metric or []),
    )

    with md_path.open("w", encoding="utf-8") as f:
        f.write(md_text)
        if not md_text.endswith("\n"):
            f.write("\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
