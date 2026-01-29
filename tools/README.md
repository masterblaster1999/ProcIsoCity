# Tools

This folder contains optional, zero-dependency helper utilities that operate on ProcIsoCity artifacts.

## `procisocity_insights.py`

A stdlib-only analytics tool for **dossier** exports (the folder produced by `proc_isocity_dossier`).

It reads (when present):

- `summary.json`
- `ticks.csv`
- `tile_metrics.csv`

…and produces:

- `insights.json` (machine-readable)
- `insights.md` (human-readable report)

### Example

```bash
python tools/procisocity_insights.py dossier_out --out dossier_out/insights.json --md dossier_out/insights.md
```

### What it computes

- Basic descriptive stats for discovered tile metrics (min/percentiles/max, mean/stdev)
- Gini coefficient (inequality) for each metric
- Top-K "hotspot" tiles for each metric
- Moran's I (4-neighbor) for land value (spatial clustering indicator)
- Pearson correlations between key metrics (if multiple exist)
- Simple tick-series sparklines from `ticks.csv`

### Notes

- The tool intentionally has **no external dependencies** (no numpy/pandas/matplotlib).
- It is schema-tolerant: it discovers column names by heuristics; if a column isn't found, it is skipped.
