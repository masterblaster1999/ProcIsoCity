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
- Top-K "hotspot" (high) tiles **and** "coldspot" (low) tiles for each metric
- Moran's I (4-neighbor) for multiple key metrics when available (spatial clustering indicator)
- Pearson correlations between key metrics (ranked by absolute strength in the Markdown report)
- Simple tick-series sparklines from `ticks.csv` (auto-discovers common series, including newer fields when present)

### Notes

- The tool intentionally has **no external dependencies** (no numpy/pandas/matplotlib).
- It is schema-tolerant: it discovers column names by heuristics; if a column isn't found, it is skipped.

## `procisocity_compare.py`

A stdlib-only **regression comparison** tool for two dossier exports.

It reads (when present) from each dossier:

- `summary.json`
- `ticks.csv`
- `tile_metrics.csv`

…and produces:

- `compare.json` (machine-readable deltas + per-metric summaries)
- `compare.md` (human-readable report)

### Example

```bash
# Compare two dossier folders and write outputs next to the current working directory
python tools/procisocity_compare.py dossier_A dossier_B

# Name the runs and write outputs into the candidate dossier folder
python tools/procisocity_compare.py dossier_A dossier_B \
  --label-a baseline --label-b candidate \
  --out dossier_B/compare.json --md dossier_B/compare.md

# Additionally list the top per-tile deltas for a couple of metrics
python tools/procisocity_compare.py dossier_A dossier_B \
  --metric land_value --metric livability --top 25
```

### What it compares

- **summary.json**: seed, size, hash (when available)
- **ticks.csv**: numeric column intersection; ranks metrics by the absolute delta on the last common day
- **tile_metrics.csv**: distribution deltas (mean/p50/p90/p99 + Gini); optionally top per-tile deltas for selected metrics

### Notes

- The tool is schema-tolerant: it only compares columns that exist in both dossiers.
- Per-tile deltas require `tile_metrics.csv` in both inputs and a consistent `(x,y)` grid.

## `proc_isocity_evolve` (C++ CLI executable)

A headless **procedural generation auto-tuner**: it searches the ProcIsoCity parameter space (seed + a curated set
of high-impact `ProcGenConfig` knobs) to find cities that score well under an objective function.

It runs `GenerateWorld(...)` + `RunAutoBuild(...)` many times, ranks the results, and can optionally write the **best**
candidate as a save file and/or a full **dossier** export.

### Example

```bash
# Maximize a custom objective: big city, happy residents, low congestion.
proc_isocity_evolve --size 96x96 --days 120 --population 80 --generations 12 \
  --score "population*(0.5+happiness) - 120*trafficCongestion - 0.05*avgCommuteTime" \
  --best-dossier out/best_dossier
```

### Notes

- The `--score` expression can reference many `Stats` fields by name (e.g. `population`, `money`, `avgLandValue`,
  `goodsSatisfaction`, `servicesOverallSatisfaction`, `transitModeShare`, `airPollutionResidentAvg01`, etc.).
- Output genomes are compact share strings you can copy-paste back into `--genome` for exact reproduction.

## `proc_isocity_chronicle` (C++ CLI executable)

A headless **procedural city newspaper** generator.

It writes:

- `chronicle.json` (machine-readable daily headline feed)
- `chronicle.md` (human-readable report)

This is also exported automatically by `proc_isocity_dossier`, and displayed in the dossier `index.html` viewer.

### Example

```bash
# Generate a chronicle from an existing save (and optionally simulate forward)
proc_isocity_chronicle --load out/save.bin --days 60 --out out/chronicle.json

# Generate from a seed and grow it with AutoBuild
proc_isocity_chronicle --seed 123 --size 128x128 --autobuild-days 120 --out out/chronicle.json
```

### Notes

- Entries include `tone` (good/neutral/bad/alert), `tags`, and a small actionable `tip`.
- Generation is deterministic: same input -> same chronicle.
