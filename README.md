# ProcIsoCity

A tiny-but-real **procedural, open-source isometric city-building simulation** starter project in **C++20**.

**Goal of this repo:** a clean *initial commit* that already has:
- an isometric tile renderer
- deterministic procedural terrain + road + zoning generation
- a tiny simulation loop (population, jobs, happiness, money)
- *zero* external art assets — **all tile textures are generated at runtime**

Built on top of **raylib** (simple, cross-platform). Controls and code are intentionally straightforward so you can evolve it into a full city builder.

---

## Build

### 1) Clone

```bash
git clone <your fork url> ProcIsoCity
cd ProcIsoCity
```

### 2) Configure + build (CMake)

```bash
# Headless CLI tools (offline-capable; no raylib)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# Interactive app (requires raylib; may download via FetchContent unless configured otherwise)
cmake -S . -B build-app -DCMAKE_BUILD_TYPE=Release -DPROCISOCITY_BUILD_APP=ON
cmake --build build-app -j
```

### CMake presets (optional)

If you have **CMake 3.22+**, you can use the included `CMakePresets.json` for common builds:

```bash
# Default (headless CLI, no raylib)
cmake --preset default
cmake --build --preset default --parallel

# Interactive app (requires raylib; may use FetchContent unless you set PROCISOCITY_USE_SYSTEM_RAYLIB=ON)
cmake --preset app
cmake --build --preset app --parallel

# Headless tests (no raylib)
cmake --preset tests
cmake --build --preset tests --parallel
ctest --preset tests

# CI / fast lite tests (no raylib, CLI off, glTF export off)
cmake --preset ci
cmake --build --preset ci --target proc_isocity_zip_tests --parallel
ctest --preset ci
```

Presets also enable `compile_commands.json` by default (handy for editor tooling).

### 3) Run

```bash
./build/proc_isocity
```

### In-game dev console (F4)

The interactive app ships with a small dev console (toggle with **F4**) that now exposes a couple of the headless tools directly in-game:

- **City dossier export** (same artifact format as `proc_isocity_dossier`):

  ```
  dossier
  dossier captures/my_dossier scale=4 iso=1 3d=0
  dossier captures/my_dossier layers=terrain,overlay,landvalue,traffic,goods_fill,district,flood_depth,ponding_depth
  ```

- **Seed mining** (same core as `proc_isocity_mine`), with background auto-stepping while the game is open:

  ```
  mine begin 500 resilient days=180
  mine status
  mine top 20 diverse=1
  mine load 1
  mine export captures/mine_top.json
  ```

Exports are written to the `captures/` folder by default.

### City Lab panel (Ctrl+F2)

There's also a small in-game UI panel to run the same headless workflows **without typing console commands**:

- start/pause seed mining, browse the current top-ranked seeds, and load one into the live game
- export a **city dossier** for the current world (or batch-export dossiers for mined seeds)

All exports still go to `captures/` by default.

### 4) Tests (headless, optional)

You can build and run a small set of **headless** tests (no raylib dependency). This is handy for CI or for quickly validating core logic changes on machines without graphics/system deps.

```bash
cmake -S . -B build-tests -DPROCISOCITY_BUILD_APP=OFF -DPROCISOCITY_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-tests -j
ctest --test-dir build-tests --output-on-failure
```

### CMake options

- `PROCISOCITY_BUILD_APP` (default: OFF) — build the interactive raylib app (`proc_isocity`).
- `PROCISOCITY_BUILD_TESTS` (default: OFF) — build `proc_isocity_tests` and enable `ctest`.
- `PROCISOCITY_BUILD_LITE_TESTS` (default: OFF) — build `proc_isocity_zip_tests` (fast unit tests) and enable `ctest` without building the full core library.
- `PROCISOCITY_BUILD_CLI` (default: ON) — build headless command-line tools (includes `proc_isocity_cli`, `proc_isocity_dossier`, `proc_isocity_mine`, and many more; see below).
- `PROCISOCITY_ENABLE_GLTF_EXPORT` (default: ON) — build the glTF/glb exporter. Set to OFF to reduce build time/memory; export commands will report a clear error.
- `PROCISOCITY_USE_SYSTEM_RAYLIB` (default: OFF) — when building the app, use a system raylib instead of FetchContent.

### Headless CLI tools (optional)

If `PROCISOCITY_BUILD_CLI=ON` (the default), CMake will also build a set of **headless** utilities that don't require raylib:

- `proc_isocity_cli`: generate a world (or load a save), step the simulation, and write artifacts:
  - JSON summary (`--out`)
  - per-tick CSV (`--csv`)
  - tile CSV (`--export-tiles-csv`)
  - PPM exports (`--export-ppm <layer> <out.ppm>`, repeatable) with optional upscaling (`--export-scale`)
  - isometric overview PPM exports (`--export-iso <layer> <out.ppm>`, repeatable) with controls:
    - `--iso-tile <WxH>` (default 16x8)
    - `--iso-height <N>` (max elevation in pixels; 0 disables vertical relief)
    - `--iso-margin <N>`, `--iso-grid <0|1>`, `--iso-cliffs <0|1>`
  - software-rendered **3D view** exports (`--export-3d <layer> <out.ppm|out.png>`, repeatable):
    - orthographic **isometric** (default) or perspective (`--3d-proj iso|persp`)
    - simple directional lighting (`--3d-light x,y,z`, `--3d-ambient`, `--3d-diffuse`)
    - anti-aliasing via SSAA (`--3d-ssaa 2` is a good starting point)
    - optional fog (`--3d-fog 1`, `--3d-fog-strength`, `--3d-fog-start`, `--3d-fog-end`)
    - optional post-fx (for a more "game-art" isometric look):
      - gamma-correct SSAA resolve (`--3d-gamma 1`)
      - screen-space ambient occlusion (`--3d-ao 1`, `--3d-ao-strength`, `--3d-ao-radius`, ...)
      - depth-based outline pass (`--3d-edge 1`, `--3d-edge-alpha`, `--3d-edge-threshold`, ...)
      - filmic tonemap / grade (`--3d-tonemap 1`, `--3d-exposure`, `--3d-contrast`, `--3d-saturation`, `--3d-vignette`)
      - ordered dithering + channel quantize (`--3d-dither 1`, `--3d-dither-bits`, `--3d-dither-strength`)
      - deterministic seed for AO/dither jitter (`--3d-post-seed`)
    - geometry knobs: `--3d-heightscale`, `--3d-quant`, `--3d-buildings`, `--3d-cliffs`
  - batch runs across multiple seeds (`--batch N`)

  Example:

  ```bash
  ./build/proc_isocity_cli --seed 1 --size 128x128 --days 200 \
    --out out_{seed}.json --csv ticks_{seed}.csv \
    --export-ppm overlay overlay_{seed}.ppm --export-scale 4 \
    --export-iso overlay iso_{seed}.ppm --iso-tile 16x8 --iso-height 14 \
    --export-3d overlay view3d_{seed}.png --3d-size 1280x720 --3d-proj iso --3d-ssaa 2 \
    --3d-ao 1 --3d-edge 1 --3d-tonemap 1 --3d-dither 1 --3d-post-seed 1337 --batch 3
  ```

- `proc_isocity_dossier`: one-command **"city dossier"** exporter — runs generation / simulation and then writes a
  self-contained folder containing:
  - `index.html` (offline-friendly viewer with layer switcher + optional per-tile inspector)
  - `summary.json` (includes config, hash, and tick snapshots)
  - `ticks.csv` (per-day stats)
  - `chronicle.json` / `chronicle.md` (procedural daily newspaper/advisor feed derived from ticks)
  - `tile_metrics.csv` (per-tile base + derived metrics: land value, traffic, goods, flood/ponding)
  - `world.bin` (final save)
  - a set of map images (`map_*.png`, optional `iso_*.png`, optional `view3d_overlay.png`)

  ```bash
  ./build/proc_isocity_dossier --seed 1 --size 128x128 --days 120 --out-dir dossier_out \
    --scale 4 --layers terrain,overlay,landvalue,traffic,goods_fill,district,flood_depth,ponding_depth \
    --iso 1 --iso-layers overlay,landvalue
  ```

- `proc_isocity_chronicle`: generate only the **City Chronicle** (no images) as `chronicle.json` / `chronicle.md`.
  Useful when you want a lightweight, deterministic daily narrative feed without exporting a full dossier.

  ```bash
  ./build/proc_isocity_chronicle --load save.bin --autobuild-days 120 --days 30 --out out/chronicle.json
  ```

- `proc_isocity_mine`: batch-run procedural seeds, simulate, compute a KPI CSV, and output the **best seeds** (optionally diversity-selected).
  Useful for finding "hero" cities for screenshots, or "pathological" cities for stress/regression tests (`--objective chaos`).

  ```bash
  # Find resilient, high-quality seeds (MMR diverse selection)
  ./build/proc_isocity_mine --seed-start 1 --samples 500 --size 128x128 --days 180 \
    --objective resilient --top 20 --diverse 1 --csv mine.csv --json mine_top.json --manifest top_seeds.txt
  ```

- `proc_isocity_timelapse`: generate a deterministic **isometric frame sequence** by stepping the simulator forward.
  Useful for regression visuals / CI artifacts.

  ```bash
  ./build/proc_isocity_timelapse --seed 1 --size 128x128 --out frames \
    --days 120 --every 2 --layers overlay,landvalue --format png
  ```

- `proc_isocity_streetnames`: deterministic **street naming** + simple **parcel addressing** derived from the road network.
  Exports streets as JSON (good for GIS) and addresses as CSV (good for scripts/spreadsheets).

  ```bash
  ./build/proc_isocity_streetnames --seed 1 --size 128x128 \
    --streets-json streets.json --addresses-csv addresses.csv
  ```

- `proc_isocity_cartography`: render a "shareable" labeled **isometric poster** (PNG) with district + street labels.
  Optionally dumps placed label boxes as JSON for external toolchains.

  ```bash
  ./build/proc_isocity_cartography --seed 1 --size 128x128 --out poster.png --labels-json labels.json
  ```

- `proc_isocity_wayfind`: **geocode** a procedural address (or street intersection) and compute a road route with
  turn-by-turn style **maneuvers**. Optionally writes a JSON route and a debug image with the route overlaid.

  ```bash
  # Route between two procedural addresses
  ./build/proc_isocity_wayfind --seed 1 --size 128x128 \
    --from "120 Asterwood Ave" --to "450 3rd St" --out-json route.json --out-image route.png --image-scale 4

  # Route between intersections
  ./build/proc_isocity_wayfind --seed 1 --size 128x128 \
    --from "Asterwood Ave & 3rd St" --to "Juniper Rd @ 7th Ave"
  ```

- `proc_isocity_tour`: generate a procedural **walking tour** by synthesizing a handful of interesting POIs
  (parks, peaks, structural bottlenecks, markets...) and connecting them with turn-by-turn **itinerary legs**.
  Optionally renders a Cartography-style poster with the **tour route** and numbered stop markers.

  ```bash
  # Generate a tour + poster from a fresh procedural world
  ./build/proc_isocity_tour --seed 1 --size 128x128 --out-json tour.json --out-md tour.md --out-image tour.png

  # Start the tour from a specific procedural address
  ./build/proc_isocity_tour --seed 1 --size 128x128 --start "120 Asterwood Ave" --stops 8 --out-image tour.png

  # Same city, different tour (tie-break salt)
  ./build/proc_isocity_tour --seed 1 --size 128x128 --seed-salt 42 --out-image tour_alt.png
  ```

- `proc_isocity_osmimport`: import an **OpenStreetMap (OSM XML)** extract into a new ProcIsoCity save.

  This tool maps common OSM tags to ProcIsoCity features:
  - `highway=*` -> roads (with tiered levels)
  - `natural=water` / `waterway=*` -> water
  - `landuse=*` -> zones (res/com/ind)
  - `leisure=park` -> parks
  - `building=*` -> zones (heuristic)

  Optional: run the deterministic **AutoBuild** bot for `--autobuild-days` after import to populate additional zoning/parks.

  ```bash
  # Roads-only import into a 512x512 world
  ./build/proc_isocity_osmimport --osm extract.osm --save osm_world.bin --seed 1 --size 512x512

  # Full import (roads + water + landuse + parks + buildings) with AutoBuild growth
  ./build/proc_isocity_osmimport --osm extract.osm --save osm_full.bin --seed 1 --size 512x512 --full \
    --autobuild-days 120
  ```

- `proc_isocity_mapexport`: export a world to a single **GeoJSON FeatureCollection** (roads + landuse polygons + optional
  districts) for GIS tooling (QGIS/GeoPandas/Leaflet, etc.).

  ```bash
  # Generate + export
  ./build/proc_isocity_mapexport --seed 1 --size 128x128 --geojson map.geojson

  # Export a loaded save and include district polygons + stats
  ./build/proc_isocity_mapexport --load save.bin --geojson map_with_districts.geojson --districts 1
  ```

- `proc_isocity_diff`: compare two save files (deep tile diff + hash) and optionally write a color-coded diff map:

  ```bash
  ./build/proc_isocity_diff saveA.bin saveB.bin --ppm diff.ppm --scale 4
  ```

- `proc_isocity_imagediff`: compare two **PPM (P6)** images and optionally write an absolute-difference image.

  ```bash
  # Exit codes: 0 match, 1 differ, 2 error
  ./build/proc_isocity_imagediff a.ppm b.ppm --out diff.ppm --threshold 0 --json diff.json
  ```

- `proc_isocity_roadgraph`: export a **compressed road network graph** (intersections/endpoints/corners) to DOT/JSON/CSV.
  Also computes simple metrics (connected components, approximate weighted diameter) and can emit a debug PPM.

  ```bash
  # Export a loaded save
  ./build/proc_isocity_roadgraph --load save.bin \
    --dot roads.dot --json roads.json --nodes-csv road_nodes.csv --edges-csv road_edges.csv

  # Generate a new world and emit a diameter highlight image
  ./build/proc_isocity_roadgraph --seed 1 --size 128x128 --diameter-ppm diameter.ppm --ppm-scale 4
  ```

- `proc_isocity_tileset`: generate a dependency-free, deterministic **sprite atlas** of the project's procedural
  textures (terrain, terrain transitions, roads, bridges, overlays) with optional **taller building sprites** and optional **civic facility sprites**
  (education/health/police/fire), plus optional **emissive**,
  **height**, **normal**, **shadow**, and **SDF** atlases (all sharing the exact same layout/metadata). This makes it easy to
  plug the generator into external renderers or mod pipelines without needing any extra art.

  Output formats:
  - default: **RGBA PNG** (color type 6)
  - optional: **indexed-color PNG** (color type 3) with a generated palette (`--indexed 1`).
    This can drastically reduce file size because the built-in encoder intentionally writes stored (uncompressed) DEFLATE blocks.
    You can also re-save the atlas in an external tool to get standard PNG compression; the loader now supports that.

  - Consumers: `LoadGfxTilesetAtlas()` (used by `--iso-tileset` for headless isometric exports) can read:
    - RGBA8 PNG (color type 6), RGB8 PNG (color type 2), and indexed-color PNG (color type 3 + PLTE/tRNS)
    - scanline filters 0..4 (None/Sub/Up/Average/Paeth)
    - zlib streams containing stored, fixed-Huffman, or dynamic-Huffman DEFLATE blocks
    - (still no interlace / 16-bit / grayscale)

  Quality / engine-integration options:
  - `--extrude <px>` duplicates sprite border pixels outward into transparent padding.
    This helps reduce texture bleeding when using **linear filtering + mipmaps** in downstream engines.
  - `--mip-dir <dir>` writes a full mip chain (mip0..mipN) to a directory as `<base>_mipN.png`.
    - `--mip-levels <n>` limits the number of mip levels written (0 = until 1x1 or `--mip-min-size`).
    - `--mip-min-size <px>` stops once both dimensions are <= this value.
    - `--mip-premultiply <0|1>` controls whether downsampling uses premultiplied-alpha filtering.
    - `--mip-alpha-coverage <0|1>` preserves alpha coverage per-sprite across mips (useful for cutout sprites like trees).
      - `--mip-alpha-threshold <f>` sets the alpha test threshold in [0,1] used for coverage matching (default: 0.5).
      - `--mip-alpha-iters <n>` sets the binary search iterations for matching (default: 12).

  Vector outline export (picking/physics/debug tooling):
  - `--outlines <json>` writes per-sprite vector geometry extracted from the alpha mask:
    - `polygons`: one or more outer rings + holes (pixel-edge polygons in sprite-local coordinates)
    - `hull`: optional convex hull ring for cheap coarse hit-testing
  - `--outline-svg <svg>` writes an SVG overlay preview on top of the atlas image.
  - `--outline-threshold <f>` alpha threshold in [0,1] used to classify pixels as inside/outside.
  - `--outline-hull <0|1>` enable/disable convex hull generation.
  - `--outline-holes <0|1>` include/strip holes.
  - `--outline-svg-scale <n>` scales the SVG output dimensions (viewBox remains in atlas pixels).

  Atlas packing / size optimization options:
  - `--pack <mode>` chooses the atlas layout: `grid` (legacy) or `maxrects` (denser for mixed sprite sizes).
  - `--pack-width <px>` sets a target atlas width for `maxrects` (0 = auto).
  - `--pow2 <0|1>` rounds the final atlas dimensions up to the next power-of-two (useful for some engines).
  - `--trim <0|1>` trims fully transparent borders per sprite before packing. Metadata includes `srcW/srcH/trimX/trimY` so consumers can reconstruct original canvases if needed.
  - `--trim-border <px>` keeps an extra transparent border when trimming (recommended when exporting SDF + mipmaps).

  Notes:
  - `--extrude` is now enforced to be `<= --pad` to prevent overlapping writes between neighboring sprites in tight pack modes.

  Palette themes (`--theme`):
  - Presets: `classic`, `autumn`, `desert`, `noir`, `neon`, `pastel`, `space_colony`
  - Seed-driven families: `procedural`, `procedural_muted`, `procedural_vibrant`
    - These synthesize new baseline hues from the palette seed, so different `--seed` values can yield dramatically different art direction.

  ```bash
  ./build/proc_isocity_tileset --out tileset.png --meta tileset.json --html tileset_preview.html \
    --seed 1 --theme classic --tile 64x32 --cols 24 --pad 2 \
    --extrude 2 --mip-dir mips --mip-levels 0 --mip-min-size 1 --mip-premultiply 1 \
    --mip-alpha-coverage 1 --mip-alpha-threshold 0.5 --mip-alpha-iters 12 \
    --transitions 1 --transition-variants 4 \
    --buildings 1 --building-variants 12 \
    --facilities 1 --facility-variants 8 \
    --indexed 1 --indexed-colors 256 --indexed-dither 0 \
    --emit tileset_emissive.png \
    --height tileset_height.png --normal tileset_normal.png --shadow tileset_shadow.png --sdf tileset_sdf.png \
    --outlines tileset_outlines.json --outline-svg tileset_outlines.svg \
    --height-from alpha_luma --normal-strength 2.0 \
    --shadow-dir 1,1 --shadow-length 18 --shadow-blur 2 --shadow-opacity 0.70 \
    --sdf-spread 8 --sdf-threshold 0.5
  ```


- `proc_isocity_roadcentrality`: compute **betweenness centrality** on the compressed `RoadGraph` (both nodes and edges)
  to highlight structural bottlenecks in the road network (independent of simulated traffic). Exports DOT/JSON/CSV and can
  emit a one-pixel-per-tile highlight image (PPM/PNG by extension).

  ```bash
  # Centrality on a loaded save, travel-time weighted
  ./build/proc_isocity_roadcentrality --load save.bin --weight-mode time \
    --json centrality.json --dot centrality.dot \
    --nodes-csv centrality_nodes.csv --edges-csv centrality_edges.csv \
    --ppm centrality.png --ppm-scale 4 --top-nodes 25 --top-edges 50

  # Approximate betweenness by sampling sources (deterministic)
  ./build/proc_isocity_roadcentrality --seed 1 --size 128x128 --max-sources 128 --json cent_sample.json
  ```


- `proc_isocity_trafficgraph`: compute a **commute traffic heatmap** (per road tile) and aggregate it onto the
  compressed `RoadGraph` (nodes/edges) so you can analyze congestion at the segment level. Exports DOT/JSON/CSV and can
  emit traffic heatmap + "top congested edges" highlight images (PPM/PNG by extension).

  ```bash
  # Generate a world, simulate 60 days to populate zones, then export graph + images
  ./build/proc_isocity_trafficgraph --seed 1 --size 128x128 --days 60     --json trafficgraph.json --dot trafficgraph.dot --nodes-csv tg_nodes.csv --edges-csv tg_edges.csv     --heatmap traffic.png --highlight top_congested.png --scale 4

  # Enable congestion-aware routing for the commute assignment
  ./build/proc_isocity_trafficgraph --seed 2 --size 128x128 --days 60     --congestion-aware 1 --passes 6 --alpha 0.15 --beta 4.0     --json trafficgraph_congestion.json
  ```


- `proc_isocity_goodsgraph`: compute a **goods shipments traffic heatmap** (per road tile) and aggregate it onto the
  compressed `RoadGraph` (nodes/edges) so you can analyze **supply chain pressure** at the segment level.
  In addition to DOT/JSON/CSV, it can export an **origin-destination (OD)** list between road access points
  (local industrial→commercial deliveries, edge imports, edge exports) as CSV and as **GeoJSON desire lines**.

  ```bash
  # Generate a world, simulate 60 days, then export goods flow + OD
  ./build/proc_isocity_goodsgraph --seed 1 --size 128x128 --days 60 \
    --json goodsgraph.json --dot goodsgraph.dot --nodes-csv gg_nodes.csv --edges-csv gg_edges.csv \
    --heatmap goods_traffic.png --fillmap goods_fill.png --highlight goods_top_edges.png --scale 4 \
    --od-csv goods_od.csv --od-geojson goods_od.geojson --od-top 250 --od-min-amount 4

  # Analyze an existing save, with imports disabled (stress local industry)
  ./build/proc_isocity_goodsgraph --load save.bin --allow-imports 0 \
    --json goods_local.json --heatmap goods_local.png
  ```


- `proc_isocity_parkopt`: suggest **new park placements** that best serve underserved zones.
  Demand can be weighted by **zone tiles** or by **occupants**, and distance is computed along the road network
  (either raw road steps or travel-time). The tool can export a JSON/CSV ranked list, render an annotated overlay,
  and optionally write a new save with the proposed parks applied (tooling-only; it does not charge money).

  ```bash
  # Generate a world, simulate 60 days, then suggest 12 parks and export artifacts
  ./build/proc_isocity_parkopt --seed 1 --size 128x128 --days 60 --add 12 \
    --json parkopt.json --csv parkopt.csv \
    --annotate parkopt.png --heat-before park_access_before.png --heat-after park_access_after.png

  # Improve an existing save and write a new save file with parks placed
  ./build/proc_isocity_parkopt --load save.bin --add 8 --save save_more_parks.bin
  ```


- `proc_isocity_servicesopt`: suggest **civic service facilities** (Education/Health/Safety)
  to improve a SimCity-style "coverage" model. Under the hood it uses a greedy, capacity-aware
  E2SFCA-style heuristic (two-step floating catchment area) on the road network (distance can be
  raw road steps or travel-time) and then reports the detailed satisfaction using the full
  `ComputeServices` evaluator.

  It exports a ranked JSON/CSV list of facility placements (tile + access road), plus optional
  before/after satisfaction heatmaps (PPM/PNG by extension).

  ```bash
  # Generate a world, simulate 120 days, then propose 8 education facilities and export a heatmap
  ./build/proc_isocity_servicesopt --seed 1 --size 128x128 --days 120 \
    --type education --add 8 --level 2 \
    --json services_edu.json --csv services_edu.csv --heat-after edu_sat.png --scale 4

  # Plan all 3 services in one run and export overall satisfaction heatmap
  ./build/proc_isocity_servicesopt --load save.bin --type all --add 6 --level 2 \
    --json services_all.json --heat-after services_overall.png
  ```


- `proc_isocity_floodrisk`: headless **flood/ponding risk** analysis on the terrain heightfield.
  It combines two complementary models:
  - **Sea-level flooding**: connectivity-based coastal inundation mask + depth.
  - **Depression filling (Priority-Flood)**: how much water would pond in terrain sinks before spilling.

  Exports masks/depth maps (PPM/PNG by extension), GeoJSON polygons per flood/depression region, and a JSON summary.
  It can optionally write a new save where flooded/ponded tiles are converted to `Terrain::Water` (tooling-only).

  ```bash
  # Coastal flood at sea level 0.45
  ./build/proc_isocity_floodrisk --seed 1 --size 128x128 --mode sea --sea-level 0.45 \
    --sea-mask sea.png --sea-depth sea_depth.png --sea-annotate sea_overlay.png \
    --sea-geojson sea.geojson --json sea_report.json

  # Ponding potential (Priority-Flood) on an existing save; only keep ponds deeper than 0.01
  ./build/proc_isocity_floodrisk --load save.bin --mode depressions --dep-min-depth 0.01 \
    --dep-depth ponds.png --dep-annotate ponds_overlay.png --dep-geojson ponds.geojson --json ponds_report.json

  # Apply sea-flooded tiles as water and write a new save
  ./build/proc_isocity_floodrisk --load save.bin --mode sea --sea-level 0.45 --apply sea --save save_flooded.bin
  ```


- `proc_isocity_evac`: headless **evacuation accessibility + bottleneck** analysis under an optional hazard mask.
  It computes which `Residential` tiles can reach a *safe* road exit on the map edge, and builds a simple
  evacuation-demand map on the road network by aggregating residents along shortest-to-exit routes.
  Exports a JSON summary, an optional CSV of the most over-utilized road tiles, and optional images.

  ```bash
  # Coastal evacuation analysis after 120 days of growth
  ./build/proc_isocity_evac --seed 1 --size 128x128 --days 120 --mode sea --sea-level 0.45 \
    --json evac.json --top-roads-csv evac_top_roads.csv \
    --hazard hazard.png --annotate evac.png --flow evac_flow.png --ppm-scale 4

  # Ponding-aware analysis on an existing save (Priority-Flood depth threshold)
  ./build/proc_isocity_evac --load save.bin --mode depressions --dep-min-depth 0.02 \
    --json evac_pond.json --annotate evac_pond.png --ppm-scale 4
  ```


- `proc_isocity_roadupgrades`: plan **road level upgrades** (street→avenue→highway) under a budget, based on a
  combined per-road-tile flow map:
  - **commute traffic** (from `Traffic`)
  - optional **goods shipments** traffic (from `Goods`)

  You can optimize for either:
  - **congestion relief** (reduce per-tile excess flow above capacity)
  - **travel time savings** (flow-weighted milli-time saved)
  - a **hybrid** of the two

  Exports JSON/CSV, can emit a highlight image, can export a DOT road graph colored by combined utilization, and can
  write a new save with the upgrades applied (tooling-only; it does not charge money).

  ```bash
  # Generate a world, simulate 60 days, then pick upgrades under a budget and export artifacts
  ./build/proc_isocity_roadupgrades --seed 1 --size 128x128 --days 60 \
    --budget 250 --objective congestion --min-util 1.0 \
    --json upgrades.json --edges-csv upgrades_edges.csv --tiles-csv upgrades_tiles.csv \
    --dot upgrades_flow.dot --highlight upgrades.png --scale 4

  # Optimize for travel-time savings and write a save with the upgrades applied
  ./build/proc_isocity_roadupgrades --load save.bin --objective time --budget 500 \
    --json upgrades_time.json --write-save save_upgraded.bin
  ```



- `proc_isocity_policyopt`: headless **policy optimizer** that searches citywide taxes + maintenance to maximize an
  objective (money / growth / balanced / happiness) by repeatedly simulating `--eval-days` from a fixed baseline.
  Outputs JSON + CSV + an iteration trace, and can write a new save with the best policy applied.

  ```bash
  # Tune a loaded save for a balanced objective and emit analysis artifacts
  ./build/proc_isocity_policyopt --load save.bin --days 30 --eval-days 60 --objective balanced \
    --json policyopt.json --csv policyopt_top.csv --trace policyopt_trace.csv

  # Brute-force small ranges (exhaustive) and write a save with the best policy
  ./build/proc_isocity_policyopt --load save.bin --method exhaustive --eval-days 30 \
    --tax-res 0..4 --tax-com 0..6 --tax-ind 0..6 --maint-road 0..3 --maint-park 0..3 \
    --objective money --write-save save_best_policy.bin
  ```

- `proc_isocity_roadresilience`: analyze **road network vulnerability** on the compressed `RoadGraph` by finding:
  - **bridge edges** (cut-edges): removing the segment disconnects the network
  - **articulation nodes** (cut-vertices): removing the intersection disconnects the network

  Optionally, it suggests **bypass roads** for the top bridge edges by running a multi-source road-build path search that
  reconnects the two sides *without* using the bridge segment. Exports JSON/CSV and can emit highlight images.

  ```bash
  # Generate a world, simulate 60 days, then export a resilience report + images
  ./build/proc_isocity_roadresilience --seed 1 --size 128x128 --days 60     --json resilience.json --bridges-csv bridges.csv --articulations-csv articulations.csv     --highlight-bridges bridges.png --highlight-bypasses bypasses.png --scale 4

  # Optimize bypass suggestions by money cost and write out a save with the best bypass applied
  ./build/proc_isocity_roadresilience --load save.bin     --bypass-money 1 --bypass-target-level 2 --bypass-allow-bridges 0     --json resilience_money.json --write-best-save save_with_bypass.bin
  ```

- `proc_isocity_blocks`: extract **road-separated land blocks** (components of non-road, non-water tiles)
  and export summary metrics to JSON/CSV. Also emits an optional debug PPM label image.

  ```bash
  ./build/proc_isocity_blocks --seed 1 --size 128x128 \
    --json blocks.json --csv blocks.csv --tile-csv block_ids.csv \
    --ppm blocks.ppm --ppm-scale 4
  ```


- `proc_isocity_blockdistricts`: assign **administrative districts** using the city-block adjacency graph.
  This yields neighborhood-like, block-contiguous districts (as opposed to the road-network-centered
  auto-districting). Exports a report to JSON/CSV/DOT and can write a new save with the districts applied.

  ```bash
  # Generate a world, assign 6 districts, export a DOT graph and a district overlay image
  ./build/proc_isocity_blockdistricts --seed 1 --size 128x128 \
    --districts 6 --fill-roads 1 \
    --json blockdistricts.json --dot blockdistricts.dot \
    --blocks-csv blockdistricts_blocks.csv --edges-csv blockdistricts_edges.csv \
    --district-ppm districts.png --scale 4

  # Apply block districts to an existing save and write out a new save
  ./build/proc_isocity_blockdistricts --load save.bin --districts 8 --write-save save_with_districts.bin
  ```

- `proc_isocity_patch`: create/apply compact **binary patches** between two saves (great for regression artifacts or sharing a small repro):

  ```bash
  # Create a patch that transforms saveA -> saveB
  ./build/proc_isocity_patch make saveA.bin saveB.bin out.isopatch

  # Apply it back onto the base save
  ./build/proc_isocity_patch apply saveA.bin out.isopatch saveA_patched.bin

  # Generate an inverse patch (saveB -> saveA) using the original base save
  ./build/proc_isocity_patch invert saveA.bin out.isopatch out_inverse.isopatch

  # Compose multiple patches into one (saveA -> saveD)
  ./build/proc_isocity_patch compose saveA.bin patch_AB.isopatch patch_BC.isopatch patch_CD.isopatch out_AD.isopatch
  ```

- `proc_isocity_blueprint`: capture/apply **rectangular blueprints** (tile stamps) from/to saves. Supports
  rotation/mirroring and two application modes:
  - `replace`: overwrite all tiles present in the blueprint
  - `stamp`: skip tiles whose blueprint overlay is `None` (useful for sparse stamps)

  Examples:

  ```sh
  # Capture a sparse stamp of just overlay/level/district from a 32x32 region.
  ./build/proc_isocity_blueprint capture saveA.bin 10 10 32 32 out.isobp \
    --fields overlay,level,district --sparse 1

  # Apply it rotated 90 degrees at (64, 64) and write a new save.
  ./build/proc_isocity_blueprint apply saveA.bin out.isobp 64 64 saveA_stamped.bin \
    --mode stamp --rotate 90
  ```

  More operations:

  ```sh
  # Create a *diff blueprint* (base -> target). By default this crops to the minimal bounds
  # containing all changed tiles and prints a suggested destination for apply.
  ./build/proc_isocity_blueprint diff saveA.bin saveB.bin delta.isobp \
    --fields overlay,level,district,variation

  # Crop any blueprint to its delta bounds (prints the crop offset).
  ./build/proc_isocity_blueprint crop big.isobp cropped.isobp --pad 2

  # Pre-transform a blueprint in blueprint-space (rotate/mirror), then apply without transforms.
  ./build/proc_isocity_blueprint transform in.isobp out_rotated.isobp --rotate 90 --mirrorx 1
  ```

- `proc_isocity_transform`: rotate/mirror/crop an entire save (useful for generating variants or normalizing orientation).

  ```bash
  # Rotate 90° clockwise
  ./build/proc_isocity_transform saveA.bin saveA_r90.bin --rotate 90

  # Mirror + crop (crop is applied after rotate/mirror)
  ./build/proc_isocity_transform saveA.bin saveA_sub.bin --mirror-x --crop 32 32 128 128
  ```

- `proc_isocity_roadgraph`: export a **compressed road network graph** (intersections/endpoints/corners) to DOT/JSON/CSV
  and compute simple connectivity metrics (including an approximate weighted diameter).

  ```bash
  # Export DOT + JSON + CSVs for an existing save
  ./build/proc_isocity_roadgraph --load saveA.bin \
    --dot roadgraph.dot --json roadgraph.json \
    --nodes-csv road_nodes.csv --edges-csv road_edges.csv

  # Also render a one-pixel-per-tile debug PPM (nodes highlighted) and a diameter-highlight PPM
  ./build/proc_isocity_roadgraph --load saveA.bin \
    --ppm roadgraph.ppm --diameter-ppm diameter.ppm --ppm-scale 4

  # Generate a world on the fly (no save required)
  ./build/proc_isocity_roadgraph --seed 1 --size 128x128 --json g.json
  ```

- `proc_isocity_roadcentrality`: compute **betweenness centrality** on the compressed `RoadGraph` (nodes + edges).
  Exports DOT/JSON/CSV and can emit a one-pixel-per-tile highlight image.

  ```bash
  ./build/proc_isocity_roadcentrality --load saveA.bin --weight-mode time \
    --json centrality.json --dot centrality.dot \
    --nodes-csv cent_nodes.csv --edges-csv cent_edges.csv \
    --ppm centrality.png --ppm-scale 4
  ```

- `proc_isocity_mesh`: export a save/world to a simple 3D mesh for debugging/interoperability:
  - **Wavefront OBJ + MTL**
  - **glTF 2.0** (`.gltf` + sibling `.bin`) and **GLB** (`.glb`)

  ```bash
  # Export an existing save to OBJ/MTL
  ./build/proc_isocity_mesh --load saveA.bin --obj saveA.obj

  # Export an existing save to GLB (single-file glTF)
  ./build/proc_isocity_mesh --load saveA.bin --glb saveA.glb

  # Export glTF JSON + BIN (writes saveA.gltf + saveA.bin)
  ./build/proc_isocity_mesh --load saveA.bin --gltf saveA.gltf

  # Generate + simulate a new world, then export only a cropped subregion
  ./build/proc_isocity_mesh --seed 42 --size 128x128 --days 120 --obj out.obj --crop 32 32 64 64

  # You can also request multiple outputs in one run (e.g. OBJ + GLB)
  ./build/proc_isocity_mesh --load saveA.bin --obj saveA.obj --glb saveA.glb
  ```

- `proc_isocity_replay`: pack/inspect/play deterministic **replay journals**.

  Replay files embed a full base save plus a stream of Tick/Patch/Snapshot events,
  so you can share a *single file* to reproduce a sequence of edits + simulation.

  New in the latest replay format: optional **Note** and **AssertHash** events.
  Notes are just metadata, and AssertHash lets you embed a deterministic checkpoint
  (useful for CI / regression playback).

  ```bash
  # Pack a replay containing (base save) + (one patch that turns base -> target)
  ./build/proc_isocity_replay pack saveA.bin saveB.bin out.isoreplay \
    --note "baseline->target" --assert-final-hash

  # Inspect the replay header
  ./build/proc_isocity_replay info out.isoreplay

  # Play it back and emit artifacts (use --ignore-asserts to skip hash checkpoints)
  ./build/proc_isocity_replay play out.isoreplay --out summary.json --csv ticks.csv --save final.bin
  ```

- `proc_isocity_inspect`: print a fast header/metadata summary for a save file (optionally verifies CRC):

  ```bash
  ./build/proc_isocity_inspect saveA.bin
  ./build/proc_isocity_inspect saveA.bin --verify-crc --json saveA_summary.json
  ```

- `proc_isocity_config`: dump/apply the embedded **ProcGenConfig** and **SimConfig** for a save file (JSON).

  ```bash
  # Dump combined config to a JSON file
  ./build/proc_isocity_config dump saveA.bin --all config.json

  # Apply overrides (merge) and write a new save
  ./build/proc_isocity_config apply saveA.bin saveA_new.bin --sim sim_overrides.json

  # Reset configs to defaults before applying overrides
  ./build/proc_isocity_config apply saveA.bin saveA_reset.bin --reset-proc --reset-sim
  ```

- `proc_isocity_script`: run a deterministic **edit/sim script** (useful for regression tests and repeatable scenarios).

  Script files are simple line-based commands (comments start with `#`). Example:

  ```txt
  # Generate a world, build a small road loop, zone it, simulate, export artifacts
  size 96x96
  seed 123
  generate

  # A loop of streets
  road_line 10 10 30 10 1
  road_line 30 10 30 30 1
  road_line 30 30 10 30 1
  road_line 10 30 10 10 1

  # Fill a small neighborhood
  fill res 11 11 29 29 1

  # Or flood-fill a whole land block bounded by roads/water
  # (great for "zone the whole block" style edits)
  # flood res 11 11 1

  tick 200
  export_ppm overlay out_{seed}_{day}.ppm 4
  expect_hash 0x0000000000000000  # replace with a real hash for CI
  ```

  Run it:

  ```bash
  ./build/proc_isocity_script scenario.txt --out summary.json --csv ticks.csv
  ```

  **New:** `proc_isocity_script` now supports **batch runs** and simple **script variables**.

  - `--batch N` runs the same script `N` times (run index available as `{run}` in templates).
  - `--seed S` sets the base seed (in batch mode, each run starts with seed `S + {run}` unless the script overrides it).
  - `--define name=value` sets a script variable (available as `{name}`), useful for parameterized scripts.

  Script helpers:

  - `set <name> <value>`: define a variable (value is stored as a *template* and expanded when used).
  - `add <name> <delta>`: add an integer delta to a variable (handy for counters).
  - `unset <name>`: remove a variable.
  - `echo <...>`: print an expanded line (useful for debugging).
  - `vars`: print all current variables.

  Most numeric arguments accept simple integer expressions, so you can write things like `seed 100+{run}` or `fill res 0 0 {w}-1 {h}-1 1`.

- `proc_isocity_suite`: run a **suite** of scenarios (scripts and/or replays) and emit CI-friendly reports.

  This is handy for automated regression checks in GitHub Actions or other CI systems: you can run
  multiple scripts/replays, write a single JSON/JUnit summary, and optionally dump per-case artifacts.

  Examples:

  ```bash
  # Run two scripts and one replay.
  ./build/proc_isocity_suite scenarios/smoke.isocity scenarios/econ.isocity artifacts/repro.isoreplay \
    --out-dir artifacts/suite_out --json-report artifacts/suite.json --junit artifacts/suite.xml

  # Discover scenarios under a directory (only .isocity + .isoreplay by default).
  ./build/proc_isocity_suite --discover scenarios --out-dir artifacts/suite_out

  # Shard across CI workers (0-based): run only shard 1 of 4.
  ./build/proc_isocity_suite --discover scenarios --shard 1/4 --junit artifacts/shard1.xml

  # Golden image regression (snapshot testing):
  #  - Create/update goldens (writes *.golden.*.ppm next to scenarios by default)
  ./build/proc_isocity_suite --discover scenarios --golden --update-golden --golden-format iso --out-dir artifacts/suite_out

  #  - Verify against committed goldens in CI (fails on mismatch)
  ./build/proc_isocity_suite --discover scenarios --golden --golden-format iso --golden-threshold 0 --junit artifacts/suite.xml

  # Golden hash regression (world state snapshot testing):
  #  - Create/update hash goldens (writes *.golden.hash next to scenarios by default)
  ./build/proc_isocity_suite --discover scenarios --hash-golden --update-hash-golden

  # HTML dashboard (summary + links to artifacts + embedded golden previews when available):
  #  - Tip: use --golden-ext png for browser-friendly images.
  ./build/proc_isocity_suite --discover scenarios --out-dir artifacts/suite_out \
    --golden --golden-ext png --hash-golden \
    --json-report artifacts/suite.json --junit artifacts/suite.xml \
    --html-report artifacts/suite_out/index.html --html-title "ProcIsoCity CI Suite"
  ```

- `proc_isocity_autobuild`: run the deterministic **city bot** headlessly (generate or load a world, then grow it for N days).

  ```bash
  # Start from a blank grass world and let the bot grow it for 365 days
  ./build/proc_isocity_autobuild --size 96x96 --seed 42 --empty --money 2000 --days 365 \
    --bot zonesPerDay 3 --bot roadsPerDay 1 --bot parksPerDay 1 \
    --out bot_{seed}.json --csv bot_{seed}.csv --save bot_{seed}.bin \
    --export-ppm overlay bot_{seed}.ppm --export-scale 4
  ```

---

## Controls

- **Right mouse drag**: pan camera  
- **Mouse wheel**: zoom (zooms around mouse cursor)
### ProcGen config keys (scripts)

You can tweak procedural generation from a `.isocity` script:

```text
proc <key> <value>
```

New in the latest saves/patches: the default terrain pass includes an **erosion + rivers** stage. You can tune (or disable) it with:

- `proc erosion 0|1` (master enable)
- `proc rivers 0|1`
- `proc thermal_iterations <int>`
- `proc thermal_talus <float>`
- `proc thermal_rate <float>`
- `proc river_min_accum <int>` (0 = auto threshold)
- `proc river_carve <float>`
- `proc river_power <float>`
- `proc smooth_iterations <int>`
- `proc smooth_rate <float>`
- `proc quantize_scale <int>` (0 disables quantization)

The original ProcGen keys are still supported:

- `terrainScale`, `waterLevel`, `sandLevel`
- `hubs`, `extraConnections`
- `roadLayout` (organic/grid/radial/tensor_field/physarum/medial_axis/voronoi_cells/space_colonization)
- `zoneChance`, `parkChance`

- **1**: Road tool
- **U**: (Road tool) cycle road class (Street/Avenue/Highway)
- **2**: Residential zone tool
- **3**: Commercial zone tool
- **4**: Industrial zone tool
- **5**: Park tool
- **0**: Bulldoze tool
- **6**: Raise terrain tool (terraform)
- **7**: Lower terrain tool (terraform)
- **8**: Smooth terrain tool (terraform)
- **9**: District tool (paint district labels 0..7)
  - **, / .**: (while in District tool) cycle active district
  - **K**: auto-generate districts from the current road network
    - **Shift+K**: use only roads connected to the map edge (outside-connected)
  - **Alt+Click**: pick the hovered tile's district as the active district
  - **Shift+Click**: flood fill the hovered region with the active district
    - **Ctrl+Shift+Click**: allow the flood to cross roads (land-block mode)
- **E**: toggle elevation rendering (flat vs elevated)
- **R**: regenerate a new world (new seed)
- **G**: toggle grid overlay
- **H**: toggle help overlay
- **M**: toggle minimap (click/drag to jump camera)
- **C**: toggle vehicle micro-sim (commuters + goods trucks)
- **O**: toggle outside connectivity overlay
- **T**: toggle road graph overlay
- **V**: toggle traffic overlay
- **B**: toggle goods overlay
- **P**: toggle policy/budget panel
  - **Tab**: select a setting
  - **[ / ]**: adjust selected value (hold **Shift** for bigger steps)
  - When the policy panel is open, **[ / ]** adjust policy instead of brush size.
- **F3**: toggle traffic model panel (congestion-aware routing tuning)
  - **Tab**: select a setting
  - **[ / ]**: adjust selected value (hold **Shift** for bigger steps)
- **F7**: toggle district panel (per-district stats + optional policy multipliers)
- **F1**: toggle city report panel (time-series graphs)
  - **Tab**: cycle report pages
- **F2**: toggle base render cache (faster rendering on large maps)
- **I**: toggle merged zone buildings (render adjacent same-type zones as larger 3D-ish blocks)
- **L**: cycle heatmap overlay (land value / park amenity / water amenity / pollution / traffic spill)
  - Hold **Shift** while pressing **L** to cycle backwards.
- **Space**: pause/resume simulation
- **N**: step simulation by one tick (while paused)
- **+ / -**: change simulation speed
- **[ / ]**: brush size (diamond radius)
- **Shift** (while painting):
  - Road tool: Shift+drag plans a cheapest **money cost** road path (includes upgrades + bridges).
    - If you can't afford the whole plan, it won't partially build.
  - Terraform tools: stronger effect.
- **Shift+Click** (build tools): flood fill the region under the cursor and apply the active tool.
  - Works for **Residential/Commercial/Industrial/Park/Bulldoze**.
  - Zoning/parks are **atomic**: if you can't afford the whole fill, nothing is built.
  - **Ctrl+Shift+Click** allows the fill to cross roads (land block mode). Use carefully.
- **Ctrl** (while using terraform tools): finer effect.
- Painting applies each tile at most once per stroke (prevents accidental multi-upgrades while holding the mouse still).
- Failed placements (no money / no road access / water) show a toast when you release the mouse.
- **F6**: cycle save slot (1..5)
- **F5**: quick save to the selected slot
- **F9**: quick load from the selected slot
- **F10**: open save manager (browse slots, preview minimap thumbnails, delete)
- Autosaves write to `isocity_autosave_slot*.bin` on a wall-clock timer.
- **Ctrl+Z**: undo last paint stroke
- **Ctrl+Y** (or **Ctrl+Shift+Z**): redo
- **Q**: Inspect tool (disables painting)
- **ESC**: quit

---

## Notes

- By default (when `PROCISOCITY_BUILD_APP=ON`), this project uses CMake **FetchContent** to download/build raylib automatically.
- On Linux you may need dev packages for X11/OpenGL/audio depending on distro.
- No textures are loaded from disk; all tiles are created procedurally at runtime.
- Roads now auto-connect visually (auto-tiling based on neighboring road tiles).
- Zones and job tiles only function when connected via roads to the **map edge** (an "outside" connection).
- Zone tiles show 1–3 pips (level) and a tiny occupancy bar when zoomed in.
- Zone tiles also render simple **procedural 3D-ish buildings** (no external assets) at higher zoom.
  - When merged buildings are enabled (**I**), neighboring zone tiles can merge into larger **rectangular** footprints.
    Rooftop details (like commercial signs) are oriented toward nearby roads.
- Parks now boost happiness based on **local coverage** (zones within a small radius of a road-connected park).
- Land value now feeds back into simulation:
  - Residential growth is biased toward high-value tiles.
  - Job assignment (commercial/industrial) is weighted by land value.
  - Budget/demand/land value summaries are shown in the HUD; taxes and maintenance can be tweaked in the in-game policy panel (**P**).
- Commute routing supports an optional **congestion-aware** multi-pass assignment model (tunable via **F3**).
- Save files are versioned; current is **v9**:
  - v2: seed + procgen config + tile deltas (small saves)
  - v3: CRC32 checksum (corruption detection)
  - v4: varint + delta-encoded diffs (smaller / faster)
  - v5: persists terraforming via height deltas
  - v6: persists SimConfig (taxes, maintenance, outside-connection rule, park radius)
  - v7: persists per-tile districts + optional district policy multipliers
  - v8: compresses the delta payload for smaller saves / faster disk IO
  - v9: persists ProcGen erosion config (keeps delta/regeneration deterministic)

---

## Roadmap ideas

- Proper pathfinding (A*), road graphs, traffic, goods flow
- Chunked world streaming + frustum culling
- Proper road graphs (nodes/edges) + traffic visualization
- Smarter undo/redo (optional: track only overlays + money, not sim state)
- Save system: multiple slots and stronger cross-version stability (v8 adds delta compression; v9 adds erosion config)
- Buildings with multi-tile footprints + richer procedural silhouettes
- Rendering performance: cached chunk layers (terrain/decals) + fewer draw calls
- Multi-layer rendering (terrain, decals, structures, overlays)

---

## License

MIT (for this template).  
raylib is licensed under the zlib/libpng license (see raylib’s LICENSE).
