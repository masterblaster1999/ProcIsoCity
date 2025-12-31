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
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### 3) Run

```bash
./build/proc_isocity
```

### 4) Tests (headless, optional)

You can build and run a small set of **headless** tests (no raylib dependency). This is handy for CI or for quickly validating core logic changes on machines without graphics/system deps.

```bash
cmake -S . -B build-tests -DPROCISOCITY_BUILD_APP=OFF -DPROCISOCITY_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-tests -j
ctest --test-dir build-tests --output-on-failure
```

### CMake options

- `PROCISOCITY_BUILD_APP` (default: ON) — build the interactive raylib app (`proc_isocity`).
- `PROCISOCITY_BUILD_TESTS` (default: OFF) — build `proc_isocity_tests` and enable `ctest`.
- `PROCISOCITY_BUILD_CLI` (default: ON) — build headless command-line tools (`proc_isocity_cli`, `proc_isocity_diff`, `proc_isocity_imagediff`, `proc_isocity_inspect`, `proc_isocity_config`, `proc_isocity_patch`, `proc_isocity_script`, `proc_isocity_replay`, `proc_isocity_blueprint`, `proc_isocity_suite`, `proc_isocity_transform`, `proc_isocity_roadgraph`, `proc_isocity_mesh`).
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
  - batch runs across multiple seeds (`--batch N`)

  Example:

  ```bash
  ./build/proc_isocity_cli --seed 1 --size 128x128 --days 200 \
    --out out_{seed}.json --csv ticks_{seed}.csv \
    --export-ppm overlay overlay_{seed}.ppm --export-scale 4 \
    --export-iso overlay iso_{seed}.ppm --iso-tile 16x8 --iso-height 14 --batch 3
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

- `proc_isocity_mesh`: export a save/world to **Wavefront OBJ + MTL** (simple 3D mesh) for debugging
  and interoperability with 3D tools.

  ```bash
  # Export an existing save to OBJ/MTL
  ./build/proc_isocity_mesh --load saveA.bin --obj saveA.obj

  # Generate + simulate a new world, then export only a cropped subregion
  ./build/proc_isocity_mesh --seed 42 --size 128x128 --days 120 --obj out.obj --crop 32 32 64 64
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
