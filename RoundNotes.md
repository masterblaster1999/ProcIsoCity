# Round 7

- Added **air quality gameplay**: simulator now computes resident-weighted air pollution exposure and applies a mild happiness penalty (headless) based on average exposure + high-exposure fraction.
  - Code: `src/isocity/Sim.cpp`, settings: `src/isocity/Sim.hpp`, stats fields: `src/isocity/World.hpp`
- Exposed runtime tuning via `Simulator::airPollutionModel()` (enable/disable + config + penalty scales).
- Fixed traffic safety stats wiring to match `TrafficSafetyResult` and existing `Stats` fields.
- Fixed traffic incident severity logic to match `TrafficIncidentSettings` (no missing fields).
- Fixed test helper that used a non-existent `World::applyTool({...})` stroke overload.

Verify:
- Run the app and zone heavy industrial upwind of residential; happiness should drop vs the same layout with buffers/less industry.
- Build tests (`-DPROCISOCITY_BUILD_APP=OFF -DPROCISOCITY_BUILD_TESTS=ON`) and run `proc_isocity_tests`.

# Round 8

- Introduced a shared per-tick CSV schema helper: `src/isocity/StatsCsv.hpp`.
  - All tools that write `ticks.csv` now use the same header/order and include a much richer set of `Stats` fields (services, trade, economy, air pollution exposure, incidents, etc.).
  - Updated: `src/isocity/Dossier.cpp`, `src/isocity/Script.cpp`, and multiple CLI tools.
- Improved dossier analytics:
  - `tools/procisocity_insights.py` now recognizes additional tile metrics (noise, heat island, air pollution, safety, accessibility, livability, etc.) and reports both hotspots and coldspots.
  - Moran's I is computed for multiple key metrics when available, and correlations are ranked by absolute strength.
- Fixed a build warning in `src/isocity/HealthCheck.cpp` by replacing `snprintf` timestamp formatting with a stream-based formatter.

Verify:
- Run any CLI exporter that writes `ticks.csv` (e.g., `proc_isocity_cli --csv out/ticks.csv`) and confirm the header contains the new columns (ending with incident fields).
- Run `python tools/procisocity_insights.py <dossier_dir>` and confirm `insights.md` includes new metrics + coldspots + multi-metric Moran's I.

# Round 9

- Added a new zero-dependency regression tool: `tools/procisocity_compare.py`.
  - Compares two dossier exports (`summary.json`, `ticks.csv`, `tile_metrics.csv`) and writes:
    - `compare.json` (machine-readable)
    - `compare.md` (human-friendly Markdown)
  - Supports optional **per-tile** delta ranking for specific metrics via `--metric <name>`.
- Updated `tools/README.md` with usage and examples.

Verify:
- Generate two dossiers (different seeds or two versions of the sim) and run:
  - `python tools/procisocity_compare.py dossier_A dossier_B --metric land_value --metric livability`
  - Confirm `compare.md` contains the top per-tick and per-tile changes.

# Round 10

- Upgraded the dossier `index.html` viewer for **faster inspection**:
  - **Time-series charting** for `ticks.csv` (no external JS libs; uses a tiny `<canvas>` renderer).
  - A **generic tile-metrics inspector** that discovers numeric columns in `tile_metrics.csv` and lets you
    choose which ones to show on hover.
  - Both `ticks.csv` and `tile_metrics.csv` still support offline use via the file pickers when the dossier
    is opened as `file://` and `fetch()` is blocked by the browser.

Verify:
- Export a dossier with `proc_isocity_dossier`.
- Open `index.html`:
  - Confirm the **Time series** chart auto-loads `ticks.csv` when served from HTTP, or loads via the file
    picker when opened locally.
  - Confirm loading `tile_metrics.csv` populates the hover metric selector and hover values update.

# Round 11

- Added a new headless **procedural evolution lab**: `proc_isocity_evolve`.
  - Uses a deterministic cross-entropy style sampler (elite-set distribution updates) plus seed mutation.
  - Supports fully-custom score functions via `--score "<expr>"`, referencing the simulator's `Stats` fields.
  - Can emit the best result as a `.bin` save and/or a complete dossier export for inspection.
- Build: added `proc_isocity_evolve` to `CMakeLists.txt`.
- Docs: updated `tools/README.md` with a quick example.

Verify:
- Build the CLI tools and run:
  - `proc_isocity_evolve --size 96x96 --days 120 --generations 6 --population 40 --best-dossier out/best`
  - Open `out/best/index.html` and sanity-check the resulting city metrics.

# Round 12

- Added a deterministic **City Chronicle** (procedural newspaper/advisor feed) exported as:
  - `chronicle.json` (machine-readable)
  - `chronicle.md` (human-readable)
  - Code: `src/isocity/Chronicle.hpp/.cpp`
- Dossier exports now include the chronicle by default, and the dossier `index.html` viewer gained a **Chronicle** section with offline-friendly loading.
  - Code: `src/isocity/Dossier.hpp`, `src/isocity/Dossier.cpp`
- Refactored `proc_isocity_dossier` to delegate export generation to the shared core dossier writer (removes duplicated exporter logic and ensures the CLI always stays in sync with the core exporter).
  - Code: `src/cli/dossier.cpp`
- Added a standalone headless tool `proc_isocity_chronicle` to generate just the chronicle from a save or from a seed (optionally simulating forward and/or using AutoBuild).
  - Build: `CMakeLists.txt`, code: `src/cli/chronicle.cpp`

Verify:
- Build CLI tools and run:
  - `proc_isocity_dossier --seed 123 --size 96x96 --autobuild-days 60 --days 30 --out-dir out/dossier`
  - Confirm `out/dossier/chronicle.json` and `out/dossier/chronicle.md` exist.
  - Open `out/dossier/index.html` and confirm the Chronicle section loads/filters entries.
- Run the standalone tool:
  - `proc_isocity_chronicle --seed 123 --size 96x96 --autobuild-days 60 --out out/chronicle.json`

# Round 13

- Added a new ProcGen macro road layout: **Voronoi Cells** (`voronoi_cells`).
  - Builds a sparse grid Voronoi tessellation from hubs + extra land "sites" and carves **arterials along cell boundaries**.
  - Produces ring-road / superblock-style neighborhood boundaries with high seed variability.
  - Each hub is connected to the nearest boundary arterial; `extraConnections` adds a few long cross-city expressways.
  - Code: `src/isocity/ProcGen.hpp`, `src/isocity/ProcGen.cpp`
- Updated save + patch readers to accept the expanded road-layout enum.
  - Code: `src/isocity/SaveLoad.cpp`, `src/isocity/WorldPatch.cpp`
- Updated CLI/script help text and docs to list the new layout option.
  - Code: `src/cli/main.cpp`, `src/cli/dossier.cpp`, `src/isocity/Script.cpp`, `README.md`

Verify:
- Generate a city with the new layout:
  - `proc_isocity_cli --seed 123 --size 96x96 --gen-road-layout voronoi_cells --out out/summary.json --export-ppm overlay out/overlay.ppm --export-scale 4`
  - Confirm the macro network has **cell-like ring roads** and hubs connect into it.
- Save/load round-trip:
  - `proc_isocity_cli --seed 123 --size 96x96 --gen-road-layout voronoi_cells --save out/city.bin`
  - `proc_isocity_cli --load out/city.bin --out out/reload.json`

# Round 14

- Added a new ProcGen macro road layout: **Physarum** (`physarum`).
  - Uses a lightweight agent/pheromone simulation (slime-mold / ant-colony inspired) to discover high-likelihood corridors between hubs.
  - Extracts arterials from the pheromone field and then runs a pheromone-biased A* pass to guarantee hub connectivity.
  - Tends to produce organic, redundancy-aware backbones with emergent loops that differ significantly from the other macro layouts.
  - Code: `src/isocity/ProcGen.hpp`, `src/isocity/ProcGen.cpp`
- Updated save + patch readers to accept the expanded road-layout enum.
  - Code: `src/isocity/SaveLoad.cpp`, `src/isocity/WorldPatch.cpp`
- Updated CLI/script help text and docs to list the new layout option.
  - Code: `src/cli/main.cpp`, `src/cli/dossier.cpp`, `src/isocity/Script.cpp`, `README.md`

Verify:
- Generate a city with the new layout:
  - `proc_isocity_cli --seed 123 --size 96x96 --gen-road-layout physarum --out out/summary.json --export-ppm overlay out/overlay.ppm --export-scale 4`
  - Confirm the macro network shows **thickened pheromone corridors** (highways/avenues) with plausible organic redundancy.
- Save/load round-trip:
  - `proc_isocity_cli --seed 123 --size 96x96 --gen-road-layout physarum --save out/city.bin`
  - `proc_isocity_cli --load out/city.bin --out out/reload.json`

# Round 15

- Added a new ProcGen macro road layout: **Medial Axis / Skeleton** (`medial_axis`).
  - Approximates the **medial axis** of the buildable landmass by computing a boundary Voronoi diagram (multi-source BFS) and keeping the Voronoi edges (where wavefronts from different boundary sources meet).
  - Carves **arterials along the skeleton** (avenues/highways depending on interior distance), then connects hubs into the spine using a skeleton-biased A* so long-distance connectors naturally "thread" through the city.
  - Produces central spines on islands, natural backbones through narrow corridors, and dramatic seed-to-seed variation while remaining terrain-aware and bridge-capable.
  - Code: `src/isocity/ProcGen.hpp`, `src/isocity/ProcGen.cpp`
- Updated save + patch readers to accept the expanded road-layout enum.
  - Code: `src/isocity/SaveLoad.cpp`, `src/isocity/WorldPatch.cpp`
- Updated CLI/script help text and docs to list the new layout option.
  - Code: `src/cli/main.cpp`, `src/isocity/Game.cpp`, `src/isocity/Script.cpp`, `README.md`

Verify:
- Generate a city with the new layout:
  - `proc_isocity_cli --seed 123 --size 96x96 --gen-road-layout medial_axis --out out/summary.json --export-ppm overlay out/overlay.ppm --export-scale 4`
  - Confirm the macro network shows a **centerline/skeleton spine** rather than pure hub-to-hub shortcuts.
- Save/load round-trip:
  - `proc_isocity_cli --seed 123 --size 96x96 --gen-road-layout medial_axis --save out/city.bin`
  - `proc_isocity_cli --load out/city.bin --out out/reload.json`

# Round 16

- Added a new ProcGen macro road layout: **Tensor Field** (`tensor_field`).
  - Synthesizes a smooth *orientation field* from a blend of:
    - terrain contour direction (roads prefer to follow contours on steep slopes)
    - low-frequency domain-warped noise (district-scale grain)
  - Traces a small number of long arterial **spines** from the most central hubs, then connects
    hubs using MST + optional long links via a tensor-field biased A*.
  - Produces sweeping boulevards / parkways that respect terrain while still having coherent
    city-scale structure.
  - Code: `src/isocity/ProcGen.hpp`, `src/isocity/ProcGen.cpp`
- Updated save + patch readers to accept the expanded road-layout enum.
  - Code: `src/isocity/SaveLoad.cpp`, `src/isocity/WorldPatch.cpp`
- Updated CLI/script/in-game help text + docs to list the new layout option.
  - Code: `src/cli/main.cpp`, `src/isocity/Game.cpp`, `src/isocity/Script.cpp`, `README.md`
- Improved `proc_isocity_evolve` genome share-codes:
  - Switched the discrete-field bit-pack from **u16 -> u32** (modern codes have a 36-hex-digit pack).
  - Parser remains backward compatible with the legacy 32-hex-digit pack.
  - Code: `src/cli/evolve.cpp`

Verify:
- Generate a city with the new layout:
  - `proc_isocity_cli --seed 123 --size 96x96 --gen-road-layout tensor_field --out out/summary.json --export-ppm overlay out/overlay.ppm --export-scale 4`
  - Confirm you see **long coherent arterials** (spines) and that hub connections tend to align
    to a consistent local orientation rather than pure shortest paths.
- Save/load round-trip:
  - `proc_isocity_cli --seed 123 --size 96x96 --gen-road-layout tensor_field --save out/city.bin`
  - `proc_isocity_cli --load out/city.bin --out out/reload.json`
- Evolve genome code:
  - `proc_isocity_evolve --iters 2 --pop 4 --best 2 --seed 1`
  - Copy a printed `genome=G...` code and verify `--genome <code>` parses.

# Round 17

- Added a new ProcGen district assignment mode: **Watershed** (`watershed`).
  - Segments the terrain into **drainage basins** using the existing hydrology flow field.
  - Chooses `kDistrictCount` seed basins via **farthest-point sampling** (spatial coverage + prefers land basins).
  - Merges basins into districts using a **multi-source Dijkstra** on the basin adjacency graph, with
    a ridge penalty proportional to shared boundary length (so long ridgelines are more likely to remain
    as district boundaries).
  - Produces natural, terrain-driven neighborhood boundaries that often follow ridges/valleys.
  - Code: `src/isocity/ProcGen.hpp`, `src/isocity/ProcGen.cpp`
- Updated save + patch readers to accept the expanded districting-mode enum.
  - Code: `src/isocity/SaveLoad.cpp`, `src/isocity/WorldPatch.cpp`
- Updated CLI/script/in-game help text to list the new mode.
  - Code: `src/cli/main.cpp`, `src/cli/dossier.cpp`, `src/isocity/Game.cpp`, `src/isocity/Script.cpp`

Verify:
- Generate with the new districting mode and export a district overlay:
  - `proc_isocity_cli --seed 123 --size 128x128 --gen-districting-mode watershed --out out/summary.json --export-ppm district out/district.ppm --export-scale 4`
  - Confirm districts tend to form **basin-like regions** separated by ridges/river valleys.
- Script usage:
  - `proc districting_mode watershed`

# Round 18

- Improved **Watershed** districting code quality + scalability:
  - Replaced per-basin hash-map adjacency with a deterministic **boundary edge list** (sort+count) to reduce memory and improve speed on large maps.
  - Seed selection now uses incremental farthest-point sampling with O(1) seed membership checks.
  - Basin merge Dijkstra now uses **64-bit distances** to avoid overflow on very large maps.
- Added an optional CMake build knob: `PROCISOCITY_ENABLE_GLTF_EXPORT` (default: ON).
  - When OFF, the build compiles a small stub (`GltfExportStub.cpp`) so the API remains link-compatible and export calls return a clear error.

Verify:
- (Default) build unchanged:
  - `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j`
- (Optional) low-memory / faster build:
  - `cmake -S . -B build -DPROCISOCITY_ENABLE_GLTF_EXPORT=OFF -DCMAKE_BUILD_TYPE=Release && cmake --build build -j`
- Watershed districting:
  - `proc_isocity_cli --seed 123 --size 128x128 --gen-districting-mode watershed --export-ppm district out/district.ppm --export-scale 4`

# Round 19

- Improved hydrology performance + robustness:
  - Flow accumulation is now computed in **O(n)** via an in-degree (topological) traversal, avoiding the previous `O(n log n)` height sort.
  - Includes a safety fallback to the legacy height-sorted pass if a malformed cyclic flow field is detected; the fallback comparator now handles **NaNs** with a strict weak ordering.
  - Code: `src/isocity/Hydrology.cpp`
- Expanded ProcGen regression tests:
  - Added determinism + road-graph connectivity checks for **all** macro road layouts:
    `organic`, `grid`, `radial`, `space_colonization`, `voronoi_cells`, `physarum`, `medial_axis`, `tensor_field`.
  - Added determinism + “not-all-one” sanity checks for **Watershed** districting mode.
  - Code: `ProcIsoCityTests.cpp`

Verify:
- Configure + build tests (`-DPROCISOCITY_BUILD_APP=OFF -DPROCISOCITY_BUILD_TESTS=ON`) and run `proc_isocity_tests`.


# Round 20

- Build system reliability improvements:
  - `PROCISOCITY_BUILD_APP` default is now **OFF** (matches docs and keeps the default build offline-capable).
  - Added a top-level `CMakePresets.json` with common presets: `default` (headless CLI), `tests`, `app`, `app-system-raylib`.
- Watershed districting optimization:
  - Basin adjacency boundary edges are now stored as packed `u64` keys instead of `std::pair<int,int>` to reduce memory and improve cache behavior when many basins are present.
  - Code: `src/isocity/ProcGen.cpp` (Watershed districting mode).

Verify:
- Headless default build:
  - `cmake --preset default && cmake --build --preset default --parallel`
- Headless tests:
  - `cmake --preset tests && cmake --build --preset tests --parallel && ctest --preset tests`
- Interactive app (raylib required):
  - `cmake --preset app && cmake --build --preset app --parallel`


# Round 21

- CLI code quality improvements:
  - Added a shared header for CLI parsing helpers: `src/cli/CliParse.hpp`.
    - Consolidates parsing of ints/u64 (including `0x` hex seeds), WxH sizes, strict finite floats,
      simple bool parsing, and a few tiny filesystem helpers.
    - Migrated several high-traffic headless tools (`proc_isocity_cli`, `proc_isocity_dossier`,
      `proc_isocity_chronicle`, `proc_isocity_evolve`, `proc_isocity_autobuild`, `proc_isocity_replay`,
      `proc_isocity_script`, `proc_isocity_suite`, `proc_isocity_timelapse`) to use the shared helpers.
      This keeps behavior consistent across tools and reduces duplicated code.

- Build convenience:
  - Added a new CMake preset: `ci`.
    - Debug build, tests ON, CLI OFF, glTF export OFF.
    - Intended for fast CI builds and quick local sanity checks.

Verify:
- CLI parsing consistency:
  - `proc_isocity_dossier --seed 0x1234 --size 64x64 --out-dir out/dossier`
  - `proc_isocity_chronicle --seed 0x1234 --size 64x64 --out out/chronicle.json`
  - Confirm hex seeds are accepted uniformly.
- CI preset:
  - `cmake --preset ci && cmake --build --preset ci --parallel && ctest --preset ci`



# Round 22

- Save/load + patch correctness fixes:
  - Fixed **terrain preset** enum validation so all presets (including `tectonic`) survive round-trips through:
    - full saves (`SaveLoad.cpp`)
    - world patches (`WorldPatch.cpp`)
  - Added `Count` sentinels and shared clamp helpers for:
    - `ProcGenTerrainPreset`
    - `ProcGenRoadLayout`
    - `ProcGenDistrictingMode`
    This prevents future “max enum value” mismatches when new generation modes are added.

- CLI/script UX polish:
  - Updated `--gen-preset` help hints to include the `tectonic` preset.

Verify:
- Generate a tectonic map, save, reload, and confirm the preset remains tectonic:
  - `proc_isocity_cli --seed 123 --size 128x128 --gen-preset tectonic --save out/tectonic.isocity`
  - `proc_isocity_cli --load out/tectonic.isocity --export-ppm terrain out/terrain.ppm`
- Patch round-trip (if you use patches):
  - Create a patch with a tectonic preset and ensure applying it preserves the preset.
