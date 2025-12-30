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
- `PROCISOCITY_USE_SYSTEM_RAYLIB` (default: OFF) — when building the app, use a system raylib instead of FetchContent.

---

## Controls

- **Right mouse drag**: pan camera  
- **Mouse wheel**: zoom (zooms around mouse cursor)
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
- **F1**: toggle city report panel (time-series graphs)
  - **Tab**: cycle report pages
- **F2**: toggle base render cache (faster rendering on large maps)
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
- Parks now boost happiness based on **local coverage** (zones within a small radius of a road-connected park).
- Land value now feeds back into simulation:
  - Residential growth is biased toward high-value tiles.
  - Job assignment (commercial/industrial) is weighted by land value.
  - Budget/demand/land value summaries are shown in the HUD; taxes and maintenance can be tweaked in the in-game policy panel (**P**).
- Commute routing supports an optional **congestion-aware** multi-pass assignment model (tunable via **F3**).
- Save files are versioned; current is **v8**:
  - v2: seed + procgen config + tile deltas (small saves)
  - v3: CRC32 checksum (corruption detection)
  - v4: varint + delta-encoded diffs (smaller / faster)
  - v5: persists terraforming via height deltas
  - v6: persists SimConfig (taxes, maintenance, outside-connection rule, park radius)
  - v7: persists per-tile districts + optional district policy multipliers
  - v8: compresses the delta payload for smaller saves / faster disk IO

---

## Roadmap ideas

- Proper pathfinding (A*), road graphs, traffic, goods flow
- Chunked world streaming + frustum culling
- Proper road graphs (nodes/edges) + traffic visualization
- Smarter undo/redo (optional: track only overlays + money, not sim state)
- Save system: multiple slots and stronger cross-version stability (v8 adds delta compression)
- Buildings with multi-tile footprints + richer procedural silhouettes
- Rendering performance: cached chunk layers (terrain/decals) + fewer draw calls
- Multi-layer rendering (terrain, decals, structures, overlays)

---

## License

MIT (for this template).  
raylib is licensed under the zlib/libpng license (see raylib’s LICENSE).
