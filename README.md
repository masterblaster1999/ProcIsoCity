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

---

## Controls

- **Right mouse drag**: pan camera  
- **Mouse wheel**: zoom (zooms around mouse cursor)
- **1**: Road tool
- **2**: Residential zone tool
- **3**: Commercial zone tool
- **4**: Industrial zone tool
- **5**: Park tool
- **0**: Bulldoze tool
- **R**: regenerate a new world (new seed)
- **G**: toggle grid overlay
- **H**: toggle help overlay
- **Space**: pause/resume simulation
- **N**: step simulation by one tick (while paused)
- **+ / -**: change simulation speed
- **[ / ]**: brush size (diamond radius)
- Painting applies each tile at most once per stroke (prevents accidental multi-upgrades while holding the mouse still).
- Failed placements (no money / no road access / water) show a toast when you release the mouse.
- **F5**: quick save (writes `isocity_save.bin`)
- **F9**: quick load
- **Ctrl+Z**: undo last paint stroke
- **Ctrl+Y** (or **Ctrl+Shift+Z**): redo
- **Q**: Inspect tool (disables painting)
- **ESC**: quit

---

## Notes

- This project uses CMake **FetchContent** to download/build raylib automatically.
- On Linux you may need dev packages for X11/OpenGL/audio depending on distro.
- No textures are loaded from disk; all tiles are created procedurally at runtime.
- Roads now auto-connect visually (auto-tiling based on neighboring road tiles).
- Zone tiles show 1–3 pips (level) and a tiny occupancy bar when zoomed in.
- Parks now boost happiness based on **local coverage** (zones within a small radius of a road-connected park).
- Save files are versioned; **v2** saves store seed + procgen config + tile deltas (smaller saves).

---

## Roadmap ideas

- Proper pathfinding (A*), road graphs, traffic, goods flow
- Chunked world streaming + frustum culling
- Proper road graphs (nodes/edges) + traffic visualization
- Smarter undo/redo (optional: track only overlays + money, not sim state)
- Save system: compression, multiple slots, and stronger cross-version stability
- Buildings with multi-tile footprints + procedural silhouettes
- Multi-layer rendering (terrain, decals, structures, overlays)

---

## License

MIT (for this template).  
raylib is licensed under the zlib/libpng license (see raylib’s LICENSE).
