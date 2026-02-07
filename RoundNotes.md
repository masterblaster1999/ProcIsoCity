# Round Notes

This round focuses on **fast CI / lite-test builds** and a **better in-game Navigator (wayfinding) experience**.

## Changes

- **CMake: optional `isocity_core` build**
  - Added `PROCISOCITY_BUILD_CORE` (default auto): ON when building app/CLI/full tests, otherwise OFF.
  - The `ci` preset builds **only** lite tests (no full core library), enabling fast `cmake --build --preset ci` + `ctest --preset ci`.

- **Lite tests**
  - Added small, dependency-free tests:
    - `proc_isocity_cli_parse_tests` for `src/cli/CliParse.hpp`.
    - `proc_isocity_deterministic_math_tests` for `src/isocity/DeterministicMath.hpp` and `src/isocity/UInt128.hpp`.
    - `proc_isocity_random_tests` for `src/isocity/Random.hpp`.

- **GitHub Actions**
  - Added a cross-platform workflow that runs the `ci` preset + lite tests on Linux/macOS/Windows.

- **Navigator UI (in-game wayfinding)**
  - Added `Ctrl+G` hotkey to toggle the Navigator panel.
  - Reworked the panel into Route / Options / POV tabs.
  - Added map click picking for From/To endpoints with optional auto-route.
  - Added routing options (metric + avoidance weights) and re-route button.
  - Overlay now shows picked endpoints and the current hovered tile while picking.

- **Docs / repo hygiene**
  - Updated `README.md` preset commands and documented `PROCISOCITY_BUILD_CORE`.
  - Added `.gitignore` for local build dirs and editor files.
