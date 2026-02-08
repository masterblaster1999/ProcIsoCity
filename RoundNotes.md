# Round Notes

This round focuses on **faster CI / lite-test builds** and build-system ergonomics.

## Changes

- **CMake: optional `isocity_core` build**
  - Added `PROCISOCITY_BUILD_CORE` (default auto): ON when building app/CLI/full tests, otherwise OFF.
  - The `ci` preset now builds **only** lite tests (no full core library), enabling fast `cmake --build --preset ci` + `ctest --preset ci`.

- **Lite tests: CLI parse coverage**
  - Added `proc_isocity_cli_parse_tests` (lite) covering `src/cli/CliParse.hpp` helpers.
  - Parsing tightened: `ParseU64` accepts leading `+`; `ParseBool01` is case-insensitive for true/false/on/off/yes/no.

- **GitHub Actions**
  - Added a cross-platform workflow that runs the `ci` preset + lite tests on Linux/macOS/Windows.

- **Docs / repo hygiene**
  - Updated `README.md` preset commands and documented `PROCISOCITY_BUILD_CORE`.
  - Added `.gitignore` for local build dirs and editor files.

## Previous patches (already applied)

- Procedural terrain presets: **atoll** and **peninsula** (+ parsing aliases).
- `proc_isocity_wayfind` upgrades: random endpoints, routing presets, new exports (MD/CSV/ISO), and output directory creation.
- ZIP safety hardening: zip-slip blocking + duplicate entry name detection (+ lite tests).
- `proc_isocity_dossier` usability: `--clean`, friendlier `--format` parsing, and optional ZIP packaging.
- MSVC hygiene tooling (`tools/fix_msvc_issues.py`).
