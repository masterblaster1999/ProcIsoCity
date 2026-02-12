# Round Notes

This round focuses on **support tooling** improvements and integrating a few disconnected elements.

## Changes

- **Support bundle improvements (bug-report tooling)**
  - Added a new headless CLI tool: `proc_isocity_support` (folder or `.zip` bundles; no raylib).
  - `SupportBundle` now sanitizes the bundle name prefix for portable filenames.
  - Support bundles now preserve colliding filenames (auto-renaming duplicates) and write a deterministic manifest with byte sizes.
  - `--extra` now accepts directories (recursive scan) with a conservative cap via `--extra-dir-max-files`.
  - Added a new lite test: `proc_isocity_support_bundle_tests` (dir + zip + ZIP entry sanity).

- **Gameplay safety: confirm destructive hotkeys**
  - `R` regenerate now requires a second press within a short window.
  - `F9` quick-load now requires confirmation and avoids prompting when the slot is empty.

- **HUD help overlay accuracy**
  - Updated the in-game help text to match current keybindings (save menu, quicksave/quickload, challenges, etc.).

- **Procgen preset integration**
  - Added `strait` as an alias for the `peninsula` terrain preset.
  - Updated the dev console preset list accordingly.

## Previous patches (already applied)

- Faster CI / lite-test builds and build-system ergonomics.
  - Optional `PROCISOCITY_BUILD_CORE` and a `ci` preset that builds only lite tests.
  - Lite tests for CLI parse helpers (`proc_isocity_cli_parse_tests`).
  - Docs + repo hygiene updates.

- Procedural terrain presets: **atoll** and **peninsula** (+ parsing aliases).
- `proc_isocity_wayfind` upgrades: random endpoints, routing presets, new exports (MD/CSV/ISO), and output directory creation.
- ZIP safety hardening: zip-slip blocking + duplicate entry name detection (+ lite tests).
- `proc_isocity_dossier` usability: `--clean`, friendlier `--format` parsing, and optional ZIP packaging.
- MSVC hygiene tooling (`tools/fix_msvc_issues.py`).
