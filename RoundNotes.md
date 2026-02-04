# Round Notes

This round focuses on procedural terrain generation and new macro presets.

## Changes

- **New terrain preset: atoll**
  - `proc preset atoll`: ring-shaped reef island with a central lagoon and 1–3 sea passes.
  - Added parsing aliases: `reef`, `lagoon`, `coral_reef`.

- **New terrain preset: peninsula**
  - `proc preset peninsula`: a large mainland attached to one edge, tapering into a long land finger.
  - Added parsing aliases: `cape`, `headland`, `promontory`, `spit`.
  - Added to `proc_isocity_evolve` default terrain preset distribution.


- **`proc_isocity_wayfind` gameplay upgrades**
  - Added `random|rand` endpoint support to quickly generate routes as lightweight “missions”.
  - Added `--preset fastest|balanced|safe|quiet` for one-flag routing profiles (time metric + tuned hazard weights).
  - Added new export outputs:
    - `--out-md` (Markdown directions)
    - `--out-csv` (path tiles)
    - `--out-iso` (isometric overview with route overlay)
  - Added isometric rendering knobs (`--iso-layer`, `--iso-tile`, `--iso-height`, `--iso-margin`, `--iso-grid`, `--iso-cliffs`).
  - Outputs now auto-create parent directories (JSON/MD/CSV/images).

- **Docs**
  - Updated `README.md` with the new `proc_isocity_wayfind` features and examples.

## Previous patches (already applied)

- GitHub Actions CI for fast lite tests (`proc_isocity_zip_tests`).
- ZIP safety hardening: zip-slip blocking + duplicate entry name detection.
- `proc_isocity_dossier` usability: `--clean` and friendlier `--format` parsing.
- Optional dossier ZIP packaging + MSVC-friendly splitting of the embedded viewer string.
