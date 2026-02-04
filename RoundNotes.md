# Round Notes

This round adds a new terrain preset focused on procedural map variety.

## Changes

- **ProcGen: new `atoll` terrain preset**
  - Ring island with a central lagoon and 1-3 deterministic passes to the open sea.
  - Added to dev console + script help lists.
  - Added a deterministic unit test checking lagoon + ring characteristics.

- **ProcGen: new `strait` terrain preset**
  - Edge-to-edge sea channel that cuts between two major landmasses.
  - Added parsing aliases, dev console/CLI hints, and a connectivity unit test.

- **GitHub Actions CI**
  - Added a workflow that configures + builds a lightweight unit test target and runs it via CTest on Linux, macOS, and Windows.

- **Lightweight unit tests**
  - Added `PROCISOCITY_BUILD_LITE_TESTS` and a new test executable `proc_isocity_zip_tests` (fast, minimal sources; does not require building the full core library).
  - Updated the `ci` preset in `CMakePresets.json` to enable lite tests and keep CI builds snappy.

- **ZIP safety**
  - `ZipWriter` refuses duplicate entry names to prevent silent overwrites inside an archive.
  - Lite tests cover duplicate detection (including slash-normalized duplicates) and zip-slip blocking.

- **`proc_isocity_dossier` CLI**
  - Added `--clean <0|1>` to delete and recreate `--out-dir` before writing.
  - `--format` is now case-insensitive and accepts a leading dot (e.g. `PNG`, `.png`).

- **Previous round (already applied)**
  - Optional dossier directory ZIP packaging and MSVC-friendly splitting of the largest embedded viewer string.
