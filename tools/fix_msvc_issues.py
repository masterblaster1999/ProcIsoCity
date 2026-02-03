#!/usr/bin/env python3
"""ProcIsoCity MSVC hygiene fixer.

This script is intentionally *self-contained* and makes conservative edits to
the repo to help MSVC builds succeed and stay warning-clean.

Fixes currently implemented:

1) Dossier.cpp: split any raw string literal > ~16k chars into multiple adjacent
   raw string literals (avoids MSVC C2026: string too big).
2) AppPaths.cpp: define _CRT_SECURE_NO_WARNINGS for MSVC to silence C4996 for
   getenv usage (keeps code portable).
3) Sim.cpp / Pathfinding.cpp: disable MSVC warning C4456 (shadowed locals) at
   the file level (push/pop).

Run from the repo root:
  python tools/fix_msvc_issues.py
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


# MSVC C2026: "Before adjacent strings get concatenated, a string can't be
# longer than 16380 single-byte characters".
# We stay safely below that limit per chunk.
MAX_RAW_LITERAL_CONTENT = 16000
RAW_CHUNK_SIZE = 8000


RAW_STRING_START_RE = re.compile(
    r"(?P<prefix>u8|u|U|L)?R\"(?P<delim>[A-Za-z0-9_]{0,16})\("
)


def _detect_newline(text: str) -> str:
    # Preserve existing newline style when inserting new blocks.
    return "\r\n" if "\r\n" in text else "\n"


def _read_text(path: Path) -> str:
    data = path.read_bytes()
    # Preserve UTF-8 BOM if present
    if data.startswith(b"\xef\xbb\xbf"):
        return data.decode("utf-8-sig")
    return data.decode("utf-8")


def _write_text(path: Path, text: str) -> None:
    # Always write UTF-8 without BOM.
    path.write_text(text, encoding="utf-8", newline="")


def split_oversize_raw_string_literals(
    text: str,
    *,
    max_literal_content: int = MAX_RAW_LITERAL_CONTENT,
    chunk_size: int = RAW_CHUNK_SIZE,
) -> tuple[str, bool]:
    """Split oversize raw string literals into multiple adjacent literals.

    This is a heuristic parser for C++11 raw string literals.
    It preserves prefix (u8/u/U/L) and delimiter.
    """

    newline = _detect_newline(text)
    out: list[str] = []
    i = 0
    changed = False

    while True:
        m = RAW_STRING_START_RE.search(text, i)
        if not m:
            out.append(text[i:])
            break

        start = m.start()
        out.append(text[i:start])

        prefix = m.group("prefix") or ""
        delim = m.group("delim")
        content_start = m.end()  # index immediately after the opening '(' token

        close_seq = ")" + delim + "\""
        content_end = text.find(close_seq, content_start)
        if content_end < 0:
            # Unmatched raw string; copy the remainder unchanged.
            out.append(text[start:])
            break

        content = text[content_start:content_end]

        if len(content) > max_literal_content:
            # Determine indentation of the original literal for nicer diffs.
            line_start = text.rfind("\n", 0, start)
            if line_start < 0:
                line_start = 0
            else:
                line_start += 1
            indent_match = re.match(r"[ \t]*", text[line_start:start])
            indent = indent_match.group(0) if indent_match else ""

            chunks = [content[j : j + chunk_size] for j in range(0, len(content), chunk_size)]
            lines = [
                f"{indent}{prefix}R\"{delim}({chunk}){delim}\"" for chunk in chunks
            ]
            out.append(newline.join(lines))
            changed = True
        else:
            # Copy original literal unchanged.
            out.append(text[start : content_end + len(close_seq)])

        i = content_end + len(close_seq)

    return "".join(out), changed


def ensure_crt_secure_no_warnings(text: str) -> tuple[str, bool]:
    marker = "PROCISOCITY_CRT_SECURE_NO_WARNINGS"
    if marker in text or "_CRT_SECURE_NO_WARNINGS" in text:
        return text, False

    nl = _detect_newline(text)
    block = (
        f"#ifdef _MSC_VER{nl}"
        f"#define _CRT_SECURE_NO_WARNINGS // {marker}{nl}"
        f"#endif{nl}{nl}"
    )
    return block + text, True


def ensure_msvc_disable_warning_4456(text: str) -> tuple[str, bool]:
    marker = "PROCISOCITY_MSVC_DISABLE_4456"
    if marker in text:
        return text, False

    nl = _detect_newline(text)
    begin = (
        f"#ifdef _MSC_VER{nl}"
        f"#pragma warning(push){nl}"
        f"#pragma warning(disable:4456) // {marker}{nl}"
        f"#endif{nl}{nl}"
    )
    end = (
        f"{nl}#ifdef _MSC_VER{nl}"
        f"#pragma warning(pop){nl}"
        f"#endif{nl}"
    )

    return begin + text + end, True


def patch_file(path: Path, transform_fn, *, write: bool) -> bool:
    if not path.exists():
        print(f"[skip] missing: {path.as_posix()}")
        return False

    original = _read_text(path)
    updated, changed = transform_fn(original)

    if changed and write:
        _write_text(path, updated)
        print(f"[ok] patched: {path.as_posix()}")
    elif changed:
        print(f"[dry] would patch: {path.as_posix()}")
    else:
        print(f"[ok] no change: {path.as_posix()}")

    return changed


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description="Fix common MSVC warnings/errors in ProcIsoCity")
    ap.add_argument(
        "--repo",
        type=Path,
        default=Path.cwd(),
        help="Repo root (default: current working directory)",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="Report changes without writing files",
    )
    args = ap.parse_args(argv)

    repo = args.repo.resolve()
    write = not args.dry_run

    # 1) Fix the compile-stopper first.
    dossier = repo / "src" / "isocity" / "Dossier.cpp"
    patched_any = patch_file(dossier, split_oversize_raw_string_literals, write=write)

    # 2) Fix/silence MSVC-only warning noise.
    app_paths = repo / "src" / "isocity" / "AppPaths.cpp"
    patched_any = patch_file(app_paths, ensure_crt_secure_no_warnings, write=write) or patched_any

    sim = repo / "src" / "isocity" / "Sim.cpp"
    patched_any = patch_file(sim, ensure_msvc_disable_warning_4456, write=write) or patched_any

    pathfinding = repo / "src" / "isocity" / "Pathfinding.cpp"
    patched_any = patch_file(pathfinding, ensure_msvc_disable_warning_4456, write=write) or patched_any

    if args.dry_run:
        return 0

    if patched_any:
        print("\nDone. Re-run your CMake configure/build.")
    else:
        print("\nNothing to change.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
