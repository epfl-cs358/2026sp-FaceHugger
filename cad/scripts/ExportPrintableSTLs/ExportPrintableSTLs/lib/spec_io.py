"""spec_io.py — spec file I/O and log formatting for ExportPrintableSTLs."""

import os

from .config import DEFAULT_UP, LOG_MARKER, ROTATIONS, SpecEntry, VALID_SUBFOLDERS


def _is_axis_token(tok: str) -> bool:
    return len(tok) == 2 and tok[0] in "+-" and tok[1] in "XYZ"


def _parse_extras(tokens: list[str]) -> tuple[str, str | None]:
    """Classify trailing tokens after body_name. Returns (up_axis, hint)."""
    up = DEFAULT_UP
    hint = None
    for tok in tokens:
        if _is_axis_token(tok):
            up = tok
        elif "=" in tok:
            k, v = tok.split("=", 1)
            if k == "hint":
                if hint is not None:
                    raise RuntimeError(f"Multiple hint tokens in {tokens!r}")
                hint = v
            else:
                raise RuntimeError(
                    f"Unknown spec keyword {k!r} (only `hint=` is supported)"
                )
        else:
            if hint is not None:
                raise RuntimeError(f"Multiple hint tokens in {tokens!r}")
            hint = tok
    if up not in ROTATIONS:
        raise RuntimeError(f"Unknown up axis {up!r}; valid: {list(ROTATIONS)}")
    return up, hint


def read_spec(path: str) -> tuple[list[str], list[SpecEntry]]:
    """Parse print_export.txt; return (user_lines_including_marker, entries)."""
    if not os.path.exists(path):
        raise RuntimeError(f"Spec file not found: {path}")
    with open(path, encoding="utf-8") as f:
        raw_lines = f.read().splitlines()

    user_lines: list[str] = []
    saw_marker = False
    for line in raw_lines:
        if line.strip() == LOG_MARKER:
            saw_marker = True
            break
        user_lines.append(line)
    if not saw_marker:
        user_lines.append("")
    user_lines.append(LOG_MARKER)

    entries: list[SpecEntry] = []
    for raw in user_lines:
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line == LOG_MARKER.lstrip("#").strip():
            continue
        parts = line.split()
        if len(parts) < 2:
            raise RuntimeError(f"Malformed spec line: {raw!r}")
        subfolder = parts[0]
        body_name = parts[1]
        if subfolder not in VALID_SUBFOLDERS:
            raise RuntimeError(
                f"Spec line {raw!r}: subfolder must be one of {sorted(VALID_SUBFOLDERS)}, "
                f"got {subfolder!r}"
            )
        up, hint = _parse_extras(parts[2:])
        entries.append(
            SpecEntry(subfolder=subfolder, body_name=body_name, up_axis=up, hint=hint)
        )
    return user_lines, entries


def write_spec_with_log(path: str, user_lines: list[str], log_lines: list[str]) -> None:
    """Rewrite the file atomically (write to .tmp, then replace)."""
    body = "\n".join(user_lines + [""] + log_lines)
    if not body.endswith("\n"):
        body += "\n"
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(body)
    os.replace(tmp, path)


def format_log(
    timestamp: str,
    doc_name: str,
    successes: list[tuple[str, str, str]],
    failures: list[tuple[str, str]],
) -> list[str]:
    """Pure formatting; returns the log line list, no file I/O."""
    lines: list[str] = []
    lines.append(f"# Last export : {timestamp}")
    if doc_name:
        lines.append(f"# Source doc  : {doc_name}")
    lines.append("# Refinement  : MeshRefinementHigh (binary STL, design units)")
    lines.append(f"# Exported    : {len(successes)} body·ies")
    for rel, src_path, up in successes:
        up_note = "" if up == DEFAULT_UP else f"  [up={up}]"
        lines.append(f"#   ✓ {rel}    ← {src_path}{up_note}")
    if failures:
        lines.append(f"# Failures    : {len(failures)}")
        for rel, reason in failures:
            lines.append(f"#   ✗ {rel}    ({reason})")
    else:
        lines.append("# Failures    : 0")
    return lines
