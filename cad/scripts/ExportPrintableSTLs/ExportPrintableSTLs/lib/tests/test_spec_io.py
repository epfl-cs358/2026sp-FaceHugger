"""Tests for spec_io — pure-Python, no adsk dependency."""

import os
import sys
import tempfile

# Add the add-in root (parent of lib/) so `lib` is importable as a package.
sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)

import pytest

from lib.config import LOG_MARKER, SpecEntry
from lib.spec_io import format_log, read_spec, write_spec_with_log


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _write_tmp(content: str) -> str:
    """Write content to a temp file and return its path."""
    fd, path = tempfile.mkstemp(suffix=".txt")
    os.close(fd)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    return path


# ---------------------------------------------------------------------------
# _parse_extras / read_spec parsing tests
# ---------------------------------------------------------------------------


def test_parse_minimal():
    path = _write_tmp("body QuadrupedBody\n")
    _, entries = read_spec(path)
    os.unlink(path)
    assert entries == [SpecEntry("body", "QuadrupedBody", "+Z", None)]


def test_parse_up_axis():
    path = _write_tmp("leg Link2 +Y\n")
    _, entries = read_spec(path)
    os.unlink(path)
    assert entries[0].up_axis == "+Y"
    assert entries[0].hint is None


def test_parse_hint_bare_word():
    path = _write_tmp("leg Link2 Link2L:1\n")
    _, entries = read_spec(path)
    os.unlink(path)
    assert entries[0].hint == "Link2L:1"
    assert entries[0].up_axis == "+Z"


def test_parse_hint_key_value():
    path = _write_tmp("leg Link2 hint=Link2L:1\n")
    _, entries = read_spec(path)
    os.unlink(path)
    assert entries[0].hint == "Link2L:1"


def test_parse_both():
    # up_axis and hint can appear in either order
    path_a = _write_tmp("leg Link2 +Y Link2L:1\n")
    path_b = _write_tmp("leg Link2 Link2L:1 +Y\n")
    _, ea = read_spec(path_a)
    _, eb = read_spec(path_b)
    os.unlink(path_a)
    os.unlink(path_b)
    assert ea[0].up_axis == "+Y"
    assert ea[0].hint == "Link2L:1"
    assert ea[0] == eb[0]


def test_parse_multiple_hints_raises():
    path = _write_tmp("leg Link2 hintA hintB\n")
    with pytest.raises(RuntimeError, match="Multiple hint"):
        read_spec(path)
    os.unlink(path)


def test_parse_unknown_axis_raises():
    path = _write_tmp("leg Link2 +W\n")
    with pytest.raises(RuntimeError, match="Unknown up axis"):
        read_spec(path)
    os.unlink(path)


def test_parse_invalid_subfolder_raises():
    path = _write_tmp("electronics PCB\n")
    with pytest.raises(RuntimeError, match="subfolder must be one of"):
        read_spec(path)
    os.unlink(path)


# ---------------------------------------------------------------------------
# read_spec marker handling
# ---------------------------------------------------------------------------


def test_read_spec_preserves_user_lines():
    content = "body QuadrupedBody\n" + LOG_MARKER + "\n# old log\n"
    path = _write_tmp(content)
    user_lines, entries = read_spec(path)
    os.unlink(path)
    assert user_lines[-1] == LOG_MARKER
    assert len(entries) == 1


def test_read_spec_appends_marker_if_absent():
    path = _write_tmp("body QuadrupedBody\n")
    user_lines, _ = read_spec(path)
    os.unlink(path)
    assert user_lines[-1] == LOG_MARKER


# ---------------------------------------------------------------------------
# write_spec_with_log + round-trip
# ---------------------------------------------------------------------------


def test_write_and_read_roundtrip():
    path = _write_tmp("body QuadrupedBody\n" + LOG_MARKER + "\n")
    user_lines, entries_before = read_spec(path)
    log_lines = ["# Last export : 2026-01-01 00:00:00 UTC", "# Failures    : 0"]
    write_spec_with_log(path, user_lines, log_lines)
    _, entries_after = read_spec(path)
    os.unlink(path)
    assert entries_before == entries_after


def test_write_is_atomic(tmp_path):
    """write_spec_with_log uses .tmp + os.replace — no partial writes."""
    spec = tmp_path / "spec.txt"
    spec.write_text("body QuadrupedBody\n" + LOG_MARKER + "\n", encoding="utf-8")
    user_lines, _ = read_spec(str(spec))
    write_spec_with_log(str(spec), user_lines, ["# Failures    : 0"])
    # .tmp file must not linger
    assert not (tmp_path / "spec.txt.tmp").exists()


# ---------------------------------------------------------------------------
# format_log
# ---------------------------------------------------------------------------


def test_format_log_no_failures():
    lines = format_log(
        "2026-01-01 00:00:00 UTC",
        "MyDoc",
        [("body/QuadrupedBody.stl", "Skeleton:1/Body:1", "+Z")],
        [],
    )
    text = "\n".join(lines)
    assert "Failures    : 0" in text
    assert "✓" in text
    assert "✗" not in text


def test_format_log_with_failures():
    lines = format_log(
        "2026-01-01 00:00:00 UTC",
        "MyDoc",
        [],
        [("leg/Missing.stl", "body 'Missing' not found")],
    )
    text = "\n".join(lines)
    assert "Failures    : 1" in text
    assert "✗" in text
