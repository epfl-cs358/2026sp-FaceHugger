"""Tests for stl_transform — pure-Python, no adsk dependency."""

import os
import struct
import sys
import tempfile

# Add the add-in root (parent of lib/) so `lib` is importable as a package.
sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)

import pytest

from lib.config import ROTATIONS
from lib.stl_transform import rotate_stl_in_place, rotation_for_up_axis

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_IDENTITY = ROTATIONS["+Z"]


def _make_stl(triangles):
    """Build a minimal binary STL bytes object.

    `triangles` is a list of (normal, v1, v2, v3) where each is a 3-tuple of floats.
    """
    header = b"\x00" * 80
    n = len(triangles)
    blob = bytearray(header)
    blob += struct.pack("<I", n)
    for nrm, v1, v2, v3 in triangles:
        blob += struct.pack("<12f", *nrm, *v1, *v2, *v3)
        blob += b"\x00\x00"  # attribute byte count
    return bytes(blob)


def _write_stl(triangles):
    """Write a binary STL to a temp file and return the path."""
    data = _make_stl(triangles)
    fd, path = tempfile.mkstemp(suffix=".stl")
    os.close(fd)
    with open(path, "wb") as f:
        f.write(data)
    return path, data


def _read_stl_bytes(path):
    with open(path, "rb") as f:
        return f.read()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_identity_leaves_bytes_unchanged():
    """Applying the +Z identity rotation must not change any byte."""
    tris = [
        ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 0.0)),
        ((1.0, 0.0, 0.0), (2.0, 0.0, 0.0), (2.0, 1.0, 0.0), (2.0, 0.0, 1.0)),
    ]
    path, original = _write_stl(tris)
    try:
        rotate_stl_in_place(path, _IDENTITY)
        assert _read_stl_bytes(path) == original
    finally:
        os.unlink(path)


def test_apply_then_inverse_restores_original():
    """+Y rotation followed by its inverse (-Y) must restore the original bytes."""
    tris = [
        ((0.0, 1.0, 0.0), (1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0)),
    ]
    path, original = _write_stl(tris)
    try:
        rotate_stl_in_place(path, ROTATIONS["+Y"])
        rotate_stl_in_place(path, ROTATIONS["-Y"])
        result = _read_stl_bytes(path)
        # float round-trips through struct.pack/unpack may introduce tiny epsilon;
        # compare byte-for-byte because single-precision is stable for these values.
        assert result == original
    finally:
        os.unlink(path)


def test_rotation_for_up_axis_raises_on_unknown():
    """rotation_for_up_axis must raise ValueError for an unrecognised axis label."""
    with pytest.raises(ValueError, match="Unknown up axis"):
        rotation_for_up_axis("+W")


def test_rotation_for_up_axis_returns_correct_matrix():
    """rotation_for_up_axis must return the same object as ROTATIONS[axis]."""
    for axis in ROTATIONS:
        assert rotation_for_up_axis(axis) is ROTATIONS[axis]
