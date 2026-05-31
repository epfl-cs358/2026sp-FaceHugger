"""Tests for lib/stl_utils.py — pure binary STL I/O."""

import struct
import sys
import tempfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from lib.stl_utils import (
    read_binary_stl,
    transform_triangles,
    translate_stl_in_place,
    write_binary_stl,
)

_IDENTITY = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

_TRIS = [
    ((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    ((0.0, 0.0, -1.0), (0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (0.0, 1.0, 1.0)),
]


def test_roundtrip():
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as f:
        path = f.name
    write_binary_stl(path, _TRIS)
    result = read_binary_stl(path)
    assert len(result) == len(_TRIS)
    for (n_got, v0_got, v1_got, v2_got), (n_exp, v0_exp, v1_exp, v2_exp) in zip(
        result, _TRIS
    ):
        for g, e in zip(n_got, n_exp):
            assert abs(g - e) < 1e-6
        for g, e in zip(v0_got, v0_exp):
            assert abs(g - e) < 1e-6


def test_write_correct_size():
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as f:
        path = f.name
    write_binary_stl(path, _TRIS)
    size = Path(path).stat().st_size
    assert size == 84 + len(_TRIS) * 50


def test_read_too_short_raises():
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as f:
        f.write(b"\x00" * 10)
        path = f.name
    with pytest.raises(RuntimeError, match="too short"):
        read_binary_stl(path)


def test_read_size_mismatch_raises():
    buf = bytearray(b"\x00" * 80)
    buf.extend(struct.pack("<I", 99))  # claims 99 triangles
    buf.extend(b"\x00" * 50)  # only 1 triangle worth of data
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as f:
        f.write(buf)
        path = f.name
    with pytest.raises(RuntimeError, match="size mismatch"):
        read_binary_stl(path)


def test_transform_identity_preserves_vertices():
    result = transform_triangles(_TRIS, _IDENTITY, (0.0, 0.0, 0.0))
    for (n_got, v0_got, v1_got, v2_got), (n_exp, v0_exp, v1_exp, v2_exp) in zip(
        result, _TRIS
    ):
        for g, e in zip(v0_got, v0_exp):
            assert abs(g - e) < 1e-9


def test_transform_translation():
    shift = (10.0, 0.0, 0.0)
    result = transform_triangles(_TRIS, _IDENTITY, shift)
    orig_v0 = _TRIS[0][1]
    got_v0 = result[0][1]
    assert abs(got_v0[0] - (orig_v0[0] + 10.0)) < 1e-9
    assert abs(got_v0[1] - orig_v0[1]) < 1e-9


def test_transform_normal_not_translated():
    shift = (99.0, 99.0, 99.0)
    result = transform_triangles(_TRIS, _IDENTITY, shift)
    orig_n = _TRIS[0][0]
    got_n = result[0][0]
    for g, e in zip(got_n, orig_n):
        assert abs(g - e) < 1e-9


def test_translate_in_place():
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as f:
        path = f.name
    write_binary_stl(path, _TRIS)
    translate_stl_in_place(path, (1.0, 0.0, 0.0))
    result = read_binary_stl(path)
    orig_v0 = _TRIS[0][1]
    got_v0 = result[0][1]
    assert abs(got_v0[0] - (orig_v0[0] - 1.0)) < 1e-5
    assert abs(got_v0[1] - orig_v0[1]) < 1e-5


def test_translate_in_place_zero_is_noop():
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as f:
        path = f.name
    write_binary_stl(path, _TRIS)
    translate_stl_in_place(path, (0.0, 0.0, 0.0))
    result = read_binary_stl(path)
    orig_v0 = _TRIS[0][1]
    got_v0 = result[0][1]
    for g, e in zip(got_v0, orig_v0):
        assert abs(g - e) < 1e-5
