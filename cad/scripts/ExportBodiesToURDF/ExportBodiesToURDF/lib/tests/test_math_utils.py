"""Tests for lib/math_utils.py — pure float/matrix helpers."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from lib.math_utils import (
    _apply_R_3x3,
    _rad_to_deg,
    apply_matrix_cm,
    apply_matrix_to_dir_cm,
    mat_multiply_arrays,
    matrix_as_row_major_cm,
    matrix_translation_mm,
    pt_mm,
    vec3,
)


# ── simple duck-typed stubs for Fusion API objects ───────────────────────────


class _Pt:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _FlatMat:
    """Minimal Matrix3D stub — stores a flat 16-element row-major list."""

    def __init__(self, a):
        self._a = list(a)

    def asArray(self):
        return self._a


def _identity_mat():
    a = [0.0] * 16
    for i in range(4):
        a[i * 4 + i] = 1.0
    return _FlatMat(a)


def _translate_mat(tx, ty, tz):
    a = [0.0] * 16
    for i in range(4):
        a[i * 4 + i] = 1.0
    a[3], a[7], a[11] = tx, ty, tz  # row-major translation
    return _FlatMat(a)


# ── tests ─────────────────────────────────────────────────────────────────────


def test_pt_mm_converts_cm_to_mm():
    p = _Pt(1.0, 2.5, 0.1)
    assert pt_mm(p) == [10.0, 25.0, 1.0]


def test_vec3_rounds_to_4dp():
    v = _Pt(0.123456, -0.999999, 0.000001)
    result = vec3(v)
    assert result == [round(v.x, 4), round(v.y, 4), round(v.z, 4)]


def test_matrix_as_row_major_cm_identity():
    m = _identity_mat()
    rows = matrix_as_row_major_cm(m)
    assert len(rows) == 4
    for i in range(4):
        assert len(rows[i]) == 4
    assert rows[0][0] == 1.0
    assert rows[1][1] == 1.0
    assert rows[0][3] == 0.0


def test_matrix_translation_mm():
    m = _translate_mat(1.0, 2.0, 3.0)  # 1.0 cm = 10 mm
    t = matrix_translation_mm(m)
    assert t == [10.0, 20.0, 30.0]


def test_apply_matrix_cm_identity():
    m = _identity_mat()
    result = apply_matrix_cm(m, (1.0, 2.0, 3.0))
    assert result == (1.0, 2.0, 3.0)


def test_apply_matrix_cm_translation():
    m = _translate_mat(1.0, 0.0, 0.0)
    result = apply_matrix_cm(m, (0.0, 0.0, 0.0))
    assert result == (1.0, 0.0, 0.0)


def test_apply_matrix_to_dir_ignores_translation():
    m = _translate_mat(99.0, 99.0, 99.0)
    result = apply_matrix_to_dir_cm(m, (1.0, 0.0, 0.0))
    assert result == (1.0, 0.0, 0.0)


def test_mat_multiply_identity():
    identity = [0.0] * 16
    for i in range(4):
        identity[i * 4 + i] = 1.0
    a = [float(i) for i in range(16)]
    assert mat_multiply_arrays(a, identity) == a
    assert mat_multiply_arrays(identity, a) == a


def test_mat_multiply_associativity():
    import random

    random.seed(42)
    a = [random.uniform(-1, 1) for _ in range(16)]
    b = [random.uniform(-1, 1) for _ in range(16)]
    c = [random.uniform(-1, 1) for _ in range(16)]
    ab_c = mat_multiply_arrays(mat_multiply_arrays(a, b), c)
    a_bc = mat_multiply_arrays(a, mat_multiply_arrays(b, c))
    for x, y in zip(ab_c, a_bc):
        assert abs(x - y) < 1e-9


def test_apply_R_3x3_identity():
    I = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    v = [1.0, 2.0, 3.0]
    assert _apply_R_3x3(I, v) == v


def test_apply_R_3x3_none_passthrough():
    v = [1.0, 2.0, 3.0]
    assert _apply_R_3x3(None, v) is v
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    assert _apply_R_3x3(R, None) is None


def test_apply_R_3x3_90deg_rotation():
    Rz90 = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    result = _apply_R_3x3(Rz90, [1.0, 0.0, 0.0])
    assert abs(result[0]) < 1e-9
    assert abs(result[1] - 1.0) < 1e-9
    assert abs(result[2]) < 1e-9


def test_rad_to_deg_known():
    assert abs(_rad_to_deg(math.pi) - 180.0) < 1e-9
    assert abs(_rad_to_deg(0.0)) < 1e-9


def test_rad_to_deg_invalid_returns_zero():
    assert _rad_to_deg("not a number") == 0.0
    assert _rad_to_deg(None) == 0.0
