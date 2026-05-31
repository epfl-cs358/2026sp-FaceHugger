"""Tests for config — orthonormality checks on ROTATIONS matrices."""

import os
import sys

# Add the add-in root (parent of lib/) so `lib` is importable as a package.
sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)

from lib.config import ROTATIONS

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _matmul(A, B):
    """3×3 matrix multiply: C = A @ B."""
    n = 3
    return tuple(
        tuple(sum(A[i][k] * B[k][j] for k in range(n)) for j in range(n))
        for i in range(n)
    )


def _det3(R):
    """Determinant of a 3×3 matrix."""
    r = R
    return (
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
    )


def _transpose(R):
    return tuple(tuple(R[j][i] for j in range(3)) for i in range(3))


def _is_close(a, b, tol=1e-9):
    return abs(a - b) < tol


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_all_rotations_are_orthonormal():
    """Every matrix in ROTATIONS must satisfy R @ R.T = I and det(R) = +1."""
    identity = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    for axis, R in ROTATIONS.items():
        RRt = _matmul(R, _transpose(R))
        for i in range(3):
            for j in range(3):
                expected = identity[i][j]
                assert _is_close(RRt[i][j], expected), (
                    f"ROTATIONS[{axis!r}]: R @ R.T [{i}][{j}] = {RRt[i][j]}, "
                    f"expected {expected}"
                )
        det = _det3(R)
        assert _is_close(det, 1.0), f"ROTATIONS[{axis!r}]: det = {det}, expected 1.0"
