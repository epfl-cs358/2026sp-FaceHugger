"""Elementary math helpers shared across pybullet_sim modules."""

import math


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _parse_xyz(s):
    return tuple(float(x) for x in s.strip().split())
