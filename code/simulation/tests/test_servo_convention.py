# code/simulation/tests/test_servo_convention.py
"""Property tests for `translate_to_servo`.

Earlier versions of this file kept a verbatim copy of the firmware
`translateToServo` switch (`_firmware_translate`) as "independent ground
truth" and asserted byte-equality at specific poses. That approach broke
the moment per-leg `CALIB_*_THIGH/KNEE` were introduced (commit 706bf9b)
and required the test fixture to drift in lockstep with every CALIB
tweak — exactly the maintenance trap the parity test was meant to prevent.

The tests below replace those fixtures with *properties* of
`translate_to_servo` that survive any CALIB / formula tweak:

- The shoulder neutral pose still maps to servo 90 (load-bearing convention).
- math thigh/knee = 0 still maps to servo = CALIB[leg][joint] (defines CALIB).
- The function is affine in math-space with the expected per-leg sign
  pattern (derived from URDF axis sign), and perturbing one math joint
  does not leak into another joint's servo output.

Byte-for-byte parity with the firmware switch is still verified, but at
the exporter side, by `animation/scripts/test_servo_parity.py`.
"""

import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from firmware_port.servo_convention import (
    CALIB_KNEE,
    CALIB_THIGH,
    LEG_FL,
    LEG_FR,
    LEG_ID_TO_URDF_AXIS_SIGN,
    LEG_RL,
    LEG_RR,
    NEUTRAL,
    ServoTriple,
    clamp_clip_servos,
    servo_to_radians,
    translate_to_servo,
)

ALL_LEGS = [LEG_FR, LEG_FL, LEG_RR, LEG_RL]


def _expected_sign(leg_id: int, joint_idx: int) -> int:
    """Expected sign of d(servo) / d(math_input) for one (leg, joint).

    Derived from the URDF axis directions (independent specification),
    not from `translate_to_servo` itself — so the test catches divergence
    between the URDF and the firmware formulas. Shoulder is always +1
    per the load-bearing yaw-uniform convention; thigh sign equals the
    URDF link2/3 axis sign for that leg; knee sign is its negation
    (knee mounts on the opposite face — motion_math.cpp).
    """
    if joint_idx == 0:  # shoulder
        return +1
    axis = LEG_ID_TO_URDF_AXIS_SIGN[leg_id]
    return axis if joint_idx == 1 else -axis  # thigh, knee


# ─── NEUTRAL value tests (load-bearing constants) ─────────────────────────────


def test_neutral_fr():
    n = NEUTRAL[LEG_FR]
    assert n.sh == 45.0 and n.th == -60.0 and n.kn == -37.0


def test_neutral_fl():
    """Change B: FL shoulder sh=135 (outward direction matches FR/BR/BL)."""
    n = NEUTRAL[LEG_FL]
    assert n.sh == 135.0 and n.th == -60.0 and n.kn == -40.0


def test_neutral_rr():
    n = NEUTRAL[LEG_RR]
    assert n.sh == -45.0 and n.th == -50.0 and n.kn == -50.0


def test_neutral_rl():
    n = NEUTRAL[LEG_RL]
    assert n.sh == -135.0 and n.th == -60.0 and n.kn == -35.0


# ─── shoulder neutral → servo 90 (the yaw-uniform convention) ───────────────


@pytest.mark.parametrize("leg_id", ALL_LEGS)
def test_shoulder_at_neutral_maps_to_servo_90(leg_id):
    """At math-space NEUTRAL shoulder, every shoulder servo = 90.
    This is the load-bearing convention that survives any CALIB tweak."""
    n = NEUTRAL[leg_id]
    got = translate_to_servo(leg_id, n.sh, n.th, n.kn)
    assert got.hip == 90.0, f"leg {leg_id} shoulder neutral did not map to 90"


# ─── CALIB is the servo angle at math zero ───────────────────────────────────


@pytest.mark.parametrize("leg_id", ALL_LEGS)
def test_thigh_servo_at_math_zero_equals_calib(leg_id):
    """math thigh = 0 → servo thigh = CALIB_THIGH[leg]. Definition of CALIB."""
    n = NEUTRAL[leg_id]
    got = translate_to_servo(leg_id, n.sh, 0.0, n.kn)
    assert got.thigh == CALIB_THIGH[leg_id]


@pytest.mark.parametrize("leg_id", ALL_LEGS)
def test_knee_servo_at_math_zero_equals_calib(leg_id):
    """math knee = 0 → servo knee = CALIB_KNEE[leg]."""
    n = NEUTRAL[leg_id]
    got = translate_to_servo(leg_id, n.sh, n.th, 0.0)
    assert got.knee == CALIB_KNEE[leg_id]


# ─── affine response + no cross-axis leakage ────────────────────────────────


@pytest.mark.parametrize("leg_id", ALL_LEGS)
@pytest.mark.parametrize("joint_idx", [0, 1, 2])
def test_servo_response_matches_sign_convention(leg_id, joint_idx):
    """Perturbing ONE math joint by Δ shifts ITS servo by ±Δ (per
    `_expected_sign`) and leaves the other two servos untouched.

    Tests three properties at once, CALIB-invariantly:
      (a) sign of the response matches the URDF axis convention,
      (b) the response is exactly Δ in magnitude (linear, not scaled),
      (c) no cross-axis leakage.
    """
    n = NEUTRAL[leg_id]
    sh, th, kn = n.sh, n.th, n.kn
    base = translate_to_servo(leg_id, sh, th, kn)

    delta = 7.0
    pert = [sh, th, kn]
    pert[joint_idx] += delta
    after = translate_to_servo(leg_id, *pert)

    field = ("hip", "thigh", "knee")[joint_idx]
    got = getattr(after, field) - getattr(base, field)
    expected = _expected_sign(leg_id, joint_idx) * delta
    assert math.isclose(got, expected), (
        f"leg {leg_id} joint {field}: Δservo={got} expected={expected}"
    )

    for other_idx, other_field in enumerate(("hip", "thigh", "knee")):
        if other_idx == joint_idx:
            continue
        before_v = getattr(base, other_field)
        after_v = getattr(after, other_field)
        assert before_v == after_v, (
            f"leg {leg_id}: perturbing math {field} leaked into {other_field} "
            f"({before_v} → {after_v})"
        )


@pytest.mark.parametrize("leg_id", ALL_LEGS)
def test_translate_is_affine_under_random_inputs(leg_id):
    """`translate_to_servo` is affine in math-space: for two random poses A and B,
    `translate(A) − translate(B)` equals the expected sign-weighted (A − B).

    This catches non-linear drift (e.g. a stray square or gating in the
    formula) without depending on what CALIB the per-leg formulas are
    centered on.
    """
    rng = random.Random(42 + leg_id)
    for _ in range(20):
        a = (rng.uniform(-180, 180), rng.uniform(-90, 90), rng.uniform(-90, 90))
        b = (rng.uniform(-180, 180), rng.uniform(-90, 90), rng.uniform(-90, 90))
        sa = translate_to_servo(leg_id, *a)
        sb = translate_to_servo(leg_id, *b)

        for joint_idx, field in enumerate(("hip", "thigh", "knee")):
            got = getattr(sa, field) - getattr(sb, field)
            expected = _expected_sign(leg_id, joint_idx) * (a[joint_idx] - b[joint_idx])
            assert math.isclose(got, expected, abs_tol=1e-9), (
                f"leg {leg_id} {field}: Δ={got} expected={expected} for a={a} b={b}"
            )


# ─── clamp_clip_servos (per-leg, CALIB-relative) ─────────────────────────────


@pytest.mark.parametrize("leg_id", [LEG_FR, LEG_FL, LEG_RR, LEG_RL])
def test_clamp_clip_servos_passthrough(leg_id):
    """Mid-range (90,90,90) sits inside every leg's window — pass-through."""
    s = ServoTriple(hip=90.0, thigh=90.0, knee=90.0)
    c = clamp_clip_servos(leg_id, s)
    assert c.hip == 90.0 and c.thigh == 90.0 and c.knee == 90.0


@pytest.mark.parametrize("leg_id", [LEG_FR, LEG_FL, LEG_RR, LEG_RL])
def test_clamp_clip_servos_hip_uniform(leg_id):
    """Hip envelope is uniform across legs: 90 ± 52 = [38, 142]."""
    assert clamp_clip_servos(leg_id, ServoTriple(0.0, 90.0, 90.0)).hip == 38.0
    assert clamp_clip_servos(leg_id, ServoTriple(200.0, 90.0, 90.0)).hip == 142.0


def test_clamp_clip_servos_thigh_FL_neutral_27():
    """The motivating case: FL NEUTRAL (servo thigh = 27 = CALIB_FL_THIGH - 60)
    must sit on the lower edge, not be clipped to 30 by an old uniform window."""
    s = ServoTriple(hip=90.0, thigh=27.0, knee=90.0)
    assert clamp_clip_servos(LEG_FL, s).thigh == 27.0
    # And anything below CALIB-60 IS clipped to CALIB-60.
    s_low = ServoTriple(hip=90.0, thigh=26.0, knee=90.0)
    assert clamp_clip_servos(LEG_FL, s_low).thigh == 27.0


def test_clamp_clip_servos_thigh_BR_upper_163():
    """CALIB_BR_THIGH = 103 → window [43, 163]. Upper edge is 163, not 150."""
    assert clamp_clip_servos(LEG_RR, ServoTriple(90.0, 163.0, 90.0)).thigh == 163.0
    assert clamp_clip_servos(LEG_RR, ServoTriple(90.0, 164.0, 90.0)).thigh == 163.0
    assert clamp_clip_servos(LEG_RR, ServoTriple(90.0, 43.0, 90.0)).thigh == 43.0
    assert clamp_clip_servos(LEG_RR, ServoTriple(90.0, 42.0, 90.0)).thigh == 43.0


def test_clamp_clip_servos_thigh_FR_lower_24():
    """CALIB_FR_THIGH = 84 → window [24, 144]. 24 unchanged; 23 clamped to 24."""
    assert clamp_clip_servos(LEG_FR, ServoTriple(90.0, 24.0, 90.0)).thigh == 24.0
    assert clamp_clip_servos(LEG_FR, ServoTriple(90.0, 23.0, 90.0)).thigh == 24.0


def test_clamp_clip_servos_knee_per_leg():
    """Knee is CALIB ± 90 per leg, NOT a uniform [0, 180]."""
    # CALIB_FR_KNEE = 95 → window [5, 185]
    assert clamp_clip_servos(LEG_FR, ServoTriple(90.0, 90.0, 4.0)).knee == 5.0
    # CALIB_FL_KNEE = 82 → window [-8, 172]
    assert clamp_clip_servos(LEG_FL, ServoTriple(90.0, 90.0, 173.0)).knee == 172.0


def test_clamp_clip_servos_unknown_leg_raises():
    with pytest.raises(ValueError):
        clamp_clip_servos(99, ServoTriple(90.0, 90.0, 90.0))


# ─── servo_to_radians ────────────────────────────────────────────────────────


def test_servo_90_is_zero_rad():
    assert abs(servo_to_radians(90.0)) < 1e-12


def test_servo_0_is_minus_half_pi():
    assert abs(servo_to_radians(0.0) - (-math.pi / 2)) < 1e-12


def test_servo_180_is_plus_half_pi():
    assert abs(servo_to_radians(180.0) - (math.pi / 2)) < 1e-12
