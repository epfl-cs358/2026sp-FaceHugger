# code/simulation/pybullet_interpreter/tests/test_servo_convention.py
"""Parity tests: assert servo_convention matches firmware translateToServo.

Structure mirrors animation/scripts/test_servo_parity.py.
_firmware_translate() is copied verbatim from firmware source as
independent ground truth — DO NOT factor it out into servo_convention.py.
Divergence between the two is precisely the bug these tests guard against.

Firmware source: code/firmware/src/nervous_system/motion_math.cpp:32-59
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from pybullet_interpreter.servo_convention import (
    LEG_FL,
    LEG_FR,
    LEG_RL,
    LEG_RR,
    NEUTRAL,
    ServoTriple,
    clamp_clip_servos,
    servo_to_radians,
    translate_to_servo,
)


def _firmware_translate(
    leg_id: int, sh: float, th: float, kn: float
) -> tuple[float, float, float]:
    """Independent ground truth — verbatim from motion_math.cpp:32-59.
    DO NOT replace with servo_convention.translate_to_servo.
    """
    if leg_id == 0:  # LEG_FR
        return (90.0 + (sh - 45.0), 90.0 - th, 90.0 + kn)
    if leg_id == 1:  # LEG_FL — Change B: 90 + (sh - 135)
        return (90.0 + (sh - 135.0), 90.0 + th, 90.0 - kn)
    if leg_id == 2:  # LEG_RR / BR
        return (90.0 - (sh + 45.0), 90.0 + th, 90.0 - kn)
    if leg_id == 3:  # LEG_RL / BL
        return (90.0 + (sh + 135.0), 90.0 - th, 90.0 + kn)
    raise ValueError(f"unknown leg_id {leg_id}")


# ─── NEUTRAL value tests ─────────────────────────────────────────────────────


def test_neutral_fr():
    """FR neutral must match neutral_pose.h:14."""
    n = NEUTRAL[LEG_FR]
    assert n.sh == 45.0 and n.th == -60.0 and n.kn == -37.0


def test_neutral_fl():
    """FL neutral must match neutral_pose.h:15 (Change B: sh=135)."""
    n = NEUTRAL[LEG_FL]
    assert n.sh == 135.0 and n.th == -60.0 and n.kn == -40.0


def test_neutral_rr():
    """RR/BR neutral must match neutral_pose.h:16."""
    n = NEUTRAL[LEG_RR]
    assert n.sh == -45.0 and n.th == -50.0 and n.kn == -50.0


def test_neutral_rl():
    """RL/BL neutral must match neutral_pose.h:17."""
    n = NEUTRAL[LEG_RL]
    assert n.sh == -135.0 and n.th == -60.0 and n.kn == -35.0


# ─── translate_to_servo parity at NEUTRAL ───────────────────────────────────


@pytest.mark.parametrize("leg_id", [LEG_FR, LEG_FL, LEG_RR, LEG_RL])
def test_translate_at_neutral_matches_firmware(leg_id):
    """At NEUTRAL pose, translate_to_servo must equal firmware formula."""
    n = NEUTRAL[leg_id]
    got = translate_to_servo(leg_id, n.sh, n.th, n.kn)
    exp_hip, exp_th, exp_kn = _firmware_translate(leg_id, n.sh, n.th, n.kn)
    assert abs(got.hip - exp_hip) < 1e-9, f"hip mismatch leg {leg_id}"
    assert abs(got.thigh - exp_th) < 1e-9, f"thigh mismatch leg {leg_id}"
    assert abs(got.knee - exp_kn) < 1e-9, f"knee mismatch leg {leg_id}"


# ─── translate_to_servo parity at NEUTRAL + uniform 30° offset ──────────────


@pytest.mark.parametrize("leg_id", [LEG_FR, LEG_FL, LEG_RR, LEG_RL])
def test_translate_at_offset_matches_firmware(leg_id):
    """At NEUTRAL + 30° on all joints, translate_to_servo must equal firmware."""
    n = NEUTRAL[leg_id]
    sh, th, kn = n.sh + 30.0, n.th + 30.0, n.kn + 30.0
    got = translate_to_servo(leg_id, sh, th, kn)
    exp_hip, exp_th, exp_kn = _firmware_translate(leg_id, sh, th, kn)
    assert abs(got.hip - exp_hip) < 1e-9
    assert abs(got.thigh - exp_th) < 1e-9
    assert abs(got.knee - exp_kn) < 1e-9


# ─── per-axis perturbation (catches cross-axis leakage) ─────────────────────


@pytest.mark.parametrize("leg_id", [LEG_FR, LEG_FL, LEG_RR, LEG_RL])
@pytest.mark.parametrize("joint", ["sh", "th", "kn"])
def test_translate_single_joint_perturb(leg_id, joint):
    """Perturbing one joint must not affect others' servo output."""
    n = NEUTRAL[leg_id]
    sh = n.sh + (20.0 if joint == "sh" else 0.0)
    th = n.th + (20.0 if joint == "th" else 0.0)
    kn = n.kn + (20.0 if joint == "kn" else 0.0)
    got = translate_to_servo(leg_id, sh, th, kn)
    exp_hip, exp_th, exp_kn = _firmware_translate(leg_id, sh, th, kn)
    assert abs(got.hip - exp_hip) < 1e-9
    assert abs(got.thigh - exp_th) < 1e-9
    assert abs(got.knee - exp_kn) < 1e-9


# ─── clamp_clip_servos ───────────────────────────────────────────────────────


def test_clamp_clip_servos_passthrough():
    """A servo triple already in range must pass through unchanged."""
    s = ServoTriple(hip=90.0, thigh=90.0, knee=90.0)
    c = clamp_clip_servos(s)
    assert c.hip == 90.0 and c.thigh == 90.0 and c.knee == 90.0


def test_clamp_clip_servos_hip_low():
    """Hip below 38 must be clamped to 38."""
    s = ServoTriple(hip=0.0, thigh=90.0, knee=90.0)
    c = clamp_clip_servos(s)
    assert c.hip == 38.0


def test_clamp_clip_servos_thigh_high():
    """Thigh above 150 must be clamped to 150."""
    s = ServoTriple(hip=90.0, thigh=200.0, knee=90.0)
    c = clamp_clip_servos(s)
    assert c.thigh == 150.0


def test_clamp_clip_servos_knee_range():
    """Knee is full-travel [0, 180]; values at edges stay unchanged."""
    s_lo = ServoTriple(hip=90.0, thigh=90.0, knee=0.0)
    s_hi = ServoTriple(hip=90.0, thigh=90.0, knee=180.0)
    assert clamp_clip_servos(s_lo).knee == 0.0
    assert clamp_clip_servos(s_hi).knee == 180.0


# ─── servo_to_radians ────────────────────────────────────────────────────────


def test_servo_90_is_zero_rad():
    assert abs(servo_to_radians(90.0)) < 1e-12


def test_servo_0_is_minus_half_pi():
    assert abs(servo_to_radians(0.0) - (-math.pi / 2)) < 1e-12


def test_servo_180_is_plus_half_pi():
    assert abs(servo_to_radians(180.0) - (math.pi / 2)) < 1e-12
