# code/simulation/firmware_port/tests/test_servo_convention.py
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

from firmware_port.servo_convention import (
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
        # BR shoulder un-mirrored (2026-05-25): +sh = +servo like the others.
        return (90.0 + (sh + 45.0), 90.0 + th, 90.0 - kn)
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
