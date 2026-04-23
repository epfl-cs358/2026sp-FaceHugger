"""Shared constants for the FaceHugger simulation (no behavior)."""

import math
import os

URDF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "facehugger.urdf")

HIP_ANGLE = math.radians(40)
KNEE_ANGLE = math.radians(-60)
SHOULDER_ANGLE = 0.0

STANCE = {
    "shoulder": SHOULDER_ANGLE,
    "hip": HIP_ANGLE,
    "knee": KNEE_ANGLE,
}

SERVO_FORCE = 1.47
SERVO_VELOCITY = 5.0
TIMESTEP = 1.0 / 240.0
