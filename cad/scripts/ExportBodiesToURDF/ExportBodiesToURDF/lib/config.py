# config.py — All whitelists, EXPORT_RULES, and shared constants.
# No adsk imports — pure data only.
# Team members adding a mesh rule or adjusting a whitelist only need to edit
# this file.

CM_TO_MM = 10.0
CM2_TO_M2 = 1e-4  # kg·cm² → kg·m²

COLLECT_PHYSICS = True  # mass/CoM/inertia — slow, set False to skip

# Leg-assembly normalization: the FaceHuggerLegAssembly:1 occurrence is
# placed in CAD with a non-identity world rotation (currently 90° about Z).
# Mesh vertices and per-joint axis_dir / axis_origin live in that rotated
# frame, which trips up the URDF generator. We capture that rotation as
# `R_la` and apply it to leg-assembly-internal joint data in collect_joints()
# so the JSON output is in a world-aligned frame.
LEG_ASSEMBLY_OCCURRENCE = "FaceHuggerLegAssembly:1"
LEG_ASSEMBLY_COMPONENT = "FaceHuggerLegAssembly"

CONSTRUCTION_POINTS = [
    # Body-side mating points (FlexibleSkeleton:1).
    "LegMountPointFL",
    "LegMountPointFR",
    "LegMountPointBR",
    "LegMountPointBL",
    # Body Z=0 datum (under the cage).
    "BodyBottomPoint",
    # Shell alignment pair.
    "BodyAlignToShellPoint",
    "ShellAlignToBodyPoint",
    # Joint origins (FaceHuggerLegAssembly:1).
    "BodyToLink1Point",
    "Link1ToLink2Point",
    "Link2ToLink3Point",
    # Bracket-side mating point (MotorMount + MotorMountR).
    "LegMountFixedPoint",
    # Servo seat (each LegBaseServoEnclosure occurrence).
    "ServoMountPoint",
    # Link3 tip defined as a vertex.
    "Link3TipPoint",
]

CONSTRUCTION_AXES = [
    "BodyToLink1Axis",
    "Link1ToLink2Axis",
    "Link2ToLink3Axis",
]

JOINTS = [
    "Link1Revolute",
    "Link2Revolute",
    "Link3Revolute",
]

# Rules driving STL export. Three rule types supported:
#
#   {"type": "occurrence", "match": "FlexibleSkeleton:1", "stl": "X.stl"}
#   {"type": "body", "match": "BodyName", "stl": "X.stl",
#    "component": "...", "origin_landmark": "..."}
#   {"type": "combined", "stl": "X.stl", "parts": [...],
#    "origin_landmark": "...", "landmark_occurrence": "..."}
#
# See the module docstring in ExportBodiesToURDF.py for full semantics.
EXPORT_RULES = [
    {
        "type": "combined",
        "stl": "QuadrupedBody.stl",
        "parts": [
            {
                "occurrence": "FlexibleSkeleton:1/QuadrupedBody:1",
                "body": "QuadrupedBody",
            },
            {
                "occurrence": "FlexibleSkeleton:1/QuadrupedBody:1",
                "body": "MiddleBridge",
            },
            {
                "occurrence": "FlexibleSkeleton:1/QuadrupedBody:1",
                "body": "BehindBridge",
            },
            {"occurrence": "FlexibleSkeleton:1/LipoCage:1", "body": "LipoCage"},
            {"occurrence": "FlexibleSkeleton:1/Shell:1", "body": "*"},
        ],
    },
    {
        "type": "combined",
        "stl": "leg_mount_L.stl",
        "origin_landmark": "LegMountFixedPoint",
        "landmark_occurrence": "FaceHuggerLegAssembly:1/MotorMount:1",
        "parts": [
            {"occurrence": "FaceHuggerLegAssembly:1/MotorMount:1", "body": "LegMountL"},
        ],
    },
    {
        "type": "combined",
        "stl": "leg_mount_R.stl",
        "origin_landmark": "LegMountFixedPoint",
        "landmark_occurrence": "FaceHuggerLegAssembly:1/MotorMountR:1",
        "parts": [
            {
                "occurrence": "FaceHuggerLegAssembly:1/MotorMountR:1",
                "body": "LegMountR",
            },
        ],
    },
    {
        "type": "combined",
        "stl": "leg_shoulder_L.stl",
        "origin_landmark": "BodyToLink1Point",
        "parts": [
            {"occurrence": "FaceHuggerLegAssembly:1/Link1L:1", "body": "Link1L"},
        ],
    },
    {
        "type": "combined",
        "stl": "leg_shoulder_R.stl",
        "origin_landmark": "BodyToLink1Point",
        "parts": [
            {"occurrence": "FaceHuggerLegAssembly:1/Link1R:1", "body": "Link1R"},
        ],
    },
    {
        "type": "combined",
        "stl": "leg_upper.stl",
        "origin_landmark": "Link1ToLink2Point",
        "parts": [
            {"occurrence": "FaceHuggerLegAssembly:1/Link2L:1", "body": "Link2"},
        ],
    },
    {
        "type": "combined",
        "stl": "leg_lower.stl",
        "origin_landmark": "Link2ToLink3Point",
        "parts": [
            {"occurrence": "FaceHuggerLegAssembly:1/Link3L:1", "body": "Link3"},
        ],
    },
    {
        "type": "combined",
        "stl": "servo.stl",
        "origin_landmark": "ServoMountPoint",
        "parts": [
            {
                "occurrence": "FaceHuggerLegAssembly:1/LegBaseServoEnclosure:1",
                "body": "ServoBase",
            },
        ],
    },
]

# ---------------------------------------------------------------------------
# FALLBACK_MASS_OVERRIDES — electronics components where Fusion cannot compute
# physics (MCAD STEP imports with no material assigned: PCB_ESP32, PCA9685,
# OLED, BuckConverter, IMU, ToF, BMS).
#
# These values belong in facehugger_config.yaml (consumed by generate_urdf.py)
# rather than here — this dict is documentation of the intended schema and
# approximate measured masses only. Move to config.yaml before baking URDF
# inertial blocks.
#
# Measured masses (kg):
#   PCB_ESP32-38Pines:1      0.010   ESP32 dev board
#   PCA9685 v18:1            0.005   16-ch PWM driver
#   OLED 128x64 0.96in:1     0.005   display module
#   IMU:1                    0.002   MPU6050 breakout
#   BuckConverter:1          0.003   5V buck module
#   BMS:1                    0.004   LiPo protection/balancer
#   ToF:1                    0.001   VL53L0X time-of-flight sensor
# ---------------------------------------------------------------------------
FALLBACK_MASS_OVERRIDES = {
    "PCB_ESP32-38Pines:1": 0.010,  # kg, measured
    "PCA9685 v18:1": 0.005,
    "OLED 128 x 64 Display 0.96 Inch:1": 0.005,
    "IMU:1": 0.002,
    "BuckConverter:1": 0.003,
    "BMS:1": 0.004,
    "ToF:1": 0.001,
    # move to facehugger_config.yaml for URDF generation
}
