"""PyBullet scene lifecycle: connect, load robot, settle, print banner."""

import math
import os

import pybullet as p
import pybullet_data

from .paths import TIMESTEP
from .math_utils import _wrap_pi
from .motor import apply_leg_pose, build_joint_map, reset_to_stance

# --------------------------------------------------------------------------- #
# Physics realism constants
# --------------------------------------------------------------------------- #
FOOT_LATERAL_FRICTION = 2.5  # high grip — models the rubber/elastic bands on the
# real feet (PyBullet multiplies foot × plane, so this is the grippy end)
FOOT_SPINNING_FRICTION = 0.3  # rubber tips resist the foot pivoting in place
# Soft contact for the feet — PyBullet's default infinitely-stiff resolution
# makes the body bounce on every footfall and prevents a planted stance.
# Tuned so each footfall absorbs over a few timesteps instead of one. Stiffness
# scales the spring force per unit penetration; damping scales the velocity at
# contact. Values below are conservative; raise stiffness if the feet visibly
# sink, lower damping if the body still oscillates after impact.
FOOT_CONTACT_STIFFNESS = 30000.0
FOOT_CONTACT_DAMPING = 100.0
SOLVER_ITERATIONS = 150  # stiffer, less-jittery contact resolution


def connect_and_setup(cfg, gui, float_mode=False):
    """Connect PyBullet and load the plane + robot.

    float_mode=True: no gravity, no floor, and the body is pinned in the air
    (useFixedBase). Use it to watch a clip's pure joint geometry — each leg
    articulates exactly as authored, with no falling/slipping/collapse from
    physics. (Normal mode loads the ground plane, real gravity, and a free
    floating base so you see dynamic balance.)
    """
    p.connect(p.GUI if gui else p.DIRECT)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0 if float_mode else -9.81)
    p.setTimeStep(TIMESTEP)
    # More solver iterations -> stiffer, less-jittery contacts (helps planted-feet
    # moves resolve cleanly). Harmless in float mode (no contacts).
    p.setPhysicsEngineParameter(numSolverIterations=SOLVER_ITERATIONS)
    if not float_mode:
        p.loadURDF("plane.urdf")

    robot_id = p.loadURDF(
        cfg.urdf_path,
        basePosition=[0, 0, cfg.body_height + 0.02],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=float_mode,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
    )
    joint_map = build_joint_map(robot_id)
    # stance_rad is already per-leg; reset + motor-command from the same dict.
    reset_to_stance(robot_id, joint_map, cfg.stance_rad)
    apply_leg_pose(
        robot_id, joint_map, cfg.stance_rad, cfg.servo_force, cfg.servo_velocity
    )

    # Foot contact (link3 = the knee joint's child = lower leg/foot). NB: joint_map
    # keys are URDF joint names (`*_link3_joint`) — they contain "link3", not
    # "knee"; the old "knee" match never fired, so feet sat at PyBullet's default
    # 0.5 friction and slipped during planted-feet moves. Grip + a little spin
    # resistance so the feet hold; no bounce.
    for name, idx in joint_map.items():
        if "link3" in name:
            p.changeDynamics(
                robot_id,
                idx,
                lateralFriction=FOOT_LATERAL_FRICTION,
                spinningFriction=FOOT_SPINNING_FRICTION,
                restitution=0.0,
                contactStiffness=FOOT_CONTACT_STIFFNESS,
                contactDamping=FOOT_CONTACT_DAMPING,
            )

    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.55,
            cameraYaw=45,
            cameraPitch=-25,
            cameraTargetPosition=[0, 0, 0.1],
        )
    return robot_id, joint_map


def settle(robot_id, joint_map, cfg, duration_s):
    """Step the sim for `duration_s` while holding stance targets. Lets
    gravity resolve any initial overlap before gait/stand loops begin.
    No visible debug draws here — just physics."""
    n_steps = int(duration_s / TIMESTEP)
    for _ in range(n_steps):
        apply_leg_pose(
            robot_id, joint_map, cfg.stance_rad, cfg.servo_force, cfg.servo_velocity
        )
        p.stepSimulation()


def print_banner(cfg):
    print("\n=== FaceHugger sim ===")
    print(f"  URDF: {os.path.basename(cfg.urdf_path)}")
    print(f"  legs: {list(cfg.legs.keys())}")
    print(f"  servo: force={cfg.servo_force} N*m  vel={cfg.servo_velocity} rad/s")
    print(f"  body_height: {cfg.body_height * 1000:.1f} mm")
    print("  stance (deg, per leg):")
    for leg_id, s in cfg.stance_rad.items():
        print(
            f"    {leg_id}: shoulder={math.degrees(s['shoulder']):+.1f}  "
            f"hip={math.degrees(s['hip']):+.1f}  knee={math.degrees(s['knee']):+.1f}"
        )
    for leg_id, foot in cfg.neutral_foot.items():
        print(
            f"    {leg_id}: foot = "
            f"({foot[0] * 1000:+6.1f}, {foot[1] * 1000:+6.1f}, {foot[2] * 1000:+6.1f}) mm"
        )

    # IK round-trip check: recover stance angles from neutral foot.
    tol = math.radians(2.0)
    for leg_id, foot in cfg.neutral_foot.items():
        s, h, k = cfg.leg_ik(cfg, foot, leg_id)
        target = cfg.stance_rad[leg_id]
        ds = abs(_wrap_pi(s - target["shoulder"]))
        dh = abs(h - target["hip"])
        dk = abs(k - target["knee"])
        ok = max(ds, dh, dk) < tol
        tag = "OK" if ok else "FAIL"
        print(
            f"    IK[{leg_id}]: ds={math.degrees(ds):+.2f} dh={math.degrees(dh):+.2f} "
            f"dk={math.degrees(dk):+.2f} [{tag}]"
        )
