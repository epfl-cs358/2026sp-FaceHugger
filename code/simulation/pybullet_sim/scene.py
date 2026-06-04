"""PyBullet scene lifecycle: connect, load robot, settle, print banner."""

import math
import os

import pybullet as p
import pybullet_data
import yaml

from .paths import SIM_CONFIG_YAML, TIMESTEP
from .math_utils import _wrap_pi
from .motor import apply_leg_pose, build_joint_map, reset_to_stance

# --------------------------------------------------------------------------- #
# Physics config (loaded from sim_config.yaml at connect time)
# --------------------------------------------------------------------------- #
_g_sim_cfg = None  # cached copy; module-level singleton lazy-loaded


def _load_sim_config():
    global _g_sim_cfg
    if _g_sim_cfg is not None:
        return _g_sim_cfg
    with open(SIM_CONFIG_YAML) as f:
        _g_sim_cfg = yaml.safe_load(f)
    return _g_sim_cfg


# --------------------------------------------------------------------------- #
# Debug slider state
# --------------------------------------------------------------------------- #


class DebugSliders:
    """PyBullet debug sliders for live physics tuning.

    Created by connect_and_setup when debug=True and gui=True.  Call
    .apply(robot_id, joint_map) each sim step to push current slider
    values into the four foot links.  On exit, .finalize() prints the
    final values as YAML for copy-paste back into sim_config.yaml.
    """

    def __init__(self, robot_id, joint_map, sim_cfg):
        self._robot_id = robot_id
        self._joint_map = joint_map

        defaults = {
            "lateralFriction": float(sim_cfg["friction"]["foot_lateral"]),
            "spinningFriction": float(sim_cfg["friction"]["foot_spinning"]),
            "massCorrection": float(sim_cfg["mass"]["mass_correction_factor"]),
            "contactStiffness": float(sim_cfg["contact"]["stiffness"]),
            "contactDamping": float(sim_cfg["contact"]["damping"]),
        }
        self._slider_ids = {
            key: p.addUserDebugParameter(
                key, low[key], high[key], defaults[key]
            )
            for key, low, high in [
                ("lateralFriction", 0.0, 5.0),
                ("spinningFriction", 0.0, 1.0),
                ("massCorrection", 0.5, 2.0),
                ("contactStiffness", 0.0, 100000.0),
                ("contactDamping", 0.0, 500.0),
            ]
        }
        self._foot_joint_indices = [idx for name, idx in joint_map.items() if "link3" in name]

    def apply(self):
        """Read current slider values and push to the four foot links."""
        lat = p.readUserDebugParameter(self._slider_ids["lateralFriction"])
        spin = p.readUserDebugParameter(self._slider_ids["spinningFriction"])
        stiff = p.readUserDebugParameter(self._slider_ids["contactStiffness"])
        damp = p.readUserDebugParameter(self._slider_ids["contactDamping"])
        mc = p.readUserDebugParameter(self._slider_ids["massCorrection"])
        for idx in self._foot_joint_indices:
            p.changeDynamics(
                self._robot_id,
                idx,
                lateralFriction=lat,
                spinningFriction=spin,
                restitution=0.0,
                contactStiffness=stiff,
                contactDamping=damp,
            )

    def finalize(self):
        """Print final slider values in YAML format."""
        vals = {
            key: p.readUserDebugParameter(sid)
            for key, sid in self._slider_ids.items()
        }
        lines = [
            "",
            "=== Debug slider final values (copy-paste into sim_config.yaml) ===",
            "contact:",
            f"  stiffness: {vals['contactStiffness']:.0f}",
            f"  damping: {vals['contactDamping']:.0f}",
            "  restitution: 0.0",
            "friction:",
            f"  foot_lateral: {vals['lateralFriction']:.3f}",
            f"  foot_spinning: {vals['spinningFriction']:.3f}",
            "mass:",
            f"  mass_correction_factor: {vals['massCorrection']:.3f}",
        ]
        # Print them one at a time so they're flush-left (PyBullet indents
        # stdout when printing alongside pybullet prints).
        for line in lines:
            print(line, flush=True)

    @property
    def sliders_active(self):
        return True


# --------------------------------------------------------------------------- #
# Public API
# --------------------------------------------------------------------------- #


def connect_and_setup(cfg, gui, float_mode=False, debug=False):
    """Connect PyBullet and load the plane + robot.

    float_mode=True: no gravity, no floor, and the body is pinned in the air
    (useFixedBase). Use it to watch a clip's pure joint geometry — each leg
    articulates exactly as authored, with no falling/slipping/collapse from
    physics. (Normal mode loads the ground plane, real gravity, and a free
    floating base so you see dynamic balance.)

    Returns (robot_id, joint_map, debug_sliders_or_None).
    """
    sim_cfg = _load_sim_config()
    contact = sim_cfg["contact"]
    friction = sim_cfg["friction"]
    solver = sim_cfg["solver"]

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
    p.setPhysicsEngineParameter(numSolverIterations=int(solver["iterations"]))
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

    # Foot contact (link3 = the knee joint's child = lower leg/foot).
    lat_fric = float(friction["foot_lateral"])
    spin_fric = float(friction["foot_spinning"])
    rest = float(contact["restitution"])
    stiff = float(contact["stiffness"])
    damp = float(contact["damping"])
    for name, idx in joint_map.items():
        if "link3" in name:
            p.changeDynamics(
                robot_id,
                idx,
                lateralFriction=lat_fric,
                spinningFriction=spin_fric,
                restitution=rest,
                contactStiffness=stiff,
                contactDamping=damp,
            )

    # ------------------------------------------------------------------- #
    # Mass audit: print per-link mass from PyBullet dynamics info.
    # ------------------------------------------------------------------- #
    _print_mass_audit(robot_id, joint_map, sim_cfg)

    # ------------------------------------------------------------------- #
    # Debug sliders (GUI only)
    # ------------------------------------------------------------------- #
    debug_sliders = None
    if debug:
        if gui:
            debug_sliders = DebugSliders(robot_id, joint_map, sim_cfg)
        else:
            print("[debug] --debug passed with headless mode; sliders skipped.")

    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.55,
            cameraYaw=45,
            cameraPitch=-25,
            cameraTargetPosition=[0, 0, 0.1],
        )
    return robot_id, joint_map, debug_sliders


def _print_mass_audit(robot_id, joint_map, sim_cfg):
    """Iterate all links via getDynamicsInfo and print a mass audit table."""
    mass = sim_cfg["mass"]
    mc_factor = float(mass.get("mass_correction_factor", 1.0))
    use_correction = abs(mc_factor - 1.0) > 1e-9

    total = 0.0
    total_corrected = 0.0
    lines = ["", "=== PyBullet mass audit ==="]
    header = "  {:<14s} {:>10s}".format("link", "mass (kg)")
    if use_correction:
        header += " {:>14s}".format("corrected (kg)")
    lines.append(header)

    # base_link is index -1 in getDynamicsInfo
    for idx in [-1] + sorted(joint_map.values()):
        if idx == -1:
            info = p.getDynamicsInfo(robot_id, -1)
            name = "base_link"
        else:
            info = p.getDynamicsInfo(robot_id, idx)
            num_joints = p.getNumJoints(robot_id)
            for jname, jidx in joint_map.items():
                if jidx == idx:
                    # Child link of this joint
                    jinfo = p.getJointInfo(robot_id, idx)
                    name = jinfo[12].decode() if jinfo[12] else f"link_{idx}"
                    break
            else:
                name = f"link_{idx}"

        m = info[0]
        total += m
        row = "  {:<14s} {:>10.3f}".format(name, m)
        if use_correction:
            mc = m * mc_factor
            total_corrected += mc
            row += " {:>14.3f}".format(mc)
        lines.append(row)

    row_total = "  {:<14s} {:>10.3f}".format("TOTAL", total)
    if use_correction:
        row_total += " {:>14.3f}".format(total_corrected)
    lines.append(row_total)
    if use_correction:
        lines.append("  (mass_correction_factor = {:.3f})".format(mc_factor))
    lines.append("===========================")
    for line in lines:
        print(line, flush=True)


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
