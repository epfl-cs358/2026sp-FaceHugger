"""Torque-based link coloring and per-step telemetry frame for the control panel.

Joint order the panel renders: FL, FR, BL, BR x shoulder, hip, knee. Each joint
carries BOTH the firmware servo-space command (0-180, what the real servos get)
and that same command in URDF-joint degrees, so the delta vs the measured joint
angle is a true tracking error rather than a constant ~90 offset between spaces.
"""

from animation_tools.servo_convention import LEG_ID_TO_SIM_NAME

_BAND_RGBA = {
    "green": (0.2, 0.8, 0.2, 1.0),  # safe continuous hold
    "yellow": (0.9, 0.7, 0.1, 1.0),  # burst-only: over continuous, under stall
    "red": (0.9, 0.2, 0.1, 1.0),  # saturated: at/over stall, can't track
}

_TELEM_LEGS = ("fl", "fr", "bl", "br")
_TELEM_JOINTS = (("sh", 1), ("hip", 2), ("knee", 3))  # label -> URDF link number
_SIM_NAME_TO_LEG_ID = {v: k for k, v in LEG_ID_TO_SIM_NAME.items()}


def torque_color(torque_nm):
    """rgba for a joint's applied torque, via the shared sim_monitor.band() — so the
    link colors mean the same thing as the --monitor [CONT]/[STALL] flags. Brief
    fast-clip spikes show yellow (honest burst); red is reserved for true saturation
    at the effort cap, not merely 'fast'."""
    from pybullet_sim import sim_monitor

    return _BAND_RGBA[sim_monitor.band(torque_nm)]


def apply_torque_colors(p, robot_id, joint_map):
    """Tint each leg link by its current applied torque (call every ~12 steps)."""
    for name, idx in joint_map.items():
        if idx < 0:
            continue
        torque = p.getJointState(robot_id, idx)[3]
        p.changeVisualShape(robot_id, idx, rgbaColor=torque_color(torque))


def build_telemetry_frame(fc, p, robot_id, joint_map, t_s, oor=None):
    """One telemetry frame: {t, joints:[12 x {servo/joint cmd, actual, delta, ...}]}.

    `oor` is the {pca_channel: requested_deg} from parse_oor(fc.drain_serial()) for
    this tick (the firmware's pre-clamp report); pre_clamp_deg is null where a servo
    had no [OOR] line. Reads firmware servo angles + channels from `fc`, measured
    joint state from `p` (any object exposing getJointState).
    """
    from math import degrees

    from firmware_sil.sil_bridge import servo_angles_to_joint_targets
    from pybullet_sim import sim_monitor

    oor = oor or {}
    angles = fc.servo_angles()
    targets = servo_angles_to_joint_targets(angles)  # urdf joint name -> radians
    channels = list(fc.servo_channels())

    joints = []
    for leg in _TELEM_LEGS:
        leg_id = _SIM_NAME_TO_LEG_ID[leg]
        for label, link in _TELEM_JOINTS:
            servo_idx = leg_id * 3 + (link - 1)
            urdf = f"{leg}_link{link}_joint"
            commanded_servo = float(angles[servo_idx])
            commanded_joint = degrees(targets[urdf])
            idx = joint_map.get(urdf)
            if idx is not None and idx >= 0:
                state = p.getJointState(robot_id, idx)
                actual_joint = degrees(state[0])
                torque = state[3]
                delta = commanded_joint - actual_joint
            else:
                actual_joint = delta = None
                torque = 0.0
            joints.append(
                {
                    "name": f"{leg}_{label}",
                    "commanded_servo_deg": round(commanded_servo, 2),
                    "commanded_joint_deg": round(commanded_joint, 2),
                    "actual_joint_deg": None
                    if actual_joint is None
                    else round(actual_joint, 2),
                    "delta_deg": None if delta is None else round(delta, 2),
                    "torque_nm": round(torque, 4),
                    "current_a": round(sim_monitor.estimate_current_a(torque), 3),
                    "pre_clamp_deg": oor.get(channels[servo_idx]),
                }
            )
    return {"t": round(t_s, 4), "joints": joints}
