"""Torque + estimated-current monitoring for the PyBullet sim (behind --monitor).

The position controller reports the torque it applied at each joint
(getJointState()[3]). From that we estimate per-servo current and the total
draw, and flag servos that are stalling or a total that exceeds the supply
budget — so you can see *why* a clip collapses (which joints saturate) without
hardware.

The pure helpers (estimate_current_a, total_current_a, format_status) take plain
numbers/dicts and are unit-tested without PyBullet; read_joint_torques /
read_joint_pos_deg do the PyBullet reads.
"""

import math

STALL_TORQUE_NM = 2.94  # 30 kg·cm servo stall torque (matches servo.effort_nm)
STALL_CURRENT_A = 2.5  # approximate stall current at 6 V
CURRENT_LIMIT_A = 10.0  # supply / multiplexer budget — warn above this total
STALL_WARN_NM = 2.5  # flag a joint whose applied torque exceeds this

_LEGS = ("fr", "fl", "br", "bl")


def estimate_current_a(torque_nm: float) -> float:
    """Estimated servo current (A) for an applied torque, linear from the
    stall point: |torque| / (stall_torque / stall_current)."""
    return abs(torque_nm) * (STALL_CURRENT_A / STALL_TORQUE_NM)


def total_current_a(torques) -> float:
    """Sum of estimated current over an iterable of joint torques (N·m)."""
    return sum(estimate_current_a(t) for t in torques)


def format_status(elapsed_s: float, joint_torques: dict, joint_pos_deg=None) -> str:
    """Compact one-line status.

    joint_torques: {joint_name: torque_nm} for the 12 joints.
    joint_pos_deg: optional {joint_name: degrees} to include per-leg angles.

    Example:
      t= 2.3s | FR:sh=+1 th=-41 kn=-57 | ... | peak_τ=1.84N·m(fr_link2_joint)
      | est_I=6.2A | [STALL] fr_link2_joint
    Adds " [WARN >10A]" when the total estimated current exceeds the budget.
    """
    total_i = total_current_a(joint_torques.values())
    peak_name, peak_tau = max(
        joint_torques.items(), key=lambda kv: abs(kv[1]), default=("-", 0.0)
    )
    stalls = [n for n, t in joint_torques.items() if abs(t) > STALL_WARN_NM]

    parts = [f"t={elapsed_s:5.1f}s"]
    if joint_pos_deg:
        for leg in _LEGS:
            sh = joint_pos_deg.get(f"{leg}_link1_joint")
            th = joint_pos_deg.get(f"{leg}_link2_joint")
            kn = joint_pos_deg.get(f"{leg}_link3_joint")
            if sh is not None:
                parts.append(f"{leg.upper()}:sh={sh:+.0f} th={th:+.0f} kn={kn:+.0f}")
    parts.append(f"peak_τ={abs(peak_tau):.2f}N·m({peak_name})")
    warn = " [WARN >10A]" if total_i > CURRENT_LIMIT_A else ""
    parts.append(f"est_I={total_i:.1f}A{warn}")
    if stalls:
        parts.append("[STALL] " + ",".join(stalls))
    return " | ".join(parts)


def read_joint_torques(robot_id, joint_map) -> dict:
    """{joint_name: applied torque (N·m)} from getJointState()[3]."""
    import pybullet as p

    return {name: p.getJointState(robot_id, idx)[3] for name, idx in joint_map.items()}


def read_joint_pos_deg(robot_id, joint_map) -> dict:
    """{joint_name: current joint angle in degrees} from getJointState()[0]."""
    import pybullet as p

    return {
        name: math.degrees(p.getJointState(robot_id, idx)[0])
        for name, idx in joint_map.items()
    }
