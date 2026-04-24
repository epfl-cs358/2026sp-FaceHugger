"""Joint-map plumbing and math utilities shared across simulation modes."""

import math

import pybullet as p

from constants import SERVO_FORCE, SERVO_VELOCITY


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def build_joint_map(robot_id):
    """Return {joint_name: joint_index} and print a one-line summary."""
    joint_map = {}
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        joint_map[info[1].decode("utf-8")] = i
    print(f"FaceHugger loaded: {num_joints} joints")
    return joint_map


def stance_angle_for(joint_name, stance):
    """Pick the right stance angle (shoulder/hip/knee) for a joint name."""
    if "shoulder" in joint_name:
        return stance["shoulder"]
    if "hip" in joint_name:
        return stance["hip"]
    if "knee" in joint_name:
        return stance["knee"]
    return 0.0


def reset_to_stance(robot_id, joint_map, stance):
    """Hard-reset every revolute joint to the given stance."""
    for joint_name, joint_idx in joint_map.items():
        info = p.getJointInfo(robot_id, joint_idx)
        if info[2] == p.JOINT_FIXED:
            continue
        p.resetJointState(robot_id, joint_idx, stance_angle_for(joint_name, stance))


def apply_per_leg_pose(robot_id, joint_map, leg_targets,
                       force=SERVO_FORCE, velocity=SERVO_VELOCITY):
    """leg_targets = {leg_prefix: {shoulder/hip/knee: angle}}."""
    for joint_name, joint_idx in joint_map.items():
        info = p.getJointInfo(robot_id, joint_idx)
        if info[2] == p.JOINT_FIXED:
            continue
        leg = joint_name[:2]                # 'fl', 'fr', 'rl', 'rr'
        stance = leg_targets.get(leg)
        if stance is None:
            continue
        p.setJointMotorControl2(
            robot_id, joint_idx, p.POSITION_CONTROL,
            targetPosition=stance_angle_for(joint_name, stance),
            force=force, maxVelocity=velocity,
        )


def all_legs(stance):
    return {leg: stance for leg in ("fl", "fr", "rl", "rr")}
