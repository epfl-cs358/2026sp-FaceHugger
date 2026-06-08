"""PyBullet joint-map plumbing and motor-control helpers."""

import pybullet as p


def _joint_type_from_name(name):
    # URDF joint names follow the Fusion-aligned `{leg_id}_link{1,2,3}_joint`
    # pattern; the role mapping (link1=shoulder, link2=hip, link3=knee)
    # lives here so callers can keep speaking in semantic stance keys.
    if "link1" in name:
        return "shoulder"
    if "link2" in name:
        return "hip"
    if "link3" in name:
        return "knee"
    return None


def build_joint_map(robot_id):
    out = {}
    n = p.getNumJoints(robot_id)
    for i in range(n):
        info = p.getJointInfo(robot_id, i)
        out[info[1].decode()] = i
    return out


def reset_to_stance(robot_id, joint_map, stance):
    # `stance` is per-leg: {leg_id: {shoulder/hip/knee: rad}}.
    for name, idx in joint_map.items():
        info = p.getJointInfo(robot_id, idx)
        if info[2] == p.JOINT_FIXED:
            continue
        jtype = _joint_type_from_name(name)
        leg_id = name[:2]
        if jtype is None or leg_id not in stance:
            continue
        p.resetJointState(robot_id, idx, stance[leg_id][jtype])


def apply_leg_pose(robot_id, joint_map, per_leg_stance, force, velocity, kp=None, kd=None):
    for name, idx in joint_map.items():
        info = p.getJointInfo(robot_id, idx)
        if info[2] == p.JOINT_FIXED:
            continue
        leg = name[:2]
        jtype = _joint_type_from_name(name)
        if jtype is None or leg not in per_leg_stance:
            continue
        kwargs = dict(
            bodyUniqueId=robot_id,
            jointIndex=idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=per_leg_stance[leg][jtype],
            force=force,
            maxVelocity=velocity,
        )
        if kp is not None:
            kwargs["positionGain"] = kp
        if kd is not None:
            kwargs["velocityGain"] = kd
        p.setJointMotorControl2(**kwargs)


def apply_joint_targets(robot_id, joint_map, targets, force, velocity, kp=None, kd=None):
    """targets = {joint_name: angle}"""
    for name, angle in targets.items():
        idx = joint_map.get(name)
        if idx is None:
            continue
        kwargs = dict(
            bodyUniqueId=robot_id,
            jointIndex=idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=angle,
            force=force,
            maxVelocity=velocity,
        )
        if kp is not None:
            kwargs["positionGain"] = kp
        if kd is not None:
            kwargs["velocityGain"] = kd
        p.setJointMotorControl2(**kwargs)
