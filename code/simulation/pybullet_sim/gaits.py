"""Gait registry, foot trajectories, debug overlay, run_stand / run_gait."""

import math

import pybullet as p



# --------------------------------------------------------------------------- #
# Gait registry
# --------------------------------------------------------------------------- #

GAITS = {
    "walk": {
        "period": 2.4,
        "step_length": 0.04,
        "step_height": 0.02,
        "duty": 0.25,
        "offsets": {"fl": 0.00, "br": 0.25, "fr": 0.50, "bl": 0.75},
        "label": "Static walk",
    },
    "trot": {
        "period": 0.8,
        "step_length": 0.05,
        "step_height": 0.025,
        "duty": 0.5,
        "offsets": {"fl": 0.0, "br": 0.0, "fr": 0.5, "bl": 0.5},
        "label": "Trot (diagonal pairs)",
    },
}

_TRAJ_COLORS = {
    "fl": (1.0, 0.30, 0.30),
    "fr": (0.30, 1.0, 0.30),
    "bl": (0.30, 0.50, 1.0),
    "br": (1.0, 1.0, 0.30),
}


# --------------------------------------------------------------------------- #
# Foot trajectory + per-tick joint targets
# --------------------------------------------------------------------------- #


def foot_target(
    neutral_foot, leg_id, phase, step_length, step_height, duty, swing_axis="y"
):
    """Body-frame foot target. swing_axis selects which body axis steps forward.
    Body +Y is forward, so the default swing_axis="y" steps in the forward direction."""
    nx, ny, nz = neutral_foot[leg_id]
    if phase < duty:
        s = phase / duty
        d = -step_length * 0.5 + s * step_length
        dz = step_height * math.sin(math.pi * s)
    else:
        s = (phase - duty) / (1.0 - duty)
        d = step_length * 0.5 - s * step_length
        dz = 0.0
    if swing_axis == "y":
        return (nx, ny + d, nz + dz)
    return (nx + d, ny, nz + dz)


def gait_joint_targets(cfg, gait, t):
    offsets = gait["offsets"]
    global_phase = (t / gait["period"]) % 1.0
    targets = {}
    for leg_id, off in offsets.items():
        phase = (global_phase - off) % 1.0
        foot = foot_target(
            cfg.neutral_foot,
            leg_id,
            phase,
            gait["step_length"],
            gait["step_height"],
            gait["duty"],
        )
        s, h, k = cfg.leg_ik(cfg, foot, leg_id)
        targets[f"{leg_id}_link1_joint"] = s
        targets[f"{leg_id}_link2_joint"] = h
        targets[f"{leg_id}_link3_joint"] = k
    return targets


# --------------------------------------------------------------------------- #
# Debug overlay
# --------------------------------------------------------------------------- #

_TRAJ_SAMPLES = 40
_LINE_IDS: dict = {}
_MARK_IDS: dict = {}


def _body_to_world(pos_body, base_pos, base_orn):
    wp, _ = p.multiplyTransforms(base_pos, base_orn, pos_body, [0, 0, 0, 1])
    return wp


def _precompute_cycle(cfg, gait):
    offsets = gait["offsets"]
    cycles = {}
    for leg_id in offsets:
        pts = [
            foot_target(
                cfg.neutral_foot,
                leg_id,
                k / _TRAJ_SAMPLES,
                gait["step_length"],
                gait["step_height"],
                gait["duty"],
            )
            for k in range(_TRAJ_SAMPLES)
        ]
        pts.append(pts[0])
        cycles[leg_id] = pts
    return cycles


def _draw_overlay(robot_id, cycles, current_targets):
    base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
    for leg_id, pts in cycles.items():
        color = _TRAJ_COLORS.get(leg_id, (1, 1, 1))
        prev = _body_to_world(pts[0], base_pos, base_orn)
        for k in range(1, len(pts)):
            cur = _body_to_world(pts[k], base_pos, base_orn)
            key = (leg_id, k - 1)
            lid = _LINE_IDS.get(key)
            _LINE_IDS[key] = p.addUserDebugLine(
                prev,
                cur,
                lineColorRGB=color,
                lineWidth=1.5,
                replaceItemUniqueId=lid if lid is not None else -1,
            )
            prev = cur
        tw = _body_to_world(current_targets[leg_id], base_pos, base_orn)
        s = 0.012
        axes = [
            ((-s, 0, 0), (s, 0, 0)),
            ((0, -s, 0), (0, s, 0)),
            ((0, 0, -s), (0, 0, s)),
        ]
        for i, (a, b) in enumerate(axes):
            p0 = tuple(tw[j] + a[j] for j in range(3))
            p1 = tuple(tw[j] + b[j] for j in range(3))
            mkey = (leg_id, i)
            mid = _MARK_IDS.get(mkey)
            _MARK_IDS[mkey] = p.addUserDebugLine(
                p0,
                p1,
                lineColorRGB=color,
                lineWidth=3.0,
                replaceItemUniqueId=mid if mid is not None else -1,
            )


# --------------------------------------------------------------------------- #
# Simulation entry points
# --------------------------------------------------------------------------- #


def _body_height_for_gait(cfg, gait, samples_per_period=100):
    """Maximum foot-depth below body origin sampled over one gait period.
    Covers swing + stance feet across all 4 legs. Used in place of the
    neutral-stance depth so the spawn Z accommodates gait trajectories
    where foot-contact Z differs from neutral_foot."""
    offsets = gait["offsets"]
    worst = 0.0
    for k in range(samples_per_period):
        global_phase = k / samples_per_period
        for leg_id, off in offsets.items():
            phase = (global_phase - off) % 1.0
            foot = foot_target(
                cfg.neutral_foot,
                leg_id,
                phase,
                gait["step_length"],
                gait["step_height"],
                gait["duty"],
            )
            if -foot[2] > worst:
                worst = -foot[2]
    return max(1e-3, worst)


# backwards-compat re-exports — import from runner directly
