"""Terrain obstacle course: spawn, run one gait, run the gait-vs-terrain matrix.

The course is a fixed sequence of obstacles laid out along +Y, so every gait
is tested on the exact same hazard profile. Each checkpoint records how the
robot crossed it (upright / tilted / never-reached) to produce a
gait-vs-terrain survival matrix.

Obstacle sequence (dimensions in meters):
  UP_RAMP   : gentle incline, rises 2 cm over 20 cm
  PLATEAU   : 20 cm long flat top at 2 cm height
  DOWN_RAMP : mirror of UP_RAMP back to ground
  BUMPS     : three 6 mm tall strips at 10 cm spacing
  STEP_UP   : 1 cm tall, 25 cm long block
  FINISH    : thin green marker line
"""

import math
import time

import pybullet as p
import pybullet_data

from constants import STANCE, SERVO_FORCE, SERVO_VELOCITY, TIMESTEP, URDF_PATH
from helpers import (all_legs, apply_per_leg_pose, build_joint_map,
                     reset_to_stance, _clamp)
from kinematics import NEUTRAL_FOOT, foot_target, leg_ik
from gaits import GAITS


COURSE = [
    dict(name="UP_RAMP",   y=0.20, max_tilt=25.0),
    dict(name="PLATEAU",   y=0.45, max_tilt=20.0),
    dict(name="DOWN_RAMP", y=0.70, max_tilt=30.0),
    dict(name="BUMPS",     y=0.95, max_tilt=30.0),
    dict(name="STEP_UP",   y=1.40, max_tilt=45.0),
    dict(name="FINISH",    y=1.70, max_tilt=25.0),
]


def _add_static_box(half, pos, orn=(0, 0, 0, 1), color=(0.6, 0.6, 0.6, 1.0),
                    friction=1.2):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=list(color))
    body = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=list(pos),
        baseOrientation=list(orn),
    )
    p.changeDynamics(body, -1, lateralFriction=friction, restitution=0.0)
    return body


def add_ramp(y_start, length, rise, width=0.60, friction=1.3,
             color=(0.35, 0.65, 0.90, 1.0)):
    """Thin slab tilted about X so its top surface rises `rise` meters
    between y=y_start and y=y_start+length."""
    angle = math.atan2(rise, length)
    hypot = math.hypot(rise, length)
    half_thickness = 0.02
    half = [width / 2.0, hypot / 2.0, half_thickness]
    cy = y_start + length / 2.0
    cz = abs(rise) / 2.0 - half_thickness * math.cos(angle)
    orn = p.getQuaternionFromEuler([angle, 0.0, 0.0])
    return _add_static_box(half, (0.0, cy, cz), orn, color, friction)


def add_box_obstacle(y_center, height, length, width=0.60, friction=1.3,
                     color=(0.9, 0.55, 0.25, 1.0)):
    """Axis-aligned box sitting on the floor with its top face at z=height."""
    half = [width / 2.0, length / 2.0, height / 2.0]
    return _add_static_box(half, (0.0, y_center, height / 2.0),
                           (0, 0, 0, 1), color, friction)


def build_terrain_course():
    """Spawn the full obstacle course. Returns list of body ids."""
    ids = []

    ramp_color = (0.35, 0.65, 0.90, 1.0)
    ids.append(add_ramp(y_start=0.10, length=0.20, rise=0.02, color=ramp_color))
    ids.append(add_box_obstacle(y_center=0.40, height=0.02, length=0.20,
                                color=ramp_color))
    ids.append(add_ramp(y_start=0.50, length=0.20, rise=-0.02, color=ramp_color))

    bump_color = (0.95, 0.80, 0.20, 1.0)
    for y in (0.85, 0.95, 1.05):
        ids.append(add_box_obstacle(y_center=y, height=0.006, length=0.010,
                                    color=bump_color))

    ids.append(add_box_obstacle(y_center=1.35, height=0.010, length=0.25,
                                color=(0.85, 0.35, 0.30, 1.0)))

    ids.append(add_box_obstacle(y_center=1.75, height=0.005, length=0.02,
                                width=0.90, color=(0.25, 0.85, 0.35, 1.0)))
    return ids


def _sample_state(robot_id):
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    return pos, (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def _tilt(rpy):
    return max(abs(rpy[0]), abs(rpy[1]))


def run_terrain(gait_name, gui=True, duration=40.0, return_report=False):
    """Run a named gait across the terrain course, logging per-checkpoint
    crossing data. Returns a summary dict when return_report=True."""
    if gait_name not in GAITS:
        raise ValueError(f"Unknown gait '{gait_name}'. Known: {sorted(GAITS)}")
    cfg = GAITS[gait_name]

    p.connect(p.GUI if gui else p.DIRECT)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(TIMESTEP)
    p.loadURDF("plane.urdf")

    build_terrain_course()

    body_height = -NEUTRAL_FOOT["fl"][2]
    robot_id = p.loadURDF(
        URDF_PATH,
        basePosition=[0.0, -0.10, body_height + 0.005],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False,
    )
    joint_map = build_joint_map(robot_id)
    reset_to_stance(robot_id, joint_map, STANCE)
    apply_per_leg_pose(robot_id, joint_map, all_legs(STANCE))

    for joint_name, joint_idx in joint_map.items():
        if "knee" in joint_name:
            p.changeDynamics(robot_id, joint_idx,
                             lateralFriction=1.2, restitution=0.1)

    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.85, cameraYaw=75, cameraPitch=-15,
            cameraTargetPosition=[0.0, 0.0, 0.10],
        )

    print(f"\n=== Terrain run: {cfg['label']} ===")
    print(f"  course: " + " -> ".join(cp['name'] for cp in COURSE))

    reached = {cp["name"]: None for cp in COURSE}
    min_z, max_tilt_seen = 1e9, 0.0
    fell, fell_at = False, None
    low_z_streak = 0
    streak_threshold = int(0.25 / TIMESTEP)

    STEER_KP = 0.4
    STEER_MAX = 0.25
    steer_cfg_left = dict(cfg)
    steer_cfg_right = dict(cfg)

    t, steps = 0.0, int(duration / TIMESTEP)
    for i in range(steps):
        if not p.isConnected():
            print("  [window closed, exiting]")
            return
        _, base_orn = p.getBasePositionAndOrientation(robot_id)
        _, _, yaw_rad = p.getEulerFromQuaternion(base_orn)
        steer = _clamp(STEER_KP * yaw_rad, -STEER_MAX, STEER_MAX)
        steer_cfg_left["step_length"]  = cfg["step_length"] * (1.0 + steer)
        steer_cfg_right["step_length"] = cfg["step_length"] * (1.0 - steer)
        targets = {}
        period = cfg["period"]
        global_phase = (t / period) % 1.0
        for leg_id, offset in cfg["offsets"].items():
            phase = (global_phase - offset) % 1.0
            side_cfg = steer_cfg_left if leg_id.endswith("l") else steer_cfg_right
            foot = foot_target(leg_id, phase, side_cfg["step_length"],
                               cfg["step_height"], cfg["duty"])
            theta_s, theta_h, theta_k = leg_ik(foot, leg_id)
            targets[f"{leg_id}_shoulder_joint"] = theta_s
            targets[f"{leg_id}_hip_joint"]      = theta_h
            targets[f"{leg_id}_knee_joint"]     = theta_k
        for joint_name, angle in targets.items():
            idx = joint_map.get(joint_name)
            if idx is None:
                continue
            p.setJointMotorControl2(
                robot_id, idx, p.POSITION_CONTROL,
                targetPosition=angle,
                force=SERVO_FORCE, maxVelocity=SERVO_VELOCITY,
            )
        p.stepSimulation()

        pos, rpy = _sample_state(robot_id)
        min_z = min(min_z, pos[2])
        max_tilt_seen = max(max_tilt_seen, _tilt(rpy))

        if gui:
            if i % 30 == 0:
                p.resetDebugVisualizerCamera(
                    cameraDistance=0.95, cameraYaw=75, cameraPitch=-14,
                    cameraTargetPosition=[0.0, pos[1], 0.10],
                )
            time.sleep(TIMESTEP)
        t += TIMESTEP

        if abs(rpy[0]) > 100.0 or abs(rpy[1]) > 80.0:
            fell = True
        if pos[2] < 0.045:
            low_z_streak += 1
            if low_z_streak > streak_threshold:
                fell = True
        else:
            low_z_streak = 0

        if fell:
            for cp in COURSE:
                if reached[cp["name"]] is None:
                    fell_at = cp["name"]
                    break
            print(f"  [t={t:5.2f}s] FELL before {fell_at} "
                  f"(y={pos[1]:+.2f} z={pos[2]:+.3f} "
                  f"roll={rpy[0]:+.0f} pitch={rpy[1]:+.0f})")
            break

        for cp in COURSE:
            if reached[cp["name"]] is None and pos[1] >= cp["y"]:
                reached[cp["name"]] = dict(t=t, pos=tuple(pos), rpy=rpy)
                tilt = _tilt(rpy)
                tag = "clean" if tilt < cp["max_tilt"] else f"tilt {tilt:.0f} deg"
                print(f"  [t={t:5.2f}s] {cp['name']:10s} y={pos[1]:+.2f} "
                      f"z={pos[2]:+.3f} roll={rpy[0]:+5.0f} "
                      f"pitch={rpy[1]:+5.0f}  [{tag}]")

        if not gui and all(v is not None for v in reached.values()):
            break

    final_pos, final_rpy = _sample_state(robot_id)
    summary = dict(
        gait=gait_name,
        reached=reached,
        fell=fell,
        fell_at=fell_at,
        final_pos=tuple(final_pos),
        final_rpy=final_rpy,
        min_z=min_z,
        max_tilt=max_tilt_seen,
        duration=t,
    )

    print(f"\n  RESULT: {'FELL at ' + str(fell_at) if fell else 'SURVIVED'} "
          f"| final y={final_pos[1]:+.2f}  min z={min_z:+.3f}  "
          f"max tilt={max_tilt_seen:.0f} deg  t={t:.2f}s")

    if gui and not fell:
        hold = int(2.0 / TIMESTEP)
        for _ in range(hold):
            if not p.isConnected():
                break
            p.stepSimulation()
            time.sleep(TIMESTEP)

    if p.isConnected():
        p.disconnect()

    return summary if return_report else None


def _status(summary, spawn_y=-0.10):
    """Classify the final outcome of a terrain run."""
    reached_count = sum(1 for v in summary["reached"].values() if v is not None)
    all_reached = reached_count == len(summary["reached"])
    if summary["fell"]:
        return f"FELL @ {summary['fell_at']}"
    if all_reached:
        return "FINISHED"
    if summary["final_pos"][1] < spawn_y - 0.15:
        return "REVERSED"
    if reached_count == 0:
        return "STUCK"
    return "TIMEOUT"


def run_terrain_matrix(gui=False, save_path=None):
    """Run all gaits over the course; print (and optionally save) a markdown
    survival table."""
    durations = {"walk": 60.0, "trot": 40.0, "bound": 40.0}
    results = []
    for gait in ("walk", "trot", "bound"):
        summary = run_terrain(gait, gui=gui, duration=durations[gait],
                              return_report=True)
        results.append(summary)

    header = ["Gait"] + [cp["name"] for cp in COURSE] + [
        "Status", "Reached y (m)", "Min z (m)", "Max tilt", "Time (s)"
    ]

    def cell(summary, cp):
        rec = summary["reached"][cp["name"]]
        if rec is None:
            return "-"
        tilt = _tilt(rec["rpy"])
        if tilt >= cp["max_tilt"]:
            return f"! {tilt:.0f}deg"
        return "ok"

    rows = []
    for r in results:
        row = [r["gait"]]
        row.extend(cell(r, cp) for cp in COURSE)
        row.append(_status(r))
        row.append(f"{r['final_pos'][1]:+.2f}")
        row.append(f"{r['min_z']:+.3f}")
        row.append(f"{r['max_tilt']:.0f}deg")
        row.append(f"{r['duration']:.1f}")
        rows.append(row)

    widths = [max(len(str(header[i])),
                  max(len(str(row[i])) for row in rows)) for i in range(len(header))]

    def fmt(row):
        return "| " + " | ".join(str(c).ljust(widths[i])
                                 for i, c in enumerate(row)) + " |"

    lines = []
    lines.append("# FaceHugger Terrain Survival Matrix")
    lines.append("")
    lines.append("Course (spawn at y = -0.10, all obstacles along +Y):")
    for cp in COURSE:
        lines.append(f"- **{cp['name']}** at y = {cp['y']:.2f} m "
                     f"(tilt budget {cp['max_tilt']:.0f} deg)")
    lines.append("")
    lines.append(fmt(header))
    lines.append("|" + "|".join("-" * (w + 2) for w in widths) + "|")
    for row in rows:
        lines.append(fmt(row))
    lines.append("")
    lines.append("**Legend**")
    lines.append("- `ok`       -- checkpoint crossed within tilt budget")
    lines.append("- `! Xdeg`   -- crossed but body tilt X deg exceeded budget")
    lines.append("- `-`        -- never reached (timeout, fall, or wrong direction)")
    lines.append("- `FINISHED` -- reached every checkpoint upright")
    lines.append("- `TIMEOUT`  -- ran out of sim time before finish")
    lines.append("- `REVERSED` -- gait pushed the robot backward (-Y)")
    lines.append("- `STUCK`    -- never left the spawn neighbourhood")
    lines.append("- `FELL @ X` -- body tipped or dragged before crossing X")

    report = "\n".join(lines)
    print("\n\n" + report)

    if save_path:
        with open(save_path, "w") as f:
            f.write(report + "\n")
        print(f"\nReport written to {save_path}")

    return results
