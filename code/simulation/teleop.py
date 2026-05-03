# Not yet integrated with the URDF pipeline — see SIM_PIPELINE.md.
# Inherited from origin/main during the feat/urdf-pipeline merge; uses
# theirs' module-state kinematics (LEG_INFO, NEUTRAL_FOOT, leg_ik) which
# is incompatible with ours' cfg-driven build_config(). Do not import
# from this file in the URDF pipeline until it's been ported to use
# RobotConfig + cfg.leg_ik / cfg.leg_fk.
"""Keyboard-driven teleop on flat ground."""

import math
import os
import time

import pybullet as p
import pybullet_data

from constants import STANCE, SERVO_FORCE, SERVO_VELOCITY, TIMESTEP, URDF_PATH
from helpers import (
    all_legs,
    apply_per_leg_pose,
    build_joint_map,
    reset_to_stance,
    _clamp,
    _wrap_pi,
)
from kinematics import NEUTRAL_FOOT, foot_target, splayed_foot_ik
from gaits import GAITS, pre_orient_splay


def run_teleop(gait_name="trot", gui=True, duration=600.0):
    """Keyboard-driven teleop on flat ground.

    Controls (PyBullet debug window must have focus, AZERTY layout):
      Z/S      forward / reverse throttle (held)
      Q/D      turn left / right (updates heading setpoint)
      SPACE    cycle gait: walk -> trot -> bound
      R        reset body to stance (teleport)
      ESC      quit
    """
    if gait_name not in GAITS:
        raise ValueError(f"Unknown gait '{gait_name}'. Known: {sorted(GAITS)}")

    p.connect(p.GUI if gui else p.DIRECT)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(TIMESTEP)
    p.loadURDF("plane.urdf")

    body_height = -NEUTRAL_FOOT["fl"][2]
    spawn_pos = [0.0, 0.0, body_height + 0.005]
    robot_id = p.loadURDF(
        URDF_PATH,
        basePosition=spawn_pos,
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False,
    )
    joint_map = build_joint_map(robot_id)
    reset_to_stance(robot_id, joint_map, STANCE)
    apply_per_leg_pose(robot_id, joint_map, all_legs(STANCE))

    for joint_name, joint_idx in joint_map.items():
        if "knee" in joint_name:
            p.changeDynamics(robot_id, joint_idx, lateralFriction=1.2, restitution=0.1)

    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.85,
            cameraYaw=50,
            cameraPitch=-25,
            cameraTargetPosition=[0.0, 0.0, 0.10],
        )

    pre_orient_splay(robot_id, joint_map, GAITS[gait_name], gui=gui)

    print(f"\n=== Teleop: {GAITS[gait_name]['label']} ===")
    print("  forward: Z / W / UP      backward: S / DOWN")
    print("  left:    Q / A / LEFT    right:    D / RIGHT")
    print("  SHIFT + fwd/back = sprint  |  SPACE cycle gait  |  R reset  |  ESC quit")
    print("  (raw key codes print on each press; export TELEOP_QUIET=1 to hide)")

    gait_cycle = ["walk", "trot", "bound"]
    gait_idx = gait_cycle.index(gait_name) if gait_name in gait_cycle else 1

    STEER_KP = 1.2
    STEER_MAX = 0.6
    TURN_RATE = 1.4
    SPEED_MULT = 1.6
    SPRINT_MULT = 2.6
    PERIOD_SPRINT_SCALE = 0.75
    desired_yaw = 0.0

    hud_id = -1
    steer_cfg_left, steer_cfg_right = {}, {}

    t, steps = 0.0, int(duration / TIMESTEP)
    last_hud_update = -1

    for i in range(steps):
        if not p.isConnected():
            print("  [window closed, exiting]")
            return

        keys = p.getKeyboardEvents()

        if keys and not os.environ.get("TELEOP_QUIET"):
            for kc, flag in keys.items():
                if flag & p.KEY_WAS_TRIGGERED:
                    ch = chr(kc) if 32 <= kc < 127 else "?"
                    print(f"  [key] code={kc} char='{ch}'")

        def _down(*codes):
            return any(c in keys and (keys[c] & p.KEY_IS_DOWN) for c in codes)

        def _triggered(*codes):
            return any(c in keys and (keys[c] & p.KEY_WAS_TRIGGERED) for c in codes)

        KEY_FWD = (ord("z"), ord("Z"), ord("w"), ord("W"), p.B3G_UP_ARROW)
        KEY_BACK = (ord("s"), ord("S"), p.B3G_DOWN_ARROW)
        KEY_LEFT = (ord("q"), ord("Q"), ord("a"), ord("A"), p.B3G_LEFT_ARROW)
        KEY_RIGHT = (ord("d"), ord("D"), p.B3G_RIGHT_ARROW)
        KEY_GAIT = (ord(" "), p.B3G_SPACE)
        KEY_RESET = (ord("r"), ord("R"))
        KEY_QUIT = (27,)
        KEY_SPRINT = (p.B3G_SHIFT,)

        if _down(*KEY_QUIT):
            print("  [quit]")
            break

        if _triggered(*KEY_GAIT):
            gait_idx = (gait_idx + 1) % len(gait_cycle)
            gait_name = gait_cycle[gait_idx]
            print(f"  [gait] -> {gait_name}")

        if _triggered(*KEY_RESET):
            p.resetBasePositionAndOrientation(
                robot_id,
                spawn_pos,
                p.getQuaternionFromEuler([0, 0, 0]),
            )
            p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0])
            reset_to_stance(robot_id, joint_map, STANCE)
            desired_yaw = 0.0
            print("  [reset]")

        sprinting = _down(*KEY_SPRINT)
        speed_scale = SPRINT_MULT if sprinting else SPEED_MULT
        throttle = 0.0
        if _down(*KEY_FWD):
            throttle += speed_scale
        if _down(*KEY_BACK):
            throttle -= speed_scale

        if _down(*KEY_LEFT):
            desired_yaw += TURN_RATE * TIMESTEP
        if _down(*KEY_RIGHT):
            desired_yaw -= TURN_RATE * TIMESTEP

        base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
        _, _, yaw_rad = p.getEulerFromQuaternion(base_orn)
        yaw_err = _wrap_pi(desired_yaw - yaw_rad)
        steer = _clamp(STEER_KP * yaw_err, -STEER_MAX, STEER_MAX)

        cfg = GAITS[gait_name]
        base_stride = cfg["step_length"] * throttle
        steer_cfg_left = dict(cfg)
        steer_cfg_right = dict(cfg)
        steer_cfg_left["step_length"] = base_stride * (1.0 - steer)
        steer_cfg_right["step_length"] = base_stride * (1.0 + steer)

        period = cfg["period"] * (PERIOD_SPRINT_SCALE if sprinting else 1.0)
        global_phase = (t / period) % 1.0
        targets = {}
        for leg_id, offset in cfg["offsets"].items():
            phase = (global_phase - offset) % 1.0
            side_cfg = steer_cfg_left if leg_id.endswith("l") else steer_cfg_right
            foot = foot_target(
                leg_id, phase, side_cfg["step_length"], cfg["step_height"], cfg["duty"]
            )
            theta_s, theta_h, theta_k = splayed_foot_ik(
                foot,
                leg_id,
                cfg.get("shoulder_splay", 0.0),
                cfg.get("splay_signs", {}),
            )
            targets[f"{leg_id}_shoulder_joint"] = theta_s
            targets[f"{leg_id}_hip_joint"] = theta_h
            targets[f"{leg_id}_knee_joint"] = theta_k

        for joint_name, angle in targets.items():
            idx = joint_map.get(joint_name)
            if idx is None:
                continue
            p.setJointMotorControl2(
                robot_id,
                idx,
                p.POSITION_CONTROL,
                targetPosition=angle,
                force=SERVO_FORCE,
                maxVelocity=SERVO_VELOCITY,
            )

        p.stepSimulation()

        if gui:
            if i % 10 == 0:
                cam_yaw_deg = math.degrees(yaw_rad) + 90.0
                p.resetDebugVisualizerCamera(
                    cameraDistance=0.85,
                    cameraYaw=cam_yaw_deg,
                    cameraPitch=-20,
                    cameraTargetPosition=[base_pos[0], base_pos[1], 0.10],
                )

            if i - last_hud_update >= 20:
                mode = "SPRINT" if sprinting else "cruise"
                hud = f"gait:{gait_name}  mode:{mode}"
                hud_kwargs = dict(
                    textPosition=[base_pos[0], base_pos[1], 0.25],
                    textColorRGB=[0.1, 0.9, 0.1],
                    textSize=1.2,
                )
                if hud_id == -1:
                    hud_id = p.addUserDebugText(hud, **hud_kwargs)
                else:
                    hud_id = p.addUserDebugText(
                        hud, replaceItemUniqueId=hud_id, **hud_kwargs
                    )
                last_hud_update = i

            time.sleep(TIMESTEP)

        t += TIMESTEP

    if p.isConnected():
        p.disconnect()
