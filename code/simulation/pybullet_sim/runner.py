"""Sim runner: run_stand, run_gait.

run_stand is shared by both the SIL and Python port paths.
run_gait is the Python IK gait (body-height reference).
run_clip_sil and run_gait_sil live in firmware_sil.sil_bridge.
"""

import time

import pybullet as p

from .paths import TIMESTEP
from .motor import apply_joint_targets
from .scene import connect_and_setup as _connect_and_setup
from .scene import print_banner as _print_banner
from .scene import settle as _settle
from .sim_monitor import finalize_log, setup_step_hook


def run_stand(cfg, gui=True, settle_s=0.5, float_mode=False, monitor=False, log=False, debug=False):
    robot_id, joint_map, debug_sliders = _connect_and_setup(cfg, gui, float_mode=float_mode, debug=debug)
    _print_banner(cfg)
    if float_mode:
        print("[float] no gravity/floor, body pinned — showing the stance pose")
    elif settle_s > 0:
        print(f"\n[settle] holding stance for {settle_s:.2f}s before idle loop")
        _settle(robot_id, joint_map, cfg, settle_s)
    print("\nStanding - Ctrl+C to exit.")
    on_step, logger = setup_step_hook(robot_id, joint_map, monitor, log)
    step = 0
    try:
        while p.isConnected():
            p.stepSimulation()
            if on_step is not None:
                on_step(step)
            if gui:
                time.sleep(TIMESTEP)
            step += 1
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()
        finalize_log(logger)


def run_gait(
    cfg, gait_name, gui=True, settle_s=0.5, float_mode=False, monitor=False, log=False, debug=False
):
    """Run a gait via the Python IK gait (also the body-height reference).
    For the EXACT compiled firmware instead, dispatch to
    firmware_sil.sil_bridge.run_gait_sil — simulate.py does this when
    --python-port is absent."""
    from .gaits import (
        GAITS,
        _body_height_for_gait,
        _draw_overlay,
        _precompute_cycle,
        gait_joint_targets,
    )

    if gait_name not in GAITS:
        raise ValueError(f"Unknown gait: {gait_name}")
    gait = GAITS[gait_name]

    # Gait-aware spawn height: worst foot Z across a full period, not just
    # neutral_foot. Prevents the body from sinking into the floor when a
    # gait's stance-phase Z differs from the neutral Z used at build time.
    gait_depth_m = _body_height_for_gait(cfg, gait)
    if gait_depth_m > cfg.body_height:
        print(
            f"[body_height] lifting spawn from {cfg.body_height * 1000:.1f} mm "
            f"to {gait_depth_m * 1000:.1f} mm for {gait_name} trajectory"
        )
        cfg.body_height = gait_depth_m

    robot_id, joint_map, debug_sliders = _connect_and_setup(cfg, gui, float_mode=float_mode, debug=debug)
    _print_banner(cfg)
    if float_mode:
        print("[float] no gravity/floor, body pinned")
    elif settle_s > 0:
        print(f"\n[settle] holding stance for {settle_s:.2f}s before gait")
        _settle(robot_id, joint_map, cfg, settle_s)
    print(
        f"\n{gait['label']}: period={gait['period']:.2f}s  "
        f"len={gait['step_length'] * 1000:.0f}mm  h={gait['step_height'] * 1000:.0f}mm  "
        f"duty={gait['duty']:.2f}"
    )

    draw_overlay = gui
    cycles = _precompute_cycle(cfg, gait) if draw_overlay else None
    draw_every = 4

    on_step, logger = setup_step_hook(robot_id, joint_map, monitor, log)
    t = 0.0
    step = 0
    try:
        while p.isConnected():
            if debug_sliders is not None:
                debug_sliders.apply()
            targets = gait_joint_targets(cfg, gait, t)
            apply_joint_targets(
                robot_id, joint_map, targets, cfg.servo_force, cfg.servo_velocity,
                kp=cfg.kp, kd=cfg.kd,
            )
            if draw_overlay and step % draw_every == 0:
                offsets = gait["offsets"]
                global_phase = (t / gait["period"]) % 1.0
                from .gaits import foot_target

                cur_targets = {
                    leg_id: foot_target(
                        cfg.neutral_foot,
                        leg_id,
                        (global_phase - off) % 1.0,
                        gait["step_length"],
                        gait["step_height"],
                        gait["duty"],
                    )
                    for leg_id, off in offsets.items()
                }
                _draw_overlay(robot_id, cycles, cur_targets)

            p.stepSimulation()
            if on_step is not None:
                on_step(step)
            if gui:
                time.sleep(TIMESTEP)
            t += TIMESTEP
            step += 1
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if debug_sliders is not None:
            debug_sliders.finalize()
        if p.isConnected():
            p.disconnect()
        finalize_log(logger)
