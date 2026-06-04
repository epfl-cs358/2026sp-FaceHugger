"""Python-port run loops: run_stand, run_clip, run_gait.

These are the no-C++-toolchain code paths driven from `simulate.py --python-port`.
The matching SIL run loops (`run_clip_sil`, `run_gait_sil`) live in
`firmware_sil.sil_bridge` — moved there in Phase 4 of the sim-reorg to
break the bidirectional pybullet_sim ↔ firmware_sil import edge. Dispatch
between the two ports is now `simulate.py`'s job.
"""

import time

import pybullet as p

from .paths import TIMESTEP
from .motor import apply_joint_targets
from .scene import connect_and_setup as _connect_and_setup
from .scene import print_banner as _print_banner
from .scene import settle as _settle
from .sim_monitor import finalize_log, setup_step_hook


def run_stand(cfg, gui=True, settle_s=0.5, float_mode=False, monitor=False, log=False):
    robot_id, joint_map = _connect_and_setup(cfg, gui, float_mode=float_mode)
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


def run_clip(
    cfg,
    clip_name,
    gui=True,
    settle_s=0.5,
    loop=False,
    float_mode=False,
    monitor=False,
    log=False,
):
    """Load and play an animation clip in PyBullet via the Python re-port
    (firmware_port.ClipPlayer). No C++ toolchain required.

    The re-port reads animation/exported_clips/clips_all.h. loop=True (GUI)
    replays continuously; float_mode pins the body; monitor/log add the
    torque/current readout + capture. For the EXACT compiled firmware
    instead, dispatch to firmware_sil.sil_bridge.run_clip_sil (simulate.py
    does this when --python-port is absent).
    """
    from firmware_port.clip_loader import (
        DEFAULT_CLIPS_H,
        get_clip_by_name,
        load_clips_all_h,
    )
    from firmware_port.clip_player import ClipPlayer

    clips = load_clips_all_h(DEFAULT_CLIPS_H)
    clip = get_clip_by_name(clips, clip_name)

    robot_id, joint_map = _connect_and_setup(cfg, gui, float_mode=float_mode)
    _print_banner(cfg)
    if float_mode:
        print("[float] no gravity/floor, body pinned — showing joint geometry")
    elif settle_s > 0:
        print(f"\n[settle] holding stance for {settle_s:.2f}s before clip")
        _settle(robot_id, joint_map, cfg, settle_s)

    loop_note = " (looping)" if loop and gui else ""
    mon_note = "  [monitor: torque/current]" if monitor else ""
    print(
        f"\n[clip] playing '{clip.name}' ({clip.duration_ms} ms){loop_note}{mon_note}"
    )
    on_step, logger = setup_step_hook(robot_id, joint_map, monitor, log)
    player = ClipPlayer(robot_id, joint_map, clip, cfg.servo_force, cfg.servo_velocity)
    try:
        player.play_blocking(gui=gui, loop=loop, on_step=on_step)
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()
        finalize_log(logger)



