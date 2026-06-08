"""Sim runner: run_stand.

All clip and gait playback is now handled by firmware_sil.sil_bridge
(run_clip_sil / run_gait_sil). run_stand remains for idle / settle testing.
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

