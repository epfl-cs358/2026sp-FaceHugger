"""Simulation run loops: run_stand, run_gait, run_clip and their SIL variants."""

import time

import pybullet as p

from .constants import TIMESTEP
from .motor import apply_joint_targets
from .scene import connect_and_setup as _connect_and_setup
from .scene import print_banner as _print_banner
from .scene import settle as _settle


# --------------------------------------------------------------------------- #
# Step monitor + logger helpers
# --------------------------------------------------------------------------- #


def _make_step_monitor(robot_id, joint_map, every=30, logger=None):
    """Return an on_step(i) callback for the sim loops.

    --monitor: prints a torque + estimated-current status line every `every`
    sim steps (~8 Hz at 240 Hz) — per-leg angles, peak joint torque, total
    estimated current ([WARN >10A]) and any stalling joint ([STALL]).

    --log: if `logger` (a sim_monitor.SimLogger) is given, records every step
    (not just every `every`) for the end-of-run summary/CSV/plot. Reads torques
    once per step and shares them with the periodic print. See sim_monitor.py.
    """
    from . import sim_monitor

    def on_step(i):
        do_print = i % every == 0
        if logger is None and not do_print:
            return
        torques = sim_monitor.read_joint_torques(robot_id, joint_map)
        if logger is not None:
            logger.record(i * TIMESTEP, torques)
        if do_print:
            pos = sim_monitor.read_joint_pos_deg(robot_id, joint_map)
            print(sim_monitor.format_status(i * TIMESTEP, torques, pos))

    return on_step


def setup_step_hook(robot_id, joint_map, monitor, log):
    """Build the (on_step, logger) pair shared by the run_* loops.

    Returns (None, None) when neither --monitor nor --log is set. When --log,
    the SimLogger is returned too so the caller can _finalize_log() it after
    the loop. See _make_step_monitor / sim_monitor.SimLogger.
    """
    if not (monitor or log):
        return None, None
    from . import sim_monitor

    logger = sim_monitor.SimLogger(list(joint_map.keys())) if log else None
    on_step = _make_step_monitor(robot_id, joint_map, logger=logger)
    return on_step, logger


def _finalize_log(logger):
    """End-of-run output for --log: summary table, CSV, and 3-panel plot.

    Called from each run_* finally block so it runs even on Ctrl+C / p.error.
    No-op when logging is disabled.
    """
    if logger is None:
        return
    logger.summary()
    logger.save_csv("sim_log.csv")
    logger.plot("sim_log.png")


# --------------------------------------------------------------------------- #
# Run loops
# --------------------------------------------------------------------------- #


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
        _finalize_log(logger)


def run_clip(
    cfg,
    clip_name,
    gui=True,
    settle_s=0.5,
    loop=False,
    float_mode=False,
    monitor=False,
    log=False,
    python_port=False,
):
    """Load and play an animation clip by name in PyBullet.

    By DEFAULT the joints are driven by the EXACT firmware code compiled to the
    host (firmware_sil) — i.e. what the robot would actually command — and the
    fh_sim module is auto-rebuilt if firmware sources changed. This requires a
    C++ toolchain; if it is unavailable the run errors (use --python instead).

    python_port=True instead uses the Python re-port (firmware_port.ClipPlayer),
    which needs no toolchain. The re-port reads animation/exported_clips/
    clips_all.h; the firmware (default) reads the firmware's own clips_all.h.
    loop=True (GUI, re-port only) replays continuously; float_mode pins the body;
    monitor/log add the torque/current readout + capture.
    """
    if not python_port:
        return _run_clip_sil(
            cfg,
            clip_name,
            gui=gui,
            settle_s=settle_s,
            float_mode=float_mode,
            monitor=monitor,
            log=log,
        )

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
        _finalize_log(logger)


def _run_clip_sil(
    cfg, clip_name, gui=True, settle_s=0.5, float_mode=False, monitor=False, log=False
):
    """Play a clip through the compiled firmware (software-in-the-loop)."""
    from firmware_sil.sil_bridge import FirmwareSILDriver

    try:
        driver = FirmwareSILDriver()  # auto-builds fh_sim if stale/missing
    except ImportError as e:
        raise SystemExit(
            f"{e}\n\nThe firmware driver is the default. Without a C++ toolchain, "
            "play clips with the Python re-port instead:\n"
            f"  python facehugger.py sim --clip {clip_name!r} --python"
        ) from e
    if clip_name not in driver.clip_names():
        raise KeyError(
            f"clip {clip_name!r} not in firmware clips {driver.clip_names()}"
        )

    robot_id, joint_map = _connect_and_setup(cfg, gui, float_mode=float_mode)
    _print_banner(cfg)
    if float_mode:
        print("[float] no gravity/floor, body pinned — showing joint geometry")
    elif settle_s > 0:
        print(f"\n[settle] holding stance for {settle_s:.2f}s before clip")
        _settle(robot_id, joint_map, cfg, settle_s)

    mon_note = "  [monitor: torque/current]" if monitor else ""
    print(f"\n[clip][SIL] playing '{clip_name}' via exact firmware code{mon_note}")
    on_step, logger = setup_step_hook(robot_id, joint_map, monitor, log)
    try:
        driver.play_clip_blocking(
            robot_id,
            joint_map,
            clip_name,
            cfg.servo_force,
            cfg.servo_velocity,
            gui=gui,
            on_step=on_step,
        )
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()
        _finalize_log(logger)


def _run_gait_sil(
    cfg, gait_name, gui=True, settle_s=0.5, float_mode=False, monitor=False, log=False
):
    """Run a gait through the compiled firmware (software-in-the-loop) — the EXACT
    tickGait/tickTrot. Spawns at the neutral-stance body height (like clips), which
    is stable since the gait oscillates around NEUTRAL[]; no Python body-height solve."""
    from firmware_sil.sil_bridge import FirmwareSILDriver

    try:
        driver = FirmwareSILDriver()  # auto-builds fh_sim if stale/missing
    except ImportError as e:
        raise SystemExit(
            f"{e}\n\nThe firmware driver is the default. Without a C++ toolchain, "
            "run the gait with the Python re-port instead:\n"
            f"  python facehugger.py sim --{gait_name} --python"
        ) from e

    robot_id, joint_map = _connect_and_setup(cfg, gui, float_mode=float_mode)
    _print_banner(cfg)
    if float_mode:
        print("[float] no gravity/floor, body pinned")
    elif settle_s > 0:
        print(f"\n[settle] holding stance for {settle_s:.2f}s before gait")
        _settle(robot_id, joint_map, cfg, settle_s)

    mon_note = "  [monitor: torque/current]" if monitor else ""
    print(f"\n[gait][SIL] running '{gait_name}' (FW) via exact firmware code{mon_note}")
    on_step, logger = setup_step_hook(robot_id, joint_map, monitor, log)
    try:
        driver.run_gait_blocking(
            robot_id,
            joint_map,
            gait_name,
            cfg.servo_force,
            cfg.servo_velocity,
            gui=gui,
            on_step=on_step,
            duration_s=None if gui else 3.0,  # headless: finite smoke run
        )
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()
        _finalize_log(logger)


def run_gait(
    cfg,
    gait_name,
    gui=True,
    settle_s=0.5,
    float_mode=False,
    monitor=False,
    log=False,
    python_port=False,
):
    """Run a gait. By DEFAULT drives the joints with the EXACT compiled firmware
    (firmware_sil) — the same tickGait/tickTrot the robot runs. python_port=True uses
    the Python IK gait below instead (no C++ toolchain; also the body-height reference)."""
    from .gaits import (
        GAITS,
        _body_height_for_gait,
        _draw_overlay,
        _precompute_cycle,
        gait_joint_targets,
    )

    if gait_name not in GAITS:
        raise ValueError(f"Unknown gait: {gait_name}")
    if not python_port:
        return _run_gait_sil(
            cfg,
            gait_name,
            gui=gui,
            settle_s=settle_s,
            float_mode=float_mode,
            monitor=monitor,
            log=log,
        )
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

    robot_id, joint_map = _connect_and_setup(cfg, gui, float_mode=float_mode)
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
            targets = gait_joint_targets(cfg, gait, t)
            apply_joint_targets(
                robot_id, joint_map, targets, cfg.servo_force, cfg.servo_velocity
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
        if p.isConnected():
            p.disconnect()
        _finalize_log(logger)
