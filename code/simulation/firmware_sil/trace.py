"""Deterministic servo-angle traces of clips and gaits through the firmware SIL."""

import json

# Firmware GaitType enum (movements.h): NONE=0, WALK=1, TROT=2, CRAB=3.
_GAIT_NAME_TO_ID = {"walk": 1, "trot": 2, "crab": 3}


def trace_clip(fc, clip_name, record_every=24, step_hz=240, preroll_ms=200):
    """Deterministic servo-angle trace of a clip through the firmware.

    Ticks the firmware at `step_hz` over the clip's duration plus `preroll_ms`
    (t_ms = step*1000/hz) and records the 12 servo angles (whole degrees, as the
    robot receives them) every `record_every` steps. The window includes the clip
    pre-roll (the eased glide from the live pose into frame 0 that playClip starts
    with), so the full motion is captured. Used by both the reference generator and
    the clip suite, so they tick the *identical* sequence — the trace is a pure
    function of the firmware code + the clip data, making any servo-angle change
    detectable. `preroll_ms` must match the firmware CLIP_PREROLL_MS.

    Returns: list of [t_ms, [12 int degrees]].
    """
    cid = fc.clip_id_by_name(clip_name)
    if cid < 0:
        raise KeyError(f"clip {clip_name!r} not found")
    duration_ms = fc.clip_duration_ms(cid)
    end_ms = duration_ms + preroll_ms
    fc.set_clock_ms(0)
    fc.play_clip(cid)
    samples = []
    step = 0
    while True:
        t_ms = int(step * 1000.0 / step_hz)
        fc.tick(t_ms)
        if step % record_every == 0:
            samples.append([t_ms, [int(round(a)) for a in fc.servo_angles()]])
        if t_ms >= end_ms:
            break
        step += 1
    return samples


def trace_gait(fc, gait_name, direction="FW", steps=480, step_hz=240, record_every=24):
    """Deterministic servo-angle trace of a firmware gait through the SIL.

    Drives the gait the way the app/robot does: set the gait once (T:5), then
    RE-ISSUE the move (T:1) every tick — the full CMD_MOVE path arms STATE_WALK and
    resets the 500 ms deadman, so the gait keeps running instead of graceful-stopping.
    Records the 12 servo angles (whole degrees) every `record_every` steps. Shared
    driving sequence with FirmwareSILDriver.run_gait_blocking, minus PyBullet.

    Returns: list of [t_ms, [12 int degrees]]. Raises KeyError for an unknown gait.
    """
    gait_id = _GAIT_NAME_TO_ID[gait_name]  # KeyError for an unknown gait name
    fc.set_clock_ms(0)
    fc.handle_message(json.dumps({"T": 5, "g": gait_id}))
    samples = []
    for step in range(steps):
        t_ms = int(step * 1000.0 / step_hz)
        fc.handle_message(json.dumps({"T": 1, "dir": direction}))  # beat the deadman
        fc.tick(t_ms)
        if step % record_every == 0:
            samples.append([t_ms, [int(round(a)) for a in fc.servo_angles()]])
    return samples
