"""SIL clip suite — golden-trace regression for every firmware clip.

Replays each clip through the EXACT firmware code (fh_sim) and asserts its full
servo-angle trace matches the committed golden under firmware_sil/golden/. This
is the exporter/firmware guard: a firmware change (or a clip re-export) that
alters ANY servo angle by >=1 deg fails here, so it can't reach the robot
unreviewed. Regenerate goldens deliberately with `python -m firmware_sil.gen_golden`.

Skips if fh_sim isn't built (no C++ toolchain). Build: see firmware_sil/README.md.
"""

import json
from pathlib import Path

import pytest

pytest.importorskip("pybullet")

SIM_DIR = Path(__file__).resolve().parent
GOLDEN_DIR = SIM_DIR / "firmware_sil" / "golden"
GOLDEN_FILES = sorted(p for p in GOLDEN_DIR.glob("*.json") if p.name != "index.json")


def _fh_or_skip():
    try:
        from firmware_sil.sil_bridge import load_fh_sim

        return load_fh_sim()
    except ImportError as e:
        pytest.skip(f"fh_sim not built: {e}")


@pytest.mark.parametrize("golden_path", GOLDEN_FILES, ids=lambda p: p.stem)
def test_clip_matches_golden_trace(golden_path):
    """The clip's servo-angle trace is bit-identical to its committed golden."""
    from firmware_sil.sil_bridge import trace_clip

    fh = _fh_or_skip()
    golden = json.loads(golden_path.read_text())
    fc = fh.FirmwareControl()
    got = trace_clip(fc, golden["clip"], record_every=golden["record_every"])

    assert len(got) == len(golden["samples"]), (
        f"{golden['clip']}: sample count {len(got)} != golden {len(golden['samples'])}"
    )
    for (gt_ms, g_angles), (t_ms, angles) in zip(golden["samples"], got):
        assert t_ms == gt_ms, f"{golden['clip']}: t_ms {t_ms} != golden {gt_ms}"
        if angles != g_angles:
            diffs = [
                (i, g, a) for i, (g, a) in enumerate(zip(g_angles, angles)) if g != a
            ]
            pytest.fail(
                f"{golden['clip']} @ t={t_ms}ms: servo angles changed vs golden "
                f"(idx, golden, got): {diffs}\n"
                f"If this is an intentional firmware/clip change, regenerate with "
                f"`python -m firmware_sil.gen_golden` and review the diff."
            )


@pytest.mark.parametrize("golden_path", GOLDEN_FILES, ids=lambda p: p.stem)
def test_clip_angles_in_range(golden_path):
    """No servo in any clip is commanded outside the electrical [0,180] range."""
    golden = json.loads(golden_path.read_text())
    for t_ms, angles in golden["samples"]:
        for i, a in enumerate(angles):
            assert 0 <= a <= 180, (
                f"{golden['clip']} @ t={t_ms}ms servo[{i}]={a} out of [0,180]"
            )


def _play_clip_collecting_oor(fc, clip_name, step_hz=240):
    """Tick a clip end-to-end, draining the firmware's [OOR] serial each tick.

    Returns [(t_ms, {channel: requested_deg}), ...] for every tick that produced an
    out-of-range request (empty list = the clip stayed in range, the healthy case).
    """
    from firmware_sil.sil_bridge import parse_oor

    cid = fc.clip_id_by_name(clip_name)
    duration_ms = fc.clip_duration_ms(cid)
    fc.set_clock_ms(0)
    fc.play_clip(cid)
    fc.drain_serial()  # clear any boot/begin() lines before we start watching
    events = []
    step = 0
    while True:
        t_ms = int(step * 1000.0 / step_hz)
        fc.tick(t_ms)
        oor = parse_oor(fc.drain_serial())
        if oor:
            events.append((t_ms, oor))
        if t_ms >= duration_ms:
            break
        step += 1
    return events


def test_clip_playback_stays_in_servo_range_soft():
    """Soft guard: warn (don't fail) if any clip drives a servo out of [0,180].

    Driven by the firmware's OWN [OOR] warnings (the same lines printed on the
    bench serial monitor), captured per tick — finer than the sampled golden
    check. Healthy clips emit nothing; a regression that pushes a servo past the
    electrical limit surfaces here as a warning naming the clip/time/joint.
    """
    import warnings

    fh = _fh_or_skip()
    from firmware_sil.sil_bridge import channel_to_joint

    fc = fh.FirmwareControl()
    ch_to_joint = channel_to_joint(fc)
    saw_any = False
    for name in fc.clip_names():
        for t_ms, oor in _play_clip_collecting_oor(fc, name):
            saw_any = True
            for ch, deg in oor.items():
                joint = ch_to_joint.get(ch, f"servo {ch}")
                warnings.warn(
                    f"[OOR] clip {name!r} @ t={t_ms}ms: {joint} requested {deg:.1f}"
                    " (firmware will clamp to [0,180])",
                    stacklevel=2,
                )
    # The mechanism ran; the invariant we expect today is that nothing was warned.
    assert saw_any is False, "see warnings above — a clip left the [0,180] range"


def test_oor_feature_reports_calibrate_overrange():
    """Locks the firmware [OOR] feature end-to-end: an over-range calibrate is
    emitted by the firmware, captured by the HAL serial mock, parsed, and mapped
    back to the right joint. Guards the warning that powers telemetry pre_clamp."""
    fh = _fh_or_skip()
    from firmware_sil.sil_bridge import channel_to_joint, parse_oor

    fc = fh.FirmwareControl()
    fc.drain_serial()  # clear boot lines
    # BR (firmware RR=2) knee (servo 2) calibrated past the electrical limit.
    assert fc.calibrate(2, 2, 200) is True
    oor = parse_oor(fc.drain_serial())
    channel = fc.servo_channels()[2 * 3 + 2]
    assert oor.get(channel) == pytest.approx(200.0)
    assert channel_to_joint(fc)[channel] == "br_link3_joint"


def test_every_firmware_clip_has_a_golden():
    """A newly added firmware clip must ship a golden (no silent gaps in coverage)."""
    fh = _fh_or_skip()
    fc = fh.FirmwareControl()
    covered = {json.loads(p.read_text())["clip"] for p in GOLDEN_FILES}
    missing = [n for n in fc.clip_names() if n not in covered]
    assert not missing, (
        f"clips without a golden trace: {missing} — run `python -m firmware_sil.gen_golden`"
    )
