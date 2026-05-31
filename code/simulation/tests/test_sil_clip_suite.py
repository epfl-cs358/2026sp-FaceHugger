"""SIL clip suite — reference-trace regression for firmware clips.

Replays each clip through the EXACT firmware code (fh_sim) and compares its
servo-angle trace to the committed reference under firmware_sil/reference_clips/.

Which clips are hard-checked is controlled by reference_clips: in
facehugger_config.yaml (the "locked" set). For clips in that list, a mismatch
fails the test. For clips with a reference trace but NOT in the list, a mismatch
only warns — useful while actively authoring new clips whose exact output is still
in flux.

Regenerate references deliberately with:
    python code/facehugger.py update-reference-clips

Skips if fh_sim isn't built (no C++ toolchain). Build: see firmware_sil/README.md.
"""

import json
import warnings
from pathlib import Path

import pytest
import yaml

pytest.importorskip("pybullet")

SIM_DIR = Path(__file__).resolve().parent.parent  # tests/ -> code/simulation/
REFERENCE_CLIPS_DIR = SIM_DIR / "firmware_sil" / "reference_clips"
REFERENCE_FILES = sorted(
    p for p in REFERENCE_CLIPS_DIR.glob("*.json") if p.name != "index.json"
)


def _load_required_clip_names():
    """Clip names listed under reference_clips: in facehugger_config.yaml."""
    cfg_path = SIM_DIR / "facehugger_config.yaml"
    try:
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        return set(cfg.get("reference_clips", []))
    except Exception:
        return set()


REQUIRED_CLIP_NAMES = _load_required_clip_names()


def _fh_or_skip():
    try:
        from firmware_sil.sil_bridge import load_fh_sim

        return load_fh_sim()
    except ImportError as e:
        pytest.skip(f"fh_sim not built: {e}")


@pytest.mark.parametrize("ref_path", REFERENCE_FILES, ids=lambda p: p.stem)
def test_clip_matches_reference_trace(ref_path):
    """Compare a clip's servo-angle trace to its committed reference.

    Clips in reference_clips: (facehugger_config.yaml) must match exactly; all
    others only warn on mismatch so CI doesn't break during active clip authoring.
    """
    from firmware_sil.sil_bridge import trace_clip

    fh = _fh_or_skip()
    ref = json.loads(ref_path.read_text())
    clip_name = ref["clip"]
    required = clip_name in REQUIRED_CLIP_NAMES

    fc = fh.FirmwareControl()
    got = trace_clip(fc, clip_name, record_every=ref["record_every"])

    def _report(msg):
        full = (
            msg + "\nRun `python code/facehugger.py update-reference-clips` to update."
        )
        if required:
            pytest.fail(full)
        else:
            warnings.warn(full, UserWarning, stacklevel=3)

    if len(got) != len(ref["samples"]):
        _report(
            f"{clip_name}: sample count {len(got)} != reference {len(ref['samples'])}"
        )
        return

    for (gt_ms, g_angles), (t_ms, angles) in zip(ref["samples"], got):
        if t_ms != gt_ms:
            _report(f"{clip_name}: t_ms {t_ms} != reference {gt_ms}")
            return
        if angles != g_angles:
            diffs = [
                (i, g, a) for i, (g, a) in enumerate(zip(g_angles, angles)) if g != a
            ]
            _report(
                f"{clip_name} @ t={t_ms}ms: servo angles changed vs reference "
                f"(idx, reference, got): {diffs}"
            )
            return


@pytest.mark.parametrize("ref_path", REFERENCE_FILES, ids=lambda p: p.stem)
def test_clip_angles_in_range(ref_path):
    """No servo in any reference trace is commanded outside the electrical [0,180] range."""
    ref = json.loads(ref_path.read_text())
    for t_ms, angles in ref["samples"]:
        for i, a in enumerate(angles):
            assert 0 <= a <= 180, (
                f"{ref['clip']} @ t={t_ms}ms servo[{i}]={a} out of [0,180]"
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
    bench serial monitor), captured per tick — finer than the sampled reference
    check. Healthy clips emit nothing; a regression that pushes a servo past the
    electrical limit surfaces here as a warning naming the clip/time/joint.
    """
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


def test_every_reference_clip_has_a_trace():
    """Clips in reference_clips: (facehugger_config.yaml) should have a reference trace.

    Missing traces warn (not fail) so adding a clip to the required list doesn't
    immediately break CI — run update-reference-clips to generate the trace.
    Clips compiled into firmware but absent from both the list and reference_clips/
    are also reported so they're visible during review.
    """
    fh = _fh_or_skip()
    fc = fh.FirmwareControl()
    covered = {json.loads(p.read_text())["clip"] for p in REFERENCE_FILES}

    missing_required = [n for n in REQUIRED_CLIP_NAMES if n not in covered]
    if missing_required:
        warnings.warn(
            f"clips in reference_clips config but missing a trace: {missing_required}"
            " — run `python code/facehugger.py update-reference-clips`",
            UserWarning,
            stacklevel=2,
        )

    untracked = [n for n in fc.clip_names() if n not in covered]
    if untracked:
        warnings.warn(
            f"clips compiled into firmware with no reference trace: {untracked}"
            " (add to reference_clips: in facehugger_config.yaml to lock them down)",
            UserWarning,
            stacklevel=2,
        )
