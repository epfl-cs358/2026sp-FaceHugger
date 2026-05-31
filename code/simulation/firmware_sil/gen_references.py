"""Regenerate the reference servo-angle traces for every firmware clip.

Run this ONLY when the firmware's clip output is *intentionally* changed (a clip
re-export, or a deliberate edit to the control math). It plays every clip through
the compiled firmware (fh_sim) and writes one reference JSON per clip under
firmware_sil/reference_clips/. test_sil_clip_suite.py replays clips listed under
reference_clips: in facehugger_config.yaml and asserts exact match — so an
*unintentional* firmware change fails the test suite, and an intentional one shows
up as a reviewable diff in these reference files.

Usage (after building fh_sim — see firmware_sil/README.md):
    python code/facehugger.py update-reference-clips
    # or directly:
    cd code/simulation
    conda run -n facehugger python -m firmware_sil.gen_references
"""

import json
import re
from pathlib import Path

from firmware_sil.sil_bridge import load_fh_sim, trace_clip

REFERENCE_CLIPS_DIR = Path(__file__).resolve().parent / "reference_clips"
RECORD_EVERY = 24  # 240 Hz / 24 = one sample per 100 ms


def _safe(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def main():
    fh = load_fh_sim()
    REFERENCE_CLIPS_DIR.mkdir(exist_ok=True)

    index = {}
    # A fresh FirmwareControl per clip so the clip pre-roll always eases from the
    # same default pose (the clip suite also makes one per clip). Reusing one
    # instance would start each pre-roll from the previous clip's leftover pose,
    # making the trace order-dependent.
    for name in fh.FirmwareControl().clip_names():
        fc = fh.FirmwareControl()
        samples = trace_clip(fc, name, record_every=RECORD_EVERY)
        path = REFERENCE_CLIPS_DIR / f"{_safe(name)}.json"
        payload = {
            "clip": name,
            "step_hz": 240,
            "record_every": RECORD_EVERY,
            "servo_order": "FR,FL,BR,BL x (hip,thigh,knee); whole degrees",
            "samples": samples,
        }
        path.write_text(json.dumps(payload, indent=1) + "\n")
        index[name] = path.name
        print(f"  wrote {path.name}  ({len(samples)} samples)")

    (REFERENCE_CLIPS_DIR / "index.json").write_text(json.dumps(index, indent=1) + "\n")
    print(f"[references] {len(index)} clips → {REFERENCE_CLIPS_DIR}")


if __name__ == "__main__":
    main()
