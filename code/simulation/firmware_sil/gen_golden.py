"""Regenerate the golden servo-angle traces for every firmware clip.

Run this ONLY when the firmware's clip output is *intentionally* changed (a clip
re-export, or a deliberate edit to the control math). It plays every clip through
the compiled firmware (fh_sim) and writes one golden JSON per clip under
firmware_sil/golden/. test_sil_clip_suite.py replays the same clips and asserts an
exact match — so an *unintentional* firmware change fails the test suite, and an
intentional one shows up as a reviewable diff in these golden files.

Usage (after building fh_sim — see firmware_sil/README.md):
    cd code/simulation
    conda run -n facehugger python -m firmware_sil.gen_golden
"""

import json
import re
from pathlib import Path

from firmware_sil.sil_bridge import load_fh_sim, trace_clip

GOLDEN_DIR = Path(__file__).resolve().parent / "golden"
RECORD_EVERY = 24  # 240 Hz / 24 = one sample per 100 ms


def _safe(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def main():
    fh = load_fh_sim()
    GOLDEN_DIR.mkdir(exist_ok=True)

    index = {}
    # A fresh FirmwareControl per clip so the clip pre-roll always eases from the
    # same default pose (the clip suite also makes one per clip). Reusing one
    # instance would start each pre-roll from the previous clip's leftover pose,
    # making the trace order-dependent.
    for name in fh.FirmwareControl().clip_names():
        fc = fh.FirmwareControl()
        samples = trace_clip(fc, name, record_every=RECORD_EVERY)
        path = GOLDEN_DIR / f"{_safe(name)}.json"
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

    (GOLDEN_DIR / "index.json").write_text(json.dumps(index, indent=1) + "\n")
    print(f"[golden] {len(index)} clips → {GOLDEN_DIR}")


if __name__ == "__main__":
    main()
