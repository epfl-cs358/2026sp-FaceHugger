#!/usr/bin/env python3
# === Plain Python — no Blender required ===
# Run as: python <this file>  (or via pytest). Stubs `bpy` at import time.
"""Pure-Python parity test — the lockstep contract between the Blender
exporter's `_frame_to_servo` and the firmware's `tickGait` →
`translateToServo` switch at origin/feat/hardware-integration-milestone-2
@ 3899ebc.

If this test ever fails, EITHER the firmware has diverged OR the exporter
has — and a clip exported here will move the wrong joint/direction on
hardware. The four per-leg formulas + the scale-from-N math are the
contract; this test re-asserts them from independent ground truth read
out of the firmware source.

No Blender required. Stubs `bpy` just enough for fh_clip_panel.py to
import, then exercises the pure conversion. Run via:

    uv run python animation/scripts/test_servo_parity.py
"""

import importlib.util
import json
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(
    REPO_ROOT, "animation/addons/fh_clip_panel.py"
)  # moved scripts/->addons/ (2f627a1)
CONV_PATH = os.path.join(REPO_ROOT, "animation/convention.json")

_LIB = os.path.join(REPO_ROOT, "animation/lib")
if _LIB not in sys.path:
    sys.path.insert(0, _LIB)


def _firmware_translate(leg: str, sh: float, th: float, kn: float) -> list[float]:
    """Independent ground truth — copied verbatim from
    `code/firmware/src/nervous_system/spinal_cord.cpp` `tickGait` switch
    at origin/feat/hardware-integration-milestone-2 @ 3899ebc
    (lines 199-218). DO NOT factor out — divergence here is exactly the
    bug we're guarding against."""
    if leg == "fr":  # LEG_FR (LegId 0)
        return [90.0 + (sh - 45.0), 90.0 - th, 90.0 + kn]
    if leg == "fl":  # LEG_FL (LegId 1)
        # Change B: regularized to 90 + (sh - 135) (was `sh`) so servo 90 = outward.
        return [90.0 + (sh - 135.0), 90.0 + th, 90.0 - kn]
    if leg == "br":  # LEG_RR (LegId 2) — Blender 'br' ↔ firmware 'RR'
        # BR shoulder un-mirrored (2026-05-25): +sh = +servo like the other
        # three (identical motor, yaw shaft on the same vertical axis).
        return [90.0 + (sh + 45.0), 90.0 + th, 90.0 - kn]
    if leg == "bl":  # LEG_RL (LegId 3) — Blender 'bl' ↔ firmware 'RL'
        return [90.0 + (sh + 135.0), 90.0 - th, 90.0 + kn]
    raise ValueError(leg)


def main() -> int:
    bpy_stub.install()
    spec = importlib.util.spec_from_file_location("fh_clip_panel", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    with open(CONV_PATH) as fh:
        conv = json.load(fh)
    neutral = conv["neutral_joint_deg"]
    scale = conv["scale"]
    legs = ("fr", "fl", "br", "bl")
    links = ("link1", "link2", "link3")

    fails: list[str] = []

    def check(name: str, cond: bool, detail: object = "") -> None:
        print(
            ("PASS " if cond else "FAIL ") + name + (f" :: {detail}" if detail else "")
        )
        if not cond:
            fails.append(name)

    # _LEGS order must match firmware LegId enum order (FR=0, FL=1, RR=2, RL=3).
    check("_LEGS order matches firmware LegId", mod._LEGS == legs, mod._LEGS)

    # Case 1 — at NEUTRAL (raw == N): scale-from-N collapses to N exactly;
    # output must equal firmware translateToServo(N) per leg.
    row_n = {"frame": 1, "time_ms": 0}
    for leg in legs:
        for j, lk in enumerate(links):
            row_n[f"{leg}_{lk}"] = neutral[leg][j]
    out_n = mod._frame_to_servo(row_n, conv)
    for leg in legs:
        sh, th, kn = neutral[leg]
        expected = [round(v) for v in _firmware_translate(leg, sh, th, kn)]
        check(
            f"{leg} @ N == firmware translateToServo(N)",
            out_n[leg] == expected,
            (out_n[leg], expected),
        )

    # Case 2 — offset raw = N + 30°. After scale-from-N: scaled = N + 30·scale.
    # Then translateToServo(scaled) must match per-leg.
    OFFSET = 30.0
    for leg in legs:
        row = dict(row_n)
        for j, lk in enumerate(links):
            row[f"{leg}_{lk}"] = neutral[leg][j] + OFFSET
        sh = neutral[leg][0] + OFFSET * scale
        th = neutral[leg][1] + OFFSET * scale
        kn = neutral[leg][2] + OFFSET * scale
        expected = [round(v) for v in _firmware_translate(leg, sh, th, kn)]
        actual = mod._frame_to_servo(row, conv)[leg]
        check(
            f"{leg} @ N+30° matches scale·{scale:.4f} then firmware translate",
            actual == expected,
            (actual, expected),
        )

    # Case 3 — only one joint perturbed per leg: catches accidental cross-axis
    # leakage in the scale or translate logic.
    for leg in legs:
        for j_perturb, lk in enumerate(links):
            row = dict(row_n)
            row[f"{leg}_{lk}"] = neutral[leg][j_perturb] + 20.0
            scaled = list(neutral[leg])
            scaled[j_perturb] = neutral[leg][j_perturb] + 20.0 * scale
            expected = [round(v) for v in _firmware_translate(leg, *scaled)]
            actual = mod._frame_to_servo(row, conv)[leg]
            check(
                f"{leg} @ N + 20° on {lk} only",
                actual == expected,
                (actual, expected),
            )

    # Sanity: scale must be the 2/3 value the firmware GAITS[] are pre-scaled to.
    check(
        "convention.scale ≈ 2/3 (pre-scale rule)",
        abs(scale - 2.0 / 3.0) < 1e-3,
        scale,
    )

    # ---- Wire-shape contract (origin/main CMD_CALIBRATE) ----------------
    # Exporter must emit, per joint:
    #   {T:4, id:_LEG_ID[leg], servo_id:j, a:_frame_to_servo(...)[leg][j]}
    # where the firmware then derives the PCA channel via
    #   LEG_SERVO_CHANNEL[id][servo_id]   (config.h:64 on origin/main)
    # The list index into _frame_to_servo's per-leg list IS servo_id
    # (0=hip, 1=thigh, 2=knee — same order as LEG_SERVO_CHANNEL row).

    check(
        "_LEG_ID maps Blender leg name -> firmware LegId",
        getattr(mod, "_LEG_ID", None) == {"fr": 0, "fl": 1, "br": 2, "bl": 3},
        getattr(mod, "_LEG_ID", None),
    )

    # Firmware LEG_SERVO_CHANNEL table — verbatim from origin/main
    # code/firmware/src/shared/config.h:19-48 (#define values) +
    # config.h:64 (the table layout). DO NOT refactor: divergence here
    # against convention.json is exactly the bug this cross-check guards.
    FIRMWARE_LEG_SERVO_CHANNEL: dict[str, list[int]] = {
        "fr": [8, 9, 10],  # FRONT_RIGHT  hip/thigh/knee
        "fl": [12, 13, 14],  # FRONT_LEFT
        "br": [4, 5, 6],  # BOTTOM_RIGHT (firmware LegId 2 = RR)
        "bl": [0, 1, 2],  # BOTTOM_LEFT  (firmware LegId 3 = RL)
    }
    channels = conv.get("channels", {})
    for leg in legs:
        check(
            f"convention.channels[{leg}] == firmware LEG_SERVO_CHANNEL "
            f"(reference cross-check)",
            channels.get(leg) == FIRMWARE_LEG_SERVO_CHANNEL[leg],
            (channels.get(leg), FIRMWARE_LEG_SERVO_CHANNEL[leg]),
        )

    # Wire well-formedness: for every (leg, servo_id) pair, the resulting
    # (id, servo_id, angle) is in the firmware's accepted ranges.
    out_n = mod._frame_to_servo(row_n, conv)
    for leg in legs:
        leg_id = mod._LEG_ID[leg]
        check(f"{leg}: leg_id in 0..3", 0 <= leg_id <= 3, leg_id)
        for j in range(3):
            a = out_n[leg][j]
            check(
                f"{leg} servo_id={j}: angle int and 0..180 (clamp range)",
                isinstance(a, int) and 0 <= a <= 180,
                a,
            )

    print("RESULT " + ("GREEN" if not fails else "RED: " + ", ".join(fails)))
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
