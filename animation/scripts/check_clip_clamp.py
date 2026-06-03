# === Plain Python — no Blender required ===
"""Check which clips/frames/joints would be clamped by clampClipServos.

Reads the current clips_all.h (pre-scaled math-space angles), runs
translateToServo + clampClipServos for every frame, and reports any
joint where the clamp fires.

Optionally also shows what happens at scale=1.0 (no 2/3 compression),
by reversing the baked scale and re-applying at 1.0. This helps decide
whether changing convention.json scale from 0.6667 to 1.0 would cause
clips to exceed servo limits.

Usage (from repo root):
    python animation/scripts/check_clip_clamp.py
    python animation/scripts/check_clip_clamp.py --scale-one
    python animation/scripts/check_clip_clamp.py --clip wave --scale-one
"""

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]  # scripts/ -> animation/ -> repo root
sys.path.insert(0, str(_REPO_ROOT / "code" / "simulation"))

from firmware_port.clip_loader import load_clips_all_h
from firmware_port.servo_convention import (
    CALIB_KNEE,
    CALIB_THIGH,
    HIP_CLAMP_FROM_NINETY,
    KNEE_CLAMP_FROM_CALIB,
    NEUTRAL,
    THIGH_CLAMP_FROM_CALIB,
    clamp_clip_servos,
    translate_to_servo,
)

_LEG_NAMES = {0: "fr", 1: "fl", 2: "br", 3: "bl"}
_JOINT_NAMES = {0: "shoulder", 1: "thigh", 2: "knee"}

# convention.json scale baked into the current clips_all.h
_BAKED_SCALE = 0.6667


def _unscale(scaled_val: float, neutral: float, baked_scale: float) -> float:
    """Reverse the N+(raw-N)*scale compression to recover Blender raw angle."""
    if abs(baked_scale) < 1e-9:
        return neutral
    return neutral + (scaled_val - neutral) / baked_scale


def _rescale(raw_val: float, neutral: float, new_scale: float) -> float:
    return neutral + (raw_val - neutral) * new_scale


def check_frame(leg_id: int, sh: float, th: float, kn: float) -> list[dict]:
    """Run one leg's math-space angles through translate+clamp; return hits."""
    raw = translate_to_servo(leg_id, sh, th, kn)
    clamped = clamp_clip_servos(leg_id, raw)
    hits = []
    for joint_idx, (r, c, name) in enumerate(
        zip(
            (raw.hip, raw.thigh, raw.knee),
            (clamped.hip, clamped.thigh, clamped.knee),
            ("shoulder", "thigh", "knee"),
        )
    ):
        if abs(r - c) > 0.01:
            # Compute the clamp envelope for display
            if name == "shoulder":
                lo, hi = 90 - HIP_CLAMP_FROM_NINETY, 90 + HIP_CLAMP_FROM_NINETY
            elif name == "thigh":
                ct = CALIB_THIGH[leg_id]
                lo, hi = ct - THIGH_CLAMP_FROM_CALIB, ct + THIGH_CLAMP_FROM_CALIB
            else:
                ck = CALIB_KNEE[leg_id]
                lo, hi = ck - KNEE_CLAMP_FROM_CALIB, ck + KNEE_CLAMP_FROM_CALIB
            hits.append(
                {
                    "joint": name,
                    "requested": r,
                    "clamped_to": c,
                    "overshoot": r - c,
                    "envelope": (lo, hi),
                }
            )
    return hits


def run(clip_filter: str | None, show_scale_one: bool) -> int:
    clips = load_clips_all_h()
    if clip_filter:
        clips = [c for c in clips if clip_filter.lower() in c.name.lower()]
        if not clips:
            print(f"No clips matching {clip_filter!r}. Available clips:")
            for c in load_clips_all_h():
                print(f"  {c.name}")
            return 1

    total_hits = 0
    scales_to_check = [_BAKED_SCALE]
    if show_scale_one:
        scales_to_check.append(1.0)

    for clip in clips:
        clip_hits = {s: [] for s in scales_to_check}

        for frame in clip.frames:
            for leg_id in range(4):
                base_sh = frame.a[leg_id * 3]
                base_th = frame.a[leg_id * 3 + 1]
                base_kn = frame.a[leg_id * 3 + 2]

                for scale in scales_to_check:
                    if abs(scale - _BAKED_SCALE) < 1e-4:
                        sh, th, kn = base_sh, base_th, base_kn
                    else:
                        # Undo baked scale, re-apply new scale
                        n = NEUTRAL[leg_id]
                        sh = _rescale(
                            _unscale(base_sh, n.sh, _BAKED_SCALE), n.sh, scale
                        )
                        th = _rescale(
                            _unscale(base_th, n.th, _BAKED_SCALE), n.th, scale
                        )
                        kn = _rescale(
                            _unscale(base_kn, n.kn, _BAKED_SCALE), n.kn, scale
                        )

                    hits = check_frame(leg_id, sh, th, kn)
                    for h in hits:
                        clip_hits[scale].append(
                            {
                                "t_ms": frame.t_ms,
                                "leg": _LEG_NAMES[leg_id],
                                **h,
                            }
                        )

        any_hits = any(clip_hits[s] for s in scales_to_check)
        if not any_hits:
            print(f"[OK]  {clip.name!r}  — no clamp hits")
            continue

        print(f"\n[CLIP]  {clip.name!r}  ({clip.frame_count} frames)")
        for scale in scales_to_check:
            hits = clip_hits[scale]
            if not hits:
                label = (
                    f"scale={scale:.4f}"
                    if scale != 1.0
                    else "scale=1.0 (Blender=hardware)"
                )
                print(f"  {label}:  no clamp hits")
                continue

            label = (
                f"scale={scale:.4f} (current baked)"
                if abs(scale - _BAKED_SCALE) < 1e-4
                else "scale=1.0  (Blender=hardware)"
            )
            print(f"  {label}:  {len(hits)} hit(s)")

            # Group by leg+joint, show worst overshoot and affected frame count
            summary: dict[str, list] = {}
            for h in hits:
                key = f"{h['leg']} {h['joint']}"
                summary.setdefault(key, []).append(h)

            for key, group in sorted(summary.items()):
                worst = max(group, key=lambda x: abs(x["overshoot"]))
                lo, hi = worst["envelope"]
                print(
                    f"    {key:18s}  {len(group):3d} frame(s)  "
                    f"worst={worst['requested']:7.1f}° (envelope [{lo:.0f}, {hi:.0f}])  "
                    f"overshoot={worst['overshoot']:+.1f}°  at t={worst['t_ms']}ms"
                )
            total_hits += len(hits)

    print()
    if total_hits == 0:
        print("All clips clean — no clamp hits.")
    else:
        print(f"Total clamp hits across all clips/scales checked: {total_hits}")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--clip", metavar="NAME", help="filter to clips whose name contains NAME"
    )
    ap.add_argument(
        "--scale-one",
        action="store_true",
        help="also simulate what happens at scale=1.0 (Blender angle = hardware angle)",
    )
    args = ap.parse_args()
    sys.exit(run(args.clip, args.scale_one))


if __name__ == "__main__":
    main()
