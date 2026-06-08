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

from animation_tools.clip_loader import load_clips_all_h
from animation_tools.servo_convention import (
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

# Same-side inter-leg shoulder buffer. Must match
# code/firmware/src/shared/config.h INTER_LEG_BUFFER_DEG so the lint and the
# (eventual) firmware safety net agree on what's a violation.
INTER_LEG_BUFFER_DEG = 5.0


def _shorter_arc_deg(a: float, b: float) -> float:
    """Shorter arc between two angles in degrees, normalised to [0, 180]."""
    d = (b - a) % 360.0
    if d > 180.0:
        d = 360.0 - d
    return d


def check_inter_leg_gap(
    shoulders_math: list[float], buffer_deg: float = INTER_LEG_BUFFER_DEG
) -> dict[str, float]:
    """Math-space shorter-arc gap between same-side front/back shoulders.

    Input is the 4 shoulder math angles in firmware LegId order
    [FR, FL, BR, BL]. Returns {side: gap_deg} for sides where the gap is
    below `buffer_deg`. Empty dict = no violations. At rest each gap is
    90° so the check passes trivially; it only fires when a front leg
    swings most of the way back into its back neighbour's quadrant.
    """
    fr, fl, br, bl = shoulders_math
    violations: dict[str, float] = {}
    right_gap = _shorter_arc_deg(fr, br)
    if right_gap < buffer_deg:
        violations["RIGHT (FR-BR)"] = right_gap
    left_gap = _shorter_arc_deg(fl, bl)
    if left_gap < buffer_deg:
        violations["LEFT (FL-BL)"] = left_gap
    return violations


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
        clip_gap_violations: dict[float, list] = {s: [] for s in scales_to_check}

        for frame in clip.frames:
            # Collect per-scale shoulder math angles for the inter-leg gap
            # check after all 4 legs are processed.
            shoulders_per_scale: dict[float, list[float]] = {
                s: [0.0, 0.0, 0.0, 0.0] for s in scales_to_check
            }

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

                    shoulders_per_scale[scale][leg_id] = sh

                    hits = check_frame(leg_id, sh, th, kn)
                    for h in hits:
                        clip_hits[scale].append(
                            {
                                "t_ms": frame.t_ms,
                                "leg": _LEG_NAMES[leg_id],
                                **h,
                            }
                        )

            # Inter-leg gap check, after all 4 legs are known for this frame.
            for scale in scales_to_check:
                gaps = check_inter_leg_gap(shoulders_per_scale[scale])
                for side, gap in gaps.items():
                    clip_gap_violations[scale].append(
                        {"t_ms": frame.t_ms, "side": side, "gap": gap}
                    )

        any_hits = any(clip_hits[s] for s in scales_to_check)
        any_gap = any(clip_gap_violations[s] for s in scales_to_check)
        if not any_hits and not any_gap:
            print(f"[OK]  {clip.name!r}  — no clamp hits, no inter-leg violations")
            continue

        print(f"\n[CLIP]  {clip.name!r}  ({clip.frame_count} frames)")
        for scale in scales_to_check:
            hits = clip_hits[scale]
            gap_v = clip_gap_violations[scale]
            label = (
                f"scale={scale:.4f} (current baked)"
                if abs(scale - _BAKED_SCALE) < 1e-4
                else "scale=1.0  (Blender=hardware)"
            )

            if not hits and not gap_v:
                print(f"  {label}:  no clamp hits, no inter-leg violations")
                continue
            print(f"  {label}:")

            # --- per-servo clamp hits ---
            if hits:
                print(f"    [clamp] {len(hits)} hit(s)")
                summary: dict[str, list] = {}
                for h in hits:
                    key = f"{h['leg']} {h['joint']}"
                    summary.setdefault(key, []).append(h)
                for key, group in sorted(summary.items()):
                    worst = max(group, key=lambda x: abs(x["overshoot"]))
                    lo, hi = worst["envelope"]
                    print(
                        f"      {key:18s}  {len(group):3d} frame(s)  "
                        f"worst={worst['requested']:7.1f}° "
                        f"(envelope [{lo:.0f}, {hi:.0f}])  "
                        f"overshoot={worst['overshoot']:+.1f}°  "
                        f"at t={worst['t_ms']}ms"
                    )
                total_hits += len(hits)

            # --- inter-leg gap violations ---
            if gap_v:
                print(
                    f"    [inter-leg] {len(gap_v)} frame(s) below "
                    f"{INTER_LEG_BUFFER_DEG:.1f}° buffer"
                )
                by_side: dict[str, list] = {}
                for v in gap_v:
                    by_side.setdefault(v["side"], []).append(v)
                for side, group in sorted(by_side.items()):
                    worst = min(group, key=lambda x: x["gap"])
                    print(
                        f"      {side:18s}  {len(group):3d} frame(s)  "
                        f"worst gap={worst['gap']:.2f}°  at t={worst['t_ms']}ms"
                    )
                total_hits += len(gap_v)

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
