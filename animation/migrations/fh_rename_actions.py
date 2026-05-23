"""
[LEGACY · one-shot · idempotent] Run once per old rig to migrate legacy
Action names; safe to re-run (no-op once migrated). Not part of the
day-to-day workflow — fh_clip_panel.py is the active tooling.

fh_rename_actions.py — one-shot migration of FaceHugger Actions to the
clip-naming convention used by fh_clip_panel.py.

Renames the 5 legacy per-object Actions:

    body_ctrlAction       → base_anim__body_ctrl
    foot_target_flAction  → base_anim__foot_target_fl
    foot_target_frAction  → base_anim__foot_target_fr
    foot_target_blAction  → base_anim__foot_target_bl
    foot_target_brAction  → base_anim__foot_target_br

Also verifies (prints) each object's currently-assigned AnimData
slot identifier. Blender's layered Action API auto-prefixes object
slots with 'OB' at creation time — `OB<target>` should already be
correct on all five objects and the rename above does not touch
slot identifiers. Any MISMATCH in the verification output is worth
investigating before the clip panel relies on the slot lookup.

------------------------------------------------------------------
Usage
------------------------------------------------------------------

From the repo root:

    blender --background --python animation/scripts/fh_rename_actions.py

The script:
  1. Opens animation/fh_rigged_latest.blend.
  2. Renames the 5 Actions listed above (skips any that already use
     the new name, so it's idempotent — safe to re-run).
  3. Verifies slot identifiers and prints a per-object report.
  4. Saves the file in place via wm.save_mainfile().

To target a different .blend, pass a path after `--`:

    blender --background --python animation/scripts/fh_rename_actions.py -- \\
            --blend animation/some-other.blend

If the blend file isn't found, the script prints the resolved path
and exits non-zero — no partial save.
"""

import argparse
import sys
from pathlib import Path

import bpy

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
DEFAULT_BLEND = REPO_ROOT / "animation" / "fh_rigged_latest.blend"

RENAME_MAP = {
    "body_ctrlAction": "base_anim__body_ctrl",
    "foot_target_flAction": "base_anim__foot_target_fl",
    "foot_target_frAction": "base_anim__foot_target_fr",
    "foot_target_blAction": "base_anim__foot_target_bl",
    "foot_target_brAction": "base_anim__foot_target_br",
}

TARGETS = [
    "body_ctrl",
    "foot_target_fl",
    "foot_target_fr",
    "foot_target_bl",
    "foot_target_br",
]


def parse_script_args():
    """Read CLI args after the `--` separator; Blender ignores those."""
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--blend",
        type=Path,
        default=DEFAULT_BLEND,
        help=f"path to the .blend file to migrate (default: {DEFAULT_BLEND})",
    )
    return p.parse_args(argv)


def rename_actions():
    renamed = 0
    skipped_existing = []
    missing = []
    for old, new in RENAME_MAP.items():
        if bpy.data.actions.get(new):
            skipped_existing.append(new)
            print(f"[skip] '{new}' already present — leaving '{old}' alone")
            continue
        action = bpy.data.actions.get(old)
        if action is None:
            missing.append(old)
            print(f"[skip] '{old}' not found in bpy.data.actions")
            continue
        action.name = new
        renamed += 1
        print(f"[rename] {old} → {new}")
    return renamed, missing, skipped_existing


def verify_slots():
    print()
    print("Verifying AnimData slot identifiers (expecting 'OB<target>'):")
    mismatches = 0
    for target in TARGETS:
        obj = bpy.data.objects.get(target)
        if obj is None:
            print(f"  {target:<22s} object not in scene")
            mismatches += 1
            continue
        adt = obj.animation_data
        if adt is None or adt.action is None:
            print(f"  {target:<22s} no animation data / action assigned")
            mismatches += 1
            continue
        slot = adt.action_slot
        if slot is None:
            print(f"  {target:<22s} action='{adt.action.name}' but action_slot is None")
            mismatches += 1
            continue
        expected = f"OB{target}"
        status = (
            "ok" if slot.identifier == expected else f"MISMATCH (expected '{expected}')"
        )
        print(
            f"  {target:<22s} action='{adt.action.name}' slot.identifier='{slot.identifier}' [{status}]"
        )
        if slot.identifier != expected:
            mismatches += 1
    return mismatches


def main():
    args = parse_script_args()
    blend = args.blend.resolve()
    if not blend.is_file():
        print(f"ERROR: blend file not found: {blend}")
        sys.exit(1)

    print(f"Opening {blend}")
    bpy.ops.wm.open_mainfile(filepath=str(blend))

    renamed, missing, skipped = rename_actions()
    mismatches = verify_slots()

    print()
    print(
        f"Renamed {renamed}/{len(RENAME_MAP)} actions"
        f" | skipped (already new name): {len(skipped)}"
        f" | missing: {missing or 'none'}"
        f" | slot mismatches: {mismatches}"
    )

    if renamed == 0:
        print("No renames performed — file left untouched.")
        return

    bpy.ops.wm.save_mainfile()
    print(f"Saved {bpy.data.filepath}")


if __name__ == "__main__":
    main()
