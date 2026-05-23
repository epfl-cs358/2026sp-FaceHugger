# animation/migrations — one-shot migration scripts

| File | What it does |
|---|---|
| `fh_rename_actions.py` | Renames the 5 legacy Action names on an old rig to the `base_anim__<target>` convention expected by `fh_clip_panel.py`. |

## fh_rename_actions.py

> **LEGACY · run once · idempotent.** Only needed for an old rig whose
> Actions still use the pre-convention names. A current rig built by
> `urdf_to_blender_rigged.py` does not need this. The day-to-day tooling
> is `animation/addons/fh_clip_panel.py`.

Renames the 5 legacy per-object Actions on `animation/fh_rigged_latest.blend`
to the clip-naming convention `fh_clip_panel.py` expects. Idempotent — skips
Actions already at the new name — and **saves the file in place**.

| Old name | New name |
|---|---|
| `body_ctrlAction` | `base_anim__body_ctrl` |
| `foot_target_flAction` | `base_anim__foot_target_fl` |
| `foot_target_frAction` | `base_anim__foot_target_fr` |
| `foot_target_blAction` | `base_anim__foot_target_bl` |
| `foot_target_brAction` | `base_anim__foot_target_br` |

After renaming, the script verifies each object's `AnimData.action_slot.identifier`
matches `OB<target>` (Blender's auto-prefix for OBJECT-type slots) and prints a
per-object report. A `MISMATCH` line is worth investigating before relying on the
panel's slot lookup.

### Run

```bash
# default: animation/fh_rigged_latest.blend
blender --background --python animation/migrations/fh_rename_actions.py

# override target .blend:
blender --background --python animation/migrations/fh_rename_actions.py -- \
    --blend animation/some-other.blend
```

The script exits non-zero (no save) if the target `.blend` can't be found, and
skips `wm.save_mainfile()` if no Actions were renamed — so a no-op re-run leaves
the file's mtime untouched.
