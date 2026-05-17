"""
fh_clip_panel.py — FaceHugger clip manager (Blender 5.1.1)

Adds an "FH Clips" panel to the 3D viewport sidebar (N-panel,
category "FaceHugger"). Manages "clips" — named groups of 5 Actions
following the convention:

    <clip_name>__body_ctrl
    <clip_name>__foot_target_fl
    <clip_name>__foot_target_fr
    <clip_name>__foot_target_bl
    <clip_name>__foot_target_br

Each clip's Actions are assigned to their target objects using
Blender 4.4+'s layered Action API (action + action_slot), not via
the legacy single-action slot.

------------------------------------------------------------------
Loading into Blender
------------------------------------------------------------------

Option A — Install as add-on (persists across sessions):
  1. Edit > Preferences > Add-ons > Install...
  2. Pick this file, install, then tick "FH Clip Panel" to enable.
  3. Open the 3D viewport, press N, switch to the "FaceHugger" tab.

Option B — Run from the Text Editor (one-shot, for development):
  1. Open this file in Blender's Text Editor.
  2. Press Alt+P (or click "Run Script").
  3. The panel appears in the N-sidebar immediately. Re-running
     after edits unregisters the previous classes first, so you
     can iterate without restarting Blender.

Works on the existing animation/fh_rigged_latest.blend out of the
box — it just needs the rig objects body_ctrl / foot_target_<leg>
to exist in the scene.
"""

bl_info = {
    "name": "FH Clip Panel",
    "author": "FaceHugger team",
    "version": (0, 2, 0),
    "blender": (4, 4, 0),
    "location": "View3D > Sidebar > FaceHugger",
    "description": "Manage FaceHugger animation clips (body_ctrl + 4 foot_targets) using the layered Action API.",
    "category": "Animation",
}

import csv
import math
import os

import bpy

CATEGORY = "FaceHugger"
CLIP_TARGETS = [
    "body_ctrl",
    "foot_target_fl",
    "foot_target_fr",
    "foot_target_bl",
    "foot_target_br",
]
ANCHOR = "body_ctrl"

# Armature object that owns the 12 joint pose bones.
# body_ctrl is a cube Empty used for clip assignment; the real armature is FaceHuggerRig.
_ARM_OBJ_NAME = "FaceHuggerRig"

# Revolute-joint bones in export/heatmap order.
JOINT_BONES = [
    "fl_link1",
    "fl_link2",
    "fl_link3",
    "fr_link1",
    "fr_link2",
    "fr_link3",
    "bl_link1",
    "bl_link2",
    "bl_link3",
    "br_link1",
    "br_link2",
    "br_link3",
]

# Per-bone rotation_euler index derived from URDF joint axes:
#   link1 (shoulder/yaw): axis Z  → euler[2]
#   link2 (hip):          axis Y  → euler[1]
#   link3 (knee):         axis Y  → euler[1]
JOINT_ROT_AXIS = {
    "fl_link1": 2,
    "fl_link2": 1,
    "fl_link3": 1,
    "fr_link1": 2,
    "fr_link2": 1,
    "fr_link3": 1,
    "bl_link1": 2,
    "bl_link2": 1,
    "bl_link3": 1,
    "br_link1": 2,
    "br_link2": 1,
    "br_link3": 1,
}

# Simultaneous-servo warning: flag frames where more than N servos move
# by more than this threshold in a single frame step.
_DELTA_THRESHOLD_DEG = 5.0

# Heatmap colour thresholds (degrees).
_HEATMAP_MID_DEG = 5.0
_HEATMAP_HIGH_DEG = 20.0

# Module-level state for the heatmap handler.
_heatmap_prev_angles: dict = {}


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


def _find_arm_obj():
    """Return the FaceHuggerRig armature object, or None."""
    return bpy.data.objects.get(_ARM_OBJ_NAME)


def _read_bone_angles(arm_eval):
    """Read the revolute-joint angle (deg) for each bone in JOINT_BONES
    from an already-evaluated armature object."""
    angles = {}
    for name in JOINT_BONES:
        pbone = arm_eval.pose.bones.get(name)
        if pbone is not None:
            angles[name] = math.degrees(pbone.rotation_euler[JOINT_ROT_AXIS[name]])
        else:
            angles[name] = 0.0
    return angles


def _set_bone_color(pbone, rgb):
    """Apply an RGB tuple to a pose bone via BoneColor.custom."""
    r, g, b = rgb
    pbone.color.palette = "CUSTOM"
    pbone.color.custom.normal = (r, g, b)
    pbone.color.custom.select = (
        min(r + 0.2, 1.0),
        min(g + 0.2, 1.0),
        min(b + 0.2, 1.0),
    )
    pbone.color.custom.active = (1.0, 1.0, 1.0)


def _delta_to_rgb(delta_deg):
    """Map an angle delta to an RGB colour: green → orange → red."""
    if delta_deg < _HEATMAP_MID_DEG:
        return (0.10, 0.80, 0.10)
    if delta_deg < _HEATMAP_HIGH_DEG:
        t = (delta_deg - _HEATMAP_MID_DEG) / (_HEATMAP_HIGH_DEG - _HEATMAP_MID_DEG)
        return (0.10 + 0.80 * t, 0.80 - 0.65 * t, 0.10)
    return (0.90, 0.15, 0.05)


def _reset_bone_colors():
    """Restore DEFAULT palette on all 12 joint bones."""
    arm_obj = _find_arm_obj()
    if arm_obj is None:
        return
    for name in JOINT_BONES:
        pbone = arm_obj.pose.bones.get(name)
        if pbone is not None:
            pbone.color.palette = "DEFAULT"


# ---------------------------------------------------------------------------
# Heatmap frame-change handler
# ---------------------------------------------------------------------------


def _heatmap_handler(scene, depsgraph):
    arm_obj = _find_arm_obj()
    if arm_obj is None:
        return
    arm_eval = arm_obj.evaluated_get(depsgraph)
    current = _read_bone_angles(arm_eval)
    for name, angle in current.items():
        delta = abs(angle - _heatmap_prev_angles.get(name, angle))
        pbone = arm_obj.pose.bones.get(name)
        if pbone is not None:
            _set_bone_color(pbone, _delta_to_rgb(delta))
    _heatmap_prev_angles.update(current)


def _toggle_heatmap(self, context):
    """BoolProperty update callback — registers or removes _heatmap_handler."""
    handlers = bpy.app.handlers.frame_change_post
    if self.fh_heatmap_active:
        if _heatmap_handler not in handlers:
            handlers.append(_heatmap_handler)
    else:
        if _heatmap_handler in handlers:
            handlers.remove(_heatmap_handler)
        _reset_bone_colors()
        _heatmap_prev_angles.clear()


# ---------------------------------------------------------------------------
# Existing clip helpers (unchanged)
# ---------------------------------------------------------------------------


def _action_clip_name(action_name):
    for target in CLIP_TARGETS:
        suffix = f"__{target}"
        if action_name.endswith(suffix):
            return action_name[: -len(suffix)]
    return None


def list_clips():
    names = set()
    for action in bpy.data.actions:
        clip = _action_clip_name(action.name)
        if clip:
            names.add(clip)
    return sorted(names)


def clip_action(clip, target):
    return bpy.data.actions.get(f"{clip}__{target}")


def active_clip():
    """The clip whose body_ctrl Action is currently assigned, or None."""
    obj = bpy.data.objects.get(ANCHOR)
    if obj is None or obj.animation_data is None or obj.animation_data.action is None:
        return None
    return _action_clip_name(obj.animation_data.action.name)


def assign_clip(clip):
    """Wire each of the 5 Actions onto its target object via the layered API.

    Returns (assigned_count, list_of_missing_strs). Missing entries are
    either 'object:<name>' (rig target absent) or 'action:<full_action_name>'
    (clip incomplete).
    """
    missing = []
    assigned = 0
    for target in CLIP_TARGETS:
        obj = bpy.data.objects.get(target)
        action = clip_action(clip, target)
        if obj is None:
            missing.append(f"object:{target}")
            continue
        if action is None:
            missing.append(f"action:{clip}__{target}")
            continue
        adt = obj.animation_data_create()
        adt.action = action
        slot = action.slots.get(f"OB{target}")
        if slot is None and adt.action_suitable_slots:
            slot = adt.action_suitable_slots[0]
        if slot is not None:
            adt.action_slot = slot
        assigned += 1
    return assigned, missing


def _redraw_view3d(context):
    for area in context.screen.areas:
        if area.type == "VIEW_3D":
            area.tag_redraw()


# ---------------------------------------------------------------------------
# Operators — clip management
# ---------------------------------------------------------------------------


class FH_OT_apply_clip(bpy.types.Operator):
    """Assign all 5 Actions of this clip to body_ctrl + foot_target_*."""

    bl_idname = "fh.apply_clip"
    bl_label = "Apply Clip"
    bl_options = {"REGISTER", "UNDO"}

    clip_name: bpy.props.StringProperty(name="Clip Name")

    def execute(self, context):
        if not self.clip_name:
            self.report({"ERROR"}, "No clip name supplied")
            return {"CANCELLED"}
        assigned, missing = assign_clip(self.clip_name)
        if missing:
            self.report(
                {"WARNING"},
                f"Applied {assigned}/{len(CLIP_TARGETS)} — missing: {', '.join(missing)}",
            )
        else:
            self.report({"INFO"}, f"Applied clip '{self.clip_name}'")
        _redraw_view3d(context)
        return {"FINISHED"}


class FH_OT_duplicate_clip(bpy.types.Operator):
    """Duplicate the active clip's 5 Actions under a new name and apply them."""

    bl_idname = "fh.duplicate_clip"
    bl_label = "Duplicate Active Clip"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        new_name = context.scene.fh_new_clip_name.strip()
        if not new_name:
            self.report({"ERROR"}, "Enter a new clip name first")
            return {"CANCELLED"}
        src = active_clip()
        if src is None:
            self.report({"ERROR"}, f"No active clip on {ANCHOR}")
            return {"CANCELLED"}
        if src == new_name:
            self.report({"ERROR"}, "New name matches the active clip")
            return {"CANCELLED"}
        if any(bpy.data.actions.get(f"{new_name}__{t}") for t in CLIP_TARGETS):
            self.report({"ERROR"}, f"Clip '{new_name}' already exists")
            return {"CANCELLED"}

        copied = 0
        for target in CLIP_TARGETS:
            src_action = clip_action(src, target)
            if src_action is None:
                continue
            dup = src_action.copy()
            dup.name = f"{new_name}__{target}"
            copied += 1

        assigned, missing = assign_clip(new_name)
        if missing:
            self.report(
                {"WARNING"},
                f"Duplicated {copied}/{len(CLIP_TARGETS)}, applied {assigned}/{len(CLIP_TARGETS)} — missing: {', '.join(missing)}",
            )
        else:
            self.report({"INFO"}, f"Duplicated '{src}' → '{new_name}' and applied")
        context.scene.fh_new_clip_name = ""
        _redraw_view3d(context)
        return {"FINISHED"}


class FH_OT_rename_clip(bpy.types.Operator):
    """Rename all 5 Actions of the active clip in one step."""

    bl_idname = "fh.rename_clip"
    bl_label = "Rename Active Clip"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        new_name = context.scene.fh_new_clip_name.strip()
        if not new_name:
            self.report({"ERROR"}, "Enter a new clip name first")
            return {"CANCELLED"}
        src = active_clip()
        if src is None:
            self.report({"ERROR"}, f"No active clip on {ANCHOR}")
            return {"CANCELLED"}
        if src == new_name:
            self.report({"INFO"}, "Name unchanged")
            return {"CANCELLED"}
        if any(bpy.data.actions.get(f"{new_name}__{t}") for t in CLIP_TARGETS):
            self.report({"ERROR"}, f"Clip '{new_name}' already exists")
            return {"CANCELLED"}

        renamed = 0
        for target in CLIP_TARGETS:
            action = clip_action(src, target)
            if action is None:
                continue
            action.name = f"{new_name}__{target}"
            renamed += 1
        self.report(
            {"INFO"},
            f"Renamed '{src}' → '{new_name}' ({renamed}/{len(CLIP_TARGETS)} Actions)",
        )
        context.scene.fh_new_clip_name = ""
        _redraw_view3d(context)
        return {"FINISHED"}


# ---------------------------------------------------------------------------
# Operator — export active clip
# ---------------------------------------------------------------------------


class FH_OT_export_clip(bpy.types.Operator):
    """Bake IK for the active clip and write <clip>.csv + <clip>.h next to the .blend file."""

    bl_idname = "fh.export_clip"
    bl_label = "Export Active Clip"
    bl_options = {"REGISTER"}

    def execute(self, context):
        scene = context.scene

        clip = active_clip()
        if clip is None:
            self.report({"ERROR"}, "No active clip")
            return {"CANCELLED"}

        arm_obj = _find_arm_obj()
        if arm_obj is None:
            self.report({"ERROR"}, f"Armature '{_ARM_OBJ_NAME}' not found in scene")
            return {"CANCELLED"}

        blend_path = bpy.data.filepath
        if not blend_path:
            self.report({"ERROR"}, "Save the .blend file before exporting")
            return {"CANCELLED"}

        export_dir = os.path.dirname(bpy.path.abspath(blend_path))

        action = clip_action(clip, "body_ctrl")
        if action is None:
            self.report({"ERROR"}, f"No body_ctrl action for clip '{clip}'")
            return {"CANCELLED"}

        frame_start = int(action.frame_range[0])
        frame_end = int(action.frame_range[1])
        fps = scene.render.fps / scene.render.fps_base

        # ── Bake: step through every frame, evaluate the depsgraph, read poses ──
        original_frame = scene.frame_current
        rows = []
        for frame in range(frame_start, frame_end + 1):
            scene.frame_set(frame)
            depsgraph = bpy.context.evaluated_depsgraph_get()
            arm_eval = arm_obj.evaluated_get(depsgraph)
            angles = _read_bone_angles(arm_eval)
            time_ms = round((frame - frame_start) / fps * 1000)
            rows.append({"frame": frame, "time_ms": time_ms, **angles})
        scene.frame_set(original_frame)

        # ── Soft guardrail: simultaneous-servo check ─────────────────────────
        max_sim_cap = scene.fh_max_simultaneous_servos
        warning_count = 0
        max_seen = 0
        for i in range(1, len(rows)):
            deltas = [abs(rows[i][b] - rows[i - 1][b]) for b in JOINT_BONES]
            simultaneous = sum(1 for d in deltas if d > _DELTA_THRESHOLD_DEG)
            max_seen = max(max_seen, simultaneous)
            if simultaneous > max_sim_cap:
                warning_count += 1
                print(
                    f"WARNING frame {rows[i]['frame']}: {simultaneous} servos moving "
                    f">{_DELTA_THRESHOLD_DEG}° in one step (limit {max_sim_cap})"
                )

        # ── Phase 1: CSV ─────────────────────────────────────────────────────
        csv_path = os.path.join(export_dir, f"{clip}.csv")
        fieldnames = ["frame", "time_ms"] + JOINT_BONES
        with open(csv_path, "w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(
                    {
                        k: (f"{row[k]:.4f}" if k in JOINT_BONES else row[k])
                        for k in fieldnames
                    }
                )

        # ── Phase 2: C header ─────────────────────────────────────────────────
        h_path = os.path.join(export_dir, f"{clip}.h")
        safe = clip.upper().replace("-", "_").replace(" ", "_")
        guard = f"FH_CLIP_{safe}_H"
        c_sym = f"fh_clip_{clip.lower().replace('-', '_').replace(' ', '_')}"
        frame_count = len(rows)
        bone_count = len(JOINT_BONES)

        h_lines = [
            f"#ifndef {guard}",
            f"#define {guard}",
            "/* Auto-generated by FH Clip Panel — do not edit */",
            f"/* clip: {clip} | frames: {frame_start}..{frame_end} | fps: {fps:.2f} */",
            f"/* bone order: {', '.join(JOINT_BONES)} */",
            "",
            f"#define FH_CLIP_{safe}_FRAMES {frame_count}",
            f"#define FH_CLIP_{safe}_BONES  {bone_count}",
            "",
            f"static const float {c_sym}[{frame_count}][{bone_count}] = {{",
        ]
        for row in rows:
            vals = ", ".join(f"{row[b]:8.4f}f" for b in JOINT_BONES)
            h_lines.append(
                f"    {{{vals}}},  /* f{row['frame']}, t={row['time_ms']}ms */"
            )
        h_lines += ["};", "", f"#endif /* {guard} */", ""]

        with open(h_path, "w") as fh:
            fh.write("\n".join(h_lines))

        summary = (
            f"Exported '{clip}': {frame_count} frames, "
            f"max simultaneous servos: {max_seen}, "
            f"warnings: {warning_count}"
        )
        print(summary)
        self.report({"INFO"}, summary)
        return {"FINISHED"}


# ---------------------------------------------------------------------------
# Panel
# ---------------------------------------------------------------------------


class FH_PT_clip_panel(bpy.types.Panel):
    bl_idname = "VIEW3D_PT_fh_clip_panel"
    bl_label = "FH Clips"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = CATEGORY

    def draw(self, context):
        layout = self.layout
        active = active_clip()

        header = layout.box()
        header.label(text=f"Active: {active or '<none>'}", icon="ACTION")

        layout.label(text="Clips:")
        clips = list_clips()
        if not clips:
            layout.label(text="(no clips found)", icon="INFO")
        else:
            col = layout.column(align=True)
            for clip in clips:
                row = col.row(align=True)
                is_active = clip == active
                icon = "RADIOBUT_ON" if is_active else "RADIOBUT_OFF"
                op = row.operator(
                    FH_OT_apply_clip.bl_idname, text=clip, icon=icon, depress=is_active
                )
                op.clip_name = clip

        layout.separator()
        layout.label(text="New clip name:")
        layout.prop(context.scene, "fh_new_clip_name", text="")
        row = layout.row(align=True)
        row.enabled = bool(active)
        row.operator(FH_OT_duplicate_clip.bl_idname, icon="DUPLICATE")
        row.operator(FH_OT_rename_clip.bl_idname, icon="FONT_DATA")

        # ── Export ────────────────────────────────────────────────────────────
        layout.separator()
        layout.label(text="Export:")
        layout.prop(
            context.scene, "fh_max_simultaneous_servos", text="Max Simultaneous Servos"
        )
        row = layout.row()
        row.enabled = bool(active)
        row.operator(FH_OT_export_clip.bl_idname, icon="EXPORT")

        # ── Heatmap ───────────────────────────────────────────────────────────
        layout.separator()
        layout.prop(
            context.scene,
            "fh_heatmap_active",
            text="Activity Heatmap",
            toggle=True,
            icon="COLORSET_01_VEC",
        )


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

CLASSES = (
    FH_OT_apply_clip,
    FH_OT_duplicate_clip,
    FH_OT_rename_clip,
    FH_OT_export_clip,
    FH_PT_clip_panel,
)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.fh_new_clip_name = bpy.props.StringProperty(
        name="New Clip Name",
        description="Used by Duplicate Clip and Rename Active Clip",
        default="",
    )
    bpy.types.Scene.fh_max_simultaneous_servos = bpy.props.IntProperty(
        name="Max Simultaneous Servos",
        description="Warn during export when more than N servos move >5° in one frame step",
        default=6,
        min=1,
        max=12,
    )
    bpy.types.Scene.fh_heatmap_active = bpy.props.BoolProperty(
        name="Activity Heatmap",
        description="Color joint bones by per-frame angle delta: green=low, orange=mid, red=high",
        default=False,
        update=_toggle_heatmap,
    )


def unregister():
    # Clean up heatmap state before class removal.
    handlers = bpy.app.handlers.frame_change_post
    if _heatmap_handler in handlers:
        handlers.remove(_heatmap_handler)
    _reset_bone_colors()
    _heatmap_prev_angles.clear()

    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
    for prop in ("fh_new_clip_name", "fh_max_simultaneous_servos", "fh_heatmap_active"):
        if hasattr(bpy.types.Scene, prop):
            delattr(bpy.types.Scene, prop)


if __name__ == "__main__":
    try:
        unregister()
    except (RuntimeError, AttributeError):
        pass
    register()
