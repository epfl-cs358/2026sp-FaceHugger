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
    "version": (0, 1, 0),
    "blender": (4, 4, 0),
    "location": "View3D > Sidebar > FaceHugger",
    "description": "Manage FaceHugger animation clips (body_ctrl + 4 foot_targets) using the layered Action API.",
    "category": "Animation",
}

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


CLASSES = (
    FH_OT_apply_clip,
    FH_OT_duplicate_clip,
    FH_OT_rename_clip,
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


def unregister():
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
    if hasattr(bpy.types.Scene, "fh_new_clip_name"):
        del bpy.types.Scene.fh_new_clip_name


if __name__ == "__main__":
    try:
        unregister()
    except (RuntimeError, AttributeError):
        pass
    register()
