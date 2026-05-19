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
import datetime
import json
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

# Every joint's rotation axis is bone-local Z (euler index 2), uniformly
# across all 12 bones. The rig builder calls EditBone.align_roll(joint_axis)
# so bone-local Z == the URDF joint axis for ALL joints, including the
# per-side ± sign — see urdf_to_blender_rigged.py and
# code/simulation/docs/API_ANIMATION_SPEC.md §2 ("No per-joint axis
# branching anywhere", "rotation_euler[2] == joint angle"). The URDF axis
# being Y for link2/link3 is the *URDF-frame* axis, NOT the Blender
# bone-local axis: do not branch per-bone here. (A prior per-bone
# {link2/3: Y} map was wrong and silently exported zeros.)
_JOINT_AXIS_EULER_IDX = 2  # bone-local Z, all 12 bones

# Simultaneous-servo warning: flag frames where more than N servos move
# by more than this threshold in a single frame step.
_DELTA_THRESHOLD_DEG = 5.0

# Heatmap colour thresholds (degrees).
_HEATMAP_MID_DEG = 5.0
_HEATMAP_HIGH_DEG = 20.0

# The heatmap colours the per-joint *servo* meshes. Only link1 (shoulder)
# and link3 (knee) export a discrete `<bone>__servo` mesh; link2 (the hip)
# has NO servo mesh — that servo was merged into a parent body during the
# CAD→URDF "combine parts" step. Consequence: the hip's activity is NOT
# visualised (8 of 12 joints are shown). This target was chosen
# deliberately over the joint bones (hidden armature) / leg-segment meshes.
_SERVO_MESH_FMT = "{bone}__servo"
_HEATMAP_NEUTRAL_COLOR = (1.0, 1.0, 1.0, 1.0)

# Module-level state for the heatmap handler / lossless toggle-off.
_heatmap_prev_angles: dict = {}
_heatmap_saved_obj_color: dict = {}  # mesh name -> prior obj.color tuple
_heatmap_saved_shading: list = []  # [(View3DShading, prior color_type)]
_heatmap_hip_gap_warned = False


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


def _find_arm_obj():
    """Return the FaceHuggerRig armature object, or None."""
    return bpy.data.objects.get(_ARM_OBJ_NAME)


def _read_bone_angles(arm_eval):
    """Read the revolute-joint angle (deg) for each bone in JOINT_BONES
    from an already-evaluated armature object.

    The IK / Damped-Track / Limit-Rotation constraint stack writes the
    solved pose into each pose bone's evaluated ``matrix`` (pose/armature
    space) — NOT into ``rotation_euler`` or ``matrix_basis``, which keep
    their pre-constraint keyed values (identity for the IK-driven hip/knee
    bones). Reading ``rotation_euler`` therefore returns 0 for every
    IK-driven joint, which is why the first export wrote all-zero CSVs.

    Instead recover the *effective* basis: the ``matrix_basis`` the bone
    would hold if its evaluated pose had been keyed directly rather than
    constraint-solved. ``Bone.convert_local_to_pose(..., invert=True)`` is
    Blender's official inverse of pose composition (it accounts for
    inherit-scale / local-location). The joint angle is that basis'
    bone-local Z rotation, uniform across all 12 joints because the rig
    roll-aligns every joint axis onto bone-local Z (see
    ``_JOINT_AXIS_EULER_IDX``). For an FK-keyed bone this reduces exactly
    to its keyed ``rotation_euler[2]``, so FK shoulders are unaffected."""
    angles = {}
    for name in JOINT_BONES:
        pbone = arm_eval.pose.bones.get(name)
        if pbone is None:
            angles[name] = 0.0
            continue
        if pbone.parent is not None:
            basis = pbone.bone.convert_local_to_pose(
                pbone.matrix,
                pbone.bone.matrix_local,
                parent_matrix=pbone.parent.matrix,
                parent_matrix_local=pbone.parent.bone.matrix_local,
                invert=True,
            )
        else:
            basis = pbone.bone.convert_local_to_pose(
                pbone.matrix,
                pbone.bone.matrix_local,
                invert=True,
            )
        angles[name] = math.degrees(basis.to_euler("XYZ")[_JOINT_AXIS_EULER_IDX])
    return angles


def _servo_meshes():
    """{bone_name: servo mesh object} for the joints that have a servo
    mesh. link2 (hip) is intentionally absent — it has no `__servo` mesh."""
    out = {}
    for name in JOINT_BONES:
        obj = bpy.data.objects.get(_SERVO_MESH_FMT.format(bone=name))
        if obj is not None:
            out[name] = obj
    return out


def _set_servo_color(obj, rgb):
    """Apply an RGB tuple to a mesh object's viewport display colour."""
    obj.color = (rgb[0], rgb[1], rgb[2], 1.0)


def _iter_view3d_shadings():
    """Yield every View3D space's shading block (none in --background)."""
    for wm in bpy.data.window_managers:
        for win in wm.windows:
            screen = win.screen
            if screen is None:
                continue
            for area in screen.areas:
                if area.type != "VIEW_3D":
                    continue
                for space in area.spaces:
                    if space.type == "VIEW_3D":
                        yield space.shading


def _delta_to_rgb(delta_deg):
    """Map an angle delta to an RGB colour: green → orange → red."""
    if delta_deg < _HEATMAP_MID_DEG:
        return (0.10, 0.80, 0.10)
    if delta_deg < _HEATMAP_HIGH_DEG:
        t = (delta_deg - _HEATMAP_MID_DEG) / (_HEATMAP_HIGH_DEG - _HEATMAP_MID_DEG)
        return (0.10 + 0.80 * t, 0.80 - 0.65 * t, 0.10)
    return (0.90, 0.15, 0.05)


def _enter_servo_heatmap():
    """Snapshot prior servo colours + View3D solid-shading colour mode,
    then switch viewports to per-Object colour so obj.color shows."""
    _heatmap_saved_obj_color.clear()
    for name, obj in _servo_meshes().items():
        _heatmap_saved_obj_color[obj.name] = tuple(obj.color)
    _heatmap_saved_shading.clear()
    for shading in _iter_view3d_shadings():
        _heatmap_saved_shading.append((shading, shading.color_type))
        shading.color_type = "OBJECT"


def _reset_servo_heatmap():
    """Lossless toggle-off: restore servo colours and the prior
    View3D shading colour mode captured by _enter_servo_heatmap."""
    for name, prev in _heatmap_saved_obj_color.items():
        obj = bpy.data.objects.get(name)
        if obj is not None:
            obj.color = prev
    _heatmap_saved_obj_color.clear()
    for shading, prev in _heatmap_saved_shading:
        try:
            shading.color_type = prev
        except (ReferenceError, AttributeError):
            pass  # space/area was closed while the heatmap was active
    _heatmap_saved_shading.clear()


# ---------------------------------------------------------------------------
# Heatmap frame-change handler
# ---------------------------------------------------------------------------


def _heatmap_handler(scene, depsgraph):
    arm_obj = _find_arm_obj()
    if arm_obj is None:
        return
    arm_eval = arm_obj.evaluated_get(depsgraph)
    current = _read_bone_angles(arm_eval)
    servos = _servo_meshes()
    for name, angle in current.items():
        mesh = servos.get(name)  # link2 (hip) has no servo mesh -> skipped
        if mesh is None:
            continue
        delta = abs(angle - _heatmap_prev_angles.get(name, angle))
        _set_servo_color(mesh, _delta_to_rgb(delta))
    _heatmap_prev_angles.update(current)


def _toggle_heatmap(self, context):
    """BoolProperty update callback — registers or removes _heatmap_handler
    and manages the servo-mesh colouring lifecycle."""
    global _heatmap_hip_gap_warned
    handlers = bpy.app.handlers.frame_change_post
    if self.fh_heatmap_active:
        _enter_servo_heatmap()
        if _heatmap_handler not in handlers:
            handlers.append(_heatmap_handler)
        if not _heatmap_hip_gap_warned:
            _heatmap_hip_gap_warned = True
            hip = [b for b in JOINT_BONES if b.endswith("_link2")]
            print(
                "[fh_clip_panel] Activity Heatmap: link2/hip joints "
                f"({', '.join(hip)}) have no `__servo` mesh and are NOT "
                "shown — only the 8 shoulder/knee servos are coloured."
            )
    else:
        if _heatmap_handler in handlers:
            handlers.remove(_heatmap_handler)
        _reset_servo_heatmap()
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


def _autocomplete_clip(clip, context):
    """Ensure `clip` defines all 5 target Actions before it is applied.

    A clip that omits a target (e.g. legacy `base_anim`, which only has
    body_ctrl + foot_target_fl) is NOT switch-safe: Apply leaves that
    target bound to whatever the *previous* clip put there, so you get a
    mixed pose. Here we create the missing `<clip>__<target>` Actions and
    key them once at the NEUTRAL pose, so the target holds neutral across
    the clip instead of inheriting the previous clip. This permanently
    repairs the clip the first time it is applied.

    Neutral = the N-pose foot positions (via _n_pose_foot_targets) for
    foot targets; identity/home for body_ctrl. Returns the list of
    targets that were auto-created."""
    fixable = [
        t
        for t in CLIP_TARGETS
        if bpy.data.objects.get(t) is not None and clip_action(clip, t) is None
    ]
    if not fixable:
        return []

    # N-pose foot positions (only needed/cost if a foot target is missing)
    npos = None
    if any(t.startswith("foot_target_") for t in fixable):
        try:
            arm = _find_arm_obj()
            if arm is not None:
                npos = _pose_foot_targets(arm, _load_convention()["neutral_joint_deg"])
        except (ValueError, KeyError) as e:
            print(
                f"[fh_clip_panel] auto-complete: no N pose ({e}); "
                "keying missing targets at their current transform"
            )

    body_act = clip_action(clip, "body_ctrl")
    frame = int(body_act.frame_range[0]) if body_act else context.scene.frame_start

    repaired = []
    for target in fixable:
        obj = bpy.data.objects[target]
        action = bpy.data.actions.new(f"{clip}__{target}")
        action.slots.new(id_type="OBJECT", name=target)
        adt = obj.animation_data_create()
        adt.action = action
        slot = action.slots.get(f"OB{target}")
        if slot is None and adt.action_suitable_slots:
            slot = adt.action_suitable_slots[0]
        if slot is not None:
            adt.action_slot = slot

        if target == "body_ctrl":
            obj.location = (0.0, 0.0, 0.0)
            obj.rotation_euler = (0.0, 0.0, 0.0)
            obj.keyframe_insert("location", frame=frame)
            obj.keyframe_insert("rotation_euler", frame=frame)
        else:
            leg = target.removeprefix("foot_target_")
            if npos is not None and leg in npos:
                mw = obj.matrix_world.copy()
                mw.translation = npos[leg]
                obj.matrix_world = mw
            obj.keyframe_insert("location", frame=frame)
        repaired.append(target)

    context.view_layer.update()
    return repaired


def _redraw_view3d(context):
    for area in context.screen.areas:
        if area.type == "VIEW_3D":
            area.tag_redraw()


# ---------------------------------------------------------------------------
# Paths + hardware convention (single source of truth: convention.json)
# ---------------------------------------------------------------------------


def _animation_dir():
    """The repo's animation/ directory. Prefer the saved .blend's folder
    (fh_rigged_latest.blend lives in animation/); fall back to this
    script's grandparent (animation/scripts/ -> animation/)."""
    if bpy.data.filepath:
        return os.path.dirname(bpy.path.abspath(bpy.data.filepath))
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _convention_path():
    return os.path.join(_animation_dir(), "convention.json")


def _load_convention():
    """Load animation/convention.json. Raises ValueError on missing/bad
    file so callers can report it. N, SCALE and CHANNELS are NEVER
    hardcoded in Python — this is their only source."""
    path = _convention_path()
    try:
        with open(path) as fh:
            data = json.load(fh)
    except FileNotFoundError as e:
        raise ValueError(f"convention.json not found at {path}") from e
    except json.JSONDecodeError as e:
        raise ValueError(f"convention.json is not valid JSON: {e}") from e
    for key in ("neutral_joint_deg", "scale", "channels"):
        if key not in data:
            raise ValueError(f"convention.json missing required key '{key}'")
    return data


def _exported_gaits_dir(clip_name):
    """animation/exported_gaits/<clip_name>/, created if absent."""
    out = os.path.join(_animation_dir(), "exported_gaits", clip_name)
    os.makedirs(out, exist_ok=True)
    return out


# ---------------------------------------------------------------------------
# N-pose: foot targets from the RIG's own kinematics (not a hand model)
# ---------------------------------------------------------------------------
#
# An earlier version used a hand-rolled planar FK with guessed mount/yaw
# constants. It was wrong by ~340 mm/leg (inverted Z, wrong quadrants:
# legs folded up/in instead of crouching out/down) because its frame and
# sign conventions did not match the rig.
#
# The rig (URDF) is the kinematic source of truth (CLAUDE.md). N in
# convention.json is in the rig's joint-angle space: Convention A makes
# every leg uniform on bone-local Z, and Part 2's `scaled = N+(raw-N)*s`
# means raw == N at the neutral anchor (the per-leg translateToServo
# offsets cancel there). So the correct foot target is simply "where the
# IK chain tip ends up when the 12 joint bones are posed at N".
#
# The IK constraint on `*_link3` uses use_tail=True targeting
# foot_ik (== foot_target), so the IK chain tip IS the link3 bone tail.
# Pose the bones at N with the constraint stack muted (pure FK), read
# each link3 tail in world, restore. Joints whose N exceeds the URDF
# limit are clamped by LIMIT_ROTATION when the IK re-solves — accepted
# as best-effort (the rig cannot represent a pose outside its own
# limits; e.g. hip ±45° vs N≈-60°).

_N_POSE_LINKS = ("link1", "link2", "link3")  # = [shoulder, hip, knee]


def _pose_foot_targets(arm, joint_deg_by_leg):
    """{leg: (x, y, z) mm} world position of each leg's link3 bone tail
    (the IK chain tip) when the 12 joint bones are posed at the given
    per-leg [shoulder, hip, knee] angles (deg) via pure FK. Works for any
    pose — N (convention.json) or flat/rest (all zeros). Restores all
    bone rotations + constraint mute states before returning, so it has
    no lasting effect on the rig."""
    saved_rot = {}
    saved_mute = []
    for leg in ("fl", "fr", "bl", "br"):
        for idx, link in enumerate(_N_POSE_LINKS):
            pb = arm.pose.bones[f"{leg}_{link}"]
            saved_rot[pb.name] = pb.rotation_euler.copy()
            for c in pb.constraints:
                saved_mute.append((c, c.mute))
                c.mute = True
            # joint axis == bone-local Z for every joint (see
            # _JOINT_AXIS_EULER_IDX); angle is that joint's value in deg.
            pb.rotation_euler = (0.0, 0.0, math.radians(joint_deg_by_leg[leg][idx]))
    bpy.context.view_layer.update()

    out = {}
    for leg in ("fl", "fr", "bl", "br"):
        pb = arm.pose.bones[f"{leg}_link3"]
        w = arm.matrix_world @ pb.tail  # link3 tail in world (mm)
        out[leg] = (w.x, w.y, w.z)

    for c, mute in saved_mute:
        c.mute = mute
    for name, rot in saved_rot.items():
        arm.pose.bones[name].rotation_euler = rot
    bpy.context.view_layer.update()
    return out


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
        # Make the clip switch-safe: fill any target it doesn't define
        # with a neutral-pose Action so it can't inherit the previous
        # clip's motion on those targets.
        repaired = _autocomplete_clip(self.clip_name, context)
        assigned, missing = assign_clip(self.clip_name)
        if missing:
            self.report(
                {"WARNING"},
                f"Applied {assigned}/{len(CLIP_TARGETS)} — missing: "
                f"{', '.join(missing)}",
            )
        elif repaired:
            self.report(
                {"INFO"},
                f"Applied clip '{self.clip_name}' — auto-completed "
                f"{len(repaired)} missing target(s) at neutral: "
                f"{', '.join(repaired)}",
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


class FH_OT_save_as_clip(bpy.types.Operator):
    """Snapshot the animation CURRENTLY bound to the 5 control objects
    into a new clip and switch to it.

    Reads each object's live `animation_data.action` (whatever its name)
    and copies it to `<new_name>__<target>`, so it works even when no
    clip name resolves (e.g. you started keyframing on a fresh rig, or
    the bound action names drifted from the convention). The source
    actions are left bound to nothing afterwards (the new copies are
    applied); they are not deleted.

    NOTE: editing while a clip is active mutates THAT clip's actions in
    place — so if you animated on top of `base_anim`, this saves your
    work to the new clip but `base_anim` already changed too. To branch
    cleanly, Save/Duplicate BEFORE animating."""

    bl_idname = "fh.save_as_clip"
    bl_label = "Save Current → New Clip"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        new_name = context.scene.fh_new_clip_name.strip()
        if not new_name:
            self.report({"ERROR"}, "Enter a new clip name first")
            return {"CANCELLED"}
        if any(bpy.data.actions.get(f"{new_name}__{t}") for t in CLIP_TARGETS):
            self.report({"ERROR"}, f"Clip '{new_name}' already exists")
            return {"CANCELLED"}

        # The clip that was active *before* we re-bind. Editing while a
        # clip is active mutates ITS actions in place, so this source
        # clip now also carries the edits — we cannot revert it (no
        # pre-edit snapshot exists), so warn precisely instead.
        src = active_clip()

        copied = 0
        no_anim = []
        for target in CLIP_TARGETS:
            obj = bpy.data.objects.get(target)
            if obj is None:
                no_anim.append(f"object:{target}")
                continue
            ad = obj.animation_data
            src_action = ad.action if ad is not None else None
            if src_action is None:
                no_anim.append(f"no-action:{target}")
                continue
            dup = src_action.copy()
            dup.name = f"{new_name}__{target}"
            copied += 1

        if copied == 0:
            self.report(
                {"ERROR"},
                "Nothing to save — no actions are bound to the control "
                f"objects ({', '.join(no_anim)})",
            )
            return {"CANCELLED"}

        assigned, missing = assign_clip(new_name)

        # If we snapshotted edits made on top of an existing clip, that
        # clip was mutated in place. We can't revert it, so tell the
        # animator exactly how to restore it.
        dirtied = src if (src and src != new_name) else None
        if dirtied:
            print(
                f"[fh_clip_panel] NOTE: '{dirtied}' was edited in place and "
                f"now also contains these changes. To restore it: Apply "
                f"'{dirtied}', go to frame 1, click Set N Pose, then re-key "
                f"frame 1 (or re-import the rig). Tip: Save/Duplicate BEFORE "
                f"animating to branch cleanly."
            )

        if no_anim or missing:
            self.report(
                {"WARNING"},
                f"Saved {copied}/{len(CLIP_TARGETS)} to '{new_name}' "
                f"(applied {assigned}/{len(CLIP_TARGETS)}, "
                f"skipped {', '.join(no_anim) or 'none'})",
            )
        elif dirtied:
            self.report(
                {"WARNING"},
                f"Saved → '{new_name}'. NOTE: '{dirtied}' was edited in "
                f"place & now holds these changes too — re-key its frame 1 "
                f"to N to restore (see console for steps).",
            )
        else:
            self.report(
                {"INFO"},
                f"Saved current animation → clip '{new_name}' and applied",
            )
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


class FH_OT_new_clip(bpy.types.Operator):
    """Create a brand-new clip from scratch: 5 fresh Actions (one per
    control object) keyed at the current pose, then apply them. The only
    authoring entry point that does NOT need an existing active clip —
    Duplicate and Rename both do, so this breaks the zero-clips
    chicken-and-egg."""

    bl_idname = "fh.new_clip"
    bl_label = "New Clip"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        new_name = context.scene.fh_new_clip_name.strip()
        if not new_name:
            self.report({"ERROR"}, "Enter a new clip name first")
            return {"CANCELLED"}
        if any(bpy.data.actions.get(f"{new_name}__{t}") for t in CLIP_TARGETS):
            self.report({"ERROR"}, f"Clip '{new_name}' already exists")
            return {"CANCELLED"}
        missing_objs = [t for t in CLIP_TARGETS if bpy.data.objects.get(t) is None]
        if missing_objs:
            self.report(
                {"ERROR"},
                f"Cannot create clip — missing rig objects: {', '.join(missing_objs)}",
            )
            return {"CANCELLED"}

        # One fresh Action + OBJECT slot per control object. The slot
        # identifier auto-becomes `OB<target>` — exactly what assign_clip()
        # looks up, so the wiring is identical to every other clip.
        for target in CLIP_TARGETS:
            action = bpy.data.actions.new(f"{new_name}__{target}")
            action.slots.new(id_type="OBJECT", name=target)

        assigned, missing = assign_clip(new_name)

        # Key the current pose so the clip is non-empty and starts at the
        # pose the animator is looking at (rest pose by convention — see
        # API_ANIMATION_SPEC.md §4). Keys land in the just-assigned
        # action/slot of each object.
        frame = context.scene.frame_current
        keyed = 0
        for target in CLIP_TARGETS:
            obj = bpy.data.objects.get(target)
            if obj is None:
                continue
            obj.keyframe_insert("location", frame=frame)
            if obj.rotation_mode == "QUATERNION":
                obj.keyframe_insert("rotation_quaternion", frame=frame)
            elif obj.rotation_mode == "AXIS_ANGLE":
                obj.keyframe_insert("rotation_axis_angle", frame=frame)
            else:
                obj.keyframe_insert("rotation_euler", frame=frame)
            keyed += 1

        if missing:
            self.report(
                {"WARNING"},
                f"Created '{new_name}' ({assigned}/{len(CLIP_TARGETS)} "
                f"applied, {keyed} keyed) — missing: {', '.join(missing)}",
            )
        else:
            self.report(
                {"INFO"},
                f"Created clip '{new_name}' — {keyed} objects keyed at frame {frame}",
            )
        context.scene.fh_new_clip_name = ""
        _redraw_view3d(context)
        return {"FINISHED"}


def _drive_to_joint_pose(context, joint_deg_by_leg, label):
    """Move the 4 foot targets so the live IK solves the rig to
    `joint_deg_by_leg` ({leg: [shoulder, hip, knee] deg}). Returns
    (status_set, report_level, report_msg). Joints whose target exceeds
    the URDF LIMIT_ROTATION range are clamped on the re-solve
    (best-effort) and the residual is reported."""
    arm = _find_arm_obj()
    if arm is None:
        return {"CANCELLED"}, "ERROR", f"Armature '{_ARM_OBJ_NAME}' not found"

    targets = _pose_foot_targets(arm, joint_deg_by_leg)
    moved, missing = [], []
    for leg, (fx, fy, fz) in targets.items():
        obj = bpy.data.objects.get(f"foot_target_{leg}")
        if obj is None:
            missing.append(f"foot_target_{leg}")
            continue
        # Set the WORLD position robustly whether or not the target is
        # parented (rig parents it to world_origin): edit a copy of
        # matrix_world and assign it back so Blender re-derives the
        # local transform through the parent inverse.
        mw = obj.matrix_world.copy()
        mw.translation = (fx, fy, fz)
        obj.matrix_world = mw
        moved.append(leg)

    context.view_layer.update()  # let the IK solve to the new targets
    _redraw_view3d(context)

    # Residual: how far the live (possibly clamped) pose lands from the
    # requested angles. ~0 for the flat pose (0 is in range); non-zero
    # for N where it exceeds the URDF limits (hips ±45° vs N≈-60°, ...).
    dg = context.evaluated_depsgraph_get()
    ang = _read_bone_angles(arm.evaluated_get(dg))
    worst = 0.0
    for leg in ("fl", "fr", "bl", "br"):
        for idx, link in enumerate(_N_POSE_LINKS):
            worst = max(worst, abs(ang[f"{leg}_{link}"] - joint_deg_by_leg[leg][idx]))

    if missing:
        return (
            {"FINISHED"},
            "WARNING",
            f"Set {label} for {len(moved)}/4 legs — missing: {', '.join(missing)}",
        )
    if worst > 2.0:
        return (
            {"FINISHED"},
            "WARNING",
            f"Rig set to {label} (best-effort: up to {worst:.0f}° residual "
            "— URDF limit clamping and/or the rig's IK/Damped-Track "
            "foot-target fidelity; shoulders carry a small uniform offset)",
        )
    return {"FINISHED"}, "INFO", f"Rig set to {label}"


class FH_OT_set_n_pose(bpy.types.Operator):
    """Move the foot targets to the hardware-neutral (N) standing pose so
    the IK solves to the robot's real starting position. Click before
    authoring a clip so frame 1 matches hardware. N comes from
    convention.json."""

    bl_idname = "fh.set_n_pose"
    bl_label = "Set N Pose"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        try:
            jd = _load_convention()["neutral_joint_deg"]
        except (ValueError, KeyError) as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        status, level, msg = _drive_to_joint_pose(
            context, jd, "N pose (hardware neutral)"
        )
        self.report({level}, msg)
        return status


class FH_OT_set_rest_pose(bpy.types.Operator):
    """Move the foot targets to the FLAT base pose — the URDF rest (all
    joint angles 0; the splayed stance the rig builder sets on every
    `--rigged` rebuild). Use it to snap the rig back to the known
    baseline. No convention.json needed (it's all zeros)."""

    bl_idname = "fh.set_rest_pose"
    bl_label = "Set Flat Pose"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        jd = {leg: (0.0, 0.0, 0.0) for leg in ("fl", "fr", "bl", "br")}
        status, level, msg = _drive_to_joint_pose(
            context, jd, "flat/rest pose (URDF θ=0)"
        )
        self.report({level}, msg)
        return status


# ---------------------------------------------------------------------------
# LAYER 1 — pure Blender data extraction (no hardware knowledge)
# ---------------------------------------------------------------------------


def bake_clip(clip_name, context):
    """Step through every frame of `clip_name`, evaluate the depsgraph and
    read the IK-solved joint angles. Returns a list of row dicts:

        [{"frame": 1, "time_ms": 0, "fl_link1": 0.0, "fl_link2": 0.0, ...},
         ...]

    Pure data extraction — knows nothing about servos, scaling or
    channels. Raises ValueError if the rig/clip is not exportable."""
    scene = context.scene
    arm_obj = _find_arm_obj()
    if arm_obj is None:
        raise ValueError(f"Armature '{_ARM_OBJ_NAME}' not found in scene")
    action = clip_action(clip_name, "body_ctrl")
    if action is None:
        raise ValueError(f"No body_ctrl action for clip '{clip_name}'")

    frame_start = int(action.frame_range[0])
    frame_end = int(action.frame_range[1])
    fps = scene.render.fps / scene.render.fps_base

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
    return rows


# ---------------------------------------------------------------------------
# LAYER 2 — converters: baked rows -> animation/exported_gaits/<clip>/
# ---------------------------------------------------------------------------

_LEGS = ("fr", "fl", "br", "bl")  # JS playback / channel order


def to_csv(frames, clip_name):
    """Raw bone angles, one row per frame (unchanged legacy format)."""
    path = os.path.join(_exported_gaits_dir(clip_name), f"{clip_name}.csv")
    fieldnames = ["frame", "time_ms"] + JOINT_BONES
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for row in frames:
            writer.writerow(
                {
                    k: (f"{row[k]:.4f}" if k in JOINT_BONES else row[k])
                    for k in fieldnames
                }
            )
    return path


def to_c_header(frames, clip_name, convention):
    """Static C array of RAW bone angles (unchanged legacy format). The
    `convention` argument is accepted for a uniform converter signature;
    the firmware header intentionally stays raw — hardware conversion is
    the JS layer's job, not the C array's."""
    del convention  # intentionally unused — header stays raw
    path = os.path.join(_exported_gaits_dir(clip_name), f"{clip_name}.h")
    safe = clip_name.upper().replace("-", "_").replace(" ", "_")
    guard = f"FH_CLIP_{safe}_H"
    c_sym = f"fh_clip_{clip_name.lower().replace('-', '_').replace(' ', '_')}"
    frame_count = len(frames)
    bone_count = len(JOINT_BONES)
    fr0 = frames[0]["frame"] if frames else 0
    fr1 = frames[-1]["frame"] if frames else 0

    h_lines = [
        f"#ifndef {guard}",
        f"#define {guard}",
        "/* Auto-generated by FH Clip Panel — do not edit */",
        f"/* clip: {clip_name} | frames: {fr0}..{fr1} */",
        f"/* bone order: {', '.join(JOINT_BONES)} */",
        "",
        f"#define FH_CLIP_{safe}_FRAMES {frame_count}",
        f"#define FH_CLIP_{safe}_BONES  {bone_count}",
        "",
        f"static const float {c_sym}[{frame_count}][{bone_count}] = {{",
    ]
    for row in frames:
        vals = ", ".join(f"{row[b]:8.4f}f" for b in JOINT_BONES)
        h_lines.append(f"    {{{vals}}},  /* f{row['frame']}, t={row['time_ms']}ms */")
    h_lines += ["};", "", f"#endif /* {guard} */", ""]
    with open(path, "w") as fh:
        fh.write("\n".join(h_lines))
    return path


def _frame_to_servo(row, convention):
    """One baked row -> {leg: [servo0, servo1, servo2] ints} applying:
    (1) scale-from-neutral, (2) translateToServo, (3) round."""
    neutral = convention["neutral_joint_deg"]
    scale = convention["scale"]
    out = {}
    for leg in _LEGS:
        n = neutral[leg]  # [shoulder, hip, knee] deg at hardware neutral
        raw = [row[f"{leg}_link1"], row[f"{leg}_link2"], row[f"{leg}_link3"]]
        # 1. compress movement toward N
        sh, th, kn = (n[j] + (raw[j] - n[j]) * scale for j in range(3))
        # 2. translateToServo (mirror/offset per leg side)
        if leg == "fl":
            servo = [sh, 90 + th, 90 - kn]
        elif leg == "fr":
            servo = [90 + (sh - 45), 90 - th, 90 + kn]
        elif leg == "bl":
            servo = [90 + (sh + 135), 90 - th, 90 + kn]
        else:  # br
            servo = [90 - (sh + 45), 90 + th, 90 - kn]
        # 3. integer servo degrees
        out[leg] = [round(v) for v in servo]
    return out


def to_js(frames, clip_name, convention):
    """Self-contained browser-console JS that plays the clip live over
    WebSocket. Applies the full hardware conversion (scale-from-neutral,
    translateToServo, round). N/SCALE/CHANNELS come from convention.json."""
    path = os.path.join(_exported_gaits_dir(clip_name), f"{clip_name}.js")
    ch = convention["channels"]
    today = datetime.date.today().isoformat()

    clip_lines = []
    for row in frames:
        s = _frame_to_servo(row, convention)
        legs = ", ".join(
            f"{leg}:[{s[leg][0]},{s[leg][1]},{s[leg][2]}]" for leg in _LEGS
        )
        clip_lines.append(f"  {{ t: {row['time_ms']}, {legs} }},")

    ch_str = ", ".join(
        f"{leg}:[{ch[leg][0]},{ch[leg][1]},{ch[leg][2]}]" for leg in _LEGS
    )

    js = f"""// FaceHugger clip: {clip_name}
// Generated {today} from Blender animation
// Paste into browser console while connected to
// FaceHugger_Net (ws://192.168.4.1:81) to play

const ws = new WebSocket("ws://192.168.4.1:81");
const CHANNELS = {{ {ch_str} }};

const CLIP = [
{chr(10).join(clip_lines)}
];

let i = 0;
function playFrame() {{
  if (i >= CLIP.length) {{ i = 0; }}
  const frame = CLIP[i++];
  for (const leg of ["fr", "fl", "br", "bl"]) {{
    const angles = frame[leg];
    for (let j = 0; j < 3; j++) {{
      if (ws.readyState === WebSocket.OPEN) {{
        ws.send(JSON.stringify({{
          "T": 4,
          "id": CHANNELS[leg][j],
          "a": angles[j]
        }}));
      }}
    }}
  }}
}}

ws.onopen = () => {{
  console.log("Connected. Playing {clip_name}...");
  setInterval(playFrame, 30);
}};
"""
    with open(path, "w") as fh:
        fh.write(js)
    return path


# ---------------------------------------------------------------------------
# Operator — export active clip
# ---------------------------------------------------------------------------


class FH_OT_export_clip(bpy.types.Operator):
    """Bake the active clip once, then write the enabled outputs to
    animation/exported_gaits/<clip>/ (.csv / .h / .js)."""

    bl_idname = "fh.export_clip"
    bl_label = "Export Active Clip"
    bl_options = {"REGISTER"}

    def execute(self, context):
        scene = context.scene

        clip = active_clip()
        if clip is None:
            self.report({"ERROR"}, "No active clip")
            return {"CANCELLED"}
        if not bpy.data.filepath:
            self.report({"ERROR"}, "Save the .blend file before exporting")
            return {"CANCELLED"}

        # ── LAYER 1: bake once ───────────────────────────────────────────────
        try:
            rows = bake_clip(clip, context)
        except ValueError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        if not rows:
            self.report({"ERROR"}, f"Clip '{clip}' produced no frames")
            return {"CANCELLED"}

        # ── Soft guardrail: simultaneous-servo check (report only) ───────────
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
                    f"WARNING frame {rows[i]['frame']}: {simultaneous} servos "
                    f"moving >{_DELTA_THRESHOLD_DEG}° in one step "
                    f"(limit {max_sim_cap})"
                )

        # convention only needed by the .h / .js converters
        convention = None
        if scene.fh_export_header or scene.fh_export_js:
            try:
                convention = _load_convention()
            except ValueError as e:
                self.report({"ERROR"}, str(e))
                return {"CANCELLED"}

        # ── LAYER 2: run the enabled converters ──────────────────────────────
        written = []
        if scene.fh_export_csv:
            to_csv(rows, clip)
            written.append("csv")
        if scene.fh_export_header:
            to_c_header(rows, clip, convention)
            written.append("h")
        if scene.fh_export_js:
            to_js(rows, clip, convention)
            written.append("js")

        if not written:
            self.report({"WARNING"}, "No export formats enabled")
            return {"CANCELLED"}

        rel = os.path.join("animation", "exported_gaits", clip)
        summary = (
            f"Exported '{clip}' ({len(rows)} frames, "
            f"{', '.join(written)}) → {rel}/  "
            f"[max simultaneous servos: {max_seen}, warnings: {warning_count}]"
        )
        print(summary)
        self.report({"INFO"}, f"Exported to {rel}/ ({', '.join(written)})")
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
        # New Clip / Save Current work with zero existing clips (always
        # enabled); Duplicate/Rename act on the active clip.
        layout.operator(FH_OT_new_clip.bl_idname, icon="ADD")
        layout.operator(FH_OT_save_as_clip.bl_idname, icon="FILE_TICK")
        pose_row = layout.row(align=True)
        pose_row.operator(FH_OT_set_n_pose.bl_idname, icon="ARMATURE_DATA")
        pose_row.operator(FH_OT_set_rest_pose.bl_idname, icon="MOD_ARMATURE")
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
        col = layout.column(align=True)
        col.prop(context.scene, "fh_export_csv", text="CSV (raw angles)")
        col.prop(context.scene, "fh_export_header", text="C header (.h)")
        col.prop(context.scene, "fh_export_js", text="Browser JS (.js)")
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
    FH_OT_new_clip,
    FH_OT_save_as_clip,
    FH_OT_set_n_pose,
    FH_OT_set_rest_pose,
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
        description="Used by New Clip, Duplicate Clip and Rename Active Clip",
        default="",
    )
    bpy.types.Scene.fh_max_simultaneous_servos = bpy.props.IntProperty(
        name="Max Simultaneous Servos",
        description="Warn during export when more than N servos move >5° in one frame step",
        default=6,
        min=1,
        max=12,
    )
    bpy.types.Scene.fh_export_csv = bpy.props.BoolProperty(
        name="Export CSV",
        description="Write the raw bone-angle CSV",
        default=True,
    )
    bpy.types.Scene.fh_export_header = bpy.props.BoolProperty(
        name="Export C Header",
        description="Write the static C array (.h) of raw bone angles",
        default=True,
    )
    bpy.types.Scene.fh_export_js = bpy.props.BoolProperty(
        name="Export Browser JS",
        description=(
            "Write a self-contained .js that plays the clip live over "
            "WebSocket (full hardware conversion from convention.json)"
        ),
        default=True,
    )
    bpy.types.Scene.fh_heatmap_active = bpy.props.BoolProperty(
        name="Activity Heatmap",
        description=(
            "Color the per-joint servo meshes by per-frame angle delta "
            "(green=low, orange=mid, red=high). Note: link2/hip has no "
            "servo mesh, so 8 of 12 joints are shown. Switches viewports "
            "to Object colour while active; restored on toggle-off"
        ),
        default=False,
        update=_toggle_heatmap,
    )


def unregister():
    # Clean up heatmap state before class removal.
    handlers = bpy.app.handlers.frame_change_post
    if _heatmap_handler in handlers:
        handlers.remove(_heatmap_handler)
    _reset_servo_heatmap()
    _heatmap_prev_angles.clear()

    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
    for prop in (
        "fh_new_clip_name",
        "fh_max_simultaneous_servos",
        "fh_export_csv",
        "fh_export_header",
        "fh_export_js",
        "fh_heatmap_active",
    ):
        if hasattr(bpy.types.Scene, prop):
            delattr(bpy.types.Scene, prop)


if __name__ == "__main__":
    try:
        unregister()
    except (RuntimeError, AttributeError):
        pass
    register()
