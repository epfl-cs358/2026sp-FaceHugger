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
import importlib
import json
import math
import os
import sys
from pathlib import Path

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

# Authoring-time torque guard: warn if any joint moves more than this many
# degrees between consecutive baked frames (fast keyframe = mechanical shock
# on hardware). Tunable; not a hard limit — the firmware clamp is the
# electrical backstop, this catches bad animation before it ships.
FRAME_DELTA_WARN_DEG = 20.0

# Recommended max authoring fps for robot export. Higher = more WS packets
# per second with no motion benefit for these slow clips. Not enforced —
# the animator sets fps in Output Properties; we only nudge.
RECOMMENDED_MAX_FPS = 12

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


def _warn_frame_deltas(rows):
    """Print a console WARNING for each joint that moves more than
    FRAME_DELTA_WARN_DEG between consecutive frames. Returns the number of
    warnings emitted (for tests)."""
    n = 0
    for prev, cur in zip(rows, rows[1:]):
        for bone in JOINT_BONES:
            delta = abs(cur[bone] - prev[bone])
            if delta > FRAME_DELTA_WARN_DEG:
                print(
                    f"WARNING: {bone} moves {delta:.0f}° between frame "
                    f"{prev['frame']} and {cur['frame']} "
                    f"(threshold: {FRAME_DELTA_WARN_DEG:.0f}°)"
                )
                n += 1
    return n


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
    """Strip a `__<target>` suffix from an Action name to recover the
    clip prefix. Returns None for anything that doesn't look like a
    well-formed clip action — including the cascading-suffix case
    (e.g. an orphan Action literally named "__foot_target_fl__foot_target_fl"
    that would otherwise leak the would-be prefix "__foot_target_fl"
    into list_clips() and the export UI). Legitimate clip names don't
    start with "__"."""
    for target in CLIP_TARGETS:
        suffix = f"__{target}"
        if action_name.endswith(suffix):
            clip = action_name[: -len(suffix)]
            if not clip or clip.startswith("__"):
                return None  # malformed / phantom
            return clip
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


def _keep_action(action):
    """Mark a clip Action with the fake-user flag so it survives
    Blender's orphan-purge-on-save when it's not the currently-bound
    one. The user model is "all my clips persist regardless of which
    one is active" — without this, every non-active clip has zero
    users and gets garbage-collected on the next .blend save."""
    if action is not None:
        action.use_fake_user = True
    return action


def _rescue_existing_clip_actions():
    """Sweep bpy.data.actions on register, mark every clip-pattern
    Action (`<clip>__<target>`) as fake_user=True. Rescues clips that
    are still in the current .blend but were created before this fix —
    without the sweep they'd be lost on the very next save."""
    rescued = 0
    for action in bpy.data.actions:
        if _action_clip_name(action.name) is None:
            continue
        if not action.use_fake_user:
            action.use_fake_user = True
            rescued += 1
    return rescued


@bpy.app.handlers.persistent
def _on_save_pre(*_args, **_kwargs):
    """Re-sweep clip-pattern actions for fake_user RIGHT BEFORE Blender
    serialises to disk. This is the final guard against the inactive-
    clip-vanishes bug: even if some other code path (rig rebuild's
    restore_actions, a misbehaving extension, a fresh-Action creation
    site that bypasses `_keep_action`) clears use_fake_user mid-session,
    save_pre flips it back on the way to disk. Persistent so the
    handler survives .blend loads."""
    n = _rescue_existing_clip_actions()
    if n:
        print(
            f"[fh_clip_panel] save_pre: re-set fake_user on {n} clip"
            f" Action(s) to prevent orphan-purge during save"
        )


@bpy.app.handlers.persistent
def _on_load_post(*_args, **_kwargs):
    """Re-sweep clip-pattern actions after every .blend load — covers
    the case where the user opens a second file in the same Blender
    session (register() only runs once per addon lifetime, so the
    register-time sweep wouldn't fire). Persistent so the handler
    survives .blend loads."""
    n = _rescue_existing_clip_actions()
    if n:
        print(f"[fh_clip_panel] load_post: re-set fake_user on {n} clip Action(s)")


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
        action = _keep_action(bpy.data.actions.new(f"{clip}__{target}"))
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
    """Force panels to redraw so they reflect data changes immediately.

    Tags every area in every open window — not just one VIEW_3D via
    `context.screen` — because the Clips panel's per-row include/exclude
    checkbox is an operator button whose icon is computed in draw(); without a
    redraw of the N-panel's region it keeps a stale visual (the toggle data
    updates, the tick doesn't move) until the next mouse event."""
    wm = getattr(bpy.context, "window_manager", None)
    if wm is None:
        if context.screen is not None:
            for area in context.screen.areas:
                area.tag_redraw()
        return
    for window in wm.windows:
        for area in window.screen.areas:
            area.tag_redraw()


# ---------------------------------------------------------------------------
# Per-clip export-include state (Option A: inline checkbox in the Clips list)
# ---------------------------------------------------------------------------
#
# The set of clips ticked for "Export Selected Clips" is stored as a
# newline-joined string on the scene (`fh_export_selected_clips`).
# Clip names can contain spaces ("Lie flat and stand up") but never
# newlines, so '\n' is the safe delimiter. Reads always intersect with
# the live clip set so stale entries (clip deleted/renamed elsewhere)
# auto-prune; writes always sort for stable diffs.


def _export_selected_set():
    """Return the set of clip names currently ticked for selective export,
    intersected with `list_clips()` so stale entries auto-prune."""
    raw = bpy.context.scene.fh_export_selected_clips
    if not raw:
        return set()
    return {n for n in raw.split("\n") if n} & set(list_clips())


def _write_export_selected(names):
    """Persist `names` (an iterable of clip names) to the scene prop.
    Sorted + newline-joined for stable diffs."""
    bpy.context.scene.fh_export_selected_clips = "\n".join(sorted(set(names)))


def _rename_export_selected(old, new):
    """If `old` is currently ticked, swap it for `new`. Called when a
    clip is renamed so the export-selection follows the rename."""
    raw = bpy.context.scene.fh_export_selected_clips
    if not raw:
        return
    names = {n for n in raw.split("\n") if n}
    if old in names:
        names.discard(old)
        names.add(new)
        _write_export_selected(names)


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


def _exported_clips_dir(clip_name):
    """animation/exported_clips/<clip_name>/, created if absent."""
    out = os.path.join(_animation_dir(), "exported_clips", clip_name)
    os.makedirs(out, exist_ok=True)
    return out


def _exported_clips_root():
    """animation/exported_clips/ (where the bundle clips_all.h lands)."""
    return os.path.join(_animation_dir(), "exported_clips")


def _repo_root():
    """Repo root = the parent of animation/. The add-on assumes it runs from
    a checkout (the .blend lives in animation/), which is the normal case."""
    return os.path.dirname(_animation_dir())


def _firmware_clips_dir():
    """The firmware dir that holds clips_all.h (the on-board source of truth)."""
    return os.path.join(_repo_root(), "code", "firmware", "src", "nervous_system")


def _copy_bundle_to_firmware():
    """Copy the freshly written exported_clips/clips_all.h over the firmware
    copy. The firmware header is the same self-contained bundle format the
    exporter writes, so this is a plain file copy. Returns the destination
    path on success; raises ValueError (no bundle, or not a repo checkout) so
    the caller can warn without failing the export."""
    import shutil

    src = os.path.join(_exported_clips_root(), "clips_all.h")
    if not os.path.exists(src):
        raise ValueError(f"no bundle to copy at {src}")
    dest_dir = _firmware_clips_dir()
    if not os.path.isdir(dest_dir):
        raise ValueError(
            f"firmware dir not found ({dest_dir}); not in a repo checkout?"
        )
    dest = os.path.join(dest_dir, "clips_all.h")
    shutil.copyfile(src, dest)
    return dest


def _app_clips_dir():
    """The remote-control app's asset dir, where clips_extra.json is bundled so
    the app can stream clips that aren't flashed (see to_clips_extra_json)."""
    return os.path.join(_repo_root(), "code", "remote-control-app", "MyApp", "assets")


def _copy_extra_to_app():
    """Copy the freshly written exported_clips/clips_extra.json into the app's
    assets dir (the app bundles it at build time). Returns the destination path;
    raises ValueError (no bundle, or not a repo checkout) so the caller can warn
    without failing the export. Mirrors _copy_bundle_to_firmware for the app."""
    import shutil

    src = os.path.join(_exported_clips_root(), "clips_extra.json")
    if not os.path.exists(src):
        raise ValueError(f"no clips_extra.json to copy at {src}")
    dest_dir = _app_clips_dir()
    if not os.path.isdir(dest_dir):
        raise ValueError(
            f"app assets dir not found ({dest_dir}); not in a repo checkout?"
        )
    dest = os.path.join(dest_dir, "clips_extra.json")
    shutil.copyfile(src, dest)
    return dest


# ---------------------------------------------------------------------------
# Pose Library — animation/poses.json (committed, like convention.json)
# ---------------------------------------------------------------------------
#
# A "pose" is a snapshot of the 5 control objects' LOCAL transforms:
# body_ctrl gets loc+rot, the 4 foot_target_* get loc only (the IK reads
# only foot-target *position*). Local transforms are stored so a pose
# round-trips with keyframes — "Key into Clip" inserts the same local
# values the clip Actions hold. JSON shape:
#
#   { "<pose>": { "body_ctrl": {"loc":[3], "rot":[3]},
#                 "foot_target_fl": {"loc":[3]}, ... } }
#
# Poses are independent of clips: applying a pose sets transforms ONLY —
# no keyframes, no Action assignment, no clip side effects.


def _poses_path():
    return os.path.join(_animation_dir(), "poses.json")


def _load_poses():
    """Load animation/poses.json -> {name: pose-dict}. A missing file is
    not an error (nothing saved yet) -> {}. Corrupt JSON raises ValueError
    so the operator can surface it rather than silently losing poses."""
    path = _poses_path()
    try:
        with open(path) as fh:
            data = json.load(fh)
    except FileNotFoundError:
        return {}
    except json.JSONDecodeError as e:
        raise ValueError(f"poses.json is not valid JSON: {e}") from e
    return data if isinstance(data, dict) else {}


def _save_poses(data):
    """Write poses.json with stable formatting (sorted keys, indent=2,
    trailing newline) so it diffs cleanly — it's a committed file."""
    path = _poses_path()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        json.dump(data, fh, indent=2, sort_keys=True)
        fh.write("\n")


def _capture_pose():
    """Snapshot the live LOCAL transforms of the control objects:
    body_ctrl -> loc+rot, the 4 foot targets -> loc only. Values are
    rounded to 6 dp (sub-micron at the rig's mm scale) to keep poses.json
    diffs stable. Controls absent from the scene are omitted."""
    pose = {}
    for target in CLIP_TARGETS:
        obj = bpy.data.objects.get(target)
        if obj is None:
            continue
        if target == ANCHOR:
            pose[target] = {
                "loc": [round(v, 6) for v in obj.location],
                "rot": [round(v, 6) for v in obj.rotation_euler],
            }
        else:
            pose[target] = {"loc": [round(v, 6) for v in obj.location]}
    return pose


def _apply_pose(pose, context):
    """Set ONLY the named control objects' local transforms to the saved
    snapshot — no keyframes, no Action/clip side effects. Runs a
    view-layer update so the IK re-solves to the new foot targets.
    Returns (applied, missing) name lists."""
    applied, missing = [], []
    for target, xf in pose.items():
        obj = bpy.data.objects.get(target)
        if obj is None:
            missing.append(target)
            continue
        if "loc" in xf:
            obj.location = xf["loc"]
        if "rot" in xf:
            obj.rotation_euler = xf["rot"]
        applied.append(target)
    context.view_layer.update()
    return applied, missing


def _seed_pose_from_angles(joint_deg_by_leg):
    """Build a Pose-Library pose dict from per-leg [shoulder, hip, knee]
    joint angles, reusing the rig's own kinematics. This is the
    repurposed N/Flat machinery (decision C): _pose_foot_targets gives
    the IK-tip world position for each leg at those angles; we drive the
    foot targets there (body_ctrl at rest/identity), capture the
    resulting LOCAL transforms, then restore the scene exactly. The
    output is a normal library pose — once seeded it is re-savable like
    any other."""
    arm = _find_arm_obj()
    if arm is None:
        raise ValueError(f"Armature '{_ARM_OBJ_NAME}' not found in scene")

    snap = {}
    for target in CLIP_TARGETS:
        obj = bpy.data.objects.get(target)
        if obj is not None:
            snap[target] = (obj.location.copy(), obj.rotation_euler.copy())

    # Let body_ctrl stay at whatever rest position the rig builder set
    # (post-pivot-move that's (0,0,-17) — the BodyBottomPoint world
    # offset; pre-move it was identity). _pose_foot_targets below is
    # computed RELATIVE to that, and the captured pose then stores body
    # + feet at consistent positions. Earlier code forced body to
    # (0,0,0) here, which silently put the rig 17 mm above canonical
    # rest in every seeded pose — trigger Set N/Flat with the rig at
    # rest (its default state on open) and the seeded pose is correct.
    bpy.context.view_layer.update()

    foot_world = _pose_foot_targets(arm, joint_deg_by_leg)  # restores the rig

    for leg, (fx, fy, fz) in foot_world.items():
        obj = bpy.data.objects.get(f"foot_target_{leg}")
        if obj is None:
            continue
        mw = obj.matrix_world.copy()
        mw.translation = (fx, fy, fz)
        obj.matrix_world = mw
    bpy.context.view_layer.update()

    pose = _capture_pose()

    for target, (loc, rot) in snap.items():
        obj = bpy.data.objects.get(target)
        if obj is None:
            continue
        obj.location = loc
        obj.rotation_euler = rot
    bpy.context.view_layer.update()
    return pose


def _seeded_pose(name):
    """Return (poses[name], poses), seeding the built-in 'flat' /
    'neutral' entries on first use and persisting poses.json. 'flat' is
    URDF θ=0 (no convention.json needed); 'neutral' uses
    convention.json's neutral_joint_deg (raises ValueError if it's
    missing). Unknown names return (None, poses)."""
    poses = _load_poses()
    if name not in poses:
        if name == "flat":
            angles = {leg: (0.0, 0.0, 0.0) for leg in ("fl", "fr", "bl", "br")}
        elif name == "neutral":
            angles = _load_convention()["neutral_joint_deg"]
        else:
            return None, poses
        poses[name] = _seed_pose_from_angles(angles)
        _save_poses(poses)
    return poses.get(name), poses


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
        # Switching the clip context makes any pending "pose applied but
        # not keyed" notice stale — the pose was relative to a different
        # clip. Clear it so the panel doesn't show a misleading prompt.
        context.scene.fh_unkeyed_pose = ""
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
        # Sync the scene's playback range to this clip's authored range so
        # the timeline + playback match the clip you just switched to.
        _sync_scene_frame_range(context, self.clip_name)
        _redraw_view3d(context)
        return {"FINISHED"}


# ---------------------------------------------------------------------------
# Per-clip frame range <-> scene sync
# ---------------------------------------------------------------------------

# msgbus owner token — lets us react to edits of the built-in Action range
# props (which cannot carry an `update=` callback) and re-sync the scene live.
_FRAME_RANGE_MSGBUS_OWNER = object()


def _clip_frame_span(clip):
    """(start, end) frame span of a clip = the UNION of `frame_range` over ALL
    its actions (body_ctrl + the four foot_targets), not just body_ctrl.

    A clip's real span is wherever ANY target is keyed. body_ctrl alone can be
    single-keyed — e.g. a clip whose body just holds a pose while the feet
    carry the motion (fallingRobot) — which would otherwise truncate the export
    to one frame and collapse the panel/scene range to 0-0. `frame_range`
    honours each action's `use_frame_range`. Returns None if the clip has no
    actions. Note the span may be degenerate (start == end) for a genuinely
    single-frame clip — callers decide what to do with that."""
    starts, ends = [], []
    for target in CLIP_TARGETS:
        action = clip_action(clip, target)
        if action is None:
            continue
        s, e = action.frame_range
        starts.append(s)
        ends.append(e)
    if not starts:
        return None
    return int(min(starts)), int(max(ends))


def _clip_export_range(clip):
    """The frame range to EXPORT (and mirror to the scene) for a clip.

    This is the per-clip "which section of the animation to export" selector:
      * "Custom range" ON  (body_ctrl `use_frame_range`) -> the body_ctrl
        action's manual `frame_start..frame_end` — the user's explicit section.
      * "Custom range" OFF -> the full clip span (`_clip_frame_span`, the union
        of all actions), so a clip whose body is single-keyed (fallingRobot)
        still exports its whole motion.
    Returns (start, end) or None if the clip has no actions. The "Custom range"
    toggle/fields live on body_ctrl, so the choice persists per clip."""
    action = clip_action(clip, "body_ctrl")
    if action is not None and action.use_frame_range:
        return int(action.frame_start), int(action.frame_end)
    return _clip_frame_span(clip)


def _sync_scene_frame_range(context, clip=None):
    """Set the scene playback range to the active clip's export range
    (`_clip_export_range`). Refuses to set a degenerate range (end <= start) —
    returns False and leaves the scene untouched rather than zeroing it.
    Returns True on a successful, non-degenerate sync."""
    if clip is None:
        clip = active_clip()
    if clip is None:
        return False
    span = _clip_export_range(clip)
    if span is None or span[1] <= span[0]:
        return False
    context.scene.frame_start, context.scene.frame_end = span
    return True


def _on_action_range_changed(*_args):
    """msgbus notify: an Action's frame_start/frame_end/use_frame_range was
    edited — re-sync the scene to the active clip. Guarded so a stray edit on
    some unrelated Action while no clip is active is a harmless no-op."""
    try:
        _sync_scene_frame_range(bpy.context)
    except Exception as e:  # never let a UI-thread notify raise
        print(f"[fh_clip_panel] frame-range sync skipped: {e}")


def _subscribe_frame_range_msgbus():
    """Subscribe to the built-in Action range props so panel edits sync the
    scene live (built-in props can't take an `update=` callback). Best-effort:
    if msgbus is unavailable the Apply-to-Scene button + apply-clip sync still
    cover it."""
    try:
        for prop in ("frame_start", "frame_end", "use_frame_range"):
            bpy.msgbus.subscribe_rna(
                key=(bpy.types.Action, prop),
                owner=_FRAME_RANGE_MSGBUS_OWNER,
                args=(),
                notify=_on_action_range_changed,
            )
    except Exception as e:
        print(f"[fh_clip_panel] frame-range msgbus not subscribed: {e}")


def _unsubscribe_frame_range_msgbus():
    try:
        bpy.msgbus.clear_by_owner(_FRAME_RANGE_MSGBUS_OWNER)
    except Exception:
        pass


class FH_OT_sync_frame_range(bpy.types.Operator):
    """Set the scene's playback range to the active clip's authored range.
    Useful after editing the clip's Custom range fields by hand."""

    bl_idname = "fh.sync_frame_range"
    bl_label = "Apply to Scene"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        if not _sync_scene_frame_range(context):
            self.report({"WARNING"}, "No active clip to sync from")
            return {"CANCELLED"}
        clip = active_clip()
        self.report(
            {"INFO"},
            f"Scene range → {context.scene.frame_start}-{context.scene.frame_end} "
            f"(clip '{clip}')",
        )
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
            dup = _keep_action(src_action.copy())
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
            dup = _keep_action(src_action.copy())
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


class FH_OT_overwrite_clip(bpy.types.Operator):
    """Replace an EXISTING clip's 5 Actions with whatever animation is
    currently bound to the 5 control objects, then apply it.

    Use this when the live animation came from somewhere else (another
    clip, a fresh keyframing session, drifted action names) and you want
    to commit it onto clip <clip_name>, overwriting it. Destructive to
    that clip's previous contents (cannot be undone via these tools) —
    hence the confirm. The SOURCE actions are copied, not moved, so
    they are left intact (same non-destructive-to-source behaviour as
    Save Current → New Clip).

    Per-target: if the live action already IS this clip's action (you
    Applied the clip then edited it), that target is left as-is — your
    edits were already in the clip in place, nothing to copy."""

    bl_idname = "fh.overwrite_clip"
    bl_label = "Overwrite Clip From Current"
    bl_options = {"REGISTER", "UNDO"}

    clip_name: bpy.props.StringProperty(name="Clip Name")

    def invoke(self, context, event):
        wm = context.window_manager
        try:
            return wm.invoke_confirm(
                self,
                event,
                title=f"Overwrite '{self.clip_name}'?",
                message=(
                    "Replace this clip's 5 Actions with the current "
                    "animation. Destructive — cannot be undone via these "
                    "tools. The source animation is left intact."
                ),
                confirm_text="Overwrite",
            )
        except TypeError:
            # Older Blender without the rich invoke_confirm kwargs.
            return wm.invoke_confirm(self, event)

    def execute(self, context):
        clip = self.clip_name.strip()
        if not clip:
            self.report({"ERROR"}, "No clip name supplied")
            return {"CANCELLED"}
        if clip not in list_clips():
            self.report({"ERROR"}, f"Clip '{clip}' does not exist")
            return {"CANCELLED"}

        replaced, already, no_anim = [], [], []
        for target in CLIP_TARGETS:
            obj = bpy.data.objects.get(target)
            if obj is None:
                no_anim.append(f"object:{target}")
                continue
            ad = obj.animation_data
            live = ad.action if ad is not None else None
            if live is None:
                no_anim.append(f"no-action:{target}")
                continue
            dest_name = f"{clip}__{target}"
            if live.name == dest_name:
                # This target already IS the clip (Applied then edited
                # in place) — nothing to copy, leave it.
                already.append(target)
                continue
            dup = _keep_action(live.copy())
            old = bpy.data.actions.get(dest_name)
            if old is not None and old != live and old != dup:
                # Remove the OLD clip content first so the rename below
                # doesn't get an auto ".001" suffix. old is unbound here
                # (live != dest, so the clip wasn't the bound one).
                bpy.data.actions.remove(old)
            dup.name = dest_name
            replaced.append(target)

        if not replaced and not already:
            self.report(
                {"ERROR"},
                "Nothing bound to overwrite with — no actions on the "
                f"control objects ({', '.join(no_anim) or 'none'})",
            )
            return {"CANCELLED"}

        assigned, missing = assign_clip(clip)

        if not replaced:
            self.report(
                {"INFO"},
                f"'{clip}' already matches the live animation — nothing to overwrite",
            )
        elif no_anim or missing:
            self.report(
                {"WARNING"},
                f"Overwrote {len(replaced)}/{len(CLIP_TARGETS)} of '{clip}' "
                f"(applied {assigned}/{len(CLIP_TARGETS)}, kept "
                f"{len(already)} already-current, skipped "
                f"{', '.join(no_anim) or 'none'})",
            )
        else:
            self.report(
                {"INFO"},
                f"Overwrote clip '{clip}' with the current animation "
                f"({len(replaced)} action(s)) and applied",
            )
        _redraw_view3d(context)
        return {"FINISHED"}


class FH_OT_toggle_export_clip(bpy.types.Operator):
    """Toggle a clip's inclusion in the 'Export Selected Clips' set.
    Per-row icon button in the Clips sub-panel; flips the named clip
    in/out of the scene-level set (fh_export_selected_clips)."""

    bl_idname = "fh.toggle_export_clip"
    bl_label = "Include In Export"
    bl_options = {"REGISTER", "UNDO"}

    clip_name: bpy.props.StringProperty(name="Clip Name")

    def execute(self, context):
        clip = self.clip_name.strip()
        if not clip:
            self.report({"ERROR"}, "No clip name supplied")
            return {"CANCELLED"}
        raw = context.scene.fh_export_selected_clips
        names = {n for n in raw.split("\n") if n} if raw else set()
        if clip in names:
            names.discard(clip)
            action = "excluded from"
        else:
            names.add(clip)
            action = "included in"
        _write_export_selected(names)
        self.report({"INFO"}, f"Clip '{clip}' {action} selected-clips export")
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
        # If this clip was ticked for export, move the tick to the new
        # name so the user's selection follows the rename.
        _rename_export_selected(src, new_name)
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
            action = _keep_action(bpy.data.actions.new(f"{new_name}__{target}"))
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


def _apply_seeded(operator, context, name):
    """Shared body for Set N / Set Flat (decision C): seed the built-in
    '<name>' library pose on first use, then apply it exactly like
    FH_OT_pose_apply (set transforms + raise the not-keyed notice). No
    special angle-based path — they are just two library poses now."""
    try:
        pose, _ = _seeded_pose(name)
    except (ValueError, KeyError) as e:
        operator.report({"ERROR"}, str(e))
        return {"CANCELLED"}
    if pose is None:
        operator.report({"ERROR"}, f"Could not seed the '{name}' pose")
        return {"CANCELLED"}
    _, missing = _apply_pose(pose, context)
    context.scene.fh_unkeyed_pose = name
    context.scene.fh_unkeyed_frame = context.scene.frame_current
    if missing:
        operator.report(
            {"WARNING"},
            f"Applied seeded pose '{name}' — missing controls: {', '.join(missing)}",
        )
    else:
        operator.report({"INFO"}, f"Applied seeded library pose '{name}'")
    _redraw_view3d(context)
    return {"FINISHED"}


class FH_OT_set_n_pose(bpy.types.Operator):
    """Apply the built-in 'neutral' library pose (hardware-neutral N
    standing pose from convention.json). Seeded into poses.json on first
    use, then it is just a normal, re-savable library pose — this button
    is a convenience shortcut for it (decision C)."""

    bl_idname = "fh.set_n_pose"
    bl_label = "Set N Pose"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        return _apply_seeded(self, context, "neutral")


class FH_OT_set_rest_pose(bpy.types.Operator):
    """Apply the built-in 'flat' library pose — the URDF rest (all joint
    angles 0; the splayed stance the rig builder sets on every
    `--rigged` rebuild). Seeded into poses.json on first use (no
    convention.json needed), then a normal re-savable library pose; this
    button is a convenience shortcut for it (decision C)."""

    bl_idname = "fh.set_rest_pose"
    bl_label = "Set Flat Pose"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        return _apply_seeded(self, context, "flat")


# ---------------------------------------------------------------------------
# Operators — Pose Library (position-based, clip-independent)
# ---------------------------------------------------------------------------


class FH_OT_pose_save(bpy.types.Operator):
    """Snapshot the current control-object transforms into the Pose
    Library under the name in the field. Re-saving an existing name
    updates it — poses are meant to be re-tuned in place."""

    bl_idname = "fh.pose_save"
    bl_label = "Save Current Pose"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        name = context.scene.fh_pose_name.strip()
        if not name:
            self.report({"ERROR"}, "Enter a pose name first")
            return {"CANCELLED"}
        try:
            poses = _load_poses()
        except ValueError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        existed = name in poses
        poses[name] = _capture_pose()
        _save_poses(poses)
        context.scene.fh_pose_name = ""
        self.report({"INFO"}, f"{'Updated' if existed else 'Saved'} pose '{name}'")
        _redraw_view3d(context)
        return {"FINISHED"}


class FH_OT_pose_apply(bpy.types.Operator):
    """Apply a library pose: set the 5 control transforms to the saved
    snapshot. No keyframes are inserted — this is a pure viewport pose
    change. Use "Key into Clip" afterwards to commit it to a clip."""

    bl_idname = "fh.pose_apply"
    bl_label = "Apply Pose"
    bl_options = {"REGISTER", "UNDO"}

    pose_name: bpy.props.StringProperty(name="Pose Name")

    def execute(self, context):
        if not self.pose_name:
            self.report({"ERROR"}, "No pose name supplied")
            return {"CANCELLED"}
        try:
            poses = _load_poses()
        except ValueError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        pose = poses.get(self.pose_name)
        if pose is None:
            self.report({"ERROR"}, f"Pose '{self.pose_name}' not found")
            return {"CANCELLED"}
        applied, missing = _apply_pose(pose, context)
        # Raise the "applied but not keyed" notice (decision B): the pose
        # is live in the viewport but nothing is committed to a clip yet.
        # Records the pose name + the frame it was applied at so the
        # one-click Key button knows exactly where to insert keyframes.
        context.scene.fh_unkeyed_pose = self.pose_name
        context.scene.fh_unkeyed_frame = context.scene.frame_current
        if missing:
            self.report(
                {"WARNING"},
                f"Applied pose '{self.pose_name}' — missing controls: "
                f"{', '.join(missing)}",
            )
        else:
            self.report({"INFO"}, f"Applied pose '{self.pose_name}'")
        _redraw_view3d(context)
        return {"FINISHED"}


class FH_OT_pose_rename(bpy.types.Operator):
    """Rename a library pose to the name currently in the field."""

    bl_idname = "fh.pose_rename"
    bl_label = "Rename Pose"
    bl_options = {"REGISTER", "UNDO"}

    pose_name: bpy.props.StringProperty(name="Pose Name")

    def execute(self, context):
        new_name = context.scene.fh_pose_name.strip()
        if not new_name:
            self.report({"ERROR"}, "Enter the new pose name in the field first")
            return {"CANCELLED"}
        try:
            poses = _load_poses()
        except ValueError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        if self.pose_name not in poses:
            self.report({"ERROR"}, f"Pose '{self.pose_name}' not found")
            return {"CANCELLED"}
        if new_name == self.pose_name:
            self.report({"INFO"}, "Name unchanged")
            return {"CANCELLED"}
        if new_name in poses:
            self.report({"ERROR"}, f"Pose '{new_name}' already exists")
            return {"CANCELLED"}
        poses[new_name] = poses.pop(self.pose_name)
        _save_poses(poses)
        context.scene.fh_pose_name = ""
        self.report({"INFO"}, f"Renamed pose '{self.pose_name}' -> '{new_name}'")
        _redraw_view3d(context)
        return {"FINISHED"}


class FH_OT_pose_delete(bpy.types.Operator):
    """Delete a library pose. poses.json is the store; this rewrites it
    (the file is git-tracked, so a mistaken delete is recoverable)."""

    bl_idname = "fh.pose_delete"
    bl_label = "Delete Pose"
    bl_options = {"REGISTER", "UNDO"}

    pose_name: bpy.props.StringProperty(name="Pose Name")

    def execute(self, context):
        try:
            poses = _load_poses()
        except ValueError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        if self.pose_name not in poses:
            self.report({"ERROR"}, f"Pose '{self.pose_name}' not found")
            return {"CANCELLED"}
        del poses[self.pose_name]
        _save_poses(poses)
        self.report({"INFO"}, f"Deleted pose '{self.pose_name}'")
        _redraw_view3d(context)
        return {"FINISHED"}


class FH_OT_key_pose_into_clip(bpy.types.Operator):
    """Commit the pending (applied-but-not-keyed) pose into the active
    clip: re-apply the saved pose, then insert its keyframes into each
    control's clip Action at the frame the pose was applied, and clear
    the notice. Keys ONLY into the active clip's own Actions — it never
    creates a stray Action or writes onto another clip."""

    bl_idname = "fh.key_pose_into_clip"
    bl_label = "Key Pose Into Clip"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        scene = context.scene
        name = scene.fh_unkeyed_pose
        if not name:
            self.report({"ERROR"}, "No pose is pending")
            return {"CANCELLED"}
        clip = active_clip()
        if clip is None:
            self.report(
                {"ERROR"},
                "No active clip to key into — Apply a clip first "
                f"(pose '{name}' stays applied so you can retry)",
            )
            return {"CANCELLED"}
        try:
            poses = _load_poses()
        except ValueError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        pose = poses.get(name)
        if pose is None:
            # Renamed/deleted since it was applied -> the notice is stale.
            scene.fh_unkeyed_pose = ""
            self.report({"WARNING"}, f"Pose '{name}' no longer exists — notice cleared")
            return {"CANCELLED"}

        frame = scene.fh_unkeyed_frame
        # Re-apply so we key the pose's stored values even if the
        # timeline was scrubbed after Apply (the bound clip would
        # otherwise have re-evaluated the controls at the new frame).
        _apply_pose(pose, context)

        keyed, skipped = [], []
        for target in pose:
            obj = bpy.data.objects.get(target)
            if obj is None:
                skipped.append(target)
                continue
            ad = obj.animation_data
            if ad is None or ad.action is None or ad.action.name != f"{clip}__{target}":
                # Not bound to THIS clip's Action — refuse to key it
                # (re-Apply the clip to bind all 5 targets, then retry).
                skipped.append(target)
                continue
            obj.keyframe_insert("location", frame=frame)
            if target == ANCHOR:
                obj.keyframe_insert("rotation_euler", frame=frame)
            keyed.append(target)

        if not keyed:
            self.report(
                {"ERROR"},
                f"Could not key pose '{name}' — no control is bound to "
                f"clip '{clip}'. Re-Apply the clip, then retry.",
            )
            return {"CANCELLED"}

        scene.fh_unkeyed_pose = ""
        if skipped:
            self.report(
                {"WARNING"},
                f"Keyed pose '{name}' into '{clip}' @ frame {frame} "
                f"({len(keyed)}/{len(pose)}) — not bound to this clip: "
                f"{', '.join(skipped)} (re-Apply the clip, then retry)",
            )
        else:
            self.report(
                {"INFO"}, f"Keyed pose '{name}' into clip '{clip}' @ frame {frame}"
            )
        _redraw_view3d(context)
        return {"FINISHED"}


# ---------------------------------------------------------------------------
# Operator — selection sets (pick control objects; touches no data)
# ---------------------------------------------------------------------------
#
# Convenience for animators: select a meaningful group of the rig's
# control objects in one click (so you can grab/key them together).
# Pure viewport selection — no transforms, keyframes or Actions touched.

_SELECTION_PRESETS = {
    "ALL": list(CLIP_TARGETS),
    "BODY": [ANCHOR],
    "FRONT": ["foot_target_fl", "foot_target_fr"],
    "BACK": ["foot_target_bl", "foot_target_br"],
    "LEGS": [
        "foot_target_fl",
        "foot_target_fr",
        "foot_target_bl",
        "foot_target_br",
    ],
    "FL": ["foot_target_fl"],
    "FR": ["foot_target_fr"],
    "BL": ["foot_target_bl"],
    "BR": ["foot_target_br"],
}

# Button order in the panel (kept stable, independent of dict order).
_SELECTION_ORDER = ("ALL", "BODY", "FRONT", "BACK", "LEGS", "FL", "FR", "BL", "BR")


def _clip_interpolation_mode(clip):
    """Current keyframe interpolation of `clip`, as the panel toggle shows
    it: "LINEAR", "BEZIER", or None (no clip / no keyframes). Reads the
    FIRST keyframe found across the clip's layered f-curves — the same
    first-keyframe heuristic FH_OT_toggle_preview uses to decide its flip
    direction, so the displayed state and the toggle's action never
    disagree. Derived from the curves, never stored, so it can't drift."""
    if clip is None:
        return None
    for target in CLIP_TARGETS:
        action = clip_action(clip, target)
        if action is None:
            continue
        for layer in action.layers:
            for strip in layer.strips:
                for channelbag in strip.channelbags:
                    for fcurve in channelbag.fcurves:
                        for kp in fcurve.keyframe_points:
                            return (
                                "LINEAR" if kp.interpolation == "LINEAR" else "BEZIER"
                            )
    return None


class FH_OT_toggle_preview(bpy.types.Operator):
    """Toggle the active clip's F-curves between BEZIER (authoring) and
    LINEAR (exactly what the robot plays). Non-destructive: Bezier handles
    are preserved on the keyframes and restored when toggled back."""

    bl_idname = "fh.toggle_preview"
    bl_label = "Preview Robot Motion"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        clip = active_clip()
        if clip is None:
            self.report({"WARNING"}, "No active clip")
            return {"CANCELLED"}
        going_linear = None  # decided from the first keyframe seen
        touched = 0
        for target in CLIP_TARGETS:
            action = clip_action(clip, target)
            if action is None:
                continue
            # Blender 5.x layered Action API: F-curves live in
            # layer → strip → channelbag → fcurves (not action.fcurves).
            for layer in action.layers:
                for strip in layer.strips:
                    for channelbag in strip.channelbags:
                        for fcurve in channelbag.fcurves:
                            for kp in fcurve.keyframe_points:
                                if going_linear is None:
                                    going_linear = kp.interpolation != "LINEAR"
                                kp.interpolation = (
                                    "LINEAR" if going_linear else "BEZIER"
                                )
                                touched += 1
        mode = "LINEAR (robot preview)" if going_linear else "BEZIER (authoring)"
        self.report({"INFO"}, f"{clip}: {touched} keyframes -> {mode}")
        _redraw_view3d(context)
        return {"FINISHED"}


class FH_OT_select_controls(bpy.types.Operator):
    """Select a preset group of rig control objects (replacing the
    current selection) and make one of them active. Selection only —
    no transforms, keyframes or Actions are modified."""

    bl_idname = "fh.select_controls"
    bl_label = "Select Controls"
    bl_options = {"REGISTER", "UNDO"}

    preset: bpy.props.StringProperty(name="Preset")

    def execute(self, context):
        names = _SELECTION_PRESETS.get(self.preset)
        if names is None:
            self.report({"ERROR"}, f"Unknown selection preset '{self.preset}'")
            return {"CANCELLED"}

        view_objs = context.view_layer.objects
        for obj in view_objs:
            obj.select_set(False)

        selected, missing = [], []
        for name in names:
            obj = view_objs.get(name)  # None if absent or not in this view layer
            if obj is None:
                missing.append(name)
                continue
            obj.select_set(True)
            selected.append(obj)

        if not selected:
            self.report({"WARNING"}, f"No '{self.preset}' controls found in the scene")
            return {"CANCELLED"}
        view_objs.active = selected[0]

        if missing:
            self.report(
                {"WARNING"},
                f"Selected {len(selected)} ({self.preset}) — missing: "
                f"{', '.join(missing)}",
            )
        else:
            self.report(
                {"INFO"}, f"Selected {len(selected)} control(s) ({self.preset})"
            )
        _redraw_view3d(context)
        return {"FINISHED"}


# ---------------------------------------------------------------------------
# LAYER 1 — pure Blender data extraction (no hardware knowledge)
# ---------------------------------------------------------------------------


# Per-leg sign applied to the link1 (shoulder/yaw) delta on export.
#
# Canon (convention PNG, DRAFT-delta-conventions.md §2): math-space +sh = CCW
# yaw, UNIFORM across all four legs ("absolute rotation values like a unit
# circle, same rotation"). The rig's link1 yaw driver writes the bone-local Z
# rotation, i.e. it multiplies the true CCW foot-yaw delta by the bone axis
# sign (URDF link1 <axis z>: fr -1, fl +1, br +1, bl -1). To recover the
# uniform CCW math-space the export must CANCEL that bone-axis sign — so this
# table IS the axis sign. translateToServo then maps uniform math-space to
# servos, where BR's documented hardware mirror (slope -1) makes BR's servo
# move opposite the other three for a body yaw. That is correct, not a bug;
# whether BR should be un-mirrored is a separate hardware question. See
# docs/CLIP_SHOULDER_CONVENTION.md.
_LINK1_DELTA_SIGN = {"fr": -1, "fl": +1, "br": +1, "bl": -1}


def _link1_delta_to_absolute(angles, convention):
    """Convert each `*_link1` (shoulder/yaw) angle from delta-to-absolute.

    The rig drives link1 with an analytic yaw driver that outputs a DELTA
    from the flat/rest pose — 0 deg at rest, deviating only as the foot
    yaws — NOT an absolute joint angle. (The IK-driven link2/link3 are
    already absolute: `_read_bone_angles` recovers the constraint-solved
    angle for those.) But the export pipeline downstream
    (`_scale_from_neutral`, `_frame_to_servo`) and the firmware
    `translateToServo` both assume raw == NEUTRAL at the standing pose,
    so that servo 90 = shoulder outward for every leg. Reading link1 as
    delta therefore mis-anchored every shoulder (servo ~60 on fr/br,
    clamped past [38,142] on the +/-135 legs fl/bl) — collapsing the
    robot on playback in BOTH sim and firmware.

    Fix: absolute = NEUTRAL + sign*delta, where NEUTRAL anchors the rest pose
    (servo 90) and the per-leg sign (_LINK1_DELTA_SIGN) flips fl so all four
    shoulders yaw the same servo direction. Shoulders only — link2/link3 are
    already absolute. See docs/CLIP_SHOULDER_CONVENTION.md.

    Mutates and returns `angles` ({bone_name: deg}). Degrees throughout
    (`_read_bone_angles` and `neutral_joint_deg` are both degrees)."""
    neutral = convention["neutral_joint_deg"]
    for leg in _LEGS:
        key = f"{leg}_link1"
        if key in angles:
            angles[key] = neutral[leg][0] + _LINK1_DELTA_SIGN[leg] * angles[key]
    return angles


def bake_clip(clip_name, context):
    """Step through every frame of `clip_name`, evaluate the depsgraph and
    read the IK-solved joint angles. Returns a list of row dicts:

        [{"frame": 1, "time_ms": 0, "fl_link1": 0.0, "fl_link2": 0.0, ...},
         ...]

    Binds `clip_name`'s Actions onto the rig BEFORE sampling so the
    depsgraph reflects this clip's motion — not whatever clip happens to
    be active (the multi-clip "Export Selected" mislabel bug). Restores
    the previously-active clip + frame afterward. Pure data extraction —
    knows nothing about servos, scaling or channels. Raises ValueError if
    the rig/clip is not exportable."""
    scene = context.scene
    arm_obj = _find_arm_obj()
    if arm_obj is None:
        raise ValueError(f"Armature '{_ARM_OBJ_NAME}' not found in scene")
    action = clip_action(clip_name, "body_ctrl")
    if action is None:
        raise ValueError(f"No body_ctrl action for clip '{clip_name}'")

    # Bind the target clip so the depsgraph samples ITS motion. Autocomplete
    # first (mirrors FH_OT_apply_clip) so a clip missing a target doesn't
    # inherit the previous clip's Action on that target. view_layer.update()
    # forces the IK constraint stack to re-solve to the rebind before frame 0.
    prev_clip = active_clip()
    _autocomplete_clip(clip_name, context)
    _assigned, missing = assign_clip(clip_name)
    if missing:
        raise ValueError(
            f"Clip '{clip_name}' incomplete — missing: {', '.join(missing)}"
        )
    context.view_layer.update()

    # Bake the clip's export range: the body_ctrl custom range if "Custom range"
    # is enabled (the per-clip section selector), else the full clip span (union
    # of all five actions — a clip can key the body at a single frame while the
    # feet carry the motion, e.g. fallingRobot, so body_ctrl alone would truncate
    # the export to one frame).
    span = _clip_export_range(clip_name)
    if span is None:
        raise ValueError(f"Clip '{clip_name}' has no actions to bake")
    frame_start, frame_end = span
    fps = scene.render.fps / scene.render.fps_base
    if fps > RECOMMENDED_MAX_FPS:
        print(
            f"WARNING: scene fps is {fps:.0f}; consider lowering to "
            f"{RECOMMENDED_MAX_FPS} in Output Properties for robot export "
            f"(fewer WebSocket packets, same motion)."
        )

    original_frame = scene.frame_current
    convention = _load_convention()
    rows = []
    for frame in range(frame_start, frame_end + 1):
        scene.frame_set(frame)
        depsgraph = bpy.context.evaluated_depsgraph_get()
        arm_eval = arm_obj.evaluated_get(depsgraph)
        angles = _link1_delta_to_absolute(_read_bone_angles(arm_eval), convention)
        time_ms = round((frame - frame_start) / fps * 1000)
        rows.append({"frame": frame, "time_ms": time_ms, **angles})
    scene.frame_set(original_frame)

    # Restore whatever clip was active before, so baking doesn't leave the
    # rig rebound to the last-baked clip (matters for "Export Selected").
    if prev_clip is not None and prev_clip != clip_name:
        _autocomplete_clip(prev_clip, context)
        assign_clip(prev_clip)
        context.view_layer.update()
    _warn_frame_deltas(rows)
    return rows


# ---------------------------------------------------------------------------
# LAYER 2 — converters: baked rows -> animation/exported_clips/<clip>/
# ---------------------------------------------------------------------------

_LEGS = ("fr", "fl", "br", "bl")  # firmware LegId order: FR=0, FL=1, BR=2, BL=3

# Blender leg name -> firmware LegId (matches `enum LegId` in
# code/firmware/src/nervous_system/movements.h on origin/main). Used as
# the wire `id` field in CMD_CALIBRATE (T:4) — firmware then does the
# PCA-channel mapping via LEG_SERVO_CHANNEL[id][servo_id] (config.h:64),
# so the exporter no longer needs convention.json's `channels` on the
# wire. The Python list-of-3 returned by _frame_to_servo is indexed by
# firmware `servo_id` (0=hip, 1=thigh, 2=knee).
_LEG_ID = {"fr": 0, "fl": 1, "br": 2, "bl": 3}


def to_csv(frames, clip_name):
    """Raw bone angles, one row per frame (unchanged legacy format)."""
    path = os.path.join(_exported_clips_dir(clip_name), f"{clip_name}.csv")
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
    path = os.path.join(_exported_clips_dir(clip_name), f"{clip_name}.h")
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


# Per-servo zero-point calibration: servo angle when the joint is at math 0°
# (thigh/knee flat). Mirror of CALIB_* in code/firmware/src/shared/config.h.
# Hip is uncalibrated (stays 90). Keep in sync with firmware whenever CALIB
# values change (and re-export every clip so the bake-time servo angles match
# the runtime). Pre-calibration these were all 90.
_CALIB_THIGH = {"fr": 84, "fl": 87, "br": 103, "bl": 84}
_CALIB_KNEE = {"fr": 95, "fl": 82, "br": 80, "bl": 87}


def _frame_to_servo(row, convention, warn=True):
    """One baked row -> {leg: [hip, thigh, knee] ints} where the per-leg
    list index IS the firmware `servo_id` (0=hip, 1=thigh, 2=knee —
    matching LEG_SERVO_CHANNEL[leg_id][servo_id] in
    code/firmware/src/shared/config.h on origin/main).

    Applies (1) scale-from-NEUTRAL, (2) per-leg translateToServo with
    per-joint CALIB zero-points — byte-identical to the firmware
    translateToServo, locked by test_clip_parity — and (3) rounds to int.

    Wire shape consumed by to_js: {T:4, id:_LEG_ID[leg], servo_id:j,
    a:result[leg][j]}. The firmware does the PCA-channel mapping;
    convention.json's `channels` field is kept for reference but is
    NOT used on the wire."""
    neutral = convention["neutral_joint_deg"]
    scale = convention["scale"]
    out = {}
    for leg in _LEGS:
        n = neutral[leg]  # [shoulder, hip, knee] deg at hardware neutral
        raw = [row[f"{leg}_link1"], row[f"{leg}_link2"], row[f"{leg}_link3"]]
        # 1. compress movement toward N
        sh, th, kn = (n[j] + (raw[j] - n[j]) * scale for j in range(3))
        # 2. translateToServo: shoulder uses literal 90 (uncalibrated), thigh/knee
        #    use per-leg CALIB (the servo angle at math 0 / flat).
        ct, ck = _CALIB_THIGH[leg], _CALIB_KNEE[leg]
        if leg == "fl":
            # Change B: FL shoulder regularized to 90 + (sh - 135) so servo 90 = outward,
            # matching fr/bl/br. Byte-identical to firmware motion_math.cpp FL branch.
            servo = [90 + (sh - 135), ct + th, ck - kn]
        elif leg == "fr":
            servo = [90 + (sh - 45), ct - th, ck + kn]
        elif leg == "bl":
            servo = [90 + (sh + 135), ct - th, ck + kn]
        else:  # br
            # BR shoulder un-mirrored (2026-05-25): +sh = +servo like fr/fl/bl
            # (identical motor, yaw shaft on the same vertical axis). Kept
            # byte-identical to firmware translateToServo by test_clip_parity.
            servo = [90 + (sh + 45), ct + th, ck - kn]
        # 3. clamp to the servo range, surfacing authoring errors at export
        # time rather than relying on the JS / firmware clamp as the only
        # backstop (the FRAME_DELTA_WARN_DEG warning's range companion).
        for i, v in enumerate(servo):
            if v < 0 or v > 180:
                if warn:
                    bone_name = ["shoulder", "hip", "knee"][i]
                    print(
                        f"WARNING: {leg} {bone_name} servo {v:.1f} out of range "
                        f"[0-180] at frame {row.get('frame', '?')} — clamped"
                    )
                servo[i] = max(0, min(180, v))
        # 4. integer servo degrees
        out[leg] = [round(v) for v in servo]
    return out


def _current_servo_angles(context):
    """Live servo degrees for the CURRENT rig pose, per leg, via the exact
    export pipeline (link1 delta->absolute, scale-from-NEUTRAL, translateToServo)
    so the numbers match what would be baked/flashed. Returns
    {leg: [shoulder, thigh, knee]} (ints) or None if the rig/convention isn't
    available. warn=False so a panel redraw never spams the clamp warning."""
    arm = _find_arm_obj()
    if arm is None:
        return None
    try:
        convention = _load_convention()
        dg = context.evaluated_depsgraph_get()
        angles = _link1_delta_to_absolute(
            _read_bone_angles(arm.evaluated_get(dg)), convention
        )
        return _frame_to_servo(angles, convention, warn=False)
    except Exception:
        return None


def _clip_frame_ms(frames):
    """Playback wall-clock period for a baked clip: the median delta between
    consecutive `time_ms` values = the authored scene's frame period (~42 ms at
    24 fps). Falls back to 33 ms (~30 fps) for clips with fewer than 2 frames.
    Shared by the .js player and the app bundle so both play at authored speed."""
    if len(frames) >= 2:
        deltas = sorted(
            frames[i + 1]["time_ms"] - frames[i]["time_ms"]
            for i in range(len(frames) - 1)
        )
        return deltas[len(deltas) // 2]
    return 33


def to_js(
    frames,
    clip_name,
    convention,
    loop=False,
    dry_run=False,
    write=True,
    esp_ip="192.168.4.1",
):
    """Self-contained browser-console JS that plays the clip.

    Spec: doc/animation-pipeline/onboard-clip-player-design.md
    §3.1 — by default play **once** and **hold the final pose** by
    continuously re-sending the last frame's servo angles each tick
    (Phase-1 ↔ Phase-2 parity with firmware `tickClip`). `loop=True` is
    opt-in, intended only for diagnostic re-play.

    `dry_run=True` emits a variant with no `new WebSocket`/`ws.send`;
    every per-frame `{T:4,id,a}` message goes to `console.log` instead.
    Lets you validate a clip's servo stream without a robot. Stop
    semantics, hold-at-end behaviour, and the clamp are identical to
    the live version.

    `write=False` returns the generated JS string without touching the
    filesystem (useful for tests/validation). Default (`write=True`) writes
    the file to `animation/exported_clips/<clip_name>/` and returns its path.

    `esp_ip` is the robot's address baked into the WebSocket URL
    (`ws://<esp_ip>:81`) and the HOW-TO-RUN comment. The board IP can change
    (AP-mode default 192.168.4.1, or a DHCP lease on a shared network), so the
    animator sets it at export time. Empty/blank falls back to 192.168.4.1.
    Port 81 is fixed (see code/API_SPEC.md). Ignored in dry-run (no socket).

    Math: applies the full hardware conversion (scale-from-NEUTRAL +
    per-leg translateToServo + round) at bake time via
    `_frame_to_servo`. N/SCALE come from convention.json (the single
    source of truth that matches firmware NEUTRAL[]).

    Wire shape (origin/main CMD_CALIBRATE): per joint,
    {T:4, id:<leg_id 0-3>, servo_id:<0-2>, a:<0-180>}. Firmware maps to
    PCA channel via LEG_SERVO_CHANNEL[id][servo_id] (config.h:64);
    convention.json's `channels` is no longer on the wire."""
    path = os.path.join(_exported_clips_dir(clip_name), f"{clip_name}.js")
    today = datetime.date.today().isoformat()
    # Blank IP -> AP-mode default; no strict regex so mDNS names
    # (facehugger.local) stay valid too.
    esp_ip = (esp_ip or "").strip() or "192.168.4.1"

    # Derive the playback wall-clock period from the baked time_ms
    # column — the median delta between consecutive frames gives the
    # source scene's frame period (typically ~42 ms at 24 fps). The
    # previous hard-coded FRAME_MS=30 made every clip play 1.4× faster
    # than authored at the default Blender scene FPS. Using the
    # source-frame period keeps playback wall-clock = authored
    # wall-clock. Falls back to 33 ms (~30 fps) if the clip has fewer
    # than 2 frames.
    frame_ms = _clip_frame_ms(frames)

    clip_lines = []
    for row in frames:
        s = _frame_to_servo(row, convention)
        legs = ", ".join(
            f"{leg}:[{s[leg][0]},{s[leg][1]},{s[leg][2]}]" for leg in _LEGS
        )
        clip_lines.append(f"  {{ t: {row['time_ms']}, {legs} }},")

    # Per-mode pieces of the template (kept as plain Python strings —
    # injected verbatim into the outer f-string so their JS-literal { }
    # braces don't need doubling).
    loop_js = "true" if loop else "false"
    if dry_run:
        header_block = (
            '// DRY RUN — prints each {"T":4,...} message to the console\n'
            "// instead of sending it. No WebSocket is opened; no robot\n"
            "// needed. Paste into any JS console to validate the servo\n"
            "// stream before connecting to hardware.\n"
            "//\n"
            "// Wire shape matches origin/main CMD_CALIBRATE (T:4):\n"
            "//   {T:4, id:<leg_id 0-3>, servo_id:<0-2>, a:<0-180>}\n"
            "// Firmware maps to PCA channel via LEG_SERVO_CHANNEL."
        )
        transport_decl = "// (dry run — no WebSocket opened)"
        open_guard = ""
        send_call = "console.log(JSON.stringify(msg));"
        stop_close = "// (no socket to close in dry-run)"
        starter = (
            'console.log("FaceHugger DRY RUN — " + CLIP_NAME + " (" +'
            ' CLIP.length + " frames). No WebSocket; messages echoed'
            ' to console. Call fhStop() to stop.");\n'
            "_timer = setInterval(playFrame, FRAME_MS);"
        )
    else:
        header_block = (
            "// HOW TO RUN (browser):\n"
            f"//   1. Join the robot Wi-Fi (FaceHugger_Net); robot at {esp_ip}.\n"
            "//   2. Open a console on a NON-HTTPS page (http://, file://, or\n"
            "//      about:blank). ws:// is BLOCKED from https:// (mixed content)\n"
            '//      — the #1 reason "nothing happens".\n'
            "//   3. Paste this whole file. Call  fhStop()  to stop at any time.\n"
            "//\n"
            "// Wire shape matches origin/main CMD_CALIBRATE (T:4):\n"
            "//   {T:4, id:<leg_id 0-3>, servo_id:<0-2>, a:<0-180>}\n"
            "// Firmware does PCA-channel mapping via LEG_SERVO_CHANNEL."
        )
        transport_decl = f'const ws = new WebSocket("ws://{esp_ip}:81");'
        open_guard = "if (ws.readyState !== WebSocket.OPEN) return;"
        send_call = "ws.send(JSON.stringify(msg));"
        stop_close = "try { ws.close(); } catch (e) {}"
        starter = (
            "ws.onopen = () => {\n"
            '  console.log("FaceHugger: connected — playing " + CLIP_NAME +'
            ' " (" + CLIP.length + " frames). Hold-at-end is on by default;'
            ' call fhStop() when done.");\n'
            "  _timer = setInterval(playFrame, FRAME_MS);\n"
            "};\n"
            "ws.onerror = () => {\n"
            '  alert("FaceHugger: could not connect to " + ws.url +\n'
            '        " - check the robot IP / Wi-Fi network and reload.");\n'
            "};\n"
            'ws.onclose = (e) => { if (!e.wasClean) console.warn("FaceHugger WS closed", e.code); };'
        )

    js = f"""// FaceHugger clip: {clip_name}
// Generated {today} from Blender animation
//
{header_block}
//
// PLAYBACK SEMANTICS (Phase-1 parity with firmware tickClip, see spec
// doc/animation-pipeline/onboard-clip-player-design.md §3.1):
//   - DEFAULT: play the clip ONCE, then HOLD the final pose by
//     re-sending the last frame's servo angles every FRAME_MS — the
//     same as the firmware's hold-at-end behaviour.
//   - LOOP=true: replay from frame 0 instead of holding (diagnostic).
//   - fhStop(): explicit safe stop — clears the interval and closes
//     the socket. The robot keeps the last commanded servo positions
//     (servos hold their last commanded angle in hardware).

const LOOP = {loop_js};
// Playback wall-clock period — derived from the Blender scene FPS at
// bake time (median delta of `t` between consecutive frames), so this
// clip plays at the speed it was authored. Each scheduled tick sends
// the next CLIP[] frame.
const FRAME_MS = {frame_ms};
const CLIP_NAME = {json.dumps(clip_name)};

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = {{ fr: 0, fl: 1, br: 2, bl: 3 }};

const CLIP = [
{chr(10).join(clip_lines)}
];

// Servos accept 0..180; clamp defensively (extreme poses / a drifted
// convention can push the converted angle out of range).
const clamp = (v) => Math.max(0, Math.min(180, v | 0));

// Delta-encode: only emit a channel when its value changed since the last
// frame. Cuts redundant traffic and ends the hold-at-end resend flood
// (a held pose = unchanged angles = nothing sent). Reset to {{}} on restart
// so the first frame after a (re)start always sends all 12 channels.
let _last = {{}};

let _i = 0;
let _timer = null;
{transport_decl}

function fhStop() {{
  if (_timer !== null) {{ clearInterval(_timer); _timer = null; }}
  {stop_close}
  console.log("FaceHugger: playback stopped (servos hold last commanded pose).");
}}
globalThis.fhStop = fhStop;

function playFrame() {{
  // End of clip: in LOOP mode wrap to the start; otherwise clamp to
  // the last frame and keep re-sending it (hold-at-end per spec §3.1).
  if (_i >= CLIP.length) {{
    if (LOOP) {{
      _i = 0;
      _last = {{}};  // reset delta cache so loop restart re-sends all channels
    }} else {{
      _i = CLIP.length - 1;
    }}
  }}
  const frame = CLIP[_i++];
  {open_guard}
  for (const leg of ["fr", "fl", "br", "bl"]) {{
    const angles = frame[leg];
    for (let j = 0; j < 3; j++) {{
      const a = clamp(angles[j]);
      const key = leg + ":" + j;
      if (_last[key] === a) continue;
      _last[key] = a;
      const msg = {{ "T": 4, "id": LEG_IDS[leg], "servo_id": j, "a": a }};
      {send_call}
    }}
  }}
}}

{starter}
"""
    if not write:
        return js
    with open(path, "w") as fh:
        fh.write(js)
    return path


def _scale_from_neutral(row, convention):
    """One baked row -> {leg: [sh, th, kn]} math-space degrees with the
    2/3 scale-from-NEUTRAL applied (the SAME step-1 math as
    _frame_to_servo, but WITHOUT translateToServo / round). The clip
    header is math-space; the firmware applies translateToServo at
    runtime, so this must NOT pre-apply it (plan §0)."""
    neutral = convention["neutral_joint_deg"]
    scale = convention["scale"]
    out = {}
    for leg in _LEGS:
        n = neutral[leg]
        raw = [row[f"{leg}_link1"], row[f"{leg}_link2"], row[f"{leg}_link3"]]
        out[leg] = [n[j] + (raw[j] - n[j]) * scale for j in range(3)]
    return out


def _c_sym(clip_name):
    return "fh_clip_" + clip_name.lower().replace("-", "_").replace(" ", "_")


def to_clips_header(clips, convention, write=True, out_dir=None):
    """Bundle every clip into one clips_all.h + a clips_manifest.json.

    `clips` is a dict {clip_name: baked_rows}. Clips are emitted **sorted
    alphabetically by name**: index in the sorted list = clip id =
    FH_CLIPS[] index. This is enforced inside the emitter so the on-flash
    clip ids stay stable across exports regardless of the caller's dict
    insertion order (Task #7 — guards the app's flashed-clip list and any
    persisted user pinning from silent shuffles).

    Each clip's a[12] is math-space degrees, scale-from-NEUTRAL applied
    ONCE, in firmware LegId order (FR,FL,RR,RL) × (shoulder, thigh, knee).
    Firmware applies ONLY translateToServo at runtime — see plan §0 /
    onboard-clip-player-design.md §5.

    Returns (header_str, manifest_json_str). When write=True also writes
    them to out_dir (default animation/exported_clips/)."""
    names = sorted(clips.keys())
    seen_syms = {}
    for name in names:
        sym = _c_sym(name)
        if sym in seen_syms:
            raise ValueError(
                f"Clip names '{seen_syms[sym]}' and '{name}' both map to C "
                f"symbol '{sym}'; rename one (duplicate symbol won't compile)"
            )
        seen_syms[sym] = name
    lines = [
        "#ifndef FH_CLIPS_ALL_H",
        "#define FH_CLIPS_ALL_H",
        "#include <stdint.h>",
        "",
        "/* Auto-generated by FH Clip Panel (to_clips_header) - do not edit.",
        " * a[12] = PRE-SCALED math-space joint degrees: the 2/3",
        " * scale-from-NEUTRAL is already applied; firmware NEVER re-scales.",
        " * Order is firmware LegId: FR,FL,RR,RL, each (shoulder,thigh,knee).",
        " * Firmware applies ONLY translateToServo() at runtime. */",
        "",
        "typedef struct { uint16_t t_ms; float a[12]; } FhClipFrame;",
        "typedef struct {",
        "    const char*        name;",
        "    const FhClipFrame* frames;",
        "    uint16_t           frame_count;",
        "    uint16_t           duration_ms;",
        "} FhClip;",
        "",
        f"#define FH_CLIP_COUNT {len(names)}",
    ]
    manifest = {"clips": []}
    for cid, name in enumerate(names):
        rows = clips[name]
        if not rows:
            raise ValueError(f"Clip '{name}' has no frames; refusing to emit")
        prev_t = None
        for r in rows:
            t = int(r["time_ms"])
            if t > 65535:
                raise ValueError(
                    f"Clip '{name}' t_ms {t} exceeds uint16_t (65535 ms); "
                    f"clip too long for the header timeline type"
                )
            if prev_t is not None and t <= prev_t:
                raise ValueError(
                    f"Clip '{name}' t_ms not strictly increasing "
                    f"({prev_t} -> {t}); firmware bracket search requires it"
                )
            prev_t = t
        sym = _c_sym(name)
        lines.append(f"static const FhClipFrame {sym}[] = {{")
        for row in rows:
            s = _scale_from_neutral(row, convention)
            a = []
            for leg in _LEGS:  # ("fr","fl","br","bl") == LegId order
                a += [f"{v:.4f}f" for v in s[leg]]
            t_ms = int(row["time_ms"])
            lines.append(f"    {{ {t_ms:5d}, {{ {', '.join(a)} }} }},")
        lines.append("};")
        frame_count = len(rows)
        duration_ms = int(rows[-1]["time_ms"]) if rows else 0
        manifest["clips"].append(
            {
                "id": cid,
                "name": name,
                "frame_count": frame_count,
                "duration_ms": duration_ms,
            }
        )
    lines.append("")
    lines.append("static const FhClip FH_CLIPS[FH_CLIP_COUNT] = {")
    for cid, name in enumerate(names):
        rows = clips[name]
        sym = _c_sym(name)
        dur = int(rows[-1]["time_ms"]) if rows else 0
        lines.append(f"    {{ {json.dumps(name)}, {sym}, {len(rows)}, {dur} }},")
    lines.append("};")
    lines.append("#endif /* FH_CLIPS_ALL_H */")
    lines.append("")
    header = "\n".join(lines)
    manifest_str = json.dumps(manifest, indent=2) + "\n"

    if write:
        base = out_dir or os.path.join(_animation_dir(), "exported_clips")
        os.makedirs(base, exist_ok=True)
        with open(os.path.join(base, "clips_all.h"), "w") as fh:
            fh.write(header)
        with open(os.path.join(base, "clips_manifest.json"), "w") as fh:
            fh.write(manifest_str)

        # Post-export consistency gate — warns loudly but does not block
        try:
            import sys as _sys

            _sys.path.insert(
                0, os.path.join(os.path.dirname(__file__), "..", "scripts")
            )
            from check_export_consistency import check_all_clips as _check

            if not _check(Path(base)):
                print(
                    "WARNING: export consistency check FAILED — review output above before flashing"
                )
        except Exception as _e:
            print(f"WARNING: consistency check could not run: {_e}")

    return header, manifest_str


def to_clips_extra_json(clips, convention, write=True, out_dir=None):
    """Bundle clips into clips_extra.json — the format the remote-control app
    streams client-side (no flash). Unlike clips_all.h (math-space, scaled; the
    firmware applies translateToServo at runtime), this stores the FINAL per-leg
    servo degrees [hip, thigh, knee] via _frame_to_servo — the exact wire values
    the app sends as CMD_CALIBRATE (T:4), identical to what the .js players emit.

    `clips` is a dict {clip_name: baked_rows}. Clips are emitted **sorted
    alphabetically by name** so the app-bundle order matches clips_all.h
    and stays stable across exports (Task #7). Returns the JSON string;
    when write=True also writes clips_extra.json to out_dir (default
    animation/exported_clips/)."""
    out = {"clips": []}
    for name in sorted(clips.keys()):
        rows = clips[name]
        if not rows:
            raise ValueError(f"Clip '{name}' has no frames; refusing to emit")
        frames = []
        for row in rows:
            s = _frame_to_servo(row, convention)
            frames.append(
                {
                    "t": int(row["time_ms"]),
                    "fr": s["fr"],
                    "fl": s["fl"],
                    "br": s["br"],
                    "bl": s["bl"],
                }
            )
        out["clips"].append(
            {
                "name": name,
                "frame_ms": _clip_frame_ms(rows),
                "duration_ms": int(rows[-1]["time_ms"]),
                "loop": False,
                "frames": frames,
            }
        )
    s = json.dumps(out, indent=2) + "\n"
    if write:
        base = out_dir or os.path.join(_animation_dir(), "exported_clips")
        os.makedirs(base, exist_ok=True)
        with open(os.path.join(base, "clips_extra.json"), "w") as fh:
            fh.write(s)
    return s


# ---------------------------------------------------------------------------
# Operators — export (active clip / a ticked selection of clips)
# ---------------------------------------------------------------------------


def _bake_and_write(clip, context, convention):
    """Bake one clip (Layer 1) then run the scene's enabled converters
    (Layer 2) into animation/exported_clips/<clip>/. Returns
    (written, n_frames, max_simultaneous, warning_count). Raises
    ValueError if the clip can't be baked or yields no frames. Shared by
    Export Active Clip and Export Selected so both stay byte-identical."""
    scene = context.scene
    rows = bake_clip(clip, context)
    if not rows:
        raise ValueError(f"Clip '{clip}' produced no frames")

    # Soft guardrail: simultaneous-servo check (report only).
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
                f"WARNING {clip} frame {rows[i]['frame']}: {simultaneous} "
                f"servos moving >{_DELTA_THRESHOLD_DEG}° in one step "
                f"(limit {max_sim_cap})"
            )

    written = []
    if scene.fh_export_csv:
        to_csv(rows, clip)
        written.append("csv")
    if scene.fh_export_header:
        to_c_header(rows, clip, convention)
        written.append("h")
    if scene.fh_export_js:
        to_js(
            rows,
            clip,
            convention,
            loop=scene.fh_export_js_loop,
            dry_run=scene.fh_export_js_dryrun,
            esp_ip=scene.fh_esp_ip,
        )
        written.append("js-dry" if scene.fh_export_js_dryrun else "js")
    return written, len(rows), max_seen, warning_count


def _regenerate_clips_all(context, convention, names=None):
    """(Re)write the bundled firmware header clips_all.h + clips_manifest.json.

    `names` is the set of clips to bundle; None means every clip. clips_all.h is
    the firmware's source of truth, and the ticked export selection is what ends
    up on the robot, so the export operators pass that selection here. Clip
    ordering (and therefore clip id) is alphabetical by name — enforced by
    to_clips_header itself, so callers cannot accidentally shuffle ids. Bakes
    each clip and calls to_clips_header — the same path export_all_clips.py
    uses. A clip that fails to bake is skipped so the bundle still regenerates
    from the rest. Returns (bundled_names, skipped_msg).
    """
    wanted = set(names) if names is not None else None
    baked = {}
    skipped = []
    for clip in list_clips():
        if wanted is not None and clip not in wanted:
            continue
        try:
            rows = bake_clip(clip, context)
        except ValueError as e:
            skipped.append(f"{clip} ({e})")
            continue
        if rows:
            baked[clip] = rows
    if not baked:
        raise ValueError("no clips could be baked for clips_all.h")
    to_clips_header(baked, convention, write=True)
    return list(baked.keys()), ("; ".join(skipped) if skipped else None)


def _regenerate_clips_extra(context, convention, names=None):
    """(Re)write clips_extra.json (the app's streamable bundle). Defaults to ALL
    clips (names=None): clips_all.h carries only the ticked firmware subset, but
    the app bundle deliberately includes every clip so an animation is playable
    in the app the moment it's exported — no flash needed. Clip ordering is
    alphabetical (enforced by to_clips_extra_json). Skips clips that fail to
    bake. Returns (bundled_names, skipped_msg)."""
    wanted = set(names) if names is not None else None
    baked = {}
    skipped = []
    for clip in list_clips():
        if wanted is not None and clip not in wanted:
            continue
        try:
            rows = bake_clip(clip, context)
        except ValueError as e:
            skipped.append(f"{clip} ({e})")
            continue
        if rows:
            baked[clip] = rows
    if not baked:
        raise ValueError("no clips could be baked for clips_extra.json")
    to_clips_extra_json(baked, convention, write=True)
    return list(baked.keys()), ("; ".join(skipped) if skipped else None)


def _app_bundle_step(operator, scene, context, convention):
    """Regenerate clips_extra.json (ALL clips) and optionally copy it into the
    app, reporting warnings via `operator`. Returns a note string for the export
    summary ("" when the app bundle is disabled). Never raises — a failed app
    bundle must not fail the rest of the export."""
    if not scene.fh_export_app_bundle:
        return ""
    note = ""
    try:
        xnames, xskipped = _regenerate_clips_extra(context, convention)
        note = f" + clips_extra.json ({len(xnames)} clips)"
        print(f"Regenerated clips_extra.json from {len(xnames)} clip(s)")
        if xskipped:
            operator.report({"WARNING"}, f"clips_extra.json skipped: {xskipped}")
        if scene.fh_copy_extra_to_app:
            try:
                dest = _copy_extra_to_app()
                note += " (copied to app)"
                print(f"Copied clips_extra.json to app: {dest}")
            except ValueError as e:
                operator.report({"WARNING"}, f"clips_extra.json NOT copied to app: {e}")
    except ValueError as e:
        operator.report({"WARNING"}, f"clips_extra.json NOT regenerated: {e}")
    return note


class FH_OT_export_clip(bpy.types.Operator):
    """Bake the active clip once, then write the enabled outputs to
    animation/exported_clips/<clip>/ (.csv / .h / .js)."""

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
        if not (
            scene.fh_export_csv
            or scene.fh_export_header
            or scene.fh_export_js
            or scene.fh_export_app_bundle
        ):
            self.report({"WARNING"}, "No export formats enabled")
            return {"CANCELLED"}

        # convention only needed by the .h / .js converters
        convention = None
        if scene.fh_export_header or scene.fh_export_js or scene.fh_export_app_bundle:
            try:
                convention = _load_convention()
            except ValueError as e:
                self.report({"ERROR"}, str(e))
                return {"CANCELLED"}

        try:
            written, nframes, max_seen, warning_count = _bake_and_write(
                clip, context, convention
            )
        except ValueError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}

        rel = os.path.join("animation", "exported_clips", clip)
        print(
            f"Exported '{clip}' ({nframes} frames, {', '.join(written)}) "
            f"→ {rel}/  [max simultaneous servos: {max_seen}, "
            f"warnings: {warning_count}]"
        )

        # clips_all.h is the firmware set = the ticked export selection (the same
        # set Export Selected bundles). Regenerate from it so an active-clip
        # export keeps the bundle in sync without silently pulling in unticked
        # WIP clips. With nothing ticked, leave the bundle alone rather than
        # wiping it.
        bundle_note = ""
        if scene.fh_export_header:
            chosen = _export_selected_set()
            if not chosen:
                self.report(
                    {"WARNING"},
                    "clips_all.h NOT regenerated: no clips ticked for the firmware bundle",
                )
            else:
                try:
                    names, skipped = _regenerate_clips_all(context, convention, chosen)
                    bundle_note = f" + clips_all.h ({len(names)} clips)"
                    print(f"Regenerated clips_all.h from {len(names)} ticked clip(s)")
                    if skipped:
                        self.report({"WARNING"}, f"clips_all.h skipped: {skipped}")
                    if scene.fh_copy_to_firmware:
                        try:
                            dest = _copy_bundle_to_firmware()
                            bundle_note += " (copied to firmware)"
                            self.report({"INFO"}, f"clips_all.h copied to {dest}")
                            print(f"Copied clips_all.h to firmware: {dest}")
                        except ValueError as e:
                            self.report(
                                {"WARNING"}, f"clips_all.h NOT copied to firmware: {e}"
                            )
                except ValueError as e:
                    self.report({"WARNING"}, f"clips_all.h NOT regenerated: {e}")

        bundle_note += _app_bundle_step(self, scene, context, convention)

        self.report({"INFO"}, f"Exported to {rel}/ ({', '.join(written)}){bundle_note}")
        return {"FINISHED"}


class FH_OT_export_selected(bpy.types.Operator):
    """Bake + export every clip ticked via the per-row checkbox in the
    Clips sub-panel (fh_export_selected_clips), each into its own
    animation/exported_clips/<clip>/ with the enabled formats. A clip
    that fails to bake is skipped and reported; the rest still export."""

    bl_idname = "fh.export_selected"
    bl_label = "Export Selected Clips"
    bl_options = {"REGISTER"}

    def execute(self, context):
        scene = context.scene
        if not bpy.data.filepath:
            self.report({"ERROR"}, "Save the .blend file before exporting")
            return {"CANCELLED"}
        # _export_selected_set() already intersects with list_clips(),
        # so stale ticks (clip renamed/deleted since) are auto-pruned.
        # Sort for stable per-clip iteration order.
        chosen = sorted(_export_selected_set())
        if not chosen:
            self.report({"ERROR"}, "No clips ticked — select clips to export")
            return {"CANCELLED"}
        if not (
            scene.fh_export_csv
            or scene.fh_export_header
            or scene.fh_export_js
            or scene.fh_export_app_bundle
        ):
            self.report({"ERROR"}, "No export formats enabled")
            return {"CANCELLED"}

        convention = None
        if scene.fh_export_header or scene.fh_export_js or scene.fh_export_app_bundle:
            try:
                convention = _load_convention()
            except ValueError as e:
                self.report({"ERROR"}, str(e))
                return {"CANCELLED"}

        ok, failed = [], []
        for clip in chosen:
            try:
                written, nframes, max_seen, warns = _bake_and_write(
                    clip, context, convention
                )
            except ValueError as e:
                failed.append(f"{clip} ({e})")
                continue
            ok.append(f"{clip}[{','.join(written)}]")
            print(
                f"Exported '{clip}' ({nframes} frames, {', '.join(written)}) "
                f"[max simultaneous servos: {max_seen}, warnings: {warns}]"
            )

        if not ok:
            self.report({"ERROR"}, f"Exported nothing — {'; '.join(failed)}")
            return {"CANCELLED"}

        # The firmware bundle clips_all.h is exactly the ticked selection, so the
        # robot gets the curated set. Regenerate it from `chosen` (not every
        # clip), which replaces the previous bundle.
        bundle_note = ""
        if scene.fh_export_header:
            try:
                names, skipped = _regenerate_clips_all(context, convention, chosen)
                bundle_note = f" + clips_all.h ({len(names)} clips)"
                print(f"Regenerated clips_all.h from {len(names)} ticked clip(s)")
                if skipped:
                    self.report({"WARNING"}, f"clips_all.h skipped: {skipped}")
                if scene.fh_copy_to_firmware:
                    try:
                        dest = _copy_bundle_to_firmware()
                        bundle_note += " (copied to firmware)"
                        self.report({"INFO"}, f"clips_all.h copied to {dest}")
                        print(f"Copied clips_all.h to firmware: {dest}")
                    except ValueError as e:
                        self.report(
                            {"WARNING"}, f"clips_all.h NOT copied to firmware: {e}"
                        )
            except ValueError as e:
                self.report({"WARNING"}, f"clips_all.h NOT regenerated: {e}")

        bundle_note += _app_bundle_step(self, scene, context, convention)

        if failed:
            self.report(
                {"WARNING"},
                f"Exported {len(ok)}/{len(chosen)} ({', '.join(ok)}){bundle_note} → "
                f"animation/exported_clips/ | failed: {'; '.join(failed)}",
            )
        else:
            self.report(
                {"INFO"},
                f"Exported {len(ok)} clip(s) ({', '.join(ok)}){bundle_note} → "
                "animation/exported_clips/",
            )
        return {"FINISHED"}


class FH_OT_open_export_dir(bpy.types.Operator):
    """Open animation/exported_clips/ in the OS file browser."""

    bl_idname = "fh.open_export_dir"
    bl_label = "Open Exports Folder"
    bl_options = {"REGISTER"}

    def execute(self, context):
        path = _exported_clips_root()
        if not os.path.isdir(path):
            self.report({"WARNING"}, f"Folder does not exist yet: {path}")
            return {"CANCELLED"}
        bpy.ops.wm.path_open(filepath=path)
        return {"FINISHED"}


class FH_OT_open_firmware_dir(bpy.types.Operator):
    """Open the firmware clips folder (code/firmware/src/nervous_system/) in
    the OS file browser, where the copied clips_all.h lands."""

    bl_idname = "fh.open_firmware_dir"
    bl_label = "Open Firmware Folder"
    bl_options = {"REGISTER"}

    def execute(self, context):
        path = _firmware_clips_dir()
        if not os.path.isdir(path):
            self.report(
                {"WARNING"}, f"Firmware dir not found ({path}); not in a repo checkout?"
            )
            return {"CANCELLED"}
        bpy.ops.wm.path_open(filepath=path)
        return {"FINISHED"}


class FH_OT_open_app_dir(bpy.types.Operator):
    """Open the remote-control app's assets folder
    (code/remote-control-app/MyApp/assets/) in the OS file browser, where the
    copied clips_extra.json lands."""

    bl_idname = "fh.open_app_dir"
    bl_label = "Open App Folder"
    bl_options = {"REGISTER"}

    def execute(self, context):
        path = _app_clips_dir()
        if not os.path.isdir(path):
            self.report(
                {"WARNING"},
                f"App assets dir not found ({path}); not in a repo checkout?",
            )
            return {"CANCELLED"}
        bpy.ops.wm.path_open(filepath=path)
        return {"FINISHED"}


class FH_OT_reload_panel(bpy.types.Operator):
    """Re-read fh_clip_panel.py from disk and re-register the add-on
    in-place. Lets animators iterate on the add-on source without
    closing and relaunching Blender."""

    bl_idname = "fh.reload_panel"
    bl_label = "Reload Add-on"
    bl_options = {"REGISTER"}

    def execute(self, context):
        mod = sys.modules.get("fh_clip_panel")
        if mod is not None:
            try:
                mod.unregister()
            except Exception:
                # Best-effort: a partially-registered state shouldn't block
                # the reload — the importlib.reload() below replaces the
                # module object anyway.
                pass
            mod = importlib.reload(mod)
            mod.register()
        self.report({"INFO"}, "FaceHugger add-on reloaded")
        return {"FINISHED"}


# ---------------------------------------------------------------------------
# Panel
# ---------------------------------------------------------------------------


_PANEL_SPACE = "VIEW_3D"
_PANEL_REGION = "UI"


class FH_PT_root(bpy.types.Panel):
    """Parent panel — always-visible status (active clip + the not-keyed
    pose notice). Feature groups live in the collapsible sub-panels
    below (Poses / Clips / Selection / Export / Display)."""

    bl_idname = "VIEW3D_PT_fh_root"
    bl_label = "FaceHugger"
    bl_space_type = _PANEL_SPACE
    bl_region_type = _PANEL_REGION
    bl_category = CATEGORY

    def draw(self, context):
        layout = self.layout
        active = active_clip()

        # Reload-from-disk button — sits at the top as the obvious
        # "I just edited fh_clip_panel.py, pick up my changes" affordance.
        layout.operator(FH_OT_reload_panel.bl_idname, icon="FILE_REFRESH")

        header = layout.box()
        header.label(text=f"Active clip: {active or '<none>'}", icon="ACTION")

        # Not-keyed notice — a pose is live in the viewport but nothing
        # is committed to a clip yet (decision B).
        pending = context.scene.fh_unkeyed_pose
        if pending:
            frame = context.scene.fh_unkeyed_frame
            note = layout.box()
            note.label(text=f"Pose '{pending}' applied — not keyed", icon="ERROR")
            if active:
                note.operator(
                    FH_OT_key_pose_into_clip.bl_idname,
                    text=f"Key into '{active}' @ frame {frame}",
                    icon="KEY_HLT",
                )
            else:
                note.label(text="Apply a clip to enable keying", icon="INFO")


class _FH_PT_child:
    """Mixin for the collapsible sub-panels: all share space/region and
    parent onto FH_PT_root."""

    bl_space_type = _PANEL_SPACE
    bl_region_type = _PANEL_REGION
    bl_category = CATEGORY
    bl_parent_id = "VIEW3D_PT_fh_root"


class FH_PT_poses(_FH_PT_child, bpy.types.Panel):
    bl_idname = "VIEW3D_PT_fh_poses"
    bl_label = "Poses"
    bl_order = 0

    def draw(self, context):
        layout = self.layout
        try:
            pose_names = sorted(_load_poses().keys())
            bad_poses = False
        except ValueError:
            pose_names, bad_poses = [], True
        if bad_poses:
            layout.label(text="(poses.json invalid)", icon="ERROR")
        elif not pose_names:
            layout.label(text="(no saved poses)", icon="INFO")
        else:
            pcol = layout.column(align=True)
            for pname in pose_names:
                prow = pcol.row(align=True)
                prow.operator(
                    FH_OT_pose_apply.bl_idname, text=pname, icon="ARMATURE_DATA"
                ).pose_name = pname
                prow.operator(
                    FH_OT_pose_rename.bl_idname, text="", icon="FONT_DATA"
                ).pose_name = pname
                prow.operator(
                    FH_OT_pose_delete.bl_idname, text="", icon="X"
                ).pose_name = pname
        prow = layout.row(align=True)
        prow.prop(context.scene, "fh_pose_name", text="")
        prow.operator(FH_OT_pose_save.bl_idname, text="", icon="ADD")


class FH_PT_clips(_FH_PT_child, bpy.types.Panel):
    bl_idname = "VIEW3D_PT_fh_clips"
    bl_label = "Clips"
    bl_order = 1

    def draw(self, context):
        layout = self.layout
        active = active_clip()
        clips = list_clips()
        if not clips:
            layout.label(text="(no clips found)", icon="INFO")
        else:
            # Compute the export-selection set once per draw (not per row)
            # so each row checkbox is just a cheap membership test.
            selected = _export_selected_set()
            col = layout.column(align=True)
            for clip in clips:
                row = col.row(align=True)
                is_active = clip == active
                icon = "RADIOBUT_ON" if is_active else "RADIOBUT_OFF"
                op = row.operator(
                    FH_OT_apply_clip.bl_idname, text=clip, icon=icon, depress=is_active
                )
                op.clip_name = clip
                # Overwrite this clip with the current bound animation
                # (confirm dialog; destructive to the clip's old content).
                row.operator(
                    FH_OT_overwrite_clip.bl_idname, text="", icon="FILE_REFRESH"
                ).clip_name = clip
                # Include-in-export checkbox: drives Export Selected Clips.
                in_export = clip in selected
                chk_icon = "CHECKBOX_HLT" if in_export else "CHECKBOX_DEHLT"
                row.operator(
                    FH_OT_toggle_export_clip.bl_idname,
                    text="",
                    icon=chk_icon,
                    depress=in_export,
                ).clip_name = clip

        layout.separator()
        layout.label(text="New clip name:")
        layout.prop(context.scene, "fh_new_clip_name", text="")
        # New Clip / Save Current work with zero existing clips (always
        # enabled); Duplicate/Rename act on the active clip.
        layout.operator(FH_OT_new_clip.bl_idname, icon="ADD")
        layout.operator(FH_OT_save_as_clip.bl_idname, icon="FILE_TICK")
        row = layout.row(align=True)
        row.enabled = bool(active)
        row.operator(FH_OT_duplicate_clip.bl_idname, icon="DUPLICATE")
        row.operator(FH_OT_rename_clip.bl_idname, icon="FONT_DATA")

        # Toggle the active clip's keyframe interpolation between BEZIER
        # (authoring) and LINEAR (what the robot plays). Non-destructive:
        # Bezier handles are preserved so toggling back restores the curves.
        # The button reflects the active clip's CURRENT mode (read live from
        # the f-curves) so you can always see which one you're in: depressed
        # + "LINEAR" when previewing robot motion, raised + "BEZIER" when
        # authoring.
        layout.separator()
        mode = _clip_interpolation_mode(active)
        is_linear = mode == "LINEAR"
        if mode is None:
            label, icon = "Preview Robot Motion", "PREVIEW_RANGE"
        elif is_linear:
            label, icon = "Preview Robot Motion: LINEAR", "IPO_LINEAR"
        else:
            label, icon = "Preview Robot Motion: BEZIER", "IPO_BEZIER"
        preview_row = layout.row()
        preview_row.enabled = bool(active)
        preview_row.operator(
            FH_OT_toggle_preview.bl_idname, text=label, icon=icon, depress=is_linear
        )

        # Frame range of the active clip. Drives the scene timeline: switching
        # clips and edits here sync scene.frame_start/end (via apply-clip,
        # the msgbus live-sync, and the Apply to Scene button).
        action = clip_action(active, "body_ctrl") if active else None
        if action is not None:
            layout.separator()
            box = layout.column(align=True)
            box.label(text="Frame Range", icon="TIME")
            box.prop(action, "use_frame_range", text="Custom range")
            if action.use_frame_range:
                rng = box.row(align=True)
                rng.prop(action, "frame_start", text="Start")
                rng.prop(action, "frame_end", text="End")
            else:
                # Clip-wide span (union of all actions), not body_ctrl alone —
                # body_ctrl can be single-keyed while the feet carry the motion.
                span = _clip_frame_span(active)
                if span is not None:
                    box.label(text=f"Keyframes: {span[0]}–{span[1]}")
            box.operator(FH_OT_sync_frame_range.bl_idname, icon="PREVIEW_RANGE")


class FH_PT_selection(_FH_PT_child, bpy.types.Panel):
    bl_idname = "VIEW3D_PT_fh_selection"
    bl_label = "Selection"
    bl_order = 2

    def draw(self, context):
        layout = self.layout
        scol = layout.column(align=True)
        scol.operator(FH_OT_select_controls.bl_idname, text="All").preset = "ALL"
        srow = scol.row(align=True)
        srow.operator(FH_OT_select_controls.bl_idname, text="Body").preset = "BODY"
        srow.operator(FH_OT_select_controls.bl_idname, text="Legs").preset = "LEGS"
        srow = scol.row(align=True)
        srow.operator(FH_OT_select_controls.bl_idname, text="Front").preset = "FRONT"
        srow.operator(FH_OT_select_controls.bl_idname, text="Back").preset = "BACK"
        srow = scol.row(align=True)
        srow.operator(FH_OT_select_controls.bl_idname, text="FL").preset = "FL"
        srow.operator(FH_OT_select_controls.bl_idname, text="FR").preset = "FR"
        srow = scol.row(align=True)
        srow.operator(FH_OT_select_controls.bl_idname, text="BL").preset = "BL"
        srow.operator(FH_OT_select_controls.bl_idname, text="BR").preset = "BR"


class FH_PT_export(_FH_PT_child, bpy.types.Panel):
    bl_idname = "VIEW3D_PT_fh_export"
    bl_label = "Export"
    bl_order = 3

    def draw(self, context):
        layout = self.layout
        active = active_clip()

        # NOTE: the old Set N Pose / Set Flat Pose buttons were removed —
        # 'neutral' / 'flat' now ship as committed entries in the Poses
        # library (poses.json), so just Apply them from the Poses panel.
        # The fh.set_n_pose / fh.set_rest_pose operators stay registered
        # as a console re-seed path if those defaults are ever deleted:
        #   bpy.ops.fh.set_n_pose()  /  bpy.ops.fh.set_rest_pose()
        layout.prop(
            context.scene, "fh_max_simultaneous_servos", text="Max Simultaneous Servos"
        )
        col = layout.column(align=True)
        col.prop(context.scene, "fh_export_csv", text="CSV (raw angles)")
        col.prop(context.scene, "fh_export_header", text="C header (.h)")
        if context.scene.fh_export_header:
            sub = col.column(align=True)
            crow = sub.row(align=True)
            crow.separator()
            crow.prop(
                context.scene,
                "fh_copy_to_firmware",
                text="Copy clips_all.h to firmware",
            )
        col.prop(context.scene, "fh_export_js", text="Browser JS (.js)")
        if context.scene.fh_export_js:
            sub = col.column(align=True)
            sub.active = True
            iprow = sub.row(align=True)
            iprow.separator()
            iprow.prop(context.scene, "fh_esp_ip", text="ESP IP")
            jsrow = sub.row(align=True)
            jsrow.separator()
            jsrow.prop(context.scene, "fh_export_js_dryrun", text="Dry run")
            jsrow.prop(context.scene, "fh_export_js_loop", text="Loop")
        col.prop(
            context.scene, "fh_export_app_bundle", text="App clips (clips_extra.json)"
        )
        if context.scene.fh_export_app_bundle:
            sub = col.column(align=True)
            arow = sub.row(align=True)
            arow.separator()
            arow.prop(
                context.scene,
                "fh_copy_extra_to_app",
                text="Copy clips_extra.json to app",
            )

        row = layout.row()
        row.enabled = bool(active)
        row.operator(
            FH_OT_export_clip.bl_idname, text="Export Active Clip", icon="EXPORT"
        )

        # Selective multi-clip export. Selection lives on the per-row
        # checkbox in the Clips sub-panel; this button just consumes it.
        # Disabled when nothing is ticked so the action is unambiguous.
        if list_clips():
            sel_count = len(_export_selected_set())
            sel_row = layout.row()
            sel_row.enabled = sel_count > 0
            sel_row.operator(
                FH_OT_export_selected.bl_idname,
                text=f"Export Selected Clips ({sel_count})",
                icon="EXPORT",
            )

        # Jump to the relevant folders in the OS file browser.
        layout.separator()
        frow = layout.row(align=True)
        frow.operator(
            FH_OT_open_export_dir.bl_idname, text="Exports", icon="FILE_FOLDER"
        )
        frow.operator(
            FH_OT_open_firmware_dir.bl_idname, text="Firmware", icon="FILE_FOLDER"
        )
        frow.operator(FH_OT_open_app_dir.bl_idname, text="App", icon="FILE_FOLDER")


class FH_PT_display(_FH_PT_child, bpy.types.Panel):
    bl_idname = "VIEW3D_PT_fh_display"
    bl_label = "Display"
    bl_order = 4
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        layout.prop(
            context.scene,
            "fh_heatmap_active",
            text="Activity Heatmap",
            toggle=True,
            icon="COLORSET_01_VEC",
        )


# Per-joint safe servo range (= clampClipServos in the firmware / sim). A value
# at or beyond these bounds is flagged red in the Servo Angles panel: it means
# the joint is being driven into its mechanical stop.
_SERVO_SAFE_RANGE = {
    "shoulder": (38, 142),
    "thigh": (30, 150),
    "knee": (0, 180),
}


class FH_PT_servos(_FH_PT_child, bpy.types.Panel):
    """Live read-out of the 12 servo angles for the current rig pose — the
    exact degrees that would be exported / flashed. Updates as the animation
    plays so you can see, frame by frame, what each leg's servos are commanded.
    A value at/beyond its safe range is shown red (driven into the stop)."""

    bl_idname = "VIEW3D_PT_fh_servos"
    bl_label = "Servo Angles"
    bl_order = 5
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        servos = _current_servo_angles(context)
        if servos is None:
            layout.label(text="(rig or convention.json unavailable)", icon="INFO")
            return

        layout.label(
            text=f"Frame {context.scene.frame_current} — exported servo degrees"
        )
        # header
        hdr = layout.row(align=True)
        hdr.label(text="Leg")
        for t in ("Shldr", "Thigh", "Knee"):
            hdr.label(text=t)

        joints = ("shoulder", "thigh", "knee")
        col = layout.column(align=True)
        for leg in ("fr", "fl", "br", "bl"):
            row = col.row(align=True)
            row.label(text=leg.upper())
            for j, v in zip(joints, servos[leg]):
                lo, hi = _SERVO_SAFE_RANGE[j]
                cell = row.row(align=True)
                cell.alert = v <= lo or v >= hi  # red when at/over the stop
                cell.label(text=f"{v}°")


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

CLASSES = (
    FH_OT_apply_clip,
    FH_OT_new_clip,
    FH_OT_save_as_clip,
    FH_OT_set_n_pose,
    FH_OT_set_rest_pose,
    FH_OT_pose_save,
    FH_OT_pose_apply,
    FH_OT_pose_rename,
    FH_OT_pose_delete,
    FH_OT_key_pose_into_clip,
    FH_OT_select_controls,
    FH_OT_duplicate_clip,
    FH_OT_overwrite_clip,
    FH_OT_toggle_export_clip,
    FH_OT_rename_clip,
    FH_OT_export_clip,
    FH_OT_export_selected,
    FH_OT_open_export_dir,
    FH_OT_open_firmware_dir,
    FH_OT_open_app_dir,
    FH_OT_reload_panel,
    FH_OT_toggle_preview,
    FH_OT_sync_frame_range,
    # Panels: parent MUST be registered before its children so the
    # bl_parent_id link resolves.
    FH_PT_root,
    FH_PT_poses,
    FH_PT_clips,
    FH_PT_selection,
    FH_PT_export,
    FH_PT_display,
    FH_PT_servos,
)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.fh_new_clip_name = bpy.props.StringProperty(
        name="New Clip Name",
        description="Used by New Clip, Duplicate Clip and Rename Active Clip",
        default="",
    )
    bpy.types.Scene.fh_pose_name = bpy.props.StringProperty(
        name="Pose Name",
        description="Used by Save Current Pose and Rename Pose",
        default="",
    )
    bpy.types.Scene.fh_unkeyed_pose = bpy.props.StringProperty(
        name="Unkeyed Pose",
        description=(
            "Name of a pose that was applied to the viewport but not yet "
            "keyed into a clip. Empty = nothing pending. Drives the "
            "panel's not-keyed notice"
        ),
        default="",
    )
    bpy.types.Scene.fh_unkeyed_frame = bpy.props.IntProperty(
        name="Unkeyed Pose Frame",
        description="Frame the pending pose was applied at (where Key inserts it)",
        default=0,
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
    bpy.types.Scene.fh_copy_to_firmware = bpy.props.BoolProperty(
        name="Copy clips_all.h to firmware",
        description=(
            "After writing the bundle, copy clips_all.h straight into "
            "code/firmware/src/nervous_system/ so the firmware picks it up "
            "without a manual copy. Assumes the add-on runs from a repo "
            "checkout; warns (does not fail) if the firmware dir is absent"
        ),
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
    bpy.types.Scene.fh_export_js_loop = bpy.props.BoolProperty(
        name="Loop (diagnostic)",
        description=(
            "Opt in to loop playback in the generated .js. Default is OFF: "
            "play once, then hold the final pose (Phase-1/Phase-2 parity "
            "with the firmware tickClip — see the on-board clip-player spec)"
        ),
        default=False,
    )
    bpy.types.Scene.fh_export_js_dryrun = bpy.props.BoolProperty(
        name="Dry run (console echo)",
        description=(
            "Emit a .js variant that console.logs each {T:4,id,a} message "
            "instead of opening a WebSocket — validate a clip without a robot"
        ),
        default=False,
    )
    bpy.types.Scene.fh_esp_ip = bpy.props.StringProperty(
        name="ESP IP",
        description=(
            "Robot board IP baked into the exported .js WebSocket URL "
            "(ws://<ip>:81). Change it when the board's address changes "
            "(AP-mode default 192.168.4.1, or a DHCP lease). Blank falls back "
            "to 192.168.4.1; port 81 is fixed (see code/API_SPEC.md)"
        ),
        default="192.168.4.1",
    )
    bpy.types.Scene.fh_export_app_bundle = bpy.props.BoolProperty(
        name="Export App Clip Bundle",
        description=(
            "Write clips_extra.json — final servo-degree frames for EVERY clip "
            "so the remote-control app can stream them (T:4) without flashing. "
            "Independent of the firmware bundle / ticked selection: clips_all.h "
            "stays the curated on-robot set, the app bundle is everything"
        ),
        default=True,
    )
    bpy.types.Scene.fh_copy_extra_to_app = bpy.props.BoolProperty(
        name="Copy clips_extra.json to app",
        description=(
            "After writing it, copy clips_extra.json into the app's assets "
            "(code/remote-control-app/MyApp/assets/) so the app bundles it on "
            "the next build. Assumes a repo checkout; warns (does not fail) if "
            "the app dir is absent"
        ),
        default=True,
    )
    bpy.types.Scene.fh_export_selected_clips = bpy.props.StringProperty(
        name="Selected clips for export",
        description=(
            "Internal: newline-joined set of clip names ticked via the "
            "per-row checkbox in the Clips sub-panel. Read by "
            "'Export Selected Clips'; intersected with list_clips() on "
            "read so stale entries auto-prune."
        ),
        default="",
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

    # Rescue any clip-pattern Actions that pre-date this fix and would
    # otherwise be culled on the next .blend save (zero users + no
    # fake-user flag). Safe to re-run — re-marks idempotently.
    n = _rescue_existing_clip_actions()
    if n:
        print(f"[fh_clip_panel] rescued {n} clip Action(s) (fake-user set)")

    # Belt-and-suspenders against the inactive-clip-vanishes bug:
    # save_pre re-sweeps fake_user right before Blender serialises, so
    # even if any code path clears the flag mid-session the save still
    # preserves the clip. load_post handles second-file-opened-in-same-
    # session (register() only ran for the first file).
    if _on_save_pre not in bpy.app.handlers.save_pre:
        bpy.app.handlers.save_pre.append(_on_save_pre)
    if _on_load_post not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_on_load_post)

    # Live-sync the scene range when the active clip's Action range props are
    # edited in the panel (built-in props can't carry an update= callback).
    _subscribe_frame_range_msgbus()


def unregister():
    _unsubscribe_frame_range_msgbus()
    # Clean up heatmap state before class removal.
    handlers = bpy.app.handlers.frame_change_post
    if _heatmap_handler in handlers:
        handlers.remove(_heatmap_handler)
    _reset_servo_heatmap()
    _heatmap_prev_angles.clear()

    # Remove our save_pre / load_post guards.
    if _on_save_pre in bpy.app.handlers.save_pre:
        bpy.app.handlers.save_pre.remove(_on_save_pre)
    if _on_load_post in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_on_load_post)

    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
    for prop in (
        "fh_new_clip_name",
        "fh_pose_name",
        "fh_unkeyed_pose",
        "fh_unkeyed_frame",
        "fh_max_simultaneous_servos",
        "fh_export_csv",
        "fh_export_header",
        "fh_copy_to_firmware",
        "fh_export_js",
        "fh_export_js_loop",
        "fh_export_js_dryrun",
        "fh_esp_ip",
        "fh_export_app_bundle",
        "fh_copy_extra_to_app",
        "fh_export_selected_clips",
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
