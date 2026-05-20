"""
urdf_to_blender_rigged.py — Armature-based URDF importer with IK

Sibling of `visualize_urdf.py`. Same URDF parsing + scene bootstrap, but
builds a real Blender Armature so the rig can be posed/animated. The
animator-facing import path (placement-only `visualize_urdf.py` stays as
the cross-check against PyBullet).

Architecture:
  1. Parse URDF, compute link world matrices at rest pose (theta=0).
  2. Build a single Armature in edit mode:
       - One bone per URDF link (12 leg bones + base_link), named to
         match the URDF link names exactly (`fl_link1`, `fr_link2`, …).
       - Bone head = parent joint pivot world.
       - Bone tail = next joint pivot (or for `*_link3`, the foot tip),
         **projected onto the plane perpendicular to that joint's URDF
         axis** so bone-Y ⊥ joint axis. The projection is the
         precondition for `EditBone.align_roll(axis)` to land bone-local
         Z exactly on the joint axis. Without it, bone-local Z drifts
         from the joint axis (≈13° on `*_link1`, ≈23° on `*_link3`),
         breaking the spec's "rotation_euler[2] = joint angle" contract.
         The bone is a rotation control; mesh geometry is set
         independently in `attach_visuals` via `matrix_world`, so the
         visible limb is unaffected.
       - Bone roll = `EditBone.align_roll(joint_axis_world)`. After the
         tail projection above, **bone-local Z = the URDF joint axis**
         for all 12 joints uniformly.
       - Parent in chain via `EditBone.parent`, `use_connect=False`.

Constraints, applied in this order so the constraint stack evaluates as
[IK first, then LIMIT_ROTATION] (LIMIT_ROTATION clamps the IK-solved
pose, never the other way around). link1 yaw is driven analytically (no
constraint stack involvement) via a scripted driver, see (3c).

  3. Per leg:
       a. `foot_target_{leg}` (visible, sphere): at the actual CAD foot
          tip in world. Foot tip per leg: `Link3TipAxis` in
          `fusion_export.json` (falls back to
          `FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M`). For R-side legs
          (FR, BL), the X component is negated to match the mirrored
          `leg_lower.stl` visual (`<visual rpy="0 π 0">`). Animator
          drags this.
       b. `foot_ik_{leg}` (hidden, plain axes): `COPY_LOCATION` from
          `foot_target` with `use_offset=False` — tracks foot_target
          exactly. Kept as a separate Empty so the animator-facing
          handle is decoupled from the IK target.
       c. `shoulder_pivot_{leg}` (hidden, plain axes): bone-parented to
          `*_link1` at zero local offset at the bone HEAD, so its world
          XY tracks the shoulder pivot in every pose. Used by the link1
          yaw driver to read the live shoulder world XY position.
       d. IK constraint on `*_link3` pose bone: `target=foot_ik`,
          `chain_count=2` (covers link3 + link2), `use_tail=True`,
          `use_rotation=False`, `use_stretch=False`.
       e. Scripted driver on `*_link1` pose bone's `rotation_euler[2]`:
              atan2(foot_y - pivot_y, foot_x - pivot_x)
              - body_rot - REST_ANGLE_<leg>
          REST_ANGLE per leg is the Z component of the URDF
          `*_link1_joint`'s `<origin rpy>`. Variables read world-space
          LOC_X/LOC_Y from `foot_target_<leg>` and
          `shoulder_pivot_<leg>` plus world-space ROT_Z from `body_ctrl`.
          Replaces the legacy DAMPED_TRACK, which aimed in world space
          and ignored body_ctrl rotation — so the legs would twist
          against the body whenever body_ctrl yawed.

     Known limitation: bone tail is the projected foot tip (drops the
     joint-axis component of the foot tip in link3 frame, ≈ −Y in link3
     frame). Foot_ik / foot_target sit at the actual CAD foot tip. The
     two differ by ~13 mm along the joint axis at rest, so IK has a
     constant ~13 mm residual it cannot close — the gap is along the
     joint axis, perpendicular to every plane the IK can rotate the
     foot through. Cosmetic only; the visible foot mesh tip remains at
     foot_target's location.
  4. `lock_ik_x = lock_ik_y = True; lock_ik_z = False` plus
     `ik_stiffness_x/y = 1.0` plus `use_ik_limit_z` with `ik_min_z` /
     `ik_max_z` from URDF on `*_link2` and `*_link3` pose bones.
     Restricts the IK solver to bone-local Z rotation (= URDF joint
     axis) within the URDF range. link1 isn't in the IK chain so it
     doesn't get IK locks.
  5. `LIMIT_ROTATION` on every joint bone:
     - link2/link3: `use_limit_x=use_limit_y=True` with `min=max=0`
       (belt-and-suspenders for the IK locks), and `use_limit_z=True`
       with `min_z/max_z` from URDF `<limit>`.
     - link1: Z-only clamp (`use_limit_z=True`, `min_z/max_z` from
       URDF). X/Y are left UNLOCKED — the driver writes
       `rotation_euler[2]` directly and the X/Y lock from the
       Damped-Track era is no longer needed (it was only there to fight
       the world-space track).
  6. Mesh attach: `<visual>` STLs imported, parented to their link's
     bone, then `obj.matrix_world = link_world @ visual_origin` to
     match `visualize_urdf.py` byte-for-byte at rest.
  7. Joint-pivot/axis markers — `hide_viewport=True` by default.

Usage (GUI — recommended):
    /Applications/Blender-5.1.app/Contents/MacOS/Blender \
        --python animation/scripts/urdf_to_blender_rigged.py

Usage (headless — smoke test):
    /Applications/Blender-5.1.app/Contents/MacOS/Blender --background \
        --python animation/scripts/urdf_to_blender_rigged.py \
        -- --save /tmp/fh_rigged.blend

CLI flags (after the `--` separator per Blender convention):
    --urdf PATH      path to facehugger.urdf
    --meshes PATH    directory containing exported STLs
    --json PATH      fusion_export.json (foot tip from `Link3TipPoint`)
    --save PATH      write a .blend here after building the scene

Targets Blender 5.x.
"""

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
from mathutils import Matrix, Vector


# ---------------------------------------------------------------------------
# Constants — match visualize_urdf.py for cross-script overlay
# ---------------------------------------------------------------------------

M_TO_MM = 1000.0  # URDF metres → mm scene

SPHERE_RADIUS_MM = 3.0
AXIS_LENGTH_MM = 20.0

ROOT_COLLECTION = "URDFRigged"
ARMATURE_COLLECTION = "Armature"
MESHES_COLLECTION = "Meshes"
JOINTS_COLLECTION = "Joint Origins"
AXES_COLLECTION = "Joint Axes"
TARGETS_COLLECTION = "IK Targets"
CONTROLS_COLLECTION = "Controls"

COLOR_JOINT = (1.0, 0.0, 0.0, 1.0)  # red
COLOR_AXIS = (1.0, 0.4, 0.0, 1.0)  # orange

ARMATURE_NAME = "FaceHuggerRig"

# Foot tip in link3 frame (mm) — leg_lower.stl max-+Y centroid, copied
# from the URDF metadata comment block. This is the AUTHORITATIVE value
# used to place each leg's foot_target Empty and the link3 bone tail
# (after axis-parallel projection). The JSON `Link3TipPoint` is read
# only as a cross-check: if it differs from this constant by > 5 mm,
# main() prints a warning so the CAD construction point gets moved to
# coincide with the mesh tip.
FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M = Vector((-0.030946, -0.013000, 0.000024))

# Naming patterns
LEG_IDS = ("fl", "fr", "bl", "br")
R_SIDE_LEGS = ("fr", "bl")  # legs whose link3 visual has rpy=(0, π, 0)
FOOT_TARGET_FMT = "foot_target_{leg}"
FOOT_IK_FMT = "foot_ik_{leg}"
SHOULDER_PIVOT_FMT = "shoulder_pivot_{leg}"
SHOULDER_BONE_FMT = "{leg}_link1"
KNEE_BONE_FMT = "{leg}_link3"


def _foot_tip_for_leg(leg, foot_tip_in_link3_m):
    """Return the foot tip in link3 frame, accounting for the L/R-side
    mesh mirroring. The leg_lower.stl mesh is shared between sides;
    L-side legs (FL, BR) use it with `<visual rpy="0 0 0">` and R-side
    legs (FR, BL) with `<visual rpy="0 π 0">` — a Y-axis 180° rotation.
    The foot tip, defined in the STL frame, therefore lives at -X in
    link3 frame for R-side legs (relative to the L-side reference value
    loaded from `Link3TipAxis`). Without this flip, FR/BL foot Empties
    are placed on the wrong side of the leg in world.

    The Z component of the foot tip mirrors too in principle, but its
    URDF magnitude is ~24μm — below visual precision, ignored.
    """
    if leg in R_SIDE_LEGS:
        return Vector(
            (-foot_tip_in_link3_m.x, foot_tip_in_link3_m.y, foot_tip_in_link3_m.z)
        )
    return foot_tip_in_link3_m


# ---------------------------------------------------------------------------
# URDF parsing — copy of visualize_urdf.py's parser plus _parse_limit
# ---------------------------------------------------------------------------


def _parse_origin(elem):
    o = elem.find("origin")
    if o is None:
        return Matrix.Identity(4)
    xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
    rx = Matrix.Rotation(rpy[0], 3, "X")
    ry = Matrix.Rotation(rpy[1], 3, "Y")
    rz = Matrix.Rotation(rpy[2], 3, "Z")
    M = (rz @ ry @ rx).to_4x4()
    M.translation = Vector(xyz)
    return M


def _parse_axis(elem):
    a = elem.find("axis")
    if a is None:
        return Vector((1.0, 0.0, 0.0))
    xyz = [float(v) for v in a.get("xyz").split()]
    v = Vector(xyz)
    return v.normalized() if v.length > 0 else Vector((1.0, 0.0, 0.0))


def _parse_limit(elem):
    """Return (lower_rad, upper_rad) from <joint><limit>, or (None, None)
    for fixed joints / missing limits."""
    lim = elem.find("limit")
    if lim is None:
        return (None, None)
    try:
        return (float(lim.get("lower")), float(lim.get("upper")))
    except (TypeError, ValueError):
        return (None, None)


def _parse_visual(v):
    g = v.find("geometry/mesh")
    if g is None:
        return None
    fname = g.get("filename")
    scale = [float(s) for s in (g.get("scale") or "1 1 1").split()]
    return (_parse_origin(v), fname, scale)


def parse_urdf(path):
    """Same shape as visualize_urdf.parse_urdf but joints carry limits too."""
    tree = ET.parse(path)
    root_elem = tree.getroot()
    if root_elem.tag != "robot":
        sys.exit(f"{path}: root tag is <{root_elem.tag}>, expected <robot>")

    links = {}
    for link_elem in root_elem.findall("link"):
        name = link_elem.get("name")
        visuals = []
        for v in link_elem.findall("visual"):
            parsed = _parse_visual(v)
            if parsed is not None:
                visuals.append(parsed)
        links[name] = visuals

    joints = {}
    for j in root_elem.findall("joint"):
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        lower, upper = _parse_limit(j)
        joints[child] = {
            "parent": parent,
            "origin": _parse_origin(j),
            "axis": _parse_axis(j),
            "name": j.get("name"),
            "type": j.get("type"),
            "lower": lower,
            "upper": upper,
        }

    children = set(joints.keys())
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        sys.exit(f"{path}: expected exactly one root link, got {roots}")
    return {"root": roots[0], "links": links, "joints": joints}


# ---------------------------------------------------------------------------
# Forward kinematics at rest pose (all joint angles 0)
# ---------------------------------------------------------------------------


def compute_link_world(robot):
    """{link_name: 4x4 matrix in metres, rest pose}."""
    link_world = {robot["root"]: Matrix.Identity(4)}
    remaining = dict(robot["joints"])
    while remaining:
        placed = []
        for child, j in remaining.items():
            if j["parent"] in link_world:
                link_world[child] = link_world[j["parent"]] @ j["origin"]
                placed.append(child)
        if not placed:
            sys.exit(
                f"Dangling joint chain — could not place: {list(remaining.keys())}"
            )
        for c in placed:
            del remaining[c]
    return link_world


def load_foot_tip_in_link3_frame(json_path):
    """Read `Link2ToLink3Axis` (construction axis) and `Link3TipPoint`
    (construction point) from `fusion_export.json` and return the foot
    tip position in link3-local frame (metres), or `None` if either is
    missing.

    `Link2ToLink3Axis` lives in each occurrence's `axes[]` (key
    `origin_mm`); `Link3TipPoint` lives in `points[]` (key `pos_mm`).
    Both are in leg-assembly-source frame; the foot-relative-to-knee
    delta lives in that same frame, so apply `R_la = Rz(90°)` — the
    same rotation `ExportBodiesToURDF` pre-applies to joint axes — to
    convert to link3-local.
    """
    if not json_path or not Path(json_path).exists():
        return None
    try:
        data = json.loads(Path(json_path).read_text())
    except (OSError, json.JSONDecodeError) as e:
        print(f"[warn] could not read {json_path}: {e}")
        return None

    knee_source_mm = [None]
    foot_source_mm = [None]

    def _walk(node):
        for axis in node.get("axes", []) or []:
            xyz = axis.get("origin_mm")
            if (
                axis.get("name") == "Link2ToLink3Axis"
                and knee_source_mm[0] is None
                and xyz
                and len(xyz) >= 3
            ):
                knee_source_mm[0] = Vector(xyz[:3])
        for point in node.get("points", []) or []:
            xyz = point.get("pos_mm")
            if (
                point.get("name") == "Link3TipPoint"
                and foot_source_mm[0] is None
                and xyz
                and len(xyz) >= 3
            ):
                foot_source_mm[0] = Vector(xyz[:3])
        for child in node.get("children", []) or []:
            if knee_source_mm[0] is not None and foot_source_mm[0] is not None:
                return
            _walk(child)

    for occ in data.get("occurrences", []) or []:
        _walk(occ)
        if knee_source_mm[0] is not None and foot_source_mm[0] is not None:
            break

    if knee_source_mm[0] is None or foot_source_mm[0] is None:
        return None

    delta_source_mm = foot_source_mm[0] - knee_source_mm[0]
    import math as _math

    R_la = Matrix.Rotation(_math.pi / 2.0, 4, "Z")
    return (R_la @ delta_source_mm) / M_TO_MM


def load_body_bottom_point(json_path):
    """World position (metres) of `BodyBottomPoint` — the body's Z=0
    datum under the cage. It lives in `FlexibleSkeleton:1`, the
    base_link occurrence whose world transform is identity, so its
    `pos_mm` is already in the base_link / rig-world frame. Returns a
    Vector (metres) or None if absent (not yet re-exported from Fusion)."""
    if not json_path or not Path(json_path).exists():
        return None
    try:
        data = json.loads(Path(json_path).read_text())
    except (OSError, json.JSONDecodeError) as e:
        print(f"[warn] could not read {json_path}: {e}")
        return None

    found = [None]

    def _walk(node):
        for point in node.get("points", []) or []:
            if point.get("name") == "BodyBottomPoint" and found[0] is None:
                xyz = point.get("pos_mm")
                if xyz and len(xyz) >= 3:
                    found[0] = Vector(xyz[:3]) / M_TO_MM
        for child in node.get("children", []) or []:
            if found[0] is None:
                _walk(child)

    for occ in data.get("occurrences", []) or []:
        _walk(occ)
        if found[0] is not None:
            break
    return found[0]


def matrix_m_to_mm(M):
    out = M.copy()
    out.translation = out.translation * M_TO_MM
    return out


# ---------------------------------------------------------------------------
# Scene bootstrap (same idiom as visualize_urdf.py)
# ---------------------------------------------------------------------------


# stash_actions / restore_actions preserve Object-level animation data
# across clear_scene(). The full-rebuild flow deletes every Object
# datablock, which is the only "real user" of the body_ctrl and
# foot_target_* Actions (no NLA strips, no bone-level F-curves in the
# current rig). Without intervention, those Actions become orphaned
# (users=0) and Blender garbage-collects them on the next save — losing
# every keyframe the animator authored. Setting use_fake_user=True
# before the wipe holds the Action alive across the rebuild; after the
# new rig is built, restore_actions() rebinds each Action by Object name
# AND by explicit slot identifier. Auto-slot assignment when setting
# `anim_data.action = action` is heuristic and unreliable in Blender 5.x
# (per the official ActionSlot docs); explicit
# `anim_data.action_slot = action.slots[identifier]` is the canonical
# pattern.
def stash_actions():
    """Capture every (object_name, action_name, slot_identifier) triple
    currently in use; set use_fake_user=True on each Action so it
    survives clear_scene(). Return the captured list."""
    stash = []
    for obj in bpy.data.objects:
        ad = obj.animation_data
        if ad is None or ad.action is None:
            continue
        action = ad.action
        slot = ad.action_slot
        if slot is None and action.slots:
            slot = action.slots[0]
        slot_identifier = slot.identifier if slot is not None else None
        action.use_fake_user = True
        stash.append((obj.name, action.name, slot_identifier))
        print(
            f"[urdf_to_blender_rigged] stash: {obj.name!r} -> "
            f"action={action.name!r} slot={slot_identifier!r}"
        )
    return stash


def restore_actions(stash):
    """Rebind each stashed Action to its now-recreated Object by name,
    explicitly setting `action_slot` from the stashed identifier (with
    `action_suitable_slots[0]` as fallback). Drop use_fake_user after
    binding — the assigned animation_data is now a real user."""
    n_restored = 0
    for obj_name, action_name, slot_identifier in stash:
        obj = bpy.data.objects.get(obj_name)
        if obj is None:
            print(
                f"[urdf_to_blender_rigged] restore: object {obj_name!r} "
                f"not in rebuilt scene — skipping action {action_name!r}"
            )
            continue
        action = bpy.data.actions.get(action_name)
        if action is None:
            print(
                f"[urdf_to_blender_rigged] restore: action {action_name!r} "
                "missing from bpy.data.actions (was it GC'd between "
                "stash and rebuild?)"
            )
            continue
        ad = obj.animation_data_create()
        ad.action = action
        slot = None
        if slot_identifier:
            slot = action.slots.get(slot_identifier)
        if slot is None and ad.action_suitable_slots:
            slot = ad.action_suitable_slots[0]
        if slot is not None:
            ad.action_slot = slot
        action.use_fake_user = False
        bound_id = slot.identifier if slot is not None else None
        print(
            f"[urdf_to_blender_rigged] restore: {obj_name!r} <- "
            f"action={action_name!r} slot={bound_id!r}"
        )
        n_restored += 1
    print(f"[urdf_to_blender_rigged] restored {n_restored}/{len(stash)} action(s)")


def clear_scene():
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for coll in list(bpy.data.collections):
        bpy.data.collections.remove(coll)
    for mat in list(bpy.data.materials):
        bpy.data.materials.remove(mat)
    for mesh in list(bpy.data.meshes):
        bpy.data.meshes.remove(mesh)
    for arm in list(bpy.data.armatures):
        bpy.data.armatures.remove(arm)
    _configure_units()


def _configure_units():
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 0.001  # 1 BU = 1 mm
    scene.unit_settings.length_unit = "MILLIMETERS"
    for area in bpy.context.screen.areas if bpy.context.screen else []:
        if area.type == "VIEW_3D":
            for space in area.spaces:
                if space.type == "VIEW_3D":
                    space.clip_start = 0.1
                    space.clip_end = 10000.0


def make_collections():
    scene_coll = bpy.context.scene.collection
    root = bpy.data.collections.new(ROOT_COLLECTION)
    scene_coll.children.link(root)
    children = {ROOT_COLLECTION: root}
    for name in (
        ARMATURE_COLLECTION,
        MESHES_COLLECTION,
        TARGETS_COLLECTION,
        CONTROLS_COLLECTION,
        JOINTS_COLLECTION,
        AXES_COLLECTION,
    ):
        c = bpy.data.collections.new(name)
        root.children.link(c)
        children[name] = c
    # Joint markers off in viewport AND render by default; toggle the
    # eye / camera icon in the outliner when debugging. Without the
    # render hide, the red/orange spheres show up as confusing dots in
    # any rendered output.
    children[JOINTS_COLLECTION].hide_viewport = True
    children[JOINTS_COLLECTION].hide_render = True
    children[AXES_COLLECTION].hide_viewport = True
    children[AXES_COLLECTION].hide_render = True
    return children


def make_material(name, rgba):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    principled = next((n for n in nodes if n.type == "BSDF_PRINCIPLED"), None)
    if principled is not None:
        principled.inputs["Base Color"].default_value = rgba
    mat.diffuse_color = rgba
    return mat


# ---------------------------------------------------------------------------
# Armature construction
# ---------------------------------------------------------------------------


def _bone_endpoints_world_mm(robot, link_world, link_name, foot_tip_in_link3_m):
    """Return (head_mm, tail_mm) for the bone representing `link_name`.

    head = link's frame origin in world (where the parent joint pivot sits).
    tail = next joint's origin world (= child link's frame origin) when the
           link has exactly one child joint, else falls back to a per-link
           heuristic (`foot_tip_in_link3_m` for `*_link3`, +Y bump for
           base_link).

    `foot_tip_in_link3_m` MUST match the value used to place each leg's
    `foot_target` Empty so that `*_link3` bone tail and the foot Empty
    coincide at rest (zero IK error). Caller responsibility — main
    threads the same value through `place_ik_targets_and_constraints`.
    """
    head_m = link_world[link_name].to_translation()

    # Find a child joint (link N+1) whose parent is this link, if any.
    child_joint_origin_m = None
    for child_link, j in robot["joints"].items():
        if j["parent"] == link_name:
            child_joint_origin_m = link_world[child_link].to_translation()
            break

    if link_name.endswith("_link1"):
        # Aim link1 bone-Y at the foot tip (instead of at the hip pivot)
        # so Damped Track on link1 is identity at the URDF rest pose.
        # If bone-Y aimed at the hip (the natural child-joint origin),
        # shoulder→hip and shoulder→foot would differ by ~19° in the
        # horizontal plane (the leg has bends at hip and knee at rest),
        # and Damped Track would yaw link1 by that angle when the
        # dep-graph evaluates — visually "contorting" the rest pose by
        # ~88 mm at the foot. With the foot-tip aim, Damped Track is
        # zero at rest and only fires when the animator drags
        # foot_target away from its rest position.
        #
        # Joint axis for link1 is world +Z, so the perpendicular-to-
        # axis projection step below drops the Z component into the
        # horizontal plane. bone-local Z stays = world +Z, alignment
        # invariant unchanged. _foot_tip_for_leg applies the R-side
        # X-flip so FR/BL legs aim correctly toward their own feet.
        leg = link_name.split("_", 1)[0]
        link3_name = KNEE_BONE_FMT.format(leg=leg)
        tail_m = link_world[link3_name] @ _foot_tip_for_leg(leg, foot_tip_in_link3_m)
    elif child_joint_origin_m is not None:
        tail_m = child_joint_origin_m
    elif link_name.endswith("_link3"):
        leg = link_name.split("_", 1)[0]
        tail_m = link_world[link_name] @ _foot_tip_for_leg(leg, foot_tip_in_link3_m)
    else:
        # base_link or any leaf without a foot definition — 50 mm bump in
        # +Y world so the bone is visible.
        tail_m = head_m + Vector((0.0, 0.050, 0.0))

    # Bone-Y MUST be perpendicular to the URDF joint axis so that
    # EditBone.align_roll(axis) lands exactly on bone-local Z = axis.
    # Where the link-to-link offset has a component along the joint axis
    # (link1 has a Z lift; link3 foot tip has a Y component), project that
    # component out — the bone becomes a guide rod for rotation, not a
    # visual limb. Mesh geometry is set independently via matrix_world in
    # attach_visuals(), so the limb still looks correct.
    j = robot["joints"].get(link_name)
    if j is not None and j["axis"].length > 1e-6:
        axis_world = (link_world[link_name].to_3x3() @ j["axis"]).normalized()
        bone_dir = tail_m - head_m
        proj = bone_dir - bone_dir.dot(axis_world) * axis_world
        if proj.length > 1e-6:
            tail_m = head_m + proj

    return head_m * M_TO_MM, tail_m * M_TO_MM


def _joint_axis_world(robot, link_world, link_name):
    """The URDF joint axis for the joint that drives `link_name`, expressed
    in world frame at rest pose. None if `link_name` has no incoming joint
    (i.e., the root)."""
    j = robot["joints"].get(link_name)
    if j is None:
        return None
    # joint axis is in joint frame == child link frame at rest.
    return link_world[link_name].to_3x3() @ j["axis"]


def build_armature(robot, link_world, collection, foot_tip_in_link3_m):
    """Create the armature, bones, parenting, and roll. Returns the
    armature object. `foot_tip_in_link3_m` is forwarded to
    `_bone_endpoints_world_mm` for `*_link3` tail placement; pass the
    same value used for IK target placement so bone tail and IK target
    coincide at rest.
    """
    bpy.ops.object.armature_add(
        enter_editmode=False,
        align="WORLD",
        location=(0.0, 0.0, 0.0),
    )
    arm_obj = bpy.context.object
    arm_obj.name = ARMATURE_NAME
    arm_obj.data.name = ARMATURE_NAME + "_data"

    # Move into the armature collection.
    for c in list(arm_obj.users_collection):
        c.objects.unlink(arm_obj)
    collection.objects.link(arm_obj)

    # Iterate-until-stable so parents always exist before children.
    bpy.ops.object.mode_set(mode="EDIT")
    edit_bones = arm_obj.data.edit_bones

    # Remove the default bone armature_add gives us.
    for b in list(edit_bones):
        edit_bones.remove(b)

    placed = set()
    pending = set(robot["links"].keys())
    while pending:
        added_this_pass = []
        for link_name in list(pending):
            j = robot["joints"].get(link_name)
            if j is not None and j["parent"] not in placed:
                continue
            head_mm, tail_mm = _bone_endpoints_world_mm(
                robot, link_world, link_name, foot_tip_in_link3_m
            )
            bone = edit_bones.new(link_name)
            bone.head = head_mm
            bone.tail = tail_mm
            axis_world = _joint_axis_world(robot, link_world, link_name)
            if axis_world is not None and axis_world.length > 1e-6:
                bone.align_roll(axis_world)
            if j is not None:
                bone.parent = edit_bones[j["parent"]]
                bone.use_connect = False
            placed.add(link_name)
            added_this_pass.append(link_name)
        if not added_this_pass:
            sys.exit(f"Dangling links — could not place: {list(pending)}")
        for n in added_this_pass:
            pending.discard(n)

    bpy.ops.object.mode_set(mode="POSE")
    for pb in arm_obj.pose.bones:
        pb.rotation_mode = "XYZ"
    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.context.view_layer.update()
    return arm_obj


def add_limit_rotation_constraints(robot, arm_obj):
    """`LIMIT_ROTATION` per joint bone:

    - link2 / link3 (in the IK chain): X and Y rotation locked to 0
      (belt-and-suspenders for the IK locks); Z limited to URDF
      `<limit lower upper>`.
    - link1 (driven analytically by a scripted driver — see
      `add_link1_yaw_drivers`): Z-only clamp. X/Y are deliberately
      LEFT UNLOCKED. The X/Y=0 lock used to exist to keep
      Damped Track from leaking pitch/roll into link1, but the
      driver writes `rotation_euler[2]` directly and the constraint
      stack no longer needs to fight a world-space track.

    Added LAST in the constraint stack so it clamps after IK. Owner
    space = LOCAL throughout (bone-local axes).
    """
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode="POSE")
    n = 0
    for child_link, j in robot["joints"].items():
        if j["lower"] is None or j["upper"] is None:
            continue
        pb = arm_obj.pose.bones.get(child_link)
        if pb is None:
            print(f"[warn] no pose bone for joint child link {child_link!r}")
            continue
        is_link1 = child_link.endswith("_link1")
        c = pb.constraints.new("LIMIT_ROTATION")
        c.use_limit_x = not is_link1
        c.min_x = 0.0
        c.max_x = 0.0
        c.use_limit_y = not is_link1
        c.min_y = 0.0
        c.max_y = 0.0
        c.use_limit_z = True
        c.min_z = j["lower"]
        c.max_z = j["upper"]
        c.owner_space = "LOCAL"
        n += 1
    bpy.ops.object.mode_set(mode="OBJECT")
    return n


def lock_ik_axes(robot, arm_obj):
    """IK-side joint constraints on `*_link2` and `*_link3` pose bones
    (the bones in the IK chain — `chain_count=2` from the constraint on
    link3 covers link3 + link2):

    - `lock_ik_x = lock_ik_y = True` plus `ik_stiffness_x/y = 1.0`:
      restrict the IK solver to bone-local Z = URDF joint axis.
    - `use_ik_limit_z = True` with `ik_min_z`/`ik_max_z` from URDF
      `<limit lower upper>`: solver respects URDF range during the
      iterative solve, not just the post-hoc `LIMIT_ROTATION` clamp.
      This keeps the solver from oscillating against an out-of-range
      target.

    link1 is not in any IK chain so it has no IK locks; its yaw-only
    motion is enforced by Damped Track + LIMIT_ROTATION instead.
    """
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode="POSE")
    n = 0
    for pb in arm_obj.pose.bones:
        if not (pb.name.endswith("_link2") or pb.name.endswith("_link3")):
            continue
        j = robot["joints"].get(pb.name)
        if j is None:
            continue
        pb.lock_ik_x = True
        pb.lock_ik_y = True
        pb.lock_ik_z = False
        pb.ik_stiffness_x = 1.0
        pb.ik_stiffness_y = 1.0
        pb.ik_stiffness_z = 0.0
        if j["lower"] is not None and j["upper"] is not None:
            pb.use_ik_limit_z = True
            pb.ik_min_z = j["lower"]
            pb.ik_max_z = j["upper"]
        n += 1
    bpy.ops.object.mode_set(mode="OBJECT")
    return n


# ---------------------------------------------------------------------------
# Mesh attachment
# ---------------------------------------------------------------------------


def _import_stl(stl_path, name):
    before = set(bpy.data.objects)
    try:
        bpy.ops.wm.stl_import(filepath=str(stl_path), global_scale=1.0)
    except Exception as e:
        print(f"[warn] STL import failed for {stl_path}: {e}")
        return None
    new_objs = [o for o in bpy.data.objects if o not in before]
    if not new_objs:
        print(f"[warn] STL import created no objects: {stl_path}")
        return None
    obj = new_objs[0]
    obj.name = name
    return obj


def attach_visuals(robot, link_world, arm_obj, meshes_dir, target_collection):
    """Import every <visual> STL and parent it to its link's bone via
    parent_type='BONE'. After parenting, set matrix_world to the
    rest-pose target so geometry overlays visualize_urdf.py byte-for-byte."""
    placed = 0
    for link_name, visuals in robot["links"].items():
        link_W = link_world[link_name]
        for visual_origin, mesh_rel, _scale in visuals:
            mesh_path = (meshes_dir / Path(mesh_rel).name).resolve()
            if not mesh_path.exists():
                mesh_path = Path(mesh_rel)
                if not mesh_path.exists():
                    print(f"[warn] mesh missing: {mesh_rel}")
                    continue
            obj_name = f"{link_name}__{Path(mesh_rel).stem}"
            obj = _import_stl(mesh_path, obj_name)
            if obj is None:
                continue
            for c in list(obj.users_collection):
                c.objects.unlink(obj)
            target_collection.objects.link(obj)
            obj.parent = arm_obj
            obj.parent_type = "BONE"
            obj.parent_bone = link_name
            obj.matrix_parent_inverse = Matrix.Identity(4)
            bpy.context.view_layer.update()
            obj.matrix_world = matrix_m_to_mm(link_W @ visual_origin)
            placed += 1
    return placed


# ---------------------------------------------------------------------------
# IK targets + IK + Damped Track
# ---------------------------------------------------------------------------


def _add_empty(name, location_mm, target_collection, kind="SPHERE", size_mm=8.0):
    bpy.ops.object.empty_add(type=kind, radius=size_mm, location=tuple(location_mm))
    obj = bpy.context.object
    obj.name = name
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    target_collection.objects.link(obj)
    return obj


def place_ik_targets_and_constraints(
    robot, link_world, arm_obj, collection, foot_tip_in_link3_m, world_origin
):
    """Per leg, set up the foot targets, the shoulder pivot, and the IK:

    `foot_target_{leg}` (visible, blue ball) — placed at the actual CAD
    foot tip in world. Per-leg foot tip from `_foot_tip_for_leg(leg, …)`
    so R-side legs (FR, BL) get the X-flipped value to match their
    mirrored leg_lower.stl visual. Animator drags this.

    `foot_ik_{leg}` (hidden) — `COPY_LOCATION` from `foot_target` with
    `use_offset=False` and zero stored offset, so foot_ik tracks
    foot_target exactly. Kept as a separate Empty for clarity (animator
    handle vs IK target are decoupled — easy to retarget later).

    `shoulder_pivot_{leg}` (hidden) — PLAIN_AXES empty bone-parented to
    `*_link1` at zero local offset at the bone HEAD. Bone parenting
    rides body_ctrl AND the link1 pre-driver transform, so the empty's
    world XY always equals the shoulder pivot's world XY. Read by the
    link1 yaw driver as `pivot_x` / `pivot_y`.

    Known limitation: bone tail is the projected foot tip (= drop the
    joint-axis component of `foot_tip_in_link3_m`), which sits ~13 mm
    away from the actual CAD foot tip along the joint axis (the
    component the projection removed, ≈ link3-frame Y). foot_ik sits
    at the actual CAD foot tip, so at rest IK has a constant ~13 mm
    residual it cannot close — the gap is along the joint axis,
    perpendicular to every plane the IK can rotate the foot through.
    Cosmetic; the visible foot mesh tip remains at foot_target's
    location to within numerical tolerance.

    IK on `*_link3` pose bone: `target=foot_ik`, `chain_count=2`,
    `use_tail=True`, `use_rotation=False`, `use_stretch=False`.

    The link1 yaw driver (replacing the legacy DAMPED_TRACK constraint)
    is wired in a separate pass — `add_link1_yaw_drivers` — once
    `body_ctrl` has been created, since the driver reads `body_ctrl`
    ROT_Z to subtract the chassis yaw.

    foot_target is parented to `world_origin` with
    matrix_parent_inverse=Identity, so it rides the anchor as a rigid
    handle. foot_ik is intentionally UNPARENTED: its COPY_LOCATION
    constraint computes
    `final = foot_target.world + owner.pre_constraint_world`
    (use_offset=True, owner_space=WORLD), and that owner pre-constraint
    world must stay constant for the offset to remain the static
    foot-tip → bone-tail delta. Parenting foot_ik to world_origin too
    would make pre_constraint_world ride the anchor as well, doubling
    the delta when world_origin moves and breaking IK.
    """
    bpy.ops.object.mode_set(mode="OBJECT")
    n_targets = 0
    n_ik = 0
    n_pivots = 0

    for leg in LEG_IDS:
        link3 = KNEE_BONE_FMT.format(leg=leg)
        link1 = SHOULDER_BONE_FMT.format(leg=leg)
        if link3 not in link_world:
            continue

        foot_tip_per_leg = _foot_tip_for_leg(leg, foot_tip_in_link3_m)
        actual_foot_world_m = link_world[link3] @ foot_tip_per_leg
        actual_foot_world_mm = actual_foot_world_m * M_TO_MM

        # Projected foot tip in world (= link3 bone tail position) —
        # drop the component of foot_tip_per_leg along link3's joint
        # axis, then transform to world. This must match what
        # _bone_endpoints_world_mm produces for the link3 tail so that
        # foot_ik and bone tail coincide at rest (zero IK error).
        joint_axis_link3 = robot["joints"][link3]["axis"]
        projected_link3 = (
            foot_tip_per_leg - foot_tip_per_leg.dot(joint_axis_link3) * joint_axis_link3
        )
        projected_foot_world_m = link_world[link3] @ projected_link3
        offset_world_mm = (projected_foot_world_m - actual_foot_world_m) * M_TO_MM

        # Visible foot target at actual CAD foot tip — animator drags this.
        print(
            f"[DEBUG] placing foot_target_{leg} at {tuple(round(v, 3) for v in actual_foot_world_mm)} mm"
        )
        foot_target = _add_empty(
            FOOT_TARGET_FMT.format(leg=leg),
            actual_foot_world_mm,
            collection,
            kind="SPHERE",
            size_mm=8.0,
        )
        foot_target.parent = world_origin
        foot_target.parent_type = "OBJECT"
        foot_target.matrix_parent_inverse = Matrix.Identity(4)
        n_targets += 1

        # Hidden foot_ik with COPY_LOCATION use_offset=True. Stored
        # location is the constant world offset (projected - actual);
        # the constraint reads foot_target's world location and adds
        # this offset, so foot_ik visually sits at the projected bone
        # tail. IK targets foot_ik with use_tail=True, so bone tail =
        # foot_ik = projected at rest → zero IK error. As the animator
        # drags foot_target, foot_ik tracks with the constant world
        # offset (foot_ik moves by the same delta).
        #
        # Trade-off: the offset is static in world space, but the leg's
        # joint axis rotates with link1 yaw — so under large leg poses
        # the actual mesh foot tip can drift up to ~13 mm from
        # foot_target. Acceptable for typical animation amplitude; for
        # exact mesh-tip-on-target tracking, swap to a driver that
        # recomputes the offset from the live link3 matrix.
        foot_ik = _add_empty(
            FOOT_IK_FMT.format(leg=leg),
            offset_world_mm,
            collection,
            kind="PLAIN_AXES",
            size_mm=4.0,
        )
        foot_ik.hide_viewport = True
        cop = foot_ik.constraints.new("COPY_LOCATION")
        cop.target = foot_target
        cop.target_space = "WORLD"
        cop.owner_space = "WORLD"
        cop.use_offset = True

        # IK on link3.
        bpy.context.view_layer.objects.active = arm_obj
        bpy.ops.object.mode_set(mode="POSE")
        knee_pb = arm_obj.pose.bones.get(link3)
        if knee_pb is not None:
            ik = knee_pb.constraints.new("IK")
            ik.target = foot_ik
            ik.chain_count = 2
            ik.use_rotation = False
            ik.use_tail = True
            ik.use_stretch = False
            n_ik += 1
        bpy.ops.object.mode_set(mode="OBJECT")

        # `shoulder_pivot_<leg>`: hidden PLAIN_AXES Empty bone-parented
        # to `*_link1` at zero local offset at the bone HEAD. Blender's
        # bone-parent origin is the bone TAIL by default, so we set
        # `matrix_parent_inverse = Translation((0, -bone.length, 0))` in
        # bone-local space to land at HEAD. With `location=(0,0,0)` the
        # empty sits at the shoulder pivot in every pose — the link1
        # yaw driver reads its world LOC_X/LOC_Y as `pivot_x`/`pivot_y`.
        link1_bone = arm_obj.data.bones.get(link1)
        if link1_bone is not None:
            pivot = _add_empty(
                SHOULDER_PIVOT_FMT.format(leg=leg),
                (0.0, 0.0, 0.0),  # overwritten by matrix_parent_inverse below
                collection,
                kind="PLAIN_AXES",
                size_mm=4.0,
            )
            pivot.parent = arm_obj
            pivot.parent_type = "BONE"
            pivot.parent_bone = link1
            pivot.matrix_parent_inverse = Matrix.Translation(
                (0.0, -link1_bone.length, 0.0)
            )
            pivot.location = (0.0, 0.0, 0.0)
            pivot.rotation_euler = (0.0, 0.0, 0.0)
            pivot.hide_viewport = True
            n_pivots += 1

    return n_targets, n_ik, n_pivots


def add_link1_yaw_drivers(robot, link_world, foot_tip_in_link3_m, arm_obj, body_ctrl):
    """Scripted driver on each `*_link1` pose bone's `rotation_euler[2]`:

        axis_sign * (
            atan2(foot_y - pivot_y, foot_x - pivot_x)
            - body_rot
            - REST_ANGLE
        )

    Variables (all `transform_space='WORLD_SPACE'`):
      * `foot_x`, `foot_y`   — `foot_target_<leg>` LOC_X / LOC_Y
      * `pivot_x`, `pivot_y` — `shoulder_pivot_<leg>` LOC_X / LOC_Y
      * `body_rot`           — `body_ctrl` ROT_Z

    Per-leg numeric literals baked into each expression:

    * `REST_ANGLE` is the world-frame angle from the rest shoulder
      pivot to the rest foot target, computed analytically from
      `link_world` (the URDF rest-pose forward-kinematics solution in
      metres) rather than from depsgraph-evaluated empties — reading
      the live empties at build time can drift by ~1° because of the
      `Target -> Driver` depsgraph cycle (shoulder_pivot is
      bone-parented to link1 which has a driver reading
      shoulder_pivot). The analytical form sidesteps the depsgraph
      and matches the steady-state evaluated geometry. The driver
      evaluates to 0 at rest by construction, regardless of URDF rpy
      choice or per-side mesh conventions — it is NOT the URDF
      `<origin rpy>` Z component (those values differ from the
      world-frame foot direction by π for FL/BR on FaceHugger).

    * `axis_sign = sign(joint.axis.z)` absorbs the per-side joint axis
      flip. For L-side legs (FL, BR) the URDF axis is `+Z` and
      `EditBone.align_roll` makes bone-local +Z = world +Z, so
      `rotation_euler[2]` = world yaw rotation — sign +1. For R-side
      legs (FR, BL) the axis is `-Z`, bone-local +Z = world -Z, so a
      positive `rotation_euler[2]` rotates the bone clockwise in
      world — sign -1 keeps the analytical formula consistent across
      all four legs without per-leg branching of the driver
      structure (only the literal `axis_sign` differs).

    Because `shoulder_pivot_<leg>` is bone-parented to `*_link1` (and
    the armature is parented to `body_ctrl`), its world XY moves
    rigidly with body_ctrl — `atan2(foot - pivot) − body_rot` is the
    link1's local yaw regardless of where the body is in world.

    Replaces the legacy DAMPED_TRACK constraint on link1, which aimed
    in world space and ignored body_ctrl rotation: with the rig
    parented under body_ctrl, the armature's local frame rotated with
    body_ctrl but Damped Track kept aiming at the world-space foot, so
    the legs twisted relative to the body instead of staying on their
    targets.

    Called AFTER `add_body_control` so `body_ctrl` exists as a driver
    target, AND after `place_ik_targets_and_constraints` so the
    `foot_target_*` / `shoulder_pivot_*` empties exist with their rest
    world positions. Returns the number of drivers installed.
    """
    import math as _math

    n = 0
    for leg in LEG_IDS:
        link1 = SHOULDER_BONE_FMT.format(leg=leg)
        link3 = KNEE_BONE_FMT.format(leg=leg)
        pb = arm_obj.pose.bones.get(link1)
        if pb is None:
            continue
        joint = robot["joints"].get(link1)
        foot = bpy.data.objects.get(FOOT_TARGET_FMT.format(leg=leg))
        pivot = bpy.data.objects.get(SHOULDER_PIVOT_FMT.format(leg=leg))
        if (
            joint is None
            or foot is None
            or pivot is None
            or link1 not in link_world
            or link3 not in link_world
        ):
            print(f"[warn] link1 driver: missing joint/foot/pivot for leg {leg!r}")
            continue

        # REST_ANGLE — world yaw from rest shoulder pivot to rest foot
        # target. Computed analytically from `link_world` (the URDF rest
        # pose forward-kinematics solution) rather than from depsgraph-
        # evaluated empties, because in some Blender 5.x cycle-resolution
        # orderings the bone-parented `shoulder_pivot_<leg>` is not yet
        # at its bone-HEAD world position when this function runs (off
        # by ~1°, see `Target -> Driver` cycle warnings on rig build).
        # Reading link_world (a numpy/mathutils Matrix in metres,
        # populated by `compute_link_world` at build start) sidesteps the
        # depsgraph entirely and matches the steady-state geometry by
        # construction.
        pivot_rest_m = link_world[link1].to_translation()
        foot_rest_m = link_world[link3] @ _foot_tip_for_leg(leg, foot_tip_in_link3_m)
        rest_yaw = _math.atan2(
            foot_rest_m.y - pivot_rest_m.y,
            foot_rest_m.x - pivot_rest_m.x,
        )

        # Per-side joint-axis sign.
        axis_sign = 1 if joint["axis"].z >= 0 else -1

        # Driver writes rotation_euler[2] directly, so enforce XYZ Euler
        # rotation mode (Blender default; defensive in case earlier code
        # changed it).
        pb.rotation_mode = "XYZ"

        fc = pb.driver_add("rotation_euler", 2)
        drv = fc.driver
        drv.type = "SCRIPTED"
        # Clear pre-existing variables (idempotent on rebuild).
        for v in list(drv.variables):
            drv.variables.remove(v)

        def _add_xform_var(name, target_obj, transform_type):
            v = drv.variables.new()
            v.name = name
            v.type = "TRANSFORMS"
            t = v.targets[0]
            t.id = target_obj
            t.transform_type = transform_type
            t.transform_space = "WORLD_SPACE"

        _add_xform_var("foot_x", foot, "LOC_X")
        _add_xform_var("foot_y", foot, "LOC_Y")
        _add_xform_var("pivot_x", pivot, "LOC_X")
        _add_xform_var("pivot_y", pivot, "LOC_Y")
        _add_xform_var("body_rot", body_ctrl, "ROT_Z")

        drv.expression = (
            f"({axis_sign:+d}) * ("
            f"atan2(foot_y - pivot_y, foot_x - pivot_x)"
            f" - body_rot - ({rest_yaw!r}))"
        )
        n += 1
    return n


# ---------------------------------------------------------------------------
# Body control (single Empty drives the whole rig)
# ---------------------------------------------------------------------------


def add_body_control(arm_obj, controls_collection, body_bottom_m=None):
    """Single `body_ctrl` Empty (CUBE, 80 mm) whose ORIGIN sits at the
    body's bottom (`BodyBottomPoint`, world ~(0,0,-17) mm) so the
    animator's handle is the body's ground-contact point — dragging its
    Z directly reads "height of the body bottom". Reparents the armature
    OBJECT to body_ctrl so moving body_ctrl translates/rotates the whole
    rig; the armature's parent-inverse cancels the body_ctrl offset so
    the rig's WORLD placement is unchanged (URDF kinematics untouched —
    joint angles / exports are origin-independent).

    `body_bottom_m` is BodyBottomPoint's world position in metres (from
    fusion_export.json). Falls back to the world origin if absent — then
    behaves exactly as before.

    The armature object is hidden afterward — animator's primary handle
    becomes the visible cube. Bones still evaluate (hide_viewport on
    the object hides display, not constraint evaluation), and the
    armature can be unhidden temporarily for per-bone pose-mode work.
    """
    if body_bottom_m is None:
        body_bottom_mm = Vector((0.0, 0.0, 0.0))
    else:
        body_bottom_mm = Vector(body_bottom_m) * M_TO_MM

    bpy.ops.object.empty_add(type="CUBE", radius=80.0, location=tuple(body_bottom_mm))
    body_ctrl = bpy.context.object
    body_ctrl.name = "body_ctrl"
    for c in list(body_ctrl.users_collection):
        c.objects.unlink(body_ctrl)
    controls_collection.objects.link(body_ctrl)

    arm_obj.parent = body_ctrl
    arm_obj.parent_type = "OBJECT"
    # Cancel body_ctrl's offset so the armature stays at the URDF world
    # origin regardless of where body_ctrl's pivot is (rig must not move).
    arm_obj.matrix_parent_inverse = Matrix.Translation(body_bottom_mm).inverted()
    arm_obj.hide_viewport = True
    return body_ctrl


def create_world_origin(controls_collection):
    """Fixed `world_origin` Empty (PLAIN_AXES, 50 mm) at world (0, 0, 0).
    Deterministic root that body_ctrl + foot Empties parent to:

        world_origin   (anchor — never animated)
          ├── body_ctrl   (chassis xform — animated)
          │     └── FaceHuggerRig   (armature)
          ├── foot_target_{leg}   (animator handle — animated)
          └── foot_ik_{leg}   (hidden; COPY_LOCATION from foot_target)

    Rationale: a future `.fhc` exporter walks the scene graph and needs a
    deterministic root to express animated transforms against. Pinning
    that root to world identity (rather than reading each object's
    matrix_world directly) keeps every animated object's local
    transform = its keyed value, independent of whatever
    matrix_parent_inverse the import history left behind.

    Caller is responsible for parenting body_ctrl (inline in main) and
    the foot Empties (in place_ik_targets_and_constraints) to the
    returned world_origin.
    """
    bpy.ops.object.empty_add(type="PLAIN_AXES", radius=50.0, location=(0.0, 0.0, 0.0))
    world_origin = bpy.context.object
    world_origin.name = "world_origin"
    for c in list(world_origin.users_collection):
        c.objects.unlink(world_origin)
    controls_collection.objects.link(world_origin)
    return world_origin


def _diagnose_foot_target_placement(robot, link_world, arm_obj):
    """Print, per leg, the actual mesh foot tip in world (max-+Y
    centroid of `leg_lower` STL) versus `foot_target` Empty's world
    position — both at rest pose.

    Uses URDF rest matrices (`link_world[link3] @ visual_origin`) to
    compute the rest-pose mesh world; Blender's `mesh_obj.matrix_world`
    reflects the LIVE pose after Damped Track + IK fire on the
    placeholder rest, which is misleading for placement diagnosis.

    Expected: mesh tip and foot_target agree to within sub-mm. If they
    don't, the JSON `Link3TipAxis` was placed at a different reference
    in CAD than the mesh max-+Y centroid (which the URDF metadata
    comment `FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M` captures). Fix is
    either to move the construction axis in CAD, or switch the rigger
    to use the fallback constant.

    NOTE: this diagnostic does NOT measure the ~13 mm bone-tail-vs-
    foot_target gap (projection that preserves align_roll exactness) —
    that's rig-internal and documented in
    `place_ik_targets_and_constraints`.
    """
    # Damped Track rest-identity check: per leg, compare each link1's
    # bone-Y (in armature space, from matrix_local) to the direction
    # from link1's head to foot_target. If they align at rest, Damped
    # Track applies zero rotation at the URDF rest pose; only when the
    # animator drags foot_target does it rotate link1 — preserving
    # auto-yaw on movement while eliminating the 88 mm rest contortion.
    print("[urdf_to_blender_rigged] Damped Track rest-identity check (link1):")
    import math as _math2

    for leg in LEG_IDS:
        link1_name = SHOULDER_BONE_FMT.format(leg=leg)
        target = bpy.data.objects.get(FOOT_TARGET_FMT.format(leg=leg))
        bone = arm_obj.data.bones.get(link1_name)
        if bone is None or target is None:
            continue
        ml = bone.matrix_local
        bone_y = Vector(ml.col[1][:3]).normalized()
        head_world = Vector(ml.col[3][:3])
        target_world = target.matrix_world.translation
        dir_to_target = (target_world - head_world).normalized()
        # Full 3D angle includes elevation; LIMIT_ROTATION X/Y locks
        # block elevation rotation, so it doesn't matter for rest pose.
        cos_3d = max(-1.0, min(1.0, bone_y.dot(dir_to_target)))
        angle_3d_deg = _math2.degrees(_math2.acos(cos_3d))
        # Yaw-only angle (horizontal projection) IS what Damped Track
        # can actually apply through the X/Y locks. ~0° at rest means
        # no rest-pose contortion; auto-yaw still fires when the
        # animator drags foot_target horizontally.
        bone_y_h = Vector((bone_y.x, bone_y.y, 0.0))
        target_h = Vector((dir_to_target.x, dir_to_target.y, 0.0))
        if bone_y_h.length > 1e-9 and target_h.length > 1e-9:
            cos_yaw = max(
                -1.0,
                min(1.0, bone_y_h.normalized().dot(target_h.normalized())),
            )
            angle_yaw_deg = _math2.degrees(_math2.acos(cos_yaw))
        else:
            angle_yaw_deg = float("nan")
        print(
            f"  [{leg}_link1] 3D={angle_3d_deg:6.3f}° "
            f"yaw-only={angle_yaw_deg:6.3f}° "
            f"(yaw-only is what passes through LIMIT_ROTATION's X/Y locks; "
            f"~0° = no rest contortion)"
        )

    # Spot-check the FL Empties before iterating: parent (should be None),
    # location vs matrix_world.translation (should match — foot_target is
    # unparented and unmoved), COPY_LOCATION settings on foot_ik.
    fl_target = bpy.data.objects.get(FOOT_TARGET_FMT.format(leg="fl"))
    fl_ik = bpy.data.objects.get(FOOT_IK_FMT.format(leg="fl"))
    if fl_target is not None:
        loc = fl_target.location
        mw = fl_target.matrix_world.translation
        print(
            f"[urdf_to_blender_rigged] foot_target_fl: "
            f"location={tuple(round(v, 3) for v in loc)} mm, "
            f"matrix_world.translation={tuple(round(v, 3) for v in mw)} mm, "
            f"parent={fl_target.parent.name if fl_target.parent else None}"
        )
    if fl_ik is not None:
        cop = next((c for c in fl_ik.constraints if c.type == "COPY_LOCATION"), None)
        if cop is not None:
            print(
                f"[urdf_to_blender_rigged] foot_ik_fl: "
                f"COPY_LOCATION target={cop.target.name if cop.target else None} "
                f"target_space={cop.target_space} owner_space={cop.owner_space} "
                f"use_offset={cop.use_offset}"
            )

    print("[urdf_to_blender_rigged] foot_target vs mesh tip (rest pose):")
    for leg in LEG_IDS:
        link3 = KNEE_BONE_FMT.format(leg=leg)
        mesh_obj = bpy.data.objects.get(f"{leg}_link3__leg_lower")
        target_obj = bpy.data.objects.get(FOOT_TARGET_FMT.format(leg=leg))
        if mesh_obj is None or target_obj is None:
            print(
                f"  [{leg}] missing object — "
                f"mesh={mesh_obj is not None}, target={target_obj is not None}"
            )
            continue
        verts = mesh_obj.data.vertices
        if not verts:
            print(f"  [{leg}] empty mesh data")
            continue
        max_y = max(v.co.y for v in verts)
        tol_y = 0.01  # mesh-local mm
        tip_verts = [v.co for v in verts if v.co.y >= max_y - tol_y]
        n = len(tip_verts)
        centroid_local = sum(tip_verts, Vector((0.0, 0.0, 0.0))) / n

        # Rest-pose mesh world matrix from URDF: link_world[link3] @
        # visual_origin (for the leg_lower visual specifically).
        visual_origin = Matrix.Identity(4)
        for vis_origin, mesh_rel, _scale in robot["links"][link3]:
            if Path(mesh_rel).stem == "leg_lower":
                visual_origin = vis_origin
                break
        mesh_rest_world_m = link_world[link3] @ visual_origin
        mesh_rest_world_mm = matrix_m_to_mm(mesh_rest_world_m)
        centroid_world = mesh_rest_world_mm @ centroid_local

        target_world = target_obj.matrix_world.translation
        delta_mm = (target_world - centroid_world).length
        print(
            f"  [{leg}] mesh tip = "
            f"({centroid_world.x:+8.2f}, {centroid_world.y:+8.2f}, {centroid_world.z:+7.2f}) mm "
            f"({n} verts at max Y={max_y:.3f}); "
            f"foot_target = ({target_world.x:+8.2f}, {target_world.y:+8.2f}, {target_world.z:+7.2f}) mm; "
            f"|delta| = {delta_mm:6.3f} mm"
        )


# ---------------------------------------------------------------------------
# Joint markers (parented to bones; hidden by default)
# ---------------------------------------------------------------------------


def _add_marker(
    world_pos_mm, name, material, target_collection, radius_mm=SPHERE_RADIUS_MM
):
    before = set(bpy.data.objects)
    bpy.ops.mesh.primitive_uv_sphere_add(
        radius=radius_mm,
        location=(world_pos_mm[0], world_pos_mm[1], world_pos_mm[2]),
        segments=16,
        ring_count=8,
    )
    new_objs = [o for o in bpy.data.objects if o not in before]
    if not new_objs:
        return None
    obj = new_objs[0]
    obj.name = name
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    target_collection.objects.link(obj)
    if obj.data.materials:
        obj.data.materials[0] = material
    else:
        obj.data.materials.append(material)
    return obj


def place_joint_markers(
    robot, link_world, arm_obj, mat_joint, mat_axis, joints_collection, axes_collection
):
    """Red sphere at each joint pivot, orange sphere along the axis tip.
    Both parented to the joint's child bone with parent_type='BONE' so
    they follow pose-mode rotations."""
    n_joints = 0
    n_axes = 0
    bpy.ops.object.mode_set(mode="OBJECT")
    for child_link, j in robot["joints"].items():
        child_W = link_world[child_link]
        pivot_mm = child_W.translation * M_TO_MM
        pivot = _add_marker(
            pivot_mm, f"{j['name']}__pivot", mat_joint, joints_collection
        )
        if pivot is not None:
            pivot.parent = arm_obj
            pivot.parent_type = "BONE"
            pivot.parent_bone = child_link
            pivot.matrix_parent_inverse = Matrix.Identity(4)
            bpy.context.view_layer.update()
            pivot.matrix_world = Matrix.Translation(pivot_mm)
            n_joints += 1

        axis_world = child_W.to_3x3() @ j["axis"]
        tip_mm = pivot_mm + axis_world * AXIS_LENGTH_MM
        tip = _add_marker(
            tip_mm, f"{j['name']}__axis_tip", mat_axis, axes_collection, radius_mm=2.0
        )
        if tip is not None:
            tip.parent = arm_obj
            tip.parent_type = "BONE"
            tip.parent_bone = child_link
            tip.matrix_parent_inverse = Matrix.Identity(4)
            bpy.context.view_layer.update()
            tip.matrix_world = Matrix.Translation(tip_mm)
            n_axes += 1
    return n_joints, n_axes


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _script_dir():
    try:
        return Path(__file__).resolve().parent
    except NameError:
        return Path.cwd()


def parse_args():
    default_urdf = (
        _script_dir()
        / ".."
        / ".."
        / "code"
        / "simulation"
        / "generated"
        / "facehugger.urdf"
    )
    default_meshes = (
        _script_dir()
        / ".."
        / ".."
        / "code"
        / "simulation"
        / "generated"
        / "exported_meshes"
    )
    default_json = (
        _script_dir()
        / ".."
        / ".."
        / "code"
        / "simulation"
        / "generated"
        / "fusion_export.json"
    )
    default_save = _script_dir() / ".." / ".." / "animation" / "fh_rigged_latest.blend"

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=default_urdf.resolve())
    parser.add_argument("--meshes", type=Path, default=default_meshes.resolve())
    parser.add_argument(
        "--json",
        type=Path,
        default=default_json.resolve(),
        help="fusion_export.json path. Used to cross-check the foot tip "
        "from the Link3TipPoint construction point against "
        "FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M (URDF metadata, mesh-derived). "
        "Warns if they disagree by more than 5 mm.",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=default_save.resolve(),
        help="Output .blend path. Defaults to animation/fh_rigged_latest.blend "
        "so re-running the script always produces a refreshed file next to "
        "the in-progress animation .blend files. Pass an explicit path to "
        "override.",
    )

    argv = sys.argv
    argv = argv[argv.index("--") + 1 :] if "--" in argv else []
    return parser.parse_args(argv)


def main():
    args = parse_args()
    print(f"[urdf_to_blender_rigged] urdf   : {args.urdf}")
    print(f"[urdf_to_blender_rigged] meshes : {args.meshes}")
    print(f"[urdf_to_blender_rigged] json   : {args.json}")

    if not args.urdf.exists():
        raise SystemExit(f"URDF not found: {args.urdf}")

    robot = parse_urdf(args.urdf)
    print(
        f"[urdf_to_blender_rigged] parsed — {len(robot['links'])} links, "
        f"{len(robot['joints'])} joints, root={robot['root']}"
    )
    link_world = compute_link_world(robot)

    # Foot tip source of truth: JSON `Link3TipPoint` is the CAD-side
    # construction point the user explicitly places at the visible foot
    # tip (the curved claw end of leg_lower.stl). The URDF metadata's
    # `FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M` is the mesh's max-+Y
    # centroid, which can sit on a different feature of the mesh
    # entirely (the upper-section +Y face, not the claw tip). Prefer
    # the JSON; fall back to the constant only when the JSON is
    # missing.
    body_bottom_m = load_body_bottom_point(args.json)
    if body_bottom_m is not None:
        print(
            "[urdf_to_blender_rigged] BodyBottomPoint: world "
            f"{tuple(round(v * M_TO_MM, 3) for v in body_bottom_m)} mm "
            "— body_ctrl pivot placed here"
        )
    else:
        print(
            "[urdf_to_blender_rigged] BodyBottomPoint not in export — "
            "body_ctrl stays at world origin (re-export from Fusion to use it)"
        )

    foot_tip_from_json = load_foot_tip_in_link3_frame(args.json)
    if foot_tip_from_json is not None:
        foot_tip_in_link3_m = foot_tip_from_json
        src = f"Link3TipPoint in {args.json.name}"
    else:
        foot_tip_in_link3_m = FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M
        src = "FOOT_TIP_FALLBACK (URDF metadata fallback — Link3TipPoint missing from JSON)"
    print(
        f"[urdf_to_blender_rigged] foot tip: link3-local "
        f"{tuple(round(v * M_TO_MM, 3) for v in foot_tip_in_link3_m)} mm ({src})"
    )

    stash = stash_actions()
    clear_scene()
    collections = make_collections()
    mat_joint = make_material("JointPivotRed", COLOR_JOINT)
    mat_axis = make_material("JointAxisOrange", COLOR_AXIS)

    # Created early so place_ik_targets_and_constraints and the
    # body_ctrl reparenting downstream can both parent to it without
    # a second pass.
    world_origin = create_world_origin(collections[CONTROLS_COLLECTION])

    arm_obj = build_armature(
        robot, link_world, collections[ARMATURE_COLLECTION], foot_tip_in_link3_m
    )
    n_bones = len(arm_obj.data.bones)

    n_meshes = attach_visuals(
        robot, link_world, arm_obj, args.meshes, collections[MESHES_COLLECTION]
    )
    # Diagnostic for FL leg foot_target placement: surfaces every
    # variable that goes into the world position so the chain is
    # visible end-to-end.
    fl_link3_world_m = link_world["fl_link3"]
    fl_foot_tip_link3_m = _foot_tip_for_leg("fl", foot_tip_in_link3_m)
    fl_foot_world_m = fl_link3_world_m @ fl_foot_tip_link3_m
    print(
        f"[urdf_to_blender_rigged] FL chain:\n"
        f"  link_world['fl_link3'].translation = "
        f"{tuple(round(v * M_TO_MM, 3) for v in fl_link3_world_m.translation)} mm\n"
        f"  _foot_tip_for_leg('fl', …)         = "
        f"{tuple(round(v * M_TO_MM, 3) for v in fl_foot_tip_link3_m)} mm "
        "(in link3 frame)\n"
        f"  link3_world @ foot_tip_link3       = "
        f"{tuple(round(v * M_TO_MM, 3) for v in fl_foot_world_m)} mm "
        "(world, where foot_target_fl will be placed)"
    )

    # Constraint stack order: IK first, LIMIT_ROTATION last (so it clamps
    # the IK-solved pose, not the other way). New constraints append to
    # the stack, so call order = stack order. link1 yaw is driven
    # analytically — no constraint stack involvement — so the driver
    # pass happens after body_ctrl exists (driver target).
    n_targets, n_ik, n_pivots = place_ik_targets_and_constraints(
        robot,
        link_world,
        arm_obj,
        collections[TARGETS_COLLECTION],
        foot_tip_in_link3_m,
        world_origin,
    )
    n_lim = add_limit_rotation_constraints(robot, arm_obj)
    n_locked = lock_ik_axes(robot, arm_obj)
    n_joints, n_axes = place_joint_markers(
        robot,
        link_world,
        arm_obj,
        mat_joint,
        mat_axis,
        collections[JOINTS_COLLECTION],
        collections[AXES_COLLECTION],
    )

    _diagnose_foot_target_placement(robot, link_world, arm_obj)
    body_ctrl = add_body_control(
        arm_obj, collections[CONTROLS_COLLECTION], body_bottom_m
    )
    body_ctrl.parent = world_origin
    body_ctrl.parent_type = "OBJECT"
    body_ctrl.matrix_parent_inverse = Matrix.Identity(4)
    n_drivers = add_link1_yaw_drivers(
        robot, link_world, foot_tip_in_link3_m, arm_obj, body_ctrl
    )
    _verify_auto_yaw(arm_obj, robot, link_world, foot_tip_in_link3_m)
    restore_actions(stash)

    max_err_deg = _check_bone_z_alignment(robot, link_world, arm_obj)
    print(
        f"[urdf_to_blender_rigged] armature: {n_bones} bones, "
        f"{n_lim} LIMIT_ROTATION (link2/3 X/Y locked, Z = URDF range; "
        f"link1 Z-only), {n_ik} IK (chain=2, no stretch), "
        f"{n_drivers} link1 yaw drivers, "
        f"{n_locked} bones with IK X/Y locked"
    )
    print(
        f"[urdf_to_blender_rigged] targets : {n_targets} foot_target "
        f"+ {n_targets} foot_ik (hidden, COPY_LOCATION exact) "
        f"+ {n_pivots} shoulder_pivot (hidden, bone-parented to link1)"
    )
    print(
        f"[urdf_to_blender_rigged] controls: {body_ctrl.name} "
        f"({body_ctrl.empty_display_type}, armature parented to it, "
        f"armature.hide_viewport=True), "
        f"parented to {world_origin.name} (PLAIN_AXES, world anchor)"
    )
    print(
        f"[urdf_to_blender_rigged] geometry: {n_meshes} visuals, "
        f"{n_joints} joint pivots, {n_axes} axis tips (markers hidden)"
    )
    print(
        f"[urdf_to_blender_rigged] check   : "
        f"max bone-Z vs joint-axis error = {max_err_deg:.4f}° "
        f"(invariant should survive IK addition)"
    )

    # ------------------------------------------------------------------
    # Startup text block: auto-register the FH Clips panel on file open.
    #
    # `use_module = True` is the Text Editor "Register" checkbox — Blender
    # runs the text as a module when the .blend loads. NOTE: this only
    # fires if "Auto Run Python Scripts" is trusted/enabled (Prefs > Save
    # & Load, or `blender --enable-autoexec`); otherwise Blender blocks it
    # with a security banner and the panel won't appear.
    #
    # The embedded text resolves fh_clip_panel.py *at load time* relative
    # to the opened .blend (blend_dir/scripts), so it works on any machine
    # or checkout with no absolute path baked in — fh_rigged_latest.blend
    # lives in animation/ and the panel in animation/scripts/.
    # ------------------------------------------------------------------
    text_name = "fh_startup.py"
    if text_name in bpy.data.texts:
        bpy.data.texts.remove(bpy.data.texts[text_name])
    startup_text = bpy.data.texts.new(text_name)
    startup_text.write(
        "import os, sys, bpy\n"
        "blend_dir = os.path.dirname(bpy.data.filepath)\n"
        'addon_path = os.path.join(blend_dir, "scripts")\n'
        "if addon_path not in sys.path:\n"
        "    sys.path.insert(0, addon_path)\n"
        "import fh_clip_panel\n"
        "fh_clip_panel.register()\n"
    )
    startup_text.use_module = True  # "Register" checkbox → runs on file load
    print(
        f"[urdf_to_blender_rigged] startup text: {text_name} "
        f"(auto-registers FH Clips panel on open)"
    )

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=str(args.save))
        print(f"[urdf_to_blender_rigged] saved: {args.save}")


def _verify_auto_yaw(arm_obj, robot, link_world, foot_tip_in_link3_m):
    """Dynamic check of the per-leg link1 yaw driver:

    1. REST sign check — with `body_ctrl` at identity rotation and
       every `foot_target_<leg>` at its rest position, evaluate the
       depsgraph and assert
           link1.rotation_euler[2] ≈ 0
       per leg (≤ 1e-4 rad). A failure here means REST_ANGLE has the
       wrong sign for that leg (typically an atan2-vs-axis convention
       mismatch in `add_link1_yaw_drivers`).

    2. Behavioural check — rotate `body_ctrl` by +30° around Z, bump
       every `foot_target_<leg>` by (+30, −20) mm in world XY, evaluate
       the depsgraph, and assert each link1's resolved
       `rotation_euler[2]` equals the analytical
           axis_sign * (atan2(fy − py, fx − px) − body_rot − REST_ANGLE)
       within ≤ 1e-4 rad. REST_ANGLE comes from `link_world` (the same
       analytical source `add_link1_yaw_drivers` bakes — matches the
       driver expression by construction; reading the empties' live
       matrix_world here can drift by ~1° because of the
       `Target → Driver` depsgraph cycle).

    Restores body_ctrl and every foot_target afterwards so the saved
    blend keeps the rest pose. Print-only: a warning is logged if any
    leg fails — the rebuild itself is left to complete so the saved
    blend can be inspected.
    """
    import math as _math3

    body_ctrl = bpy.data.objects.get("body_ctrl")
    if body_ctrl is None:
        print("[urdf_to_blender_rigged] auto-yaw : body_ctrl missing, skipping")
        return

    bpy.context.view_layer.update()
    legs = []
    for leg in LEG_IDS:
        link1 = SHOULDER_BONE_FMT.format(leg=leg)
        link3 = KNEE_BONE_FMT.format(leg=leg)
        pb = arm_obj.pose.bones.get(link1)
        foot = bpy.data.objects.get(FOOT_TARGET_FMT.format(leg=leg))
        pivot = bpy.data.objects.get(SHOULDER_PIVOT_FMT.format(leg=leg))
        joint = robot["joints"].get(link1)
        if (
            pb is None
            or foot is None
            or pivot is None
            or joint is None
            or link1 not in link_world
            or link3 not in link_world
        ):
            print(f"[warn] auto-yaw : missing handles for leg {leg!r}, skipping")
            continue
        # Analytical REST_ANGLE — same source `add_link1_yaw_drivers` bakes.
        pivot_rest_m = link_world[link1].to_translation()
        foot_rest_m = link_world[link3] @ _foot_tip_for_leg(leg, foot_tip_in_link3_m)
        rest_yaw = _math3.atan2(
            foot_rest_m.y - pivot_rest_m.y, foot_rest_m.x - pivot_rest_m.x
        )
        axis_sign = 1 if joint["axis"].z >= 0 else -1
        legs.append((leg, pb, foot, pivot, rest_yaw, axis_sign))
    if not legs:
        return

    def _eval():
        bpy.context.view_layer.update()
        bpy.context.evaluated_depsgraph_get().update()

    # Save scene state we are about to perturb.
    saved_body_loc = body_ctrl.location.copy()
    saved_body_rot = Vector(body_ctrl.rotation_euler)
    saved_foot = {leg: foot.location.copy() for leg, _, foot, _, _, _ in legs}

    # --- REST sign check -------------------------------------------------
    # body_ctrl identity rotation + foot targets at their rest positions
    # (already in place — saved_foot is the rest snapshot).
    body_ctrl.rotation_euler = (0.0, 0.0, 0.0)
    _eval()
    rest_errs = [(leg, pb.rotation_euler[2]) for leg, pb, _, _, _, _ in legs]

    # --- Behavioural check ----------------------------------------------
    BODY_ROT = _math3.radians(30.0)
    body_ctrl.rotation_euler = (0.0, 0.0, BODY_ROT)
    for leg, _, foot, _, _, _ in legs:
        s = saved_foot[leg]
        foot.location = (s[0] + 30.0, s[1] - 20.0, s[2])
    _eval()

    drive_results = []
    for leg, pb, foot, pivot, rest_yaw, axis_sign in legs:
        fx, fy, _ = foot.matrix_world.translation
        px, py, _ = pivot.matrix_world.translation
        expected = axis_sign * (_math3.atan2(fy - py, fx - px) - BODY_ROT - rest_yaw)
        actual = pb.rotation_euler[2]
        delta = (actual - expected + _math3.pi) % (2 * _math3.pi) - _math3.pi
        drive_results.append((leg, actual, expected, delta))

    # Restore.
    body_ctrl.location = saved_body_loc
    body_ctrl.rotation_euler = saved_body_rot
    for leg, _, foot, _, _, _ in legs:
        foot.location = saved_foot[leg]
    _eval()

    TOL = 1e-4
    print(
        "[urdf_to_blender_rigged] auto-yaw : REST sign check "
        + ", ".join(f"{leg}={z:+.2e}" for leg, z in rest_errs)
        + " rad"
    )
    bad_rest = [leg for leg, z in rest_errs if abs(z) > TOL]
    if bad_rest:
        print(
            f"[urdf_to_blender_rigged] WARNING: REST_ANGLE sign mismatch on "
            f"{bad_rest} — link1.rotation_euler[2] should be ≈ 0 at rest"
        )

    print(
        "[urdf_to_blender_rigged] auto-yaw : driver check (body=+30°, "
        "foot=(+30,−20) mm)"
    )
    worst = 0.0
    for leg, actual, expected, delta in drive_results:
        print(
            f"                              {leg}: actual={actual:+.6f}, "
            f"expected={expected:+.6f}, |Δ|={abs(delta):.2e} rad"
        )
        worst = max(worst, abs(delta))
    if worst > TOL:
        print(
            f"[urdf_to_blender_rigged] WARNING: driver worst error "
            f"{worst:.2e} rad > {TOL:.0e} — driver math may be off"
        )


def _check_bone_z_alignment(robot, link_world, arm_obj):
    """Return the worst-case angle (degrees) between each bone's local Z
    (armature-space, read from `bone.matrix_local`) and the URDF joint
    axis it should equal. A clean rig prints < 1e-3°.

    NOTE: `bone.y_axis` / `bone.z_axis` are in parent-bone-local space,
    not armature space — use `bone.matrix_local.col[1]` (Y) and
    `.col[2]` (Z) for armature-space directions.
    """
    import math as _math

    worst = 0.0
    worst_bone = None
    per_bone = []
    for child_link, j in robot["joints"].items():
        bone = arm_obj.data.bones.get(child_link)
        if bone is None:
            continue
        axis_world = (link_world[child_link].to_3x3() @ j["axis"]).normalized()
        ml = bone.matrix_local
        z_world = Vector(ml.col[2][:3]).normalized()
        cos_t = max(-1.0, min(1.0, abs(z_world.dot(axis_world))))
        err_deg = _math.degrees(_math.acos(cos_t))
        per_bone.append((child_link, err_deg))
        if err_deg > worst:
            worst = err_deg
            worst_bone = child_link
    print(
        "[urdf_to_blender_rigged] bone-Z alignment per bone: "
        + ", ".join(f"{n}={e:.4f}°" for n, e in per_bone)
    )
    return worst


if __name__ == "__main__":
    main()
