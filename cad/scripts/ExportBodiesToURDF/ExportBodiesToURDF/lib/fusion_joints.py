# fusion_joints.py — Fusion-dependent joint extraction.
# Imports adsk (unavoidable — queries design.allComponents and joint API).


from config import CM_TO_MM, JOINTS, LEG_ASSEMBLY_COMPONENT, LEG_ASSEMBLY_OCCURRENCE
from math_utils import _apply_R_3x3


def _find_leg_assembly_R_la(occurrences_json):
    """Return 3x3 world rotation of FaceHuggerLegAssembly:1, or None."""
    for occ in occurrences_json or []:
        if occ.get("name") == LEG_ASSEMBLY_OCCURRENCE:
            wtf = occ.get("world_transform_rm_cm")
            if not wtf:
                return None
            return [
                [wtf[0][0], wtf[0][1], wtf[0][2]],
                [wtf[1][0], wtf[1][1], wtf[1][2]],
                [wtf[2][0], wtf[2][1], wtf[2][2]],
            ]
    return None


def collect_joints(design, R_la=None):
    """Walk every joint in the design and return a list of dicts.

    Schema per joint: name, owner_component, kind, type,
    axis_dir_local_unit, axis_origin_local_mm, axis_construction_name,
    origin_construction_name, limits_rad, parent_occurrence_path,
    parent_body, child_occurrence_path, child_body, normalized_by_R_la.
    """
    joints_data = []
    try:
        all_components = design.allComponents
    except Exception:
        return joints_data

    for component in all_components:
        try:
            joints = component.joints
        except Exception:
            joints = None
        if joints:
            for joint in joints:
                if not _whitelisted_joint_name(joint):
                    continue
                entry = _extract_joint(joint, component, kind="joint", R_la=R_la)
                if entry is not None:
                    joints_data.append(entry)

        try:
            as_built = component.asBuiltJoints
        except Exception:
            as_built = None
        if as_built:
            for joint in as_built:
                if not _whitelisted_joint_name(joint):
                    continue
                entry = _extract_joint(joint, component, kind="asbuilt", R_la=R_la)
                if entry is not None:
                    joints_data.append(entry)

    return joints_data


def _whitelisted_joint_name(joint):
    try:
        return joint.name in JOINTS
    except Exception:
        return False


def _extract_joint(joint, owner_component, kind="joint", R_la=None):
    """Pull data for one Fusion Joint or AsBuiltJoint into JSON-friendly schema."""
    try:
        name = joint.name
    except Exception:
        return None

    motion = None
    motion_type = "unknown"
    try:
        motion = joint.jointMotion
        motion_type = motion.objectType.split("::")[-1] if motion else "unknown"
    except Exception:
        pass

    if "Revolute" in motion_type:
        type_tag = "revolute"
    elif "Slider" in motion_type:
        type_tag = "prismatic"
    elif "Rigid" in motion_type:
        type_tag = "rigid"
    elif "Pin" in motion_type or "Cylindrical" in motion_type:
        type_tag = "cylindrical"
    elif "Ball" in motion_type:
        type_tag = "ball"
    else:
        type_tag = motion_type.lower()

    axis_dir = None
    axis_construction_name = None
    if motion is not None and type_tag in ("revolute", "prismatic", "cylindrical"):
        axis_dir, axis_construction_name = _joint_axis(motion)

    origin_local_mm, origin_construction_name = _joint_origin(joint)

    limits_rad = None
    if motion is not None and type_tag in ("revolute", "prismatic"):
        limits_rad = _joint_limits(motion, type_tag)

    parent_path = _safe_full_path(getattr(joint, "occurrenceTwo", None))
    child_path = _safe_full_path(getattr(joint, "occurrenceOne", None))
    parent_body = _first_body_name(getattr(joint, "occurrenceTwo", None))
    child_body = _first_body_name(getattr(joint, "occurrenceOne", None))

    owner_name = ""
    try:
        owner_name = owner_component.name or ""
    except Exception:
        pass

    normalized = False
    if R_la is not None and owner_name == LEG_ASSEMBLY_COMPONENT:
        if axis_dir is not None:
            axis_dir = _apply_R_3x3(R_la, axis_dir)
        if origin_local_mm is not None:
            origin_local_mm = _apply_R_3x3(R_la, origin_local_mm)
        normalized = True

    return {
        "name": name,
        "owner_component": owner_name,
        "kind": kind,
        "type": type_tag,
        "axis_dir_local_unit": axis_dir,
        "axis_origin_local_mm": origin_local_mm,
        "axis_construction_name": axis_construction_name,
        "origin_construction_name": origin_construction_name,
        "limits_rad": limits_rad,
        "parent_occurrence_path": parent_path,
        "parent_body": parent_body,
        "child_occurrence_path": child_path,
        "child_body": child_body,
        "normalized_by_R_la": normalized,
    }


def _joint_axis(motion):
    """Return (axis_dir_unit, construction_axis_name) for a revolute/prismatic motion."""
    try:
        axis_kind = motion.rotationAxis
    except AttributeError:
        try:
            axis_kind = motion.slideDirection
        except AttributeError:
            axis_kind = None
    except Exception:
        axis_kind = None

    custom_entity = None
    try:
        custom_entity = motion.customRotationAxisEntity
    except AttributeError:
        try:
            custom_entity = motion.customSlideDirectionEntity
        except AttributeError:
            custom_entity = None
    except Exception:
        custom_entity = None

    if custom_entity is not None:
        name = getattr(custom_entity, "name", None)
        try:
            geom = custom_entity.geometry
            d = geom.direction
            mag = (d.x * d.x + d.y * d.y + d.z * d.z) ** 0.5
            if mag > 0:
                return [d.x / mag, d.y / mag, d.z / mag], name
        except Exception:
            return None, name
        return None, name

    if axis_kind is None:
        return None, None
    try:
        kind_int = int(axis_kind)
    except Exception:
        return None, None
    if kind_int == 0:
        return [1.0, 0.0, 0.0], None
    if kind_int == 1:
        return [0.0, 1.0, 0.0], None
    if kind_int == 2:
        return [0.0, 0.0, 1.0], None
    return None, None


def _joint_origin(joint):
    """Return (origin_local_mm, construction_point_name)."""
    geo_one = None
    for attr in ("geometryOrOriginOne", "geometry"):
        try:
            geo_one = getattr(joint, attr, None)
        except Exception:
            geo_one = None
        if geo_one is not None:
            break
    if geo_one is None:
        return None, None

    name = None
    pos_mm = None
    try:
        origin_entity = geo_one
        if hasattr(origin_entity, "geometry"):
            inner = origin_entity.geometry
            if inner is not None and hasattr(inner, "origin"):
                origin_entity = inner.origin
        elif hasattr(origin_entity, "origin"):
            origin_entity = origin_entity.origin

        if hasattr(origin_entity, "name"):
            name = origin_entity.name or None

        pt = None
        if hasattr(origin_entity, "geometry") and origin_entity.geometry is not None:
            g = origin_entity.geometry
            if hasattr(g, "x") and hasattr(g, "y") and hasattr(g, "z"):
                pt = g
        if pt is None and hasattr(origin_entity, "x") and hasattr(origin_entity, "y"):
            pt = origin_entity
        if pt is not None:
            pos_mm = [
                round(pt.x * CM_TO_MM, 3),
                round(pt.y * CM_TO_MM, 3),
                round(pt.z * CM_TO_MM, 3),
            ]
    except Exception:
        pass

    return pos_mm, name


def _joint_limits(motion, type_tag):
    """Return a `limits_rad` dict (radians) or None on failure."""
    try:
        if type_tag == "revolute":
            lim = motion.rotationLimits
        else:
            lim = motion.slideLimits
        rest = lim.restValue if hasattr(lim, "restValue") else 0.0
    except Exception:
        return None
    out = {"rest": float(rest)}
    try:
        out["min_enabled"] = bool(lim.isMinimumValueEnabled)
        out["min"] = float(lim.minimumValue) if lim.isMinimumValueEnabled else None
    except Exception:
        out["min_enabled"] = False
        out["min"] = None
    try:
        out["max_enabled"] = bool(lim.isMaximumValueEnabled)
        out["max"] = float(lim.maximumValue) if lim.isMaximumValueEnabled else None
    except Exception:
        out["max_enabled"] = False
        out["max"] = None
    return out


def _safe_full_path(occ):
    if occ is None:
        return None
    try:
        return occ.fullPathName
    except Exception:
        return None


def _first_body_name(occ):
    if occ is None:
        return None
    try:
        bodies = occ.component.bRepBodies
        if bodies.count > 0:
            return bodies[0].name
    except Exception:
        pass
    return None
