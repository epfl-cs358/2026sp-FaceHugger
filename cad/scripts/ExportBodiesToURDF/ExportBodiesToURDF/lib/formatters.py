# formatters.py — Human-readable text renderers for fusion_export.txt and the
# Fusion message box. Pure dict/string operations — no adsk dependency.

from .config import (
    COLLECT_PHYSICS,
    CONSTRUCTION_AXES,
    CONSTRUCTION_POINTS,
    JOINTS,
    LEG_ASSEMBLY_COMPONENT,
)
from .math_utils import _rad_to_deg


def fmt_tree(nodes, depth=0):
    lines = []
    pad = "  " * depth
    for n in nodes:
        xref_tag = " [xref]" if n["xref"] else ""
        vis_tag = "" if n["visible"] else " [hidden]"
        lines.append(f"{pad}{n['name']}{xref_tag}{vis_tag}")
        lines.append(
            f"{pad}  parent_origin_mm: {tuple(n['parent_origin_mm'])}  "
            f"world_origin_mm: {tuple(n['world_origin_mm'])}"
        )
        for b in n["bodies"]:
            v = "" if b["visible"] else " [hidden]"
            lines.append(
                f"{pad}  body: {b['name']}{v} | bbox_center_mm: {tuple(b['bbox_center_mm'])}"
            )
        for a in n["axes"]:
            if "error" in a:
                lines.append(f"{pad}  caxis: {a['name']} | ERROR: {a['error']}")
            else:
                v = "" if a["visible"] else " [hidden]"
                lines.append(
                    f"{pad}  caxis: {a['name']}{v} "
                    f"| local origin_mm: {tuple(a['origin_mm'])} dir: {tuple(a['dir'])} "
                    f"| world origin_mm: {tuple(a['origin_world_mm'])} dir: {tuple(a['dir_world'])}"
                )
        for p in n["points"]:
            if "error" in p:
                lines.append(f"{pad}  cpoint: {p['name']} | ERROR: {p['error']}")
            else:
                v = "" if p["visible"] else " [hidden]"
                lines.append(
                    f"{pad}  cpoint: {p['name']}{v} "
                    f"| local pos_mm: {tuple(p['pos_mm'])} "
                    f"| world pos_mm: {tuple(p['pos_world_mm'])}"
                )
        if COLLECT_PHYSICS and "physics" in n and "error" not in n["physics"]:
            ph = n["physics"]
            lines.append(
                f"{pad}  mass: {ph['mass_kg']} kg | com local: {tuple(ph['com_mm'])} "
                f"| com world: {tuple(ph['com_world_mm'])}"
            )
        if n["children"]:
            lines += fmt_tree(n["children"], depth + 1)
    return lines


def fmt_joints(joints):
    """Render the joints array as a human-readable block for fusion_export.txt."""
    if not joints:
        return ["=== Joints ===", "  (none)"]
    lines = ["=== Joints ==="]
    for j in joints:
        name = j.get("name", "?")
        type_tag = j.get("type", "?")
        kind = j.get("kind", "joint")
        kind_suffix = "" if kind == "joint" else f" [{kind}]"
        lines.append(f"{name} ({type_tag}){kind_suffix}")
        axis_name = j.get("axis_construction_name") or "(principal axis)"
        origin_name = j.get("origin_construction_name") or "(implicit origin)"
        axis_dir = j.get("axis_dir_local_unit")
        origin = j.get("axis_origin_local_mm")
        bits = [f"axis: {axis_name}"]
        if axis_dir is not None:
            bits.append(
                f"dir=({axis_dir[0]:+.3f}, {axis_dir[1]:+.3f}, {axis_dir[2]:+.3f})"
            )
        bits.append(f"origin: {origin_name}")
        if origin is not None:
            bits.append(
                f"local_mm=({origin[0]:+.2f}, {origin[1]:+.2f}, {origin[2]:+.2f})"
            )
        lines.append("  " + "  ".join(bits))
        lim = j.get("limits_rad") or {}
        if lim:
            min_d = (
                f"{_rad_to_deg(lim['min']):+.1f}deg"
                if lim.get("min") is not None
                else "--"
            )
            max_d = (
                f"{_rad_to_deg(lim['max']):+.1f}deg"
                if lim.get("max") is not None
                else "--"
            )
            rest_d = _rad_to_deg(lim.get("rest", 0.0) or 0.0)
            lines.append(f"  limits: rest={rest_d:+.1f}deg  [{min_d}, {max_d}]")
        parent = j.get("parent_occurrence_path") or "?"
        child = j.get("child_occurrence_path") or "?"
        parent_body = j.get("parent_body") or "?"
        child_body = j.get("child_body") or "?"
        lines.append(f"  parent: {parent} ({parent_body})")
        lines.append(f"  child:  {child} ({child_body})")
    return lines


def _summarize_joint(j):
    """One-line joint summary for the post-export message box."""
    name = j.get("name", "?")
    type_tag = j.get("type", "?")
    lim = j.get("limits_rad") or {}
    range_txt = ""
    if lim and lim.get("min") is not None and lim.get("max") is not None:
        range_txt = (
            f"  [{_rad_to_deg(lim['min']):+.0f}deg, {_rad_to_deg(lim['max']):+.0f}deg]"
        )
    axis_name = j.get("axis_construction_name") or "?"
    return f"  {name} ({type_tag}, axis={axis_name}){range_txt}"


def fmt_header(export, R_la):
    """Build the fusion_export.txt header block (before the occurrence tree)."""
    lines = [
        "=" * 60,
        "FaceHugger Fusion Export",
        f"Document        : {export['document']}",
        f"Physics         : {COLLECT_PHYSICS}",
        f"Cpoint whitelist: {', '.join(CONSTRUCTION_POINTS)}",
        f"Caxis whitelist : {', '.join(CONSTRUCTION_AXES)}",
        f"Joint whitelist : {', '.join(JOINTS)}",
        "=" * 60,
        "",
        "[Leg-assembly normalization]",
        f"  Owner: {LEG_ASSEMBLY_COMPONENT}",
        "  R_la (3x3, row-major):",
        f"    [{R_la[0][0]:+.4f}, {R_la[0][1]:+.4f}, {R_la[0][2]:+.4f}]",
        f"    [{R_la[1][0]:+.4f}, {R_la[1][1]:+.4f}, {R_la[1][2]:+.4f}]",
        f"    [{R_la[2][0]:+.4f}, {R_la[2][1]:+.4f}, {R_la[2][2]:+.4f}]",
        "  Applied to: joint axis_dir_local_unit and "
        "axis_origin_local_mm (leg-assembly-internal joints only).",
        "  Mesh STLs are produced by combined-rule, already bakes world-frame vertices.",
        "",
    ]
    root_axes = export.get("root_axes") or []
    root_points = export.get("root_points") or []
    if root_axes or root_points:
        lines.append("[root]")
        for a in root_axes:
            lines.append(
                f"  caxis: {a['name']} | origin_mm: {tuple(a['origin_mm'])} | dir: {tuple(a['dir'])}"
            )
        for p in root_points:
            lines.append(f"  cpoint: {p['name']} | pos_mm: {tuple(p['pos_mm'])}")
        lines.append("")
    return lines


def fmt_message(sim_dir, exported, failed, joints, diag_lines):
    """Build the post-export message box string."""
    msg = f"Export complete.\n\nJSON + TXT → {sim_dir}\n"
    if exported:
        msg += f"\nSTLs exported ({len(exported)}):\n" + "\n".join(
            f"  {e['stl']}" for e in exported
        )
    if failed:
        msg += f"\n\nFailed ({len(failed)}):\n" + "\n".join(f"  {s}" for s in failed)
    if joints:
        msg += f"\n\nJoints captured ({len(joints)}):\n" + "\n".join(
            _summarize_joint(j) for j in joints
        )
    if diag_lines:
        msg += "\n\nAssembly check:\n" + "\n".join(diag_lines)
    return msg
