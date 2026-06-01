"""
ExportBodiesToURDF.py  —  FaceHugger Fusion assembly exporter (entry point)

Design philosophy: **the export is the raw material, not the curated
output**. The JSON `occurrences` tree contains every occurrence in the
design — PCBs, OLEDs, capacitors, every screw — because downstream
tools (the URDF generator, the Blender visualizer) navigate that tree
by occurrence path to look up world transforms, joint origins, mount
points, etc. Pruning the tree would break path resolution. So we keep
it complete and filter at consumption time.

Filtering happens via four explicit whitelists, each shaping a
DIFFERENT slice of the output:

    EXPORT_RULES         bodies → STL files (8 files in exported_meshes/)
    CONSTRUCTION_POINTS  named cpoints land in each occurrence's points[]
    CONSTRUCTION_AXES    named caxes  land in each occurrence's axes[]
    JOINTS               named joints land in the top-level joints[] array

Outputs to code/simulation/ (relative to this script's repo location):
  fusion_export.json / fusion_export.txt / exported_meshes/*.stl

Run via: Shift+S → Scripts and Add-Ins → ExportBodiesToURDF → Run
"""

import json
import os
import traceback

import adsk.core  # ty:ignore[unresolved-import]
import adsk.fusion  # ty:ignore[unresolved-import]

from lib.config import (
    COLLECT_PHYSICS,
    CONSTRUCTION_AXES,
    CONSTRUCTION_POINTS,
    JOINTS,
    LEG_ASSEMBLY_COMPONENT,
    LEG_ASSEMBLY_OCCURRENCE,
)
from lib.formatters import fmt_header, fmt_joints, fmt_message, fmt_tree
from lib.census import _find_leg_assembly_R_la, collect_joints
from lib.census import collect_axes, collect_points, identity_matrix, traverse
from lib.stl_export import export_stls
from lib.manifest import build_mesh_files_manifest
from lib.verify import verify_against_assembly_hierarchy

app = adsk.core.Application.get()
ui = app.userInterface

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIM_DIR = os.path.normpath(
    os.path.join(_HERE, "..", "..", "..", "..", "code", "simulation")
)
_GENERATED_DIR = os.path.join(_SIM_DIR, "generated")
_MESH_DIR = os.path.join(_GENERATED_DIR, "exported_meshes")


def run(_context: str):
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active Fusion design.")
            return

        root = design.rootComponent
        os.makedirs(_GENERATED_DIR, exist_ok=True)
        json_path = os.path.join(_GENERATED_DIR, "fusion_export.json")

        preserved_roles = None
        if os.path.exists(json_path):
            try:
                with open(json_path, "r", encoding="utf-8") as f:
                    prior = json.load(f)
                preserved_roles = (prior.get("mesh_files") or {}).get(
                    "_servo_role_assignment"
                )
            except Exception:
                preserved_roles = None

        root_world = identity_matrix()
        occurrences_json = traverse(root.occurrences, root_world)

        exported, failed = export_stls(design, root, occurrences_json, _MESH_DIR)
        mesh_files = build_mesh_files_manifest(exported, preserved_roles)

        R_la = _find_leg_assembly_R_la(occurrences_json)
        if R_la is None:
            raise RuntimeError(
                f"Could not find {LEG_ASSEMBLY_OCCURRENCE} world transform; "
                "leg-assembly normalization cannot proceed."
            )

        joints = collect_joints(design, R_la=R_la)
        root_axes = collect_axes(root, root_world)
        root_points = collect_points(root, root_world)

        export = {
            "document": root.name,
            "collect_physics": COLLECT_PHYSICS,
            "mesh_files": mesh_files,
            "construction_points_whitelist": list(CONSTRUCTION_POINTS),
            "construction_axes_whitelist": list(CONSTRUCTION_AXES),
            "joints_whitelist": list(JOINTS),
            "leg_assembly_normalization": {
                "applied": True,
                "occurrence": LEG_ASSEMBLY_OCCURRENCE,
                "owner_component": LEG_ASSEMBLY_COMPONENT,
                "rotation_3x3_row_major": R_la,
            },
            "joints": joints,
            "root_axes": root_axes,
            "root_points": root_points,
            "occurrences": occurrences_json,
        }

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(export, f, indent=2)

        txt_path = os.path.join(_GENERATED_DIR, "fusion_export.txt")
        with open(txt_path, "w", encoding="utf-8") as f:
            f.write(
                "\n".join(
                    fmt_header(export, R_la)
                    + fmt_tree(export["occurrences"])
                    + [""]
                    + fmt_joints(joints)
                )
            )

        diag_lines = verify_against_assembly_hierarchy(
            occurrences_json, joints, mesh_files
        )
        ui.messageBox(fmt_message(_SIM_DIR, exported, failed, joints, diag_lines))

    except:  # pylint: disable=bare-except  # noqa: E722
        app.log(f"Failed:\n{traceback.format_exc()}")
