"""
FaceHuggerExport.py — Add-in entry point consolidating URDF and printable STL exports.

Registers two commands in a "FaceHuggerExport" dropdown in the ADD-INS panel:
  • Export URDF Bodies      — writes fusion_export.json + STLs to code/simulation/generated/
  • Export Printable STLs   — reads cad/print_export.txt and writes per-body STLs to cad/body|leg/
"""

import json
import os
import sys
import time
import traceback

import adsk.core  # ty:ignore[unresolved-import]
import adsk.fusion  # ty:ignore[unresolved-import]

# Add-in root is the directory containing this file (non-double-nested layout).
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# cad/ is two levels up from FaceHuggerExport/
_CAD_DIR = os.path.normpath(os.path.join(_HERE, "../.."))
# code/simulation/ is three levels up + code/simulation
_SIM_DIR = os.path.normpath(os.path.join(_HERE, "../../..", "code", "simulation"))
_GENERATED_DIR = os.path.join(_SIM_DIR, "generated")
_MESH_DIR = os.path.join(_GENERATED_DIR, "exported_meshes")
_SPEC_FILE = os.path.join(_CAD_DIR, "print_export.txt")

# Keep references alive to prevent GC of event handlers
_handlers = []
_controls = []


# ── Command handler factory ───────────────────────────────────────────────────


def _make_handler(fn):
    """Return a CommandCreatedEventHandler that calls fn(app, ui)."""

    class _Handler(adsk.core.CommandCreatedEventHandler):
        def __init__(self):
            super().__init__()

        def notify(self, args):
            app = adsk.core.Application.get()
            ui = app.userInterface
            try:
                fn(app, ui)
            except Exception:
                ui.messageBox(f"FaceHuggerExport error:\n{traceback.format_exc()}")

    return _Handler()


# ── URDF export command ───────────────────────────────────────────────────────


def _run_urdf_export(app, ui):
    from lib.census import (
        _find_leg_assembly_R_la,
        collect_axes,
        collect_joints,
        collect_points,
        identity_matrix,
        traverse,
    )
    from lib.config import (
        COLLECT_PHYSICS,
        CONSTRUCTION_AXES,
        CONSTRUCTION_POINTS,
        JOINTS,
        LEG_ASSEMBLY_COMPONENT,
        LEG_ASSEMBLY_OCCURRENCE,
    )
    from lib.formatters import fmt_header, fmt_joints, fmt_message, fmt_tree
    from lib.manifest import build_mesh_files_manifest
    from lib.stl_export import export_stls
    from lib.verify import verify_against_assembly_hierarchy

    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        ui.messageBox("No active Fusion design.")
        return

    root = design.rootComponent
    os.makedirs(_GENERATED_DIR, exist_ok=True)
    json_path = os.path.join(_GENERATED_DIR, "fusion_export.json")

    # Preserve user edits to _servo_role_assignment across re-exports.
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

    diag_lines = verify_against_assembly_hierarchy(occurrences_json, joints, mesh_files)
    ui.messageBox(fmt_message(_SIM_DIR, exported, failed, joints, diag_lines))


# ── Printable STL export command ──────────────────────────────────────────────


def _run_print_stls(app, ui):
    from lib.print_stls.config import DEFAULT_UP
    from lib.print_stls.fusion_api import export_body_stl, find_body, iter_occ_tree
    from lib.print_stls.spec_io import format_log, read_spec, write_spec_with_log
    from lib.print_stls.stl_transform import rotate_stl_in_place, rotation_for_up_axis

    design = adsk.fusion.Design.cast(app.activeProduct)
    if design is None:
        ui.messageBox(
            "ExportPrintableSTLs: no active Fusion design.\n"
            "Open the assembly and run the script again."
        )
        return

    user_lines, entries = read_spec(_SPEC_FILE)
    if not entries:
        ui.messageBox(
            f"ExportPrintableSTLs: no spec entries found.\n"
            f"Edit {_SPEC_FILE} and add lines like:\n"
            f"    body  QuadrupedBody\n"
            f"    leg   LegMountL  +Y"
        )
        return

    # Pre-walk the occurrence tree once.
    root = design.rootComponent
    tree = list(iter_occ_tree(root.occurrences))

    mgr = design.exportManager
    successes = []  # (rel_path, src_path, up_axis)
    failures = []  # (rel_path, reason)

    for entry in entries:
        out_dir = os.path.join(_CAD_DIR, entry.subfolder)
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, f"{entry.body_name}.stl")
        rel_path = os.path.relpath(out_path, _CAD_DIR)

        occ, body, path = find_body(tree, entry.body_name, entry.hint)
        if body is None:
            detail = f" (no match for hint {entry.hint!r})" if entry.hint else ""
            failures.append((rel_path, f"body {entry.body_name!r} not found{detail}"))
            continue

        try:
            export_body_stl(mgr, body, out_path)
            if entry.up_axis != DEFAULT_UP:
                rotate_stl_in_place(out_path, rotation_for_up_axis(entry.up_axis))
        except Exception as e:  # pragma: no cover — Fusion runtime
            failures.append((rel_path, f"export failed: {e}"))
            continue
        successes.append((rel_path, path, entry.up_axis))

    # Rewrite the EXPORT LOG block at the bottom of the spec file.
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S %Z").strip()
    doc_name = ""
    try:
        doc_name = app.activeDocument.name
    except Exception:
        pass
    log_lines = format_log(timestamp, doc_name, successes, failures)
    write_spec_with_log(_SPEC_FILE, user_lines, log_lines)

    # User-facing dialog.
    summary_lines = [
        f"Exported {len(successes)} of {len(successes) + len(failures)} bodies",
        f"at HIGH mesh refinement to {os.path.basename(_CAD_DIR)}/body+leg/.",
        "",
    ]
    if successes:
        summary_lines.append("Successes:")
        for rel, _, up in successes:
            up_note = "" if up == DEFAULT_UP else f"  [up={up}]"
            summary_lines.append(f"  ✓ {rel}{up_note}")
    if failures:
        summary_lines.append("")
        summary_lines.append("Failures:")
        for rel, reason in failures:
            summary_lines.append(f"  ✗ {rel}    ({reason})")
    summary_lines.append("")
    summary_lines.append(f"Log appended to {os.path.relpath(_SPEC_FILE, _CAD_DIR)}.")
    ui.messageBox("\n".join(summary_lines))


# ── run / stop ────────────────────────────────────────────────────────────────


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        panel = ui.allToolbarPanels.itemById("SolidScriptsAddinsPanel")
        if not panel:
            ui.messageBox("Could not find ADD-INS panel.")
            return

        dropdown = panel.controls.addDropDown(
            "FaceHuggerExport", "", "FaceHuggerExportDropDown"
        )
        _controls.append(dropdown)

        commands = [
            (
                "FaceHuggerExportURDF",
                "Export URDF Bodies",
                "Export assembly to fusion_export.json + STLs for the URDF pipeline",
                _run_urdf_export,
            ),
            (
                "FaceHuggerExportPrintSTLs",
                "Export Printable STLs",
                "Export print-ready STLs per cad/print_export.txt spec",
                _run_print_stls,
            ),
        ]

        for cmd_id, name, tooltip, fn in commands:
            existing = ui.commandDefinitions.itemById(cmd_id)
            if existing:
                existing.deleteMe()

            cmd_def = ui.commandDefinitions.addButtonDefinition(
                cmd_id, name, tooltip, ""
            )
            handler = _make_handler(fn)
            cmd_def.commandCreated.add(handler)
            _handlers.append(handler)

            btn = dropdown.controls.addCommand(cmd_def)
            _controls.append(btn)

    except Exception:
        if ui:
            ui.messageBox("FaceHuggerExport failed to load:\n" + traceback.format_exc())


def stop(context):
    try:
        for ctrl in reversed(_controls):
            if ctrl and ctrl.isValid:
                ctrl.deleteMe()
        _controls.clear()

        app = adsk.core.Application.get()
        ui = app.userInterface
        for cmd_id in ("FaceHuggerExportURDF", "FaceHuggerExportPrintSTLs"):
            defn = ui.commandDefinitions.itemById(cmd_id)
            if defn:
                defn.deleteMe()

        _handlers.clear()
    except Exception:
        pass
