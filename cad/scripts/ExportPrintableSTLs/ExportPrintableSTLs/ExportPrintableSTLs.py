"""
ExportPrintableSTLs.py  —  FaceHugger print-ready STL exporter

Reads cad/print_export.txt (one body per line) and exports each named body
as a binary STL at HIGH mesh refinement into cad/body/ or cad/leg/, with
an optional per-body "up axis" rotation baked into the STL so the slicer
loads each part pre-oriented.

This is the SISTER script of ExportBodiesToURDF.py:

    ExportBodiesToURDF.py    →  code/simulation/generated/  (Medium refinement,
                                merged STLs, transformed vertices for URDF/sim)
    ExportPrintableSTLs.py   →  cad/body/ + cad/leg/        (High refinement,
                                one STL per body, optional up-axis rotation —
                                ready for slicer)

Spec file format (cad/print_export.txt):

    <subfolder>    <body_name>    [up_axis]    [hint]

Where each trailing token is identified by shape, in any order:

    up_axis        one of {+X, -X, +Y, -Y, +Z, -Z}. The body's named axis
                   is rotated to point along the slicer's +Z (up). Default
                   is +Z (no rotation). Rotates BOTH vertices and normals
                   after Fusion writes the STL.
    hint           any other bare word, or `hint=<value>`. Substring matched
                   against the occurrence path. Use only when the body name
                   appears in multiple components.

    subfolder      "body" or "leg" (creates cad/<subfolder>/ if missing)
    body_name      BRep body name in the active design (case-sensitive)

Examples:

    body   QuadrupedBody
    body   QuadrupedBody    +Y                        # +Y axis points up
    leg    Link2            -Z                        # flipped upside down
    leg    Link2            hint=Link2L:1             # disambiguate by path
    leg    Link2            +Y    Link2L:1            # both

After every run, the script rewrites the section below the marker line

    # === EXPORT LOG (auto-managed below — do not edit) ===

with a timestamp + per-body ✓/✗ status. User entries above the marker are
preserved verbatim.

Run via:  Shift+S → Scripts and Add-Ins → ExportPrintableSTLs → Run.
The active document must be the assembly containing every body in the
spec (xrefs loaded). Defaults from Fusion's STL exporter are used
(binary, design units = mm assuming the design is set to mm).
"""

import os
import time
import traceback

import adsk.core  # ty:ignore[unresolved-import]
import adsk.fusion  # ty:ignore[unresolved-import]

from lib.config import DEFAULT_UP
from lib.fusion_api import export_body_stl, find_body, iter_occ_tree
from lib.spec_io import format_log, read_spec, write_spec_with_log
from lib.stl_transform import rotate_stl_in_place, rotation_for_up_axis

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_HERE = os.path.dirname(os.path.abspath(__file__))
# This script lives at cad/scripts/ExportPrintableSTLs/ExportPrintableSTLs/.
# Three levels up = the repo's cad/ directory.
_CAD_DIR = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))
_SPEC_FILE = os.path.join(_CAD_DIR, "print_export.txt")

# Set to True to skip actual export/rotation and do a trial run only.
DRY_RUN: bool = False


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def run(_context: str):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
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
                failures.append(
                    (rel_path, f"body {entry.body_name!r} not found{detail}")
                )
                continue

            if DRY_RUN:
                successes.append((rel_path, path, entry.up_axis))
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
        dry_tag = "[DRY RUN] " if DRY_RUN else ""
        summary_lines = [
            f"{dry_tag}Exported {len(successes)} of {len(successes) + len(failures)} bodies",
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
        summary_lines.append(
            f"Log appended to {os.path.relpath(_SPEC_FILE, _CAD_DIR)}."
        )
        ui.messageBox("\n".join(summary_lines))

    except Exception:  # pragma: no cover — Fusion runtime
        if ui is not None:
            ui.messageBox("ExportPrintableSTLs failed:\n" + traceback.format_exc())
        else:
            raise
