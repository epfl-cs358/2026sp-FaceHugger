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
import struct
import time
import traceback

import adsk.core  # ty:ignore[unresolved-import]
import adsk.fusion  # ty:ignore[unresolved-import]

from lib.config import DEFAULT_UP, ROTATIONS  # noqa: F401 — used below
from lib.spec_io import format_log, read_spec, write_spec_with_log  # noqa: F401

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_HERE = os.path.dirname(os.path.abspath(__file__))
# This script lives at cad/scripts/ExportPrintableSTLs/ExportPrintableSTLs/.
# Three levels up = the repo's cad/ directory.
_CAD_DIR = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))
_SPEC_FILE = os.path.join(_CAD_DIR, "print_export.txt")


# ---------------------------------------------------------------------------
# Occurrence-tree walking
# ---------------------------------------------------------------------------


def _iter_occ_tree(occurrences):
    """Yield (occurrence, dotted_path) for every occurrence in the design.
    Same traversal pattern as ExportBodiesToURDF.py."""
    for occ in occurrences:
        path = occ.name
        yield occ, path
        children = occ.childOccurrences
        if children:
            for sub_occ, sub_path in _iter_occ_subtree(children, path):
                yield sub_occ, sub_path


def _iter_occ_subtree(occurrences, parent_path):
    for occ in occurrences:
        path = f"{parent_path}/{occ.name}"
        yield occ, path
        children = occ.childOccurrences
        if children:
            yield from _iter_occ_subtree(children, path)


def _find_body(tree, body_name, hint):
    """First (occurrence, body, path) whose body name matches and (if `hint`
    is given) whose path contains `hint` as a substring. Returns
    (None, None, None) when nothing matches."""
    for occ, path in tree:
        if hint is not None and hint not in path:
            continue
        comp = occ.component
        if comp is None:
            continue
        for body in comp.bRepBodies:
            if body.name == body_name:
                return occ, body, path
    return None, None, None


# ---------------------------------------------------------------------------
# STL export + post-rotation
# ---------------------------------------------------------------------------


def _export_body(mgr, body, out_path):
    """Export a single BRep body to a binary STL at HIGH refinement.
    Vertices retain the body's native (component-local) frame — no
    world-transform bake-in."""
    opts = mgr.createSTLExportOptions(body, out_path)
    opts.sendToPrintUtility = False
    opts.isBinaryFormat = True
    opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    mgr.execute(opts)


def _rotate_stl_in_place(path, rotation):
    """Read binary STL at `path`, rotate every normal and vertex by
    `rotation` (3 rows of 3 floats), write it back.

    Binary STL layout:
      80 bytes  : header (preserved)
       4 bytes  : uint32 triangle count
      per tri  : 12 floats (3 normal + 9 vertex) + 2 bytes attribute
    """
    with open(path, "rb") as f:
        header = f.read(80)
        (n_tri,) = struct.unpack("<I", f.read(4))
        tri_blob = f.read(50 * n_tri)

    r = rotation

    def _rot(v):
        return (
            r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
            r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
            r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
        )

    out = bytearray()
    out += header
    out += struct.pack("<I", n_tri)
    off = 0
    for _ in range(n_tri):
        nrm = struct.unpack("<3f", tri_blob[off : off + 12])
        v1 = struct.unpack("<3f", tri_blob[off + 12 : off + 24])
        v2 = struct.unpack("<3f", tri_blob[off + 24 : off + 36])
        v3 = struct.unpack("<3f", tri_blob[off + 36 : off + 48])
        attr = tri_blob[off + 48 : off + 50]
        n2 = _rot(nrm)
        v1r = _rot(v1)
        v2r = _rot(v2)
        v3r = _rot(v3)
        out += struct.pack("<12f", *n2, *v1r, *v2r, *v3r)
        out += attr
        off += 50

    with open(path, "wb") as f:
        f.write(bytes(out))


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
        tree = list(_iter_occ_tree(root.occurrences))

        mgr = design.exportManager
        successes = []  # (rel_path, src_path, up_axis)
        failures = []  # (rel_path, reason)

        for entry in entries:
            out_dir = os.path.join(_CAD_DIR, entry.subfolder)
            os.makedirs(out_dir, exist_ok=True)
            out_path = os.path.join(out_dir, f"{entry.body_name}.stl")
            rel_path = os.path.relpath(out_path, _CAD_DIR)

            occ, body, path = _find_body(tree, entry.body_name, entry.hint)
            if body is None:
                detail = f" (no match for hint {entry.hint!r})" if entry.hint else ""
                failures.append(
                    (rel_path, f"body {entry.body_name!r} not found{detail}")
                )
                continue

            try:
                _export_body(mgr, body, out_path)
                if entry.up_axis != DEFAULT_UP:
                    _rotate_stl_in_place(out_path, ROTATIONS[entry.up_axis])
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
        summary_lines.append(
            f"Log appended to {os.path.relpath(_SPEC_FILE, _CAD_DIR)}."
        )
        ui.messageBox("\n".join(summary_lines))

    except Exception:  # pragma: no cover — Fusion runtime
        if ui is not None:
            ui.messageBox("ExportPrintableSTLs failed:\n" + traceback.format_exc())
        else:
            raise
