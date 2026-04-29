"""
phase0_verify.py — verify fusion_export.json matches the new CAD spec.

Reads code/simulation/fusion_export.json and checks it against the
authoritative ASSEMBLY_HIERARCHY.md / PIPELINE_SPEC.md structure. Emits
PASS/FAIL per check, then a tabular summary. Exit code is non-zero if
any required check fails.

Usage:
    python phase0_verify.py [--export PATH]
"""

import argparse
import json
import math
import sys
from pathlib import Path


# --- expected structure (from ASSEMBLY_HIERARCHY.md) -------------------------

REQUIRED_FS_POINTS = [
    "LegMountPointFL", "LegMountPointFR",
    "LegMountPointBR", "LegMountPointBL",
]
LEGACY_FS_POINTS = [
    "LegMountFL", "LegMountFR", "LegMountBR", "LegMountBL",
]

REQUIRED_FHLA_POINTS = [
    "BodyToLink1Point", "Link1ToLink2Point", "Link2ToLink3Point",
]

REQUIRED_FHLA_OCCURRENCES = {
    "Link1L:1":              {"body": "Link1"},
    "Link1R:1":              {"body": "Link1R"},
    "Link2L:1":              {"body": "Link2"},
    "Link3L:1":              {"body": "Link3"},
    "MotorMount:1":          {"body": "LegMountL", "point": "LegMountFixedPoint"},
    "MotorMountR:1":         {"body": "LegMountR", "point": "LegMountFixedPoint"},
    "Servo_Mouser_Model:1":  {"body": "ServoBase"},
    "Servo_Mouser_Model:2":  {"body": "ServoBase"},
    "Servo_Mouser_Model:3":  {"body": "ServoBase"},
}

REQUIRED_JOINTS = ["Link1Revolute", "Link2Revolute", "Link3Revolute"]

XYZ_TOLERANCE_MM = 1.0


# --- output helpers ----------------------------------------------------------


def _ok(label, detail=""):
    print(f"  [OK ]  {label}" + (f"  — {detail}" if detail else ""))


def _fail(label, detail=""):
    print(f"  [FAIL] {label}" + (f"  — {detail}" if detail else ""))


def _info(label, detail=""):
    print(f"  [..]  {label}" + (f"  — {detail}" if detail else ""))


def _section(name):
    print(f"\n--- {name} ---")


# --- traversal helpers -------------------------------------------------------


def find_occurrence(occs, name):
    for o in occs:
        if o.get("name") == name:
            return o
    return None


def has_body(occ, body_name):
    """The export tree records each body inside an occurrence as a string in
    `bodies[]`, OR (more rigorously) as a child sub-occurrence — depending on
    the exporter's schema. Look in both."""
    bodies = occ.get("bodies") or []
    if any((b.get("name") if isinstance(b, dict) else b) == body_name for b in bodies):
        return True
    # Fallback: child occurrences with that name (some exporters list bodies as occs)
    for child in occ.get("children", []) or []:
        if child.get("name") == body_name:
            return True
    return False


def has_point(occ, point_name):
    return any(p.get("name") == point_name for p in occ.get("points", []) or [])


def get_point(occ, point_name):
    for p in occ.get("points", []) or []:
        if p.get("name") == point_name:
            return p
    return None


def vec_dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


# --- checks ------------------------------------------------------------------


def check_flexible_skeleton_points(fs):
    _section("Check 1 — FlexibleSkeleton:1 construction points")
    if fs is None:
        _fail("FlexibleSkeleton:1 not found in occurrences")
        return False, {}

    found = {p.get("name"): p for p in fs.get("points", []) or []}
    legacy_present = [n for n in LEGACY_FS_POINTS if n in found]
    if legacy_present:
        _fail(
            f"legacy point names still present: {legacy_present} — "
            f"rename to LegMountPointXX in Fusion"
        )

    all_ok = True
    pos_by_name = {}
    for name in REQUIRED_FS_POINTS:
        if name in found:
            pos = found[name].get("pos_world_mm") or found[name].get("pos_mm")
            pos_by_name[name] = pos
            _ok(name, f"world={fmt_xyz(pos)}")
        else:
            _fail(name, "missing")
            all_ok = False
    return all_ok, pos_by_name


def check_legassembly_points(fhla):
    _section("Check 2 — FaceHuggerLegAssembly:1 construction points")
    if fhla is None:
        _fail("FaceHuggerLegAssembly:1 not found in occurrences")
        return False, {}

    found = {p.get("name"): p for p in fhla.get("points", []) or []}
    all_ok = True
    pos_by_name = {}
    for name in REQUIRED_FHLA_POINTS:
        if name in found:
            local = found[name].get("pos_mm")
            world = found[name].get("pos_world_mm")
            pos_by_name[name] = {"local": local, "world": world}
            _ok(name, f"local={fmt_xyz(local)}  world={fmt_xyz(world)}")
        else:
            _fail(name, "missing")
            all_ok = False
    return all_ok, pos_by_name


def check_world_pos_crosscheck(fs_pos, fhla):
    """LegMountFixedPoint inside MotorMount:1 is the bracket-side mating
    anchor — when assembled, it must coincide with LegMountPointFL on the
    body. (BodyToLink1Point is the rotation-axis point and is offset from
    the mating face by the bracket thickness — printed informationally.)
    """
    _section("Check 3 — MotorMount:1/LegMountFixedPoint world ≈ LegMountPointFL world")
    fl_pos = fs_pos.get("LegMountPointFL")
    if fl_pos is None or fhla is None:
        _fail("can't run cross-check (missing inputs)")
        return False
    mm = next((c for c in fhla.get("children", []) or []
               if c.get("name") == "MotorMount:1"), None)
    if mm is None:
        _fail("MotorMount:1 not found")
        return False
    fixed = get_point(mm, "LegMountFixedPoint")
    if fixed is None:
        _fail("MotorMount:1/LegMountFixedPoint missing")
        return False
    fixed_world = fixed.get("pos_world_mm")
    delta = vec_dist(fl_pos, fixed_world)
    detail = (
        f"LegMountPointFL={fmt_xyz(fl_pos)}, "
        f"LegMountFixedPoint={fmt_xyz(fixed_world)}, "
        f"|Δ|={delta:.2f}mm"
    )
    if delta <= XYZ_TOLERANCE_MM:
        _ok("bracket mates on FL within 1mm", detail)
        ok = True
    else:
        _fail("CAD-alignment invariant broken", detail)
        print(
            "         ↳ MotorMount:1's LegMountFixedPoint should land on\n"
            "           LegMountPointFL on the body. Fix in Fusion: re-position\n"
            "           the FaceHuggerLegAssembly:1 occurrence so the bracket's\n"
            "           mating face lands on the FL mount point."
        )
        ok = False

    # Informational: where does MotorMountR's LegMountFixedPoint land?
    # (Not expected to match any LegMountPointXX — R brackets are virtually
    # instanced at FR/BL by the URDF generator, not in CAD.)
    mmr = next((c for c in fhla.get("children", []) or []
                if c.get("name") == "MotorMountR:1"), None)
    if mmr is not None:
        fixed_r = get_point(mmr, "LegMountFixedPoint")
        if fixed_r is not None:
            _info(
                "MotorMountR:1/LegMountFixedPoint (informational)",
                f"world={fmt_xyz(fixed_r.get('pos_world_mm'))}",
            )
    return ok


def check_legassembly_occurrences(fhla, mesh_files):
    """Visibility-tolerant: an occurrence counts as "found" if it's in the
    (visibility-filtered) JSON tree OR mentioned in any mesh_files entry's
    source_occurrences (which the export rules populate from the unfiltered
    live tree). Construction points only resolve through the JSON tree, so
    a hidden occurrence's expected point becomes a warning instead of a
    hard fail."""
    _section("Check 4 — FaceHuggerLegAssembly:1 occurrences and bodies")
    if fhla is None:
        _fail("FaceHuggerLegAssembly:1 not found")
        return False

    children = {c.get("name"): c for c in fhla.get("children", []) or []}

    # Pull every occurrence path that any STL was sourced from. Add the
    # leaf component:N too so an entry like
    # "FaceHuggerLegAssembly:1/MotorMountR:1" registers as MotorMountR:1.
    seen_paths = set()
    for entry in (mesh_files or {}).values():
        if not isinstance(entry, dict):
            continue
        for p in entry.get("source_occurrences") or []:
            seen_paths.add(p)
            seen_paths.add(p.rsplit("/", 1)[-1])

    all_ok = True
    for occ_name, expectations in REQUIRED_FHLA_OCCURRENCES.items():
        occ = children.get(occ_name)
        in_mesh = occ_name in seen_paths
        if occ is None and not in_mesh:
            _fail(f"{occ_name}", "occurrence missing (not in JSON tree, no mesh_files entry)")
            all_ok = False
            continue

        body = expectations.get("body")
        pt = expectations.get("point")
        bits = []
        sub_ok = True

        if occ is None:
            # Hidden occurrence — body presence implied by mesh_files entry.
            bits.append(f"body={body} (via mesh_files; occurrence hidden)")
            if pt:
                bits.append(
                    f"point={pt} unverified (occurrence hidden)"
                )
        else:
            if body and not has_body(occ, body):
                _fail(f"{occ_name}", f"body '{body}' missing")
                sub_ok = False
                all_ok = False
            elif body:
                bits.append(f"body={body}")
            if pt and not has_point(occ, pt):
                _fail(f"{occ_name}", f"construction point '{pt}' missing")
                sub_ok = False
                all_ok = False
            elif pt:
                bits.append(f"point={pt}")

        if sub_ok:
            _ok(occ_name, ", ".join(bits) if bits else "present")

    return all_ok


def check_joints(joints):
    _section("Check 5 — top-level joints array (informational on first run)")
    if not joints:
        if joints == []:
            _info(
                "joints array empty",
                "exporter ran but found no joints — check that revolute "
                "joints exist in Fusion AND that collect_joints walks the "
                "right component scope (xref'd FaceHuggerLegAssembly).",
            )
        else:
            _info("joints key missing", "expected ✗ before Phase A3 lands")
            print(
                "         ↳ exporter needs Phase A3 (collect_joints) — re-run\n"
                "           Phase 0 after that lands."
            )
        return False
    by_name = {j.get("name"): j for j in joints}
    all_ok = True
    for name in REQUIRED_JOINTS:
        j = by_name.get(name)
        if not j:
            _fail(name, "joint missing")
            all_ok = False
            continue
        axis = j.get("axis_dir_local_unit") or j.get("axis_dir_local")
        origin = j.get("axis_origin_local_mm") or j.get("origin_mm")
        lim = j.get("limits_rad") or {}
        lim_min_deg = (
            math.degrees(lim["min"]) if lim.get("min") is not None else None
        )
        lim_max_deg = (
            math.degrees(lim["max"]) if lim.get("max") is not None else None
        )
        rest_deg = (
            math.degrees(lim["rest"]) if lim.get("rest") is not None else None
        )
        bits = []
        if axis:
            bits.append(f"axis={fmt_xyz(axis)}")
        if origin:
            bits.append(f"origin_mm={fmt_xyz(origin)}")
        if lim_min_deg is not None and lim_max_deg is not None:
            bits.append(f"[{lim_min_deg:.1f}°, {lim_max_deg:.1f}°]")
        if rest_deg is not None:
            bits.append(f"rest={rest_deg:.1f}°")
        _ok(name, "  ".join(bits) if bits else "present")
    return all_ok


# --- formatting --------------------------------------------------------------


def fmt_xyz(v):
    if v is None:
        return "?"
    return "(" + ", ".join(f"{x:+.2f}" for x in v) + ")"


# --- entry point -------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    default_export = Path(__file__).parent / "fusion_export.json"
    parser.add_argument("--export", type=Path, default=default_export)
    args = parser.parse_args()

    if not args.export.exists():
        print(f"error: {args.export} does not exist")
        return 2

    with open(args.export) as f:
        export = json.load(f)

    occs = export.get("occurrences", []) or []
    fs = find_occurrence(occs, "FlexibleSkeleton:1")
    fhla = find_occurrence(occs, "FaceHuggerLegAssembly:1")
    joints = export.get("joints", [])

    print(f"phase0_verify — reading {args.export}")
    print(f"top-level keys: {sorted(export.keys())}")

    fs_ok, fs_pos = check_flexible_skeleton_points(fs)
    fhla_pts_ok, fhla_pos = check_legassembly_points(fhla)
    crosscheck_ok = check_world_pos_crosscheck(fs_pos, fhla) if fs_ok else False
    mesh_files = (export.get("mesh_files") or {})
    occ_ok = check_legassembly_occurrences(fhla, mesh_files)
    joints_ok = check_joints(joints)

    _section("Summary")
    rows = [
        ("FlexibleSkeleton points",  fs_ok,         True),
        ("LegAssembly points",       fhla_pts_ok,   True),
        ("World-pos cross-check",    crosscheck_ok, True),
        ("LegAssembly occurrences",  occ_ok,        True),
        ("Joints array",             joints_ok,     False),  # informational
    ]
    name_w = max(len(r[0]) for r in rows)
    for name, ok, required in rows:
        marker = "✓" if ok else "✗"
        suffix = "" if required else "  (informational)"
        print(f"  {name:<{name_w}}  : {marker}{suffix}")

    required_pass = all(ok for _, ok, req in rows if req)
    print()
    if required_pass:
        print("Phase 0 PASS — required checks all green. Phase A can proceed.")
        if not joints_ok:
            print("(joints array still pending — captured by Phase A3.)")
        return 0
    print("Phase 0 FAIL — fix the items marked ✗ in Fusion / re-export, then re-run.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
