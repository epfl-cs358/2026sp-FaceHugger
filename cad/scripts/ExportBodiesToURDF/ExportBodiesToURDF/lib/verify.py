# verify.py — Assembly hierarchy checks and export-rule physics validation.
# Pure dict operations — no adsk dependency.

_REQUIRED_FS_POINTS = (
    "LegMountPointFL",
    "LegMountPointFR",
    "LegMountPointBR",
    "LegMountPointBL",
)
_REQUIRED_FHLA_POINTS = (
    "BodyToLink1Point",
    "Link1ToLink2Point",
    "Link2ToLink3Point",
)
_REQUIRED_FHLA_OCCS = {
    "Link1L:1": {"body": "Link1L"},
    "Link1R:1": {"body": "Link1R"},
    "Link2L:1": {"body": "Link2"},
    "Link3L:1": {"body": "Link3"},
    "MotorMount:1": {"body": "LegMountL", "point": "LegMountFixedPoint"},
    "MotorMountR:1": {"body": "LegMountR", "point": "LegMountFixedPoint"},
    "LegBaseServoEnclosure:1": {"body": "ServoBase"},
    "LegBaseServoEnclosure:2": {"body": "ServoBase"},
    "LegBaseServoEnclosure:3": {"body": "ServoBase"},
}
_REQUIRED_JOINTS = ("Link1Revolute", "Link2Revolute", "Link3Revolute")


def verify_against_assembly_hierarchy(occurrences_json, joints, mesh_files):
    """Lightweight diagnostic against ASSEMBLY_HIERARCHY.md. Returns a list of
    human-readable lines (✓/✗ rows). Does not fail the export — just reports."""
    fs = next(
        (o for o in occurrences_json if o.get("name") == "FlexibleSkeleton:1"),
        None,
    )
    fhla = next(
        (o for o in occurrences_json if o.get("name") == "FaceHuggerLegAssembly:1"),
        None,
    )

    def _has_point(node, name):
        return any(p.get("name") == name for p in (node.get("points") or []))

    def _has_body(node, body_name):
        bodies = node.get("bodies") or []
        return any(
            (b.get("name") if isinstance(b, dict) else b) == body_name for b in bodies
        )

    def _mark(ok):
        return "✓" if ok else "✗"

    rows = []

    # 1. FlexibleSkeleton points
    fs_ok = bool(fs) and all(_has_point(fs, n) for n in _REQUIRED_FS_POINTS)
    rows.append(
        f"  {_mark(fs_ok)} FlexibleSkeleton points ({', '.join(_REQUIRED_FS_POINTS)})"
    )

    # 2. FaceHuggerLegAssembly points
    fhla_pts_ok = bool(fhla) and all(_has_point(fhla, n) for n in _REQUIRED_FHLA_POINTS)
    rows.append(
        f"  {_mark(fhla_pts_ok)} FaceHuggerLegAssembly points "
        f"({', '.join(_REQUIRED_FHLA_POINTS)})"
    )

    # 3. Bracket alignment cross-check
    cross_ok = False
    if fs and fhla:
        fs_pts = {p.get("name"): p for p in (fs.get("points") or [])}
        fl = fs_pts.get("LegMountPointFL")
        mm = next(
            (
                c
                for c in (fhla.get("children") or [])
                if c.get("name") == "MotorMount:1"
            ),
            None,
        )
        if fl and mm:
            fp = next(
                (
                    p
                    for p in (mm.get("points") or [])
                    if p.get("name") == "LegMountFixedPoint"
                ),
                None,
            )
            if fp:
                a = fl.get("pos_world_mm") or [0, 0, 0]
                b = fp.get("pos_world_mm") or [0, 0, 0]
                d = sum((a[i] - b[i]) ** 2 for i in range(3)) ** 0.5
                cross_ok = d <= 1.0
    rows.append(
        f"  {_mark(cross_ok)} MotorMount LegMountFixedPoint ≈ LegMountPointFL (≤ 1mm)"
    )

    # 3b. Leg X-axis reach (informational)
    if fhla:
        tip = next(
            (p for p in (fhla.get("points") or []) if p.get("name") == "Link3TipPoint"),
            None,
        )
        mm1 = next(
            (
                c
                for c in (fhla.get("children") or [])
                if c.get("name") == "MotorMount:1"
            ),
            None,
        )
        anchor = None
        if mm1:
            anchor = next(
                (
                    p
                    for p in (mm1.get("points") or [])
                    if p.get("name") == "LegMountFixedPoint"
                ),
                None,
            )
        if tip and anchor:
            tw = tip.get("pos_world_mm") or [0, 0, 0]
            aw = anchor.get("pos_world_mm") or [0, 0, 0]
            dx = abs(tw[0] - aw[0])
            rows.append(
                f"    • leg X reach (|Δx| Link3TipPoint → MotorMount:1 "
                f"LegMountFixedPoint) = {dx:.1f} mm"
            )

    # 4. FaceHuggerLegAssembly occurrences + bodies
    occ_ok = bool(fhla)
    point_warnings = []
    if fhla:
        children = {c.get("name"): c for c in (fhla.get("children") or [])}
        seen_paths = set()
        for entry in (mesh_files or {}).values():
            if not isinstance(entry, dict):
                continue
            for p in entry.get("source_occurrences") or []:
                seen_paths.add(p)
                seen_paths.add(p.rsplit("/", 1)[-1])
        for occ_name, expect in _REQUIRED_FHLA_OCCS.items():
            child = children.get(occ_name)
            in_mesh = occ_name in seen_paths
            if not child and not in_mesh:
                occ_ok = False
                break
            if child and "body" in expect and not _has_body(child, expect["body"]):
                occ_ok = False
                break
            if "point" in expect:
                if child:
                    if not _has_point(child, expect["point"]):
                        occ_ok = False
                        break
                else:
                    point_warnings.append(
                        f"{occ_name} is hidden — can't verify {expect['point']}"
                    )
    rows.append(
        f"  {_mark(occ_ok)} LegAssembly occurrences + bodies "
        "(via JSON tree or mesh_files)"
    )
    for w in point_warnings:
        rows.append(f"    note: {w}")

    # 5. Joints
    joint_names = {j.get("name") for j in joints}
    j_ok = all(n in joint_names for n in _REQUIRED_JOINTS)
    rows.append(f"  {_mark(j_ok)} Joints array ({', '.join(_REQUIRED_JOINTS)})")

    return rows


def verify_export_rules_have_physics(mesh_files, occurrences_json):
    """Check that every occurrence referenced by mesh_files has a valid physics
    block in the occurrence tree. Returns a list of warning strings (empty = all OK).

    This catches the silent-fallback problem where a MCAD component gets
    FALLBACK_INERTIA in generate_urdf.py without any warning during export.
    """
    warnings = []

    def _find_physics(nodes, occ_path):
        """Walk the occurrence tree to find the physics block at occ_path."""
        parts = occ_path.split("/")
        current_nodes = nodes
        node = None
        for part in parts:
            node = next(
                (n for n in (current_nodes or []) if n.get("name") == part), None
            )
            if node is None:
                return None
            current_nodes = node.get("children", [])
        return node.get("physics") if node else None

    seen_paths = set()
    for stl_name, entry in (mesh_files or {}).items():
        if not isinstance(entry, dict):
            continue
        for occ_path in entry.get("source_occurrences") or []:
            if occ_path in seen_paths:
                continue
            seen_paths.add(occ_path)
            physics = _find_physics(occurrences_json, occ_path)
            if physics is None:
                warnings.append(
                    f"{stl_name}: {occ_path} has no physics block in occurrence tree"
                )
            elif "error" in physics:
                warnings.append(
                    f"{stl_name}: {occ_path} physics error: {physics['error']}"
                )

    return warnings
