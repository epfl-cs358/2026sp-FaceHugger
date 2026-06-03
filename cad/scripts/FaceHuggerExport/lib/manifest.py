# manifest.py — mesh_files manifest builder and role-path migration.
# Pure dict operations — no adsk dependency.


def migrate_stale_role_paths(preserved, servo_paths):
    """Detect & repair stale occurrence paths in `_servo_role_assignment` after
    the servo component is renamed in CAD.

    If every preserved role path's leaf component differs from the new export's
    leaf component AND shares a single old name, rewrite each path by
    component-name substitution. Returns the (possibly migrated) assignment
    dict — values otherwise untouched."""
    if not preserved or not servo_paths:
        return preserved
    new_leaf = servo_paths[0].rsplit("/", 1)[-1]
    new_comp = new_leaf.rsplit(":", 1)[0]
    old_comps = set()
    role_paths = []
    for k, v in preserved.items():
        if k == "comment" or not isinstance(v, str) or "/" not in v:
            continue
        leaf = v.rsplit("/", 1)[-1]
        comp = leaf.rsplit(":", 1)[0]
        role_paths.append((k, v, comp))
        old_comps.add(comp)
    if new_comp in old_comps or len(old_comps) != 1:
        return preserved
    (old_comp,) = old_comps
    migrated = dict(preserved)
    for role, path, _ in role_paths:
        migrated[role] = path.replace(f"/{old_comp}:", f"/{new_comp}:")
    return migrated


def build_mesh_files_manifest(exported, preserved_role_assignment=None):
    """Turn the list of `exported` dicts from export_stls into the keyed
    `mesh_files` manifest section written to fusion_export.json.

    `preserved_role_assignment` is the `_servo_role_assignment` block from a
    prior JSON (if it exists), so user edits survive re-export. If none is
    passed, defaults are written: the first 3 occurrences with a ServoBase
    body are mapped to shoulder/hip/knee by list order."""
    manifest = {}
    servo_paths = []
    for entry in exported:
        stl_name = entry["stl"]
        manifest[stl_name] = {k: v for k, v in entry.items() if k != "stl"}
        if entry.get("source_body") == "ServoBase":
            servo_paths = list(entry.get("source_occurrences", []))

    if preserved_role_assignment is not None:
        manifest["_servo_role_assignment"] = migrate_stale_role_paths(
            preserved_role_assignment, servo_paths
        )
    else:
        roles = ["shoulder", "hip", "knee"]
        assignment = {
            "comment": (
                "USER-EDIT: map servo occurrence paths to kinematic roles. "
                "Defaults follow the order servos appear in the export; "
                "adjust after inspecting meshes in Blender or the sim. "
                "Re-exports preserve edits to this block."
            ),
        }
        for i, role in enumerate(roles):
            assignment[role] = servo_paths[i] if i < len(servo_paths) else None
        manifest["_servo_role_assignment"] = assignment

    return manifest
