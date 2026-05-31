"""fusion_api.py — Fusion 360 API helpers for ExportPrintableSTLs.

No top-level adsk import — adsk objects are passed in as parameters and the
module must remain importable outside Fusion (e.g. in tests or CI).
"""


def iter_occ_tree(occurrences):
    """Yield (occ, path) for every occurrence in the tree, depth-first.

    `occurrences` is a Fusion 360 OccurrenceList (root level).
    """
    for occ in occurrences:
        path = occ.name
        yield occ, path
        children = occ.childOccurrences
        if children:
            yield from _iter_occ_subtree(children, path)


def _iter_occ_subtree(occurrences, parent_path):
    """Recursively yield (occ, path) for child occurrences."""
    for occ in occurrences:
        path = f"{parent_path}/{occ.name}"
        yield occ, path
        children = occ.childOccurrences
        if children:
            yield from _iter_occ_subtree(children, path)


def find_body(tree, body_name, hint):
    """Search a pre-built tree for a body by name, optionally filtered by hint.

    `tree` is an iterable of (occ, path) pairs as produced by iter_occ_tree.
    Returns (occ, body, path) on success, or (None, None, None) if not found.
    """
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


def export_body_stl(mgr, body, out_path):
    """Export a single BRep body to a binary STL file via the Fusion export manager.

    `mgr` is a Fusion 360 ExportManager.
    Uses MeshRefinementHigh; does not send to the print utility.
    """
    import adsk.fusion  # local import — only available inside Fusion at runtime

    opts = mgr.createSTLExportOptions(body, out_path)
    opts.sendToPrintUtility = False
    opts.isBinaryFormat = True
    opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    mgr.execute(opts)
