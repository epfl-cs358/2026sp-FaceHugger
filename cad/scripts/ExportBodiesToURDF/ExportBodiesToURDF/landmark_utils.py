# landmark_utils.py — Landmark resolution from the occurrences_json dict.
# Pure Python — no adsk dependency.
# The local-frame tree walk (_find_landmark_pos_mm, _find_landmark_in_component)
# stays in fusion_stl_export.py because it walks the live Fusion root object.


def find_landmark_world_pos(occurrences_json, landmark_name):
    """Walk the JSON occurrence tree and return the WORLD position of the first
    matching construction point. Returns None if not found."""

    def walk(nodes):
        for n in nodes or []:
            for p in n.get("points") or []:
                if p.get("name") == landmark_name:
                    return p.get("pos_world_mm")
            r = walk(n.get("children"))
            if r is not None:
                return r
        return None

    return walk(occurrences_json)


def find_landmark_world_pos_at_occurrence(occurrences_json, occ_path, landmark_name):
    """Like find_landmark_world_pos, but scope the lookup to a specific
    occurrence path (e.g. 'FaceHuggerLegAssembly:1/MotorMount:1'). Returns
    None if the occurrence or landmark isn't found."""
    parts = occ_path.split("/")
    nodes = occurrences_json
    node = None
    for part in parts:
        node = next((n for n in (nodes or []) if n.get("name") == part), None)
        if node is None:
            return None
        nodes = node.get("children", [])
    if node is None:
        return None
    for p in node.get("points") or []:
        if p.get("name") == landmark_name:
            return p.get("pos_world_mm")
    return None
