"""
dump_blender_scene.py  —  plain-text Blender scene listing

Prints the current Blender scene's collection tree + per-object metadata to
stdout. Used alongside visualize_fusion_export.py to debug placement issues:
the Blender view shows WHAT got placed, this shows WHERE (in mm) in text.

Usage (chain after visualize, one Blender run builds + dumps):
    /Applications/Blender-3.3-LTS.app/Contents/MacOS/Blender \\
        --background \\
        --python animation/scripts/visualize_fusion_export.py \\
        --python animation/scripts/dump_blender_scene.py \\
      > /tmp/scene.txt

Usage (on a saved .blend from a prior visualize run):
    /Applications/Blender-3.3-LTS.app/Contents/MacOS/Blender \\
        --background /tmp/fh_debug.blend \\
        --python animation/scripts/dump_blender_scene.py \\
      > /tmp/scene.txt

Output format: human-readable lines. Locations read from obj.matrix_world
so the numbers shown are world-frame mm (visualize_fusion_export sets
`scene.unit_settings.scale_length = 0.001`, so 1 Blender unit = 1 mm).
"""

import bpy


def _fmt_loc(v):
    return f"({v.x:+8.2f}, {v.y:+8.2f}, {v.z:+8.2f})"


def _mesh_verts(obj):
    try:
        return len(obj.data.vertices)
    except Exception:
        return None


def _material_names(obj):
    try:
        return [m.name for m in obj.data.materials if m]
    except Exception:
        return []


def dump_object(obj, indent):
    """Single object line. `indent` is a string prefix."""
    loc = obj.matrix_world.translation
    parts = [f"{indent}- {obj.name}: loc={_fmt_loc(loc)} mm"]
    if obj.type == "MESH":
        verts = _mesh_verts(obj)
        if verts is not None:
            parts.append(f"{verts} verts")
        mats = _material_names(obj)
        if mats:
            parts.append(f"material={','.join(mats)}")
    parts.append(f"type={obj.type}")
    print(" ".join(parts) if len(parts) == 1 else parts[0] + ", " + ", ".join(parts[1:]))


def dump_collection(coll, depth, seen):
    """Recursive walk: print header, then objects, then child collections."""
    indent = "  " * depth
    obj_count = len(coll.objects)
    child_count = len(coll.children)
    suffix = []
    if obj_count:
        suffix.append(f"{obj_count} objects")
    if child_count:
        suffix.append(f"{child_count} sub-collections")
    tag = f"  [{', '.join(suffix)}]" if suffix else ""
    print(f"{indent}[{coll.name}]{tag}")
    for obj in sorted(coll.objects, key=lambda o: o.name):
        dump_object(obj, indent + "  ")
        seen.add(obj.name)
    for child in coll.children:
        dump_collection(child, depth + 1, seen)


def main():
    scene = bpy.context.scene
    us = scene.unit_settings

    print("=== Fusion scene dump ===")
    print(
        f"units: system={us.system}, scale_length={us.scale_length} m/unit, "
        f"display={us.length_unit}"
    )
    print(f"scene: {scene.name}")
    print(f"total objects in file: {len(bpy.data.objects)}")
    print()

    print("Collections:")
    seen = set()
    root_coll = scene.collection
    # Scene's root collection is implicit — walk its children (FusionExport etc.)
    if root_coll.children:
        for coll in root_coll.children:
            dump_collection(coll, 0, seen)
    # Plus any objects directly under the scene root collection (should be rare
    # for our visualizer output, but exists for stray objects).
    direct_objs = list(root_coll.objects)
    if direct_objs:
        print()
        print("[<scene root>] (objects not in a named collection)")
        for obj in sorted(direct_objs, key=lambda o: o.name):
            dump_object(obj, "  ")
            seen.add(obj.name)

    # Safety check: anything in bpy.data.objects we didn't visit? This
    # surfaces orphaned objects (created but never linked into a collection).
    orphans = [o.name for o in bpy.data.objects if o.name not in seen]
    print()
    if orphans:
        print(f"Orphan objects ({len(orphans)}, not linked to any visited collection):")
        for name in sorted(orphans):
            print(f"  - {name}")
    else:
        print("Orphan objects: 0")


if __name__ == "__main__":
    main()
