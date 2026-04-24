"""
inspect_stl.py  —  connected-component analysis for binary STL files

Groups triangles that share vertices into connected components, prints
each component's triangle count, bounding box, and centroid. Useful for
answering "what's actually in this STL?" when a Fusion export looks wrong
— e.g. the chassis STL should contain one welded shell but a ghost body
leaked in via a shared-component visibility quirk.

Pure stdlib; no Blender, no numpy.

Usage:
    python3 animation/scripts/inspect_stl.py <path-to.stl> [--top N]

Example:
    python3 animation/scripts/inspect_stl.py \\
        code/simulation/exported_meshes/QuadrupedBody.stl
"""

import argparse
import struct
import sys
from collections import defaultdict
from pathlib import Path


def read_stl(path):
    """Return list of triangles, each a 3-tuple of (x, y, z) vertex tuples."""
    with open(path, "rb") as f:
        f.read(80)   # header
        ntri = struct.unpack("<I", f.read(4))[0]
        tris = []
        for _ in range(ntri):
            f.read(12)   # normal (skip)
            v = [struct.unpack("<fff", f.read(12)) for _ in range(3)]
            tris.append(v)
            f.read(2)    # attribute byte count (skip)
    return tris


def _rkey(v, grid=0.1):
    """Round a vertex to a grid (mm) so coincident vertices hash together."""
    return (round(v[0] / grid) * grid,
            round(v[1] / grid) * grid,
            round(v[2] / grid) * grid)


def connected_components(tris, grid_mm=0.1):
    """Union-find over triangles sharing rounded vertices. Returns a list
    of component dicts, each with: tri_ids, tri_count, bbox, centroid."""
    n = len(tris)
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    seen_at = {}
    for i, tri in enumerate(tris):
        for v in tri:
            k = _rkey(v, grid_mm)
            prev = seen_at.get(k)
            if prev is not None:
                union(i, prev)
            else:
                seen_at[k] = i

    groups = defaultdict(list)
    for i in range(n):
        groups[find(i)].append(i)

    components = []
    for tri_ids in groups.values():
        vs = [v for i in tri_ids for v in tris[i]]
        xs = [v[0] for v in vs]
        ys = [v[1] for v in vs]
        zs = [v[2] for v in vs]
        components.append({
            "tri_count": len(tri_ids),
            "bbox": ((min(xs), min(ys), min(zs)),
                     (max(xs), max(ys), max(zs))),
            "centroid": (sum(xs) / len(xs),
                         sum(ys) / len(ys),
                         sum(zs) / len(zs)),
        })
    components.sort(key=lambda c: -c["tri_count"])
    return components


def format_component(idx, c):
    (mn, mx) = c["bbox"]
    cx, cy, cz = c["centroid"]
    return (
        f"[{idx}] {c['tri_count']:6d} tris  "
        f"centroid=({cx:+8.2f},{cy:+8.2f},{cz:+8.2f})  "
        f"bbox x=[{mn[0]:+7.2f},{mx[0]:+7.2f}] "
        f"y=[{mn[1]:+7.2f},{mx[1]:+7.2f}] "
        f"z=[{mn[2]:+7.2f},{mx[2]:+7.2f}]"
    )


def main():
    parser = argparse.ArgumentParser(description="Inspect an STL by connected components.")
    parser.add_argument("stl", type=Path, help="Path to binary STL file")
    parser.add_argument("--top", type=int, default=None,
                        help="Only print the top-N largest components")
    parser.add_argument("--grid", type=float, default=0.1,
                        help="Vertex rounding grid in mm (default 0.1)")
    args = parser.parse_args()

    if not args.stl.exists():
        print(f"not found: {args.stl}", file=sys.stderr)
        sys.exit(1)

    tris = read_stl(args.stl)
    comps = connected_components(tris, args.grid)

    # Overall bbox
    vs = [v for tri in tris for v in tri]
    xs = [v[0] for v in vs]; ys = [v[1] for v in vs]; zs = [v[2] for v in vs]

    print(f"=== {args.stl} ===")
    print(f"total triangles: {len(tris)}")
    print(
        f"overall bbox: x=[{min(xs):+7.2f},{max(xs):+7.2f}] "
        f"y=[{min(ys):+7.2f},{max(ys):+7.2f}] "
        f"z=[{min(zs):+7.2f},{max(zs):+7.2f}]"
    )
    print(f"connected components: {len(comps)}")
    print()

    shown = comps if args.top is None else comps[: args.top]
    for i, c in enumerate(shown):
        print(format_component(i, c))
    if args.top is not None and len(comps) > args.top:
        hidden_tris = sum(c["tri_count"] for c in comps[args.top:])
        print(f"... {len(comps) - args.top} more components ({hidden_tris} tris total)")


if __name__ == "__main__":
    main()
