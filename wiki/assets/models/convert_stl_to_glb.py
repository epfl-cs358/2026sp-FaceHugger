#!/usr/bin/env python3
"""Convert an STL mesh to GLB for the <model-viewer> 3D embed.

<model-viewer> loads .glb/.gltf natively, not .stl, so convert first:

    pip install "trimesh[easy]"
    python convert_stl_to_glb.py ../../../cad/body/BodySkeleton.stl --color "#0FFF4B"
    python convert_stl_to_glb.py ../../../cad/body/HipMount.stl --up X --color white

Writes <stem>.glb next to this script.

--up axis     Which STL axis is "up" (default: Z).
              Z → rotate -90° around X  (standard Fusion/PrusaSlicer export)
              X → rotate +90° around Z  (e.g. HipMount where +X is up in CAD)
              Y → no rotation needed    (already Y-up)

--color hex   Hex color (#RRGGBB) or "white" to set the PBR base color.
              Uses a low-roughness PBR material so edges read clearly under
              the neutral HDRI in model-viewer.
"""

import pathlib
import sys

import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

# Maps STL "up" axis → rotation to apply so that axis ends up as +Y in GLB.
_UP_ROTATIONS = {
    "Z": trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0]),
    "X": trimesh.transformations.rotation_matrix(np.pi / 2, [0, 0, 1]),
    "Y": np.eye(4),
}

_NAMED_COLORS = {
    "white": "#FFFFFF",
}


def _parse_hex(hex_str: str) -> tuple[float, float, float, float]:
    h = hex_str.lstrip("#")
    if len(h) != 6:
        sys.exit(f"--color must be a 6-digit hex color (got {hex_str!r})")
    r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    return r / 255, g / 255, b / 255, 1.0


def _pop_flag(args: list, flag: str, default=None):
    if flag in args:
        idx = args.index(flag)
        val = args[idx + 1]
        del args[idx : idx + 2]
        return val
    return default


def main() -> None:
    args = sys.argv[1:]

    up = (_pop_flag(args, "--up") or "Z").upper()
    color_raw = _pop_flag(args, "--color")

    if len(args) != 1:
        sys.exit(
            "usage: convert_stl_to_glb.py <path/to/mesh.stl> [--up Z|X|Y] [--color #RRGGBB]"
        )
    if up not in _UP_ROTATIONS:
        sys.exit(f"--up must be Z, X, or Y (got {up!r})")

    src = pathlib.Path(args[0])
    out = pathlib.Path(__file__).parent / (src.stem + ".glb")

    mesh = trimesh.load(src)
    mesh.apply_transform(_UP_ROTATIONS[up])

    if color_raw is not None:
        hex_str = _NAMED_COLORS.get(color_raw.lower(), color_raw)
        r, g, b, a = _parse_hex(hex_str)
        # PBR material: matte finish so surface details read clearly
        mesh.visual = trimesh.visual.TextureVisuals(
            material=PBRMaterial(
                baseColorFactor=np.array([r, g, b, a]),
                roughnessFactor=0.75,
                metallicFactor=0.0,
            )
        )

    mesh.export(out)
    note = f"color={color_raw}" if color_raw else "default color"
    print(f"wrote {out}  [up={up}, {note}]")


if __name__ == "__main__":
    main()
