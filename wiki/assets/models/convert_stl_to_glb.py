#!/usr/bin/env python3
"""Convert an STL mesh to GLB for the <model-viewer> 3D embed.

<model-viewer> loads .glb/.gltf natively, not .stl, so convert first:

    pip install "trimesh[easy]"
    python convert_stl_to_glb.py ../../../cad/body/QuadrupedBody.stl

Writes QuadrupedBody.glb next to this script. Reference it from a CAD page:

    <model-viewer src="../assets/models/QuadrupedBody.glb" camera-controls
                  auto-rotate style="width:100%;height:400px;"></model-viewer>
"""

import pathlib
import sys

import trimesh


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit("usage: convert_stl_to_glb.py <path/to/mesh.stl>")
    src = pathlib.Path(sys.argv[1])
    out = pathlib.Path(__file__).parent / (src.stem + ".glb")
    trimesh.load(src).export(out)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
