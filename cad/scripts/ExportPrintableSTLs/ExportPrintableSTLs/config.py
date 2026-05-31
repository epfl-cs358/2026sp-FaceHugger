"""config.py — pure-data constants for ExportPrintableSTLs."""

# NOTE: These STLs go to cad/body/ and cad/leg/ for 3D printing.
# They are NOT the URDF-source STLs (in code/simulation/generated/exported_meshes/).
# Do not use these paths in generate_urdf.py or facehugger_config.yaml.

from dataclasses import dataclass

VALID_SUBFOLDERS: frozenset[str] = frozenset({"body", "leg"})
DEFAULT_UP: str = "+Z"
LOG_MARKER: str = "# === EXPORT LOG (auto-managed below — do not edit) ==="

# 3×3 rotation matrices: R such that R · v_up = (0, 0, 1),
# where v_up is the body-local axis named by the key.
# Applied to every vertex AND normal of the exported STL.
# Identity for "+Z" (no rotation).
ROTATIONS: dict[str, tuple[tuple[float, ...], ...]] = {
    "+Z": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    "-Z": ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)),  # 180° about +X
    "+Y": ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)),  # +90° about +X
    "-Y": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0)),  # -90° about +X
    "+X": ((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)),  # -90° about +Y
    "-X": ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0)),  # +90° about +Y
}


@dataclass(frozen=True)
class SpecEntry:
    subfolder: str  # e.g. "body" or "leg"
    body_name: str  # e.g. "QuadrupedBody"
    up_axis: str  # e.g. "+Z", "-X"
    hint: str | None  # occurrence path substring filter, or None
