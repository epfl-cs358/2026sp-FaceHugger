# URDF pipeline

!!! todo "Stub — to be written"
    How the URDF is generated and what it guarantees. Merge
    `_context/urdf-pipeline.md`, `_context/pipeline-spec.md`, and
    `_context/urdf-conventions.md` (tightly related).

## Fusion export → URDF

!!! todo
    The ExportBodiesToURDF add-in → `generate_urdf.py` + `facehugger_config.yaml`
    → `facehugger.urdf`. The URDF is the kinematic source of truth.

## Mesh re-origining

!!! todo
    Leg STLs re-origined to URDF joint landmarks; `origin_shift_mm` record.

## Joint zero & limits

!!! todo
    θ=0 = Fusion rest pose; per-joint limits; mirror plane.

## Conventions

!!! todo
    The URDF naming and frame conventions authors must respect.
