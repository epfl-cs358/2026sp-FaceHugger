# Fusion 360 export scripts

Two Fusion 360 scripts live in `cad/scripts/`, both written for this project. They
run inside Fusion, not in the host Python environment, so you load them through
Fusion's own script runner. (Their manifests declare `type: script`, so they are
Scripts you run on demand, not persistent Add-Ins.)

To load either one: open Fusion 360, go to **Utilities -> ADD-INS -> Scripts and
Add-Ins**, stay on the **Scripts** tab, click the green `+` next to **My Scripts**,
and browse to the script's inner folder (the one containing the `.py` and
`.manifest`). It then appears in the list; select it and click **Run**. You only
need to add it once; afterwards it stays in **My Scripts**.

!!! todo "Add install screenshots"
    Add screenshots of the Scripts and Add-Ins dialog, the green `+`, and the
    folder picker, so a first-time user can follow along.

## ExportBodiesToURDF

Folder: `cad/scripts/ExportBodiesToURDF/`

This add-in is the first stage of the URDF pipeline. It reads the Fusion assembly
and writes three outputs to `code/simulation/generated/`:

- `fusion_export.json`: CAD tree, mesh manifest (`mesh_files`), per-occurrence
  world transforms, and the `_servo_role_assignment` field that maps meshes to
  firmware servo roles. Re-runs preserve any edits you have made to
  `_servo_role_assignment`.
- `fusion_export.txt`: human-readable version of the same tree, for quick
  inspection.
- `exported_meshes/*.stl`: eight STL files (chassis, leg mounts, shoulders,
  upper and lower links, servo). Each leg mesh is re-origined to its URDF joint
  landmark, so the URDF generator can emit `<origin xyz="0 0 0"/>` on visuals
  without offset adjustments.

After running the add-in, run `python code/facehugger.py urdf` from the repo root
to regenerate `facehugger.urdf` from the new export.

If the chassis STL contains unexpected electronics or PCBs, the `combined.parts`
list in the add-in source (`ExportBodiesToURDF.py`) is out of date; add or remove
the relevant occurrences and re-run.

## ExportPrintableSTLs

Folder: `cad/scripts/ExportPrintableSTLs/`

This add-in is independent of the URDF pipeline. It exports print-ready STLs with
a per-body up-axis rotation applied, suitable for slicing. Its output is not
consumed by `generate_urdf.py` or any other pipeline script.
