# Fusion 360 export add-ins

Two Fusion 360 add-ins live in `cad/scripts/`. They run inside Fusion, not in the
host Python environment, so you load them through Fusion's own script runner.

To load either add-in: open Fusion 360, go to **Utilities -> Add-Ins -> Scripts
and Add-Ins**, switch to the **Add-Ins** tab, click the `+` icon next to **My
Add-Ins**, and browse to the add-in folder. Run it from the same panel.

## ExportBodiesToURDF

Folder: `cad/scripts/ExportBodiesToURDF/`

This add-in is the first stage of the URDF pipeline. It reads the Fusion assembly
and writes three outputs to `code/simulation/generated/`:

- `fusion_export.json` - CAD tree, mesh manifest (`mesh_files`), per-occurrence
  world transforms, and the `_servo_role_assignment` field that maps meshes to
  firmware servo roles. Re-runs preserve any edits you have made to
  `_servo_role_assignment`.
- `fusion_export.txt` - human-readable version of the same tree, for quick
  inspection.
- `exported_meshes/*.stl` - eight STL files (chassis, leg mounts, shoulders,
  upper and lower links, servo). Each leg mesh is re-origined to its URDF joint
  landmark, so the URDF generator can emit `<origin xyz="0 0 0"/>` on visuals
  without offset adjustments.

After running the add-in, run `python facehugger.py urdf` from `code/simulation/`
to regenerate `facehugger.urdf` from the new export.

If the chassis STL contains unexpected electronics or PCBs, the `combined.parts`
list in the add-in source (`ExportBodiesToURDF.py`) is out of date - add or remove
the relevant occurrences and re-run.

## ExportPrintableSTLs

Folder: `cad/scripts/ExportPrintableSTLs/`

This add-in is independent of the URDF pipeline. It exports print-ready STLs with
a per-body up-axis rotation applied, suitable for slicing. Its output is not
consumed by `generate_urdf.py` or any other pipeline script.
