# 3D Printing

!!! todo "Stub - to be written"
    What to print and how. Adapted from `cad/README.md` and
    `cad/print_export.txt`; part list cross-references
    [Printed parts](parts.md#printed-parts).

## Printer & filament

!!! todo
    Recommended printer, material (PLA/PETG), nozzle, layer height.

You may use either Prusa MK3S or MK4S printers. Those are available in the printing room of the SPOT. Make sure to download the specific settings for the EPFL printers. You can find the steps to do so in the SPOT website. 

## Print settings

!!! todo
    Infill, walls, and supports, varying per part. From `print_export.txt`.

Our prints only use **SPOT-PETG** filament with **0.20 mm** muzzle width. Whether you use **STRUCTURAL** or **SPEED** settings is up to your timing. They yield similar results but try to use **STRUCTURAL** if you are not in a hurry. You do not need to change the infill percentage (15%). Make sure to select the correct printer model in PrusaSlicer (M43S or MK4S).

## Orientation per part

!!! todo
    How each body/leg part should be oriented on the bed (the
    ExportPrintableSTLs add-in applies per-body up-axis rotation).

`BodySkeleton.stl` : 

Do not change the orientation that appears when you first import the file in PrusaSlicer. 

You must add **Grid** or **Fitted** supports under every uncovered zone, including under the ESP32 hood and the little Lipo protector wing. Make sure to only add supports for "support generators" in PrusaSlicer. Do not add supports inside the screw holes.

![The body in Prusa-slicer](../assets/img/printing/BodySkeletonPrint.png)

`Middle_bridge.stl` and `Behind_bridge.stl` : Print as is.

![The bridges in Prusa-slicer](../assets/img/printing/BridgesPrint.png)

`HipMount.stl` :

Print with the back (largest surface) of the mount on the print bed. Add Grid or Fitted supports to the extrusion with the hex nut recess. Do not add supports inside the recess itself. 

Before printing this file, copy it once, then mirror it twice along the back. You should have a total of 4 hip mounts.

![The hip mounts in Prusa-slicer](../assets/img/printing/HipMountsPrint.png)


## Files

!!! todo
    Where the `.3mf` / gcode / STL files live in the repo.
