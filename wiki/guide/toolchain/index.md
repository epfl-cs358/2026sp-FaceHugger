# Toolchain

FaceHugger's build pipeline connects four tools in a fixed order. Understanding
each stage prevents surprises when you change something upstream and wonder why
nothing moved downstream.

**Fusion 360 -> URDF export.** The two Fusion add-ins in
`cad/scripts/ExportBodiesToURDF/` and `cad/scripts/ExportPrintableSTLs/` are the
first stage. The URDF exporter writes a JSON manifest and STL meshes that the
Python generator (`generate_urdf.py`) turns into `facehugger.urdf`. Everything
downstream is derived from that URDF - never hand-edit the generated files. See
[Fusion 360 export add-ins](fusion-export.md) for the step-by-step.

**URDF -> Blender rig.** Running `facehugger.py blender --rigged` ingests the
URDF and builds an armature in Blender 5.x. The rigged scene
(`animation/fh_rigged_latest.blend`) is the starting point for clip authoring.

**Blender rig -> clip authoring and export.** The FH Clip Panel (N-panel add-on
in Blender) lets you pose the rig, record keyframe bundles, and export them as C
headers and JavaScript players that the firmware can play back. See
[Blender clip authoring and export](blender-clips.md) for the workflow.

**Exported clips -> firmware flash.** The final step compiles the PlatformIO
project (which embeds the exported clip headers) and flashes it to the ESP32 over
USB. After flashing, the robot joins its own Wi-Fi network and accepts commands
over WebSocket. See [Flashing the firmware](flashing.md) for the commands.
