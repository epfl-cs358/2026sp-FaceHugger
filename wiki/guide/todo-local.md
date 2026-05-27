# Wiki TODO (local, not published)

## Content stubs

- [x] `guide/printing.md` — part viewer, leg orientation, support screenshots, print times, SPOT context
- [x] `guide/wiring.md` — complete
- [x] `guide/assembly.md` — complete
- [x] `guide/extending.md` — complete
- [x] `guide/calibration.md` — written in recent merge
- [ ] `guide/design.md` — partially written (concept + leg anatomy done); still missing:
  - [ ] How a leg moves (kinematics intuition, zero pose, sign conventions at a glance)
  - [ ] Sizing & loads (why links/servos sized this way, weight per leg)
- [x] `reference/firmware/kinematics.md` — removed; IK explanation folded into `guide/design.md` (IMU loop concept, hip/knee law-of-cosines, atan2 for shoulder, implemented in sim+Blender but not in firmware)
- [x] `reference/remote-control/websocket-api.md` — written: connection overview, mermaid sequence, safety behaviours, link to api.md
- [ ] `reference/animation/urdf-pipeline.md` — one todo left: design tips for construction-point placement

## Guide fixes

- [ ] `assembly.md` — all legs called identical, but Link1L/R and HipMount are L/R mirror pairs; clarify and use FL/FR/BL/BR (front-left etc.) consistently across assembly + printing
- [ ] `toolchain/fusion-export.md` — add "Scripts and Add-Ins" install screenshots (Scripts tab, green +, folder picker) for ExportBodiesToURDF and ExportPrintableSTLs
- [ ] `guide/printing.md` — add the Fusion files download link for the design section (CAD source)
- [ ] Fix em-dashes consistently across the whole wiki

## Structure

- [ ] Add `guide/index.md` section landing (Build Guide currently opens straight on Design)
- [ ] Port remaining reference/convention doc changes from local notes

## Diagrams

- [ ] Replace ASCII-art sketches in `reference/conventions.md` (body-frame and NEUTRAL top-view) with real figures
- [x] `reference/firmware/kinematics.md` — removed; no diagrams needed
- [ ] `reference/remote-control/websocket-api.md` — needs a message-flow diagram

## Housekeeping

- [ ] Decide whether to gitignore `.gcode` / `.bgcode` files in `cad/leg/prusa/link2/` (19 MB gcode file committed)
- [ ] Check `.blend1` backup files in `animation/` root — seem committed, probably should be gitignored
- [x] Shell `.3mf` added by teammate (`cad/body/prusa/ShellPrint.3mf`) — added to Files section in printing.md
