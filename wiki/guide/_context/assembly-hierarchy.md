<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  code/simulation/docs/ASSEMBLY_HIERARCHY.md
Original kind:  spec
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        guide/parts/printed-parts.md
-->

> **Reference material.** Verbatim copy of `code/simulation/docs/ASSEMBLY_HIERARCHY.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `code/simulation/docs/ASSEMBLY_HIERARCHY.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
<!--
CONSISTENCY-CHECK 2026-05-21
Verified against: feat/wiki-setup @ 0a1a138
- [ok]    Body-side landmarks LegMountPointFL/FR/BR/BL and joint origins BodyToLink1Point/Link1ToLink2Point/Link2ToLink3Point + LegMountFixedPoint all present in the exporter's CONSTRUCTION_POINTS (cad/scripts/ExportBodiesToURDF/.../ExportBodiesToURDF.py:114-135).
- [ok]    Link bodies match cad/leg/: Link1L, Link1R, Link2 (Link2L), Link3 (Link3L), LegMountL, LegMountR — and EXPORT_RULES references the same occurrences (Link1L:1/Link1R:1/Link2L:1/Link3L:1, MotorMount:1/MotorMountR:1). Joints Link1/2/3Revolute confirmed (_REQUIRED_JOINTS).
- [stale] Servo xrefs are out of date. Doc shows "Servo_Mouser_Model:1/2/3" with body "ServoBase". The exporter has 0 references to Servo_Mouser_Model; servos are now LegBaseServoEnclosure:N occurrences with a ServoMountPoint landmark (10 references). Replace Servo_Mouser_Model/ServoBase → LegBaseServoEnclosure/ServoMountPoint.
- [drift] Root node "FullBodyAssembly" is not used by the exporter — occurrence paths are rooted at FlexibleSkeleton:1/... and FaceHuggerLegAssembly:1/... (active-design root). Hierarchy is otherwise consistent.
- [drift] base mesh combine-rule: doc/QuadrupedBody:1 only; EXPORT_RULES combines QuadrupedBody:1 + LipoCage:1 into one base STL (ExportBodiesToURDF.py:204-215). cad/body/ also has BehindBridge.stl / MiddleBridge.stl not represented in this tree.
- [ok]    There's an extra construction point in the exporter not shown in the tree (BodyBottomPoint, Link3TipPoint) — additive, doesn't contradict the doc.
- [todo] (hidden) electronics components (PCB/PCA9685/Buck/OLED/vl53l0x/MPU6050/Lipo_Protector) and the VISIBLE_ONLY exclusion are CAD-state assertions not checkable from the exporter source alone.
-->
# FullBodyAssembly — Component Hierarchy

Notation:
  [asm]     = assembly component (has sub-components)
  [xref]    = external reference (linked file)
  [body]    = body inside a component
  (hidden)  = not visible, excluded by VISIBLE_ONLY=True

Only ServoBase body shown for servo xrefs (other servo bodies omitted for brevity).

---

[asm] FullBodyAssembly
│
├── [xref] FlexibleSkeleton:1
│   │
│   ├── Construction/
│   │   ├── LipoProtectorSketchPlane
│   │   ├── LegMountPointFL          ← body-side shoulder anchor, FL
│   │   ├── LegMountPointBL          ← body-side shoulder anchor, BL
│   │   ├── LegMountPointFR          ← body-side shoulder anchor, FR
│   │   └── LegMountPointBR          ← body-side shoulder anchor, BR
│   │
│   ├── [comp] QuadrupedBody:1
│   │   └── [body] QuadrupedBody
│   │
│   ├── [comp] PCB_ESP32-38Pins:1    (hidden)
│   ├── [comp] PCA9685 v18:1         (hidden)
│   ├── [comp] BuckConverter:1       (hidden)
│   ├── [comp] OLED 128x64 Display:1 (hidden)
│   ├── [comp] vl53l0x:1             (hidden)
│   ├── [comp] Lipo_Protector:1      (hidden)
│   ├── [comp] MPU6050 v2:1          (hidden)
│   ├── [comp] vl53l0x:2             (hidden)
│   │
│   └── [xref] LipoCage:1
│       └── [body] LipoCage
│
└── [xref] FaceHuggerLegAssembly:1
    │
    ├── Relationships/
    │   └── Joints/
    │       ├── Link3Revolute         ← revolute, Link2L→Link3L, axis=Link2ToLink3Axis
    │       ├── Link2Revolute         ← revolute, Link1L→Link2L, axis=Link1ToLink2Axis
    │       └── Link1Revolute         ← revolute, MotorMount→Link1L, axis=BodyToLink1Axis
    │
    ├── Construction/
    │   ├── Link1Alignment            (hidden)
    │   ├── Link2Alignment            (hidden)
    │   ├── Link3Alignment            (hidden)
    │   ├── BodyToLink1Axis           ← shoulder rotation axis
    │   ├── Link1ToLink2Axis          ← hip rotation axis
    │   ├── Link2ToLink3Axis          ← knee rotation axis
    │   ├── BodyToLink1Point          ← shoulder joint origin (world = LegMountPointXX)
    │   ├── Link1ToLink2Point         ← hip joint origin
    │   └── Link2ToLink3Point         ← knee joint origin
    │
    ├── [xref] Servo_Mouser_Model:1
    │   └── [body] ServoBase          (other servo bodies omitted)
    │
    ├── [xref] Servo_Mouser_Model:2
    │   └── [body] ServoBase
    │
    ├── [xref] Servo_Mouser_Model:3
    │   └── [body] ServoBase
    │
    ├── [comp] Link1L:1
    │   └── [body] Link1              ← L-side shoulder link
    │
    ├── [comp] Link2L:1
    │   └── [body] Link2              ← upper leg (shared, no R sibling yet)
    │
    ├── [comp] Link3L:1
    │   └── [body] Link3              ← lower leg (shared, no R sibling yet)
    │
    ├── [xref] MotorMount:1
    │   ├── [body] LegMountL          ← L-side bracket body
    │   ├── Construction/
    │   │   └── LegMountFixedPoint    ← bracket-side anchor (mates with LegMountPointXX)
    │   └── [xref] Servo_Mouser_Model:1
    │       └── [body] ServoBase
    │
    ├── [comp] Link1R:1
    │   └── [body] Link1R             ← R-side shoulder link (XZ mirror of Link1L)
    │
    └── [xref] MotorMountR:1
        ├── [body] LegMountR          ← R-side bracket body (XZ mirror of LegMountL)
        ├── Construction/
        │   └── LegMountFixedPoint    ← bracket-side anchor (same name as L side)
        └── [xref] Servo_Mouser_Model:1
            └── [body] ServoBase
