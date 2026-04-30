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
