# === Plain Python — no Blender required ===
# Test utility — stubs `bpy` in sys.modules so addons can be import-tested
# without launching Blender.
"""Shared bpy shim for importing Blender add-ons under plain Python."""

import sys
import types


def install() -> None:
    """Install a minimal bpy shim into sys.modules.

    Only the import-time surface is covered: class-body subclassing of
    bpy.types.Operator/Panel and the bpy.props.* calls used as annotation
    values. Bodies of operator methods are not executed.

    Idempotent — returns immediately if a real or stub bpy is already present.
    """
    if "bpy" in sys.modules:
        return
    bpy = types.ModuleType("bpy")
    bpy.types = types.SimpleNamespace(
        Operator=type("Operator", (), {}),
        Panel=type("Panel", (), {}),
        Scene=type("Scene", (), {}),
    )
    bpy.props = types.SimpleNamespace(
        StringProperty=lambda **kw: None,
        IntProperty=lambda **kw: None,
        BoolProperty=lambda **kw: None,
        EnumProperty=lambda **kw: None,
    )
    bpy.app = types.SimpleNamespace(
        handlers=types.SimpleNamespace(
            frame_change_post=[],
            save_pre=[],
            load_post=[],
            persistent=lambda fn: fn,
        )
    )
    bpy.data = types.SimpleNamespace(
        objects=types.SimpleNamespace(get=lambda *a, **kw: None),
        actions=[],
        filepath="",
    )
    bpy.context = types.SimpleNamespace(scene=None, view_layer=None)
    bpy.utils = types.SimpleNamespace(
        register_class=lambda x: None, unregister_class=lambda x: None
    )
    bpy.path = types.SimpleNamespace(abspath=lambda p: p)
    sys.modules["bpy"] = bpy
