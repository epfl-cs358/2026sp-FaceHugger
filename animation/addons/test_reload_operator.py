#!/usr/bin/env python3
"""Pure-Python test for the FH_OT_reload_panel operator — the in-place
"Reload Add-on" button so animators don't have to relaunch Blender after
editing fh_clip_panel.py.

Blender's import/registration machinery can't be exercised without a
running Blender; this test only verifies the operator class exists with
the expected bl_idname / bl_label and is wired into the CLASSES tuple
that register() iterates over. Run via:

    uv run python animation/addons/test_reload_operator.py
"""

import importlib.util
import os
import sys
import types

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(REPO_ROOT, "animation/addons/fh_clip_panel.py")


def _stub_bpy() -> None:
    """Same minimal bpy shim as test_servo_parity._stub_bpy()."""
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


def main() -> int:
    _stub_bpy()
    spec = importlib.util.spec_from_file_location("fh_clip_panel", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    fails: list[str] = []

    def check(name: str, cond: bool, detail: object = "") -> None:
        print(
            ("PASS " if cond else "FAIL ") + name + (f" :: {detail}" if detail else "")
        )
        if not cond:
            fails.append(name)

    op = getattr(mod, "FH_OT_reload_panel", None)
    check("FH_OT_reload_panel class exists on module", op is not None)
    if op is not None:
        check(
            "FH_OT_reload_panel.bl_idname == 'fh.reload_panel'",
            getattr(op, "bl_idname", None) == "fh.reload_panel",
            getattr(op, "bl_idname", None),
        )
        check(
            "FH_OT_reload_panel.bl_label == 'Reload Add-on'",
            getattr(op, "bl_label", None) == "Reload Add-on",
            getattr(op, "bl_label", None),
        )
        check(
            "FH_OT_reload_panel is in CLASSES registration tuple",
            op in mod.CLASSES,
        )

    print("RESULT " + ("GREEN" if not fails else "RED: " + ", ".join(fails)))
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
