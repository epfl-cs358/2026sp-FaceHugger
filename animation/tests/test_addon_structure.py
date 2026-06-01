"""Phase 1 of animation-reorg — pin the file layout of `animation/addons/`.

The addon is installed into Blender via Preferences; its filenames are
load-bearing for end users with the addon already enabled. This test
asserts each expected file is present and is syntactically valid Python
(AST-parseable without importing — no `bpy` required).
"""

import ast
from pathlib import Path

import pytest

ADDONS_DIR = Path(__file__).resolve().parent.parent / "addons"

EXPECTED_FILES = [
    "fh_clip_panel.py",
    "export_all_clips.py",
    "README.md",
]


@pytest.mark.parametrize("name", EXPECTED_FILES)
def test_addon_file_present(name: str):
    assert (ADDONS_DIR / name).is_file(), f"{ADDONS_DIR / name} missing"


@pytest.mark.parametrize("name", [f for f in EXPECTED_FILES if f.endswith(".py")])
def test_addon_file_parses(name: str):
    source = (ADDONS_DIR / name).read_text()
    ast.parse(source, filename=name)
