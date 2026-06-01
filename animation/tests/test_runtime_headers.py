"""Phase 1 / Phase 2 of animation-reorg — enforce the runtime-requirement
header convention on every `.py` file under `scripts/`, `addons/`, `lib/`.

Each file must start (after optional shebang and module docstring) with
one of two canonical header forms:

    # === Blender-only ===
    ...

    # === Plain Python — no Blender required ===
    ...

Phase 1 ships this test deliberately RED — no file has the header yet.
Phase 2 adds the headers (file by file) and this test goes GREEN.

The header signals runtime requirement to readers without folder renames.
"""

import re
from pathlib import Path

import pytest

ANIMATION_DIR = Path(__file__).resolve().parent.parent
SEARCH_ROOTS = [ANIMATION_DIR / d for d in ("scripts", "addons", "lib")]

HEADER_RE = re.compile(
    r"^#\s*===\s*(Blender-only|Plain Python\s*—\s*no Blender required)\s*===",
    re.MULTILINE,
)


def _py_files():
    for root in SEARCH_ROOTS:
        for path in root.rglob("*.py"):
            if path.name == "__init__.py":
                continue
            if "__pycache__" in path.parts:
                continue
            yield path


@pytest.mark.parametrize(
    "path", list(_py_files()), ids=lambda p: str(p.relative_to(ANIMATION_DIR))
)
def test_runtime_header_present(path: Path):
    text = path.read_text()
    assert HEADER_RE.search(text), (
        f"{path.relative_to(ANIMATION_DIR)} is missing the runtime-requirement "
        "header. Add either `# === Blender-only ===` or "
        "`# === Plain Python — no Blender required ===` near the top of the "
        "file (Phase 2 of animation-reorg)."
    )
