"""Phase 1 of animation-reorg — pin the file paths `code/facehugger.py`
passes to Blender.

`facehugger.py blender [--rigged]` resolves two constants, `VISUALIZE` and
`VISUALIZE_RIGGED`, to the script files under `animation/scripts/`. The
reorg explicitly does not rename or move those files; if any later phase
breaks the contract, this test goes RED before the CLI does.
"""

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
FACEHUGGER = REPO_ROOT / "code" / "facehugger.py"


def _grep_assignment(name: str) -> Path:
    """Pull a RHS like `REPO_ROOT / "animation" / "scripts" / "<file>.py"` out
    of facehugger.py without executing it. Static parsing keeps this test
    cheap and Blender-independent."""
    source = FACEHUGGER.read_text()
    match = re.search(
        rf'^{name}\s*=\s*REPO_ROOT\s*((?:/\s*"[^"]+"\s*)+)$',
        source,
        re.MULTILINE,
    )
    assert match, f"could not find `{name} = REPO_ROOT / ...` in {FACEHUGGER}"
    parts = re.findall(r'"([^"]+)"', match.group(1))
    return REPO_ROOT.joinpath(*parts)


def test_visualize_resolves_to_existing_file():
    path = _grep_assignment("VISUALIZE")
    assert path.is_file(), f"{path} does not exist"
    assert path.name == "visualize_urdf.py"


def test_visualize_rigged_resolves_to_existing_file():
    path = _grep_assignment("VISUALIZE_RIGGED")
    assert path.is_file(), f"{path} does not exist"
    assert path.name == "urdf_to_blender_rigged.py"
