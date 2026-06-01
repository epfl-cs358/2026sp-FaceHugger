"""Freeze the per-folder CLI surfaces.

`code/facehugger.py` is the single global entrypoint. It dispatches to per-folder
`-m` entrypoints (`urdf_gen.generate_urdf`, `pybullet_sim.simulate`,
`firmware_sil.ws_sim`). The simulation-reorg refactor must not drift any of
those `--help` outputs — this test pins them via fixture snapshots.

If the CLI is *intentionally* changed, regenerate the matching fixture under
`tests/fixtures/cli_help/` and review the diff.

`firmware_sil.build` and `firmware_sil.gen_references` are intentionally
excluded: `build` is a library called by other modules and has no argparse;
`gen_references` runs immediately on import without `--help` support. Both
are invoked by `facehugger.py` without flags.
"""

import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("pybullet")

SIM_DIR = Path(__file__).resolve().parent.parent  # tests/ -> code/simulation/
REPO_ROOT = SIM_DIR.parent.parent
FIXTURES = Path(__file__).resolve().parent / "fixtures" / "cli_help"

# Pin terminal width so argparse doesn't wrap absolute paths based on the local
# tty — wrapping happens *before* path normalization, so unstable widths defeat
# the substitution below.
HELP_ENV = {**os.environ, "COLUMNS": "400"}

CASES = [
    ("urdf_gen.generate_urdf", "generate_urdf.txt"),
    ("pybullet_sim.simulate", "simulate.txt"),
    ("firmware_sil.ws_sim", "ws_sim.txt"),
]


def _normalize(text: str) -> str:
    """Replace the absolute repo root with <REPO> so snapshots are portable.

    Also strips pybullet's `pybullet build time: ...` stderr-leak prefix that
    appears on stdout when `import pybullet` runs in simulate.py.
    """
    out = text.replace(str(REPO_ROOT), "<REPO>")
    out = re.sub(r"^pybullet build time:.*\n", "", out)
    return out.rstrip() + "\n"


@pytest.mark.parametrize(
    "module,fixture", CASES, ids=lambda x: x if isinstance(x, str) else ""
)
def test_cli_help_frozen(module, fixture):
    result = subprocess.run(
        [sys.executable, "-m", module, "--help"],
        cwd=str(SIM_DIR),
        capture_output=True,
        text=True,
        timeout=60,
        env=HELP_ENV,
    )
    assert result.returncode == 0, f"{module} --help failed: {result.stderr}"

    actual = _normalize(result.stdout)
    expected = (FIXTURES / fixture).read_text()
    assert actual == expected, (
        f"\n{module} --help drifted from {fixture}.\n"
        f"If intentional, regenerate the fixture and review the diff.\n"
        f"--- expected ---\n{expected}\n"
        f"--- actual ---\n{actual}"
    )
