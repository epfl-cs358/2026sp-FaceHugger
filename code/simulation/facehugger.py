"""
facehugger — single-entry CLI wrapping the simulation pipeline.

Subcommands:
  urdf      regenerate generated/facehugger.urdf from generated/fusion_export.json
  sim       run simulate.py (default: stand; --walk / --trot for gaits)
  view      open generated/facehugger.urdf in PyBullet's viewer (no physics)
  blender   open the Fusion-export visualizer in Blender
  all       urdf → sim (smoke shortcut)

Examples:
  python facehugger.py urdf
  python facehugger.py sim --walk
  python facehugger.py view
  python facehugger.py blender
  python facehugger.py blender --headless --save /tmp/scene.blend
  python facehugger.py all --headless

Env:
  BLENDER_BIN  path to the Blender executable (default: macOS Blender 3.3 LTS).
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent

GENERATE_URDF = HERE / "generate_urdf.py"
SIMULATE      = HERE / "simulate.py"
VIEW_URDF     = HERE / "view_urdf.py"
VISUALIZE     = REPO_ROOT / "animation" / "scripts" / "visualize_fusion_export.py"

BLENDER_DEFAULT_DARWIN = "/Applications/Blender-3.3-LTS.app/Contents/MacOS/Blender"


def _resolve_blender_bin():
    env = os.environ.get("BLENDER_BIN")
    if env:
        if Path(env).exists():
            return env
        sys.exit(f"BLENDER_BIN={env} does not exist")
    if sys.platform == "darwin" and Path(BLENDER_DEFAULT_DARWIN).exists():
        return BLENDER_DEFAULT_DARWIN
    found = shutil.which("blender")
    if found:
        return found
    sys.exit("Could not locate Blender. Set BLENDER_BIN or install Blender.")


def _run(cmd, cwd=HERE):
    """Forward stdout/stderr; return the child's exit code."""
    print(f"$ {' '.join(str(c) for c in cmd)}")
    return subprocess.run([str(c) for c in cmd], cwd=str(cwd)).returncode


def cmd_urdf(args):
    cli = [sys.executable, GENERATE_URDF]
    if args.export:  cli += ["--export", args.export]
    if args.config:  cli += ["--config", args.config]
    if args.out:     cli += ["--out", args.out]
    return _run(cli)


def cmd_sim(args):
    cli = [sys.executable, SIMULATE]
    if args.walk:     cli.append("--walk")
    if args.trot:     cli.append("--trot")
    if args.headless: cli.append("--headless")
    if args.settle is not None:
        cli += ["--settle", str(args.settle)]
    return _run(cli)


def cmd_view(_args):
    return _run([sys.executable, VIEW_URDF])


def cmd_blender(args):
    blender = _resolve_blender_bin()
    cli = [blender]
    if args.headless:
        cli.append("--background")
    cli += ["--python", str(VISUALIZE)]
    extra = []
    if args.save:
        extra += ["--save", args.save]
    if extra:
        cli += ["--", *extra]
    return _run(cli)


def cmd_all(args):
    rc = cmd_urdf(args)
    if rc != 0: return rc
    return cmd_sim(args)


def main():
    p = argparse.ArgumentParser(
        prog="facehugger",
        description=__doc__.splitlines()[1],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    pu = sub.add_parser("urdf", help="regenerate the URDF")
    pu.add_argument("--export", help="path to fusion_export.json")
    pu.add_argument("--config", help="path to facehugger_config.yaml")
    pu.add_argument("--out",    help="output URDF path")
    pu.set_defaults(func=cmd_urdf)

    ps = sub.add_parser("sim", help="run the PyBullet simulator")
    ps.add_argument("--walk", action="store_true")
    ps.add_argument("--trot", action="store_true")
    ps.add_argument("--headless", action="store_true")
    ps.add_argument("--settle", type=float, default=None)
    ps.set_defaults(func=cmd_sim)

    pv = sub.add_parser("view", help="open URDF in PyBullet viewer")
    pv.set_defaults(func=cmd_view)

    pb = sub.add_parser("blender", help="open the export in Blender")
    pb.add_argument("--headless", action="store_true",
                    help="run Blender in --background mode")
    pb.add_argument("--save", help="save the built scene to this .blend path")
    pb.set_defaults(func=cmd_blender)

    pa = sub.add_parser("all", help="urdf → sim (smoke run)")
    pa.add_argument("--export"); pa.add_argument("--config"); pa.add_argument("--out")
    pa.add_argument("--walk", action="store_true")
    pa.add_argument("--trot", action="store_true")
    pa.add_argument("--headless", action="store_true")
    pa.add_argument("--settle", type=float, default=None)
    pa.set_defaults(func=cmd_all)

    args = p.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
