"""
facehugger — single-entry CLI wrapping the simulation pipeline.

Subcommands:
  urdf      regenerate generated/facehugger.urdf from generated/fusion_export.json
  sim       run simulate.py (default: stand; --walk / --trot for gaits)
  view      open generated/facehugger.urdf in PyBullet's viewer (no physics)
  blender   import the URDF into Blender (placement-only, no rig by default;
            --rigged builds an armature with IK + foot-target Empties for
            animation work)
  all       urdf → sim (smoke shortcut)

Examples:
  python facehugger.py urdf
  python facehugger.py sim --walk
  python facehugger.py view
  python facehugger.py blender                                # default 5.1, placement-only
  python facehugger.py blender --rigged                       # armature + IK rig
  python facehugger.py blender --blender-version 5.2          # specific version
  python facehugger.py blender --headless --save /tmp/scene.blend
  python facehugger.py all --headless

Env:
  BLENDER_BIN  full path to a Blender executable. Wins over --blender-version
               and the /Applications search.
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
SIMULATE = HERE / "simulate.py"
VIEW_URDF = HERE / "view_urdf.py"
VISUALIZE = REPO_ROOT / "animation" / "scripts" / "visualize_urdf.py"
VISUALIZE_RIGGED = REPO_ROOT / "animation" / "scripts" / "urdf_to_blender_rigged.py"

BLENDER_DEFAULT_VERSION = "5.1"


def _darwin_app_candidates(version):
    """The /Applications layouts we know about, in priority order.
    Different Blender releases use different naming conventions:
      - 3.x LTS:    Blender-3.3-LTS.app
      - 4.x / 5.x:  "Blender 5.1.app" (with a space) or "Blender-5.1.app"
      - generic:    Blender.app
    """
    return [
        f"/Applications/Blender-{version}-LTS.app/Contents/MacOS/Blender",
        f"/Applications/Blender {version}.app/Contents/MacOS/Blender",
        f"/Applications/Blender-{version}.app/Contents/MacOS/Blender",
        f"/Applications/Blender{version}.app/Contents/MacOS/Blender",
    ]


def _resolve_blender_bin(version):
    """Return a path to a Blender executable for `version`.

    Search order (first hit wins):
      1. $BLENDER_BIN — full override; must exist or we fail loud
      2. macOS /Applications candidates for the requested version
      3. `blender{version}` on $PATH (e.g. `blender3.3`)
      4. plain `blender` on $PATH
      5. exit with the list of paths we tried
    """
    env = os.environ.get("BLENDER_BIN")
    if env:
        if Path(env).exists():
            return env
        sys.exit(f"BLENDER_BIN={env!r} does not exist")

    tried = []
    if sys.platform == "darwin":
        for candidate in _darwin_app_candidates(version):
            tried.append(candidate)
            if Path(candidate).exists():
                return candidate

    versioned = f"blender{version}"
    found = shutil.which(versioned)
    if found:
        return found
    tried.append(f"$PATH/{versioned}")

    found = shutil.which("blender")
    if found:
        return found
    tried.append("$PATH/blender")

    sys.exit(
        f"Could not locate Blender {version}. Tried:\n  "
        + "\n  ".join(tried)
        + "\nSet BLENDER_BIN to override, or pick a different --blender-version."
    )


def _run(cmd, cwd=HERE):
    """Forward stdout/stderr; return the child's exit code."""
    print(f"$ {' '.join(str(c) for c in cmd)}")
    return subprocess.run([str(c) for c in cmd], cwd=str(cwd)).returncode


def cmd_urdf(args):
    cli = [sys.executable, GENERATE_URDF]
    if args.export:
        cli += ["--export", args.export]
    if args.config:
        cli += ["--config", args.config]
    if args.out:
        cli += ["--out", args.out]
    return _run(cli)


def cmd_sim(args):
    cli = [sys.executable, SIMULATE]
    if args.walk:
        cli.append("--walk")
    if args.trot:
        cli.append("--trot")
    if args.headless:
        cli.append("--headless")
    if args.settle is not None:
        cli += ["--settle", str(args.settle)]
    return _run(cli)


def cmd_view(_args):
    return _run([sys.executable, VIEW_URDF])


def cmd_blender(args):
    blender = _resolve_blender_bin(args.blender_version)
    cli = [blender]
    if args.headless:
        cli.append("--background")
    script = VISUALIZE_RIGGED if args.rigged else VISUALIZE
    cli += ["--python", str(script)]
    extra = []
    if args.save:
        extra += ["--save", args.save]
    if extra:
        cli += ["--", *extra]
    return _run(cli)


def cmd_all(args):
    rc = cmd_urdf(args)
    if rc != 0:
        return rc
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
    pu.add_argument("--out", help="output URDF path")
    pu.set_defaults(func=cmd_urdf)

    ps = sub.add_parser("sim", help="run the PyBullet simulator")
    ps.add_argument("--walk", action="store_true")
    ps.add_argument("--trot", action="store_true")
    ps.add_argument("--headless", action="store_true")
    ps.add_argument("--settle", type=float, default=None)
    ps.set_defaults(func=cmd_sim)

    pv = sub.add_parser("view", help="open URDF in PyBullet viewer")
    pv.set_defaults(func=cmd_view)

    pb = sub.add_parser("blender", help="open the URDF in Blender")
    pb.add_argument(
        "--blender-version",
        default=BLENDER_DEFAULT_VERSION,
        metavar="VERSION",
        help="Blender major.minor version to launch (e.g. 5.1, "
        "5.2). Default %(default)s. The script requires "
        "Blender 5.0+; older versions will be rejected on "
        "startup. Override with BLENDER_BIN env var if your "
        "install path doesn't match the /Applications "
        "conventions.",
    )
    pb.add_argument(
        "--headless", action="store_true", help="run Blender in --background mode"
    )
    pb.add_argument("--save", help="save the built scene to this .blend path")
    pb.add_argument(
        "--rigged",
        action="store_true",
        help="build a posable armature with IK + foot-target Empties "
        "(animation rig). Default is placement-only via "
        "visualize_urdf.py — useful for cross-checking the URDF rest "
        "pose against PyBullet but not animateable.",
    )
    pb.set_defaults(func=cmd_blender)

    pa = sub.add_parser("all", help="urdf → sim (smoke run)")
    pa.add_argument("--export")
    pa.add_argument("--config")
    pa.add_argument("--out")
    pa.add_argument("--walk", action="store_true")
    pa.add_argument("--trot", action="store_true")
    pa.add_argument("--headless", action="store_true")
    pa.add_argument("--settle", type=float, default=None)
    pa.set_defaults(func=cmd_all)

    args = p.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
