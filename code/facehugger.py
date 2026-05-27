"""
facehugger — single-entry CLI wrapping the simulation pipeline.

Subcommands:
  urdf      regenerate generated/facehugger.urdf from generated/fusion_export.json
  sim       run the robot in software (firmware-in-loop PyBullet). Default: stand.
            Drive it from the CLI (--walk / --trot / --clip NAME), or from an
            external client over the T: WebSocket API (--serve; --app also launches
            the web app). --list-clips prints the available clips and exits.
  blender   import the URDF into Blender (placement-only, no rig by default;
            --rigged builds an armature with IK + foot-target Empties)
  app       serve the remote-control app on the web (Expo)
  flash     build and upload the firmware (PlatformIO)
  serve     deprecated alias for `sim --serve`

Examples:
  python facehugger.py urdf
  python facehugger.py sim --walk
  python facehugger.py sim --app                              # sim + WebSocket API + app
  python facehugger.py sim --list-clips
  python facehugger.py flash                                  # build + upload firmware
  python facehugger.py app --port 8080
  python facehugger.py blender --rigged                       # armature + IK rig
  python facehugger.py blender --headless --save /tmp/scene.blend

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

# This file lives at code/facehugger.py. The simulation packages it shells into
# (pybullet_sim / urdf_gen / firmware_sil) live in code/simulation/, so module
# invocations run with cwd=SIM_DIR. Paths resolve from __file__, so the CLI works
# from any working directory (e.g. `python code/facehugger.py sim`).
REPO_ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = REPO_ROOT / "code" / "simulation"

# Runtime + build steps are packages; invoked as `python -m <pkg>.<mod>` with
# cwd=SIM_DIR (see _run) so pybullet_sim / urdf_gen / firmware_sil are importable.
GENERATE_URDF = ["-m", "urdf_gen.generate_urdf"]
SIMULATE = ["-m", "pybullet_sim.simulate"]
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


def _run(cmd, cwd=SIM_DIR):
    """Forward stdout/stderr; return the child's exit code.

    cwd=None runs in the caller's working directory (with SIM_DIR added to
    PYTHONPATH so the `-m pybullet_sim...` / `-m urdf_gen...` packages still
    import). Used by `sim` so its --log output (sim_log.csv/png) lands where
    the user invoked the command, not in code/simulation/.
    """
    print(f"$ {' '.join(str(c) for c in cmd)}")
    env = None
    if cwd is None:
        env = {**os.environ}
        env["PYTHONPATH"] = os.pathsep.join(
            [str(SIM_DIR), env.get("PYTHONPATH", "")]
        ).rstrip(os.pathsep)
    return subprocess.run(
        [str(c) for c in cmd], cwd=(str(cwd) if cwd else None), env=env
    ).returncode


def cmd_urdf(args):
    cli = [sys.executable, *GENERATE_URDF]
    if args.export:
        cli += ["--export", args.export]
    if args.config:
        cli += ["--config", args.config]
    if args.out:
        cli += ["--out", args.out]
    return _run(cli)


def _list_clips():
    """Print the clip names + ids the sim/robot will run, parsed from the
    firmware clips_all.h FH_CLIPS[] table (no C++ toolchain needed)."""
    import re

    header = REPO_ROOT / "code" / "firmware" / "src" / "nervous_system" / "clips_all.h"
    if not header.is_file():
        sys.exit(f"clips_all.h not found: {header}")
    text = header.read_text()
    m = re.search(r"FH_CLIPS\[[^\]]*\]\s*=\s*\{(.*?)\};", text, re.S)
    entries = (
        re.findall(r'\{\s*"([^"]+)"\s*,\s*\w+\s*,\s*(\d+)\s*,\s*(\d+)\s*\}', m.group(1))
        if m
        else []
    )
    print(f"{len(entries)} clip(s) in {header.relative_to(REPO_ROOT)}:")
    for i, (name, frames, dur) in enumerate(entries):
        print(f"  {i:2d}  {name}  ({frames} frames, {dur} ms)")


def cmd_sim(args):
    if getattr(args, "list_clips", False):
        _list_clips()
        return 0
    if (
        getattr(args, "serve", False)
        or getattr(args, "app", False)
        or getattr(args, "panel", False)
    ):
        # Drive the sim from an external client (app / panel) over the T: WebSocket
        # API instead of from CLI flags. GUI on by default; --headless turns it off.
        return _serve_session(
            host=args.host,
            port=args.port,
            gui=not args.headless,
            app=args.app,
            app_port=args.app_port,
            panel=args.panel,
        )
    cli = [sys.executable, *SIMULATE]
    if args.clip:
        cli += ["--clip", args.clip]
    if args.loop:
        cli.append("--loop")
    if args.float_mode:
        cli.append("--float")
    if args.monitor:
        cli.append("--monitor")
    if args.log:
        cli.append("--log")
    if args.python_port:
        cli.append("--python")
    if args.walk:
        cli.append("--walk")
    if args.trot:
        cli.append("--trot")
    if args.headless:
        cli.append("--headless")
    if args.settle is not None:
        cli += ["--settle", str(args.settle)]
    # cwd=None: run in the user's directory so --log artifacts land there.
    return _run(cli, cwd=None)


def _urdf_stale():
    """True if generated/facehugger.urdf is missing or older than its inputs."""
    urdf = SIM_DIR / "generated" / "facehugger.urdf"
    if not urdf.exists():
        return True
    inputs = [
        SIM_DIR / "generated" / "fusion_export.json",
        SIM_DIR / "facehugger_config.yaml",
    ]
    u = urdf.stat().st_mtime
    return any(p.exists() and p.stat().st_mtime > u for p in inputs)


def cmd_blender(args):
    # Refresh a stale URDF first so the rig is never built against an old one.
    if not getattr(args, "skip_urdf_check", False) and _urdf_stale():
        print("URDF is missing or older than its inputs; regenerating it first.")
        rc = _run([sys.executable, *GENERATE_URDF])
        if rc != 0:
            return rc

    blender = _resolve_blender_bin(args.blender_version)

    # `fh_rigged_latest.blend` is the animation LIBRARY — it holds the
    # rig + all authored clips, and only the rigged builder
    # (`urdf_to_blender_rigged.py`) has the stash/restore logic that
    # preserves those clips across a rebuild. Placement-only
    # (`visualize_urdf.py`) clears the scene with no stash/restore, so
    # it must NEVER default-save over the library — doing so would
    # silently destroy every clip. So:
    #   --save PATH        → save there (either mode, explicit intent)
    #   --rigged, no --save → default to the library
    #   placement-only, no --save → don't persist at all (GUI view only)
    if args.save:
        save_path = Path(args.save)
    elif args.rigged:
        save_path = REPO_ROOT / "animation" / "fh_rigged_latest.blend"
    else:
        save_path = None

    cli = [blender]
    # Only reopen the existing library in rigged mode, where stash/restore
    # protects the clips. Placement-only always starts from a blank scene, and
    # --rebuild-rig forces a from-scratch rig build (no reopen, so clips are lost).
    reopen = (
        args.rigged
        and not args.reset
        and not getattr(args, "rebuild_rig", False)
        and save_path is not None
        and save_path.exists()
    )
    if reopen:
        cli.append(str(save_path))
    if args.headless:
        cli.append("--background")
    script = VISUALIZE_RIGGED if args.rigged else VISUALIZE
    cli += ["--python", str(script)]
    if save_path is not None:
        cli += ["--", "--save", str(save_path)]
    return _run(cli)


def _serve_session(*, host, port, gui, app, app_port, panel=False):
    """Run the firmware-backed WebSocket API (ws_sim), optionally launching the web
    app alongside it and hosting the browser control panel. Backs `sim --serve` /
    `sim --app` / `sim --panel` and the legacy `serve`."""
    app_proc = None
    if app:
        app_dir = REPO_ROOT / "code" / "remote-control-app" / "MyApp"
        if not app_dir.is_dir():
            sys.exit(f"app dir not found: {app_dir}")
        print(f"Web app:   http://localhost:{app_port}  (auto-connects to the sim)")
        # Point the launched web app at the sim by default (config.ts reads these
        # EXPO_PUBLIC_ vars as its startup target). The Settings screen can still
        # switch. localhost is correct for the web app on this machine; from a
        # phone, use the Settings screen with the dev machine's LAN IP.
        app_env = {
            **os.environ,
            "EXPO_PUBLIC_WS_IP": "localhost",
            "EXPO_PUBLIC_WS_PORT": str(port),
        }
        app_proc = subprocess.Popen(
            ["npx", "expo", "start", "--web", "--port", str(app_port)],
            cwd=str(app_dir),
            env=app_env,
        )
    print(f"WebSocket: ws://{host}:{port}   (telemetry SSE on :8082)")
    if panel:
        print(f"Panel:     http://{host}:8082/panel")
    cli = [
        sys.executable,
        "-m",
        "firmware_sil.ws_sim",
        "--host",
        host,
        "--port",
        str(port),
    ]
    if gui:
        cli.append("--gui")
    if panel:
        cli.append("--panel")
    # cwd=None: run in the user's dir (SIM_DIR on PYTHONPATH) so firmware_sil imports.
    try:
        return _run(cli, cwd=None)
    finally:
        if app_proc is not None:
            app_proc.terminate()
            try:
                app_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                app_proc.kill()


def cmd_serve(args):
    # Deprecated alias: the WebSocket API is now `sim --serve` (and `sim --app`).
    print("note: `serve` is now `sim --serve` / `sim --app`; this alias still works.")
    return _serve_session(
        host=args.host, port=args.port, gui=args.gui, app=False, app_port=8080
    )


def cmd_flash(args):
    fw_dir = REPO_ROOT / "code" / "firmware"
    if not fw_dir.is_dir():
        sys.exit(f"firmware dir not found: {fw_dir}")
    target = [] if args.build_only else ["-t", "upload"]
    rc = _run(["pio", "run", "-e", args.env, *target], cwd=fw_dir)
    if rc != 0 or not args.monitor:
        return rc
    return _run(["pio", "device", "monitor", "-e", args.env], cwd=fw_dir)


def cmd_app(args):
    app_dir = REPO_ROOT / "code" / "remote-control-app" / "MyApp"
    if not app_dir.is_dir():
        sys.exit(f"app dir not found: {app_dir}")
    url = f"http://localhost:{args.port}"
    print(f"Serving the remote-control app (Expo web) at {url}")
    print("Point it at the robot or the sim from the app's Settings screen.")
    print("(Ctrl-C to stop.)\n")
    # cwd in MyApp so expo finds package.json; works from any caller cwd.
    return _run(
        ["npx", "expo", "start", "--web", "--port", str(args.port)], cwd=app_dir
    )


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
    ps.add_argument("--clip", metavar="NAME", help="play a named animation clip")
    ps.add_argument(
        "--loop",
        action="store_true",
        help="replay the clip continuously (GUI only) to observe over time",
    )
    ps.add_argument(
        "--float",
        dest="float_mode",
        action="store_true",
        help="no gravity/floor, body pinned — watch joint geometry only",
    )
    ps.add_argument(
        "--monitor",
        action="store_true",
        help="print torque + estimated-current status (peak τ, total A, stalls)",
    )
    ps.add_argument(
        "--log",
        action="store_true",
        help="record per-step torque/current → summary + sim_log.csv + sim_log.png",
    )
    ps.add_argument(
        "--python",
        dest="python_port",
        action="store_true",
        help="drive clips AND gaits with the Python re-port instead of the default "
        "exact compiled firmware (firmware_sil); use when you have no C++ toolchain",
    )
    ps.add_argument("--walk", action="store_true")
    ps.add_argument("--trot", action="store_true")
    ps.add_argument("--headless", action="store_true")
    ps.add_argument("--settle", type=float, default=None)
    ps.add_argument(
        "--list-clips",
        dest="list_clips",
        action="store_true",
        help="list the available clip names and ids (from clips_all.h) and exit, "
        "instead of running the simulator",
    )
    # Interface: drive the sim from an external client instead of CLI flags.
    ps.add_argument(
        "--serve",
        action="store_true",
        help="expose the T: WebSocket API (drive from the app / panel) instead of "
        "driving from the CLI",
    )
    ps.add_argument(
        "--app",
        action="store_true",
        help="imply --serve and also launch the web app (full interactive session)",
    )
    ps.add_argument(
        "--host",
        default="localhost",
        help="WebSocket bind host for --serve (use 0.0.0.0 to reach it from a phone)",
    )
    ps.add_argument(
        "--port",
        type=int,
        default=8081,
        help="WebSocket port for --serve (default 8081; the robot uses 81)",
    )
    ps.add_argument(
        "--app-port",
        type=int,
        default=8080,
        dest="app_port",
        help="web host port for --app (default 8080)",
    )
    ps.add_argument(
        "--panel",
        action="store_true",
        help="imply --serve and host the browser control panel over HTTP "
        "(at http://localhost:8082/panel)",
    )
    ps.set_defaults(func=cmd_sim)

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
    pb.add_argument(
        "--reset",
        action="store_true",
        help="start from a blank scene, discarding any existing animations",
    )
    pb.add_argument(
        "--rebuild-rig",
        dest="rebuild_rig",
        action="store_true",
        help="rebuild the rig from the URDF even if fh_rigged_latest.blend exists "
        "(does not reopen it, so its clips are discarded)",
    )
    pb.add_argument(
        "--skip-urdf-check",
        dest="skip_urdf_check",
        action="store_true",
        help="do not regenerate a stale URDF before opening Blender",
    )
    pb.set_defaults(func=cmd_blender)

    pflash = sub.add_parser("flash", help="build and upload the firmware (PlatformIO)")
    pflash.add_argument(
        "--build-only",
        dest="build_only",
        action="store_true",
        help="compile only; do not upload to the board",
    )
    pflash.add_argument(
        "--monitor",
        action="store_true",
        help="open the serial monitor after a successful upload",
    )
    pflash.add_argument(
        "--env",
        default="upesy_wroom",
        help="PlatformIO environment (default upesy_wroom)",
    )
    pflash.set_defaults(func=cmd_flash)

    pserve = sub.add_parser(
        "serve",
        help="deprecated alias for `sim --serve` (firmware-backed WebSocket API)",
    )
    pserve.add_argument("--host", default="localhost")
    pserve.add_argument(
        "--port", type=int, default=8081, help="default 8081 (81 is privileged)"
    )
    pserve.add_argument("--gui", action="store_true", help="show the PyBullet window")
    pserve.set_defaults(func=cmd_serve)

    papp = sub.add_parser(
        "app", help="serve the remote-control app on the web (Expo / npm)"
    )
    papp.add_argument(
        "--port",
        type=int,
        default=8080,
        help="web host port (default 8080; the sim's serve uses 8081)",
    )
    papp.set_defaults(func=cmd_app)

    args = p.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
