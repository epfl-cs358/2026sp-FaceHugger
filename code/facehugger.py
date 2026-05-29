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
import signal
import subprocess
import sys
import time
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


# Where to point people when a dependency is missing. Descriptive, not a URL:
# the wiki is hosted but its address is not pinned yet.
TOOLCHAIN_DOCS = "the toolchain setup guide (wiki: guide/toolchain/)"
SIM_INSTALL = (
    "cd code/simulation && conda env create -f environment.yml "
    "&& conda activate facehugger"
)


def _missing(mod):
    import importlib.util

    return importlib.util.find_spec(mod) is None


def _die_missing(what, intro, fix, extra=None):
    """Exit with a clear 'X not found. <fix>. Docs: ...' message."""
    lines = [
        "",
        f"✗ {what} not found. {intro}",
        f"  {fix}",
        f"  Docs: {TOOLCHAIN_DOCS}",
    ]
    if extra:
        lines.append(f"  {extra}")
    sys.exit("\n".join(lines))


def _require_sim_deps(need_build):
    """Exit with an actionable message if the sim's Python deps are missing.
    `need_build` is True when the firmware SIL is compiled (needs pybind11)."""
    missing = ["pybullet"] if _missing("pybullet") else []
    if need_build and _missing("pybind11"):
        missing.append("pybind11")
    if not missing:
        return
    extra = None
    if "pybind11" in missing and "pybullet" not in missing:
        extra = "Or skip the C++ toolchain: rerun with --python"
    _die_missing(
        ", ".join(missing), "Install the sim dependencies:", SIM_INSTALL, extra
    )


def cmd_sim(args):
    if getattr(args, "list_clips", False):
        _list_clips()
        return 0
    serving = bool(
        getattr(args, "serve", False)
        or getattr(args, "app", False)
        or getattr(args, "panel", False)
    )
    # Default path (non --python) and any serve build the firmware SIL.
    _require_sim_deps(need_build=serving or not getattr(args, "python_port", False))
    if serving:
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


def _signal_group(proc, sig):
    """Send `sig` to the child's whole process group, so child trees die too
    (e.g. Expo's Metro/node, not just the `npx` launcher). Children are started
    with start_new_session=True, putting each in its own group. Falls back to
    signalling the single process on non-POSIX or if the group is already gone."""
    if proc.poll() is not None:
        return
    if os.name == "posix":
        try:
            os.killpg(os.getpgid(proc.pid), sig)
            return
        except (ProcessLookupError, PermissionError):
            return
    try:
        proc.kill() if sig == signal.SIGKILL else proc.terminate()
    except ProcessLookupError:
        pass


def _shutdown_children(procs):
    """Tear down every (label, Popen) still running: SIGINT the whole group for a
    graceful stop, wait up to 5s total, then SIGKILL any survivor. Used so one
    Ctrl-C on `sim --app`/`--serve`/`--panel` takes the sim AND the app down."""
    alive = [(label, p) for label, p in procs if p.poll() is None]
    for label, p in alive:
        print(f"  stopping {label}…")
        _signal_group(p, signal.SIGINT)
    deadline = time.time() + 5
    for _label, p in alive:
        try:
            p.wait(timeout=max(0.0, deadline - time.time()))
        except subprocess.TimeoutExpired:
            pass
    for label, p in alive:
        if p.poll() is None:
            print(f"  force-killing {label}…")
            _signal_group(p, signal.SIGKILL)


def _ensure_app_deps(app_dir):
    """Make sure the Expo app has its npm deps installed before we spawn it.

    Without this, `sim --app` / `app` would call `npx expo start` against a
    bare checkout, which fails silently inside the spawned process while the
    sim keeps printing "Web app: …" — confusing on a fresh clone. Idempotent:
    if `node_modules/.package-lock.json` is newer than `package.json` we
    short-circuit; otherwise run `npm ci` (or `npm install` if no lock file).
    Streams stdout/stderr so the user sees install progress + any errors."""
    pkg = app_dir / "package.json"
    if not pkg.is_file():
        sys.exit(f"app package.json not found: {pkg}")
    if shutil.which("npm") is None:
        _die_missing(
            "Node / npm", "Install Node.js 18+ (the app uses Expo):", "nodejs.org"
        )
    nm = app_dir / "node_modules"
    nm_stamp = nm / ".package-lock.json"
    fresh = (
        nm.is_dir()
        and nm_stamp.is_file()
        and nm_stamp.stat().st_mtime >= pkg.stat().st_mtime
    )
    if fresh:
        return
    has_lock = (app_dir / "package-lock.json").is_file()
    cmd = ["npm", "ci"] if has_lock else ["npm", "install"]
    print(f"Installing app deps ({' '.join(cmd)} in {app_dir.name})…")
    rc = subprocess.call(cmd, cwd=str(app_dir))
    if rc != 0:
        sys.exit(
            f"app dep install failed (exit {rc}); run `{' '.join(cmd)}` in {app_dir} manually"
        )


def _serve_session(*, host, port, gui, app, app_port, panel=False):
    """Run the firmware-backed WebSocket API (ws_sim), optionally launching the web
    app alongside it and hosting the browser control panel. Backs `sim --serve` /
    `sim --app` / `sim --panel` and the legacy `serve`.

    Both children run in their own process groups (start_new_session=True) so the
    terminal's Ctrl-C doesn't race them directly; the parent catches it once and
    tears both down via _shutdown_children, killing whole trees (Expo's Metro/node
    included)."""
    procs = []  # (label, Popen) — torn down newest-first on exit

    app_proc = None
    if app:
        app_dir = REPO_ROOT / "code" / "remote-control-app" / "MyApp"
        if not app_dir.is_dir():
            sys.exit(f"app dir not found: {app_dir}")
        if shutil.which("npx") is None:
            _die_missing(
                "Node / npx", "Install Node.js 18+ (the app uses Expo):", "nodejs.org"
            )
        _ensure_app_deps(app_dir)
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
            start_new_session=True,
        )
        procs.append(("web app", app_proc))
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
    # ws_sim imports the firmware_sil package, so run with SIM_DIR on PYTHONPATH
    # (mirrors _run(cwd=None)) but in the user's working dir.
    sim_env = {**os.environ}
    sim_env["PYTHONPATH"] = os.pathsep.join(
        [str(SIM_DIR), sim_env.get("PYTHONPATH", "")]
    ).rstrip(os.pathsep)
    print(f"$ {' '.join(str(c) for c in cli)}")
    sim_proc = subprocess.Popen(
        [str(c) for c in cli], env=sim_env, start_new_session=True
    )
    procs.append(("sim", sim_proc))

    rc = 0
    try:
        rc = sim_proc.wait()
    except KeyboardInterrupt:
        print("\nShutting down (Ctrl-C)…")
    finally:
        _shutdown_children(procs)
    return rc


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
    if shutil.which("pio") is None:
        _die_missing(
            "PlatformIO (pio)",
            "Install it:",
            "pip install platformio  (or the VS Code PlatformIO extension)",
        )
    target = [] if args.build_only else ["-t", "upload"]
    rc = _run(["pio", "run", "-e", args.env, *target], cwd=fw_dir)
    if rc != 0 or not args.monitor:
        return rc
    return _run(["pio", "device", "monitor", "-e", args.env], cwd=fw_dir)


def cmd_app(args):
    app_dir = REPO_ROOT / "code" / "remote-control-app" / "MyApp"
    if not app_dir.is_dir():
        sys.exit(f"app dir not found: {app_dir}")
    if shutil.which("npx") is None:
        _die_missing(
            "Node / npx", "Install Node.js 18+ (the app uses Expo):", "nodejs.org"
        )
    _ensure_app_deps(app_dir)
    url = f"http://localhost:{args.port}"
    print(f"Serving the remote-control app (Expo web) at {url}")
    print("Point it at the robot or the sim from the app's Settings screen.")
    print("(Ctrl-C to stop.)\n")
    # Own process group + managed teardown so Ctrl-C takes the whole Expo tree
    # (Metro/node), not just the npx launcher — same as `sim --app`.
    print(f"$ npx expo start --web --port {args.port}")
    proc = subprocess.Popen(
        ["npx", "expo", "start", "--web", "--port", str(args.port)],
        cwd=str(app_dir),
        start_new_session=True,
    )
    try:
        rc = proc.wait()
    except KeyboardInterrupt:
        print("\nShutting down (Ctrl-C)…")
        rc = 0
    finally:
        _shutdown_children([("web app", proc)])
    return rc


def main():
    p = argparse.ArgumentParser(
        prog="facehugger",
        description=__doc__.splitlines()[1],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    # metavar omits the deprecated `serve` alias from the listing (it still works).
    sub = p.add_subparsers(
        dest="cmd", required=True, metavar="{urdf,sim,blender,flash,app}"
    )

    pu = sub.add_parser("urdf", help="regenerate the URDF")
    pu.add_argument("--export", help="path to fusion_export.json")
    pu.add_argument("--config", help="path to facehugger_config.yaml")
    pu.add_argument("--out", help="output URDF path")
    pu.set_defaults(func=cmd_urdf)

    ps = sub.add_parser(
        "sim", help="run the robot in software (gaits/clips; --serve/--app to drive it)"
    )
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
    ps.add_argument(
        "--walk",
        action="store_true",
        help="run the walk gait (firmware tickGait via the SIL)",
    )
    ps.add_argument(
        "--trot",
        action="store_true",
        help="run the trot gait (firmware tickTrot via the SIL)",
    )
    ps.add_argument(
        "--headless",
        action="store_true",
        help="no PyBullet GUI window (CI smoke check)",
    )
    ps.add_argument(
        "--settle",
        type=float,
        default=None,
        metavar="SECONDS",
        help="seconds to hold the stance before the run starts (lets gravity settle)",
    )
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

    # Deprecated alias for `sim --serve`. Omitting help= leaves it out of the
    # listing (and the metavar above drops it from the choices line); it still
    # works if typed.
    pserve = sub.add_parser("serve")
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
        help="web host port (default 8080; the sim's --serve uses 8081)",
    )
    papp.set_defaults(func=cmd_app)

    args = p.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
