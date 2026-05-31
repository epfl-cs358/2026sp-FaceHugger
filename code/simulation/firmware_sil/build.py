"""fh_sim build system: staleness check, CMake configure, auto-rebuild."""

import os
import subprocess
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_BUILD = _HERE / "build"
_FW_SRC = (_HERE / ".." / ".." / "firmware" / "src").resolve()

_SOURCE_DIRS = [_FW_SRC, _HERE / "hal"]
_SOURCE_FILES = [_HERE / "bindings.cpp", _HERE / "CMakeLists.txt"]
_SOURCE_EXTS = {".cpp", ".c", ".h", ".hpp", ".inl", ".txt"}


def _compiled_so():
    """The built module file, or None if not present."""
    hits = sorted(_BUILD.glob("fh_sim*.so")) + sorted(_BUILD.glob("fh_sim*.pyd"))
    return hits[0] if hits else None


def _newest_source_mtime():
    """mtime of the most recently changed firmware / hal / binding source."""
    newest = 0.0
    for d in _SOURCE_DIRS:
        if d.is_dir():
            for f in d.rglob("*"):
                if f.suffix in _SOURCE_EXTS and f.is_file():
                    newest = max(newest, f.stat().st_mtime)
    for f in _SOURCE_FILES:
        if f.is_file():
            newest = max(newest, f.stat().st_mtime)
    return newest


def staleness():
    """(is_stale, reason) — is the compiled .so missing or older than its sources?"""
    so = _compiled_so()
    if so is None:
        return True, "fh_sim is not built"
    src = _newest_source_mtime()
    if src > so.stat().st_mtime:
        return True, "firmware/hal/binding sources are newer than the built fh_sim"
    return False, "fh_sim is up to date"


def build(quiet=False):
    """Configure (if needed) and compile fh_sim via CMake. Raises on failure."""
    out = subprocess.DEVNULL if quiet else None
    if not (_BUILD / "CMakeCache.txt").is_file():
        subprocess.run(
            [
                "cmake",
                "-S",
                str(_HERE),
                "-B",
                str(_BUILD),
                f"-DPython_EXECUTABLE={sys.executable}",
            ],
            check=True,
            stdout=out,
            stderr=out,
        )
    subprocess.run(
        ["cmake", "--build", str(_BUILD)], check=True, stdout=out, stderr=out
    )


def load_fh_sim(auto_build=True):
    """Import the compiled fh_sim module, rebuilding it first if it is stale.

    --sil must never run a stale firmware: by default this checks the .so against
    the firmware/hal/binding sources and, if older or missing, recompiles before
    importing. Set auto_build=False (or env FH_SIL_NO_BUILD=1) to skip the rebuild
    and instead warn loudly / error if stale.
    """
    if os.environ.get("FH_SIL_NO_BUILD"):
        auto_build = False

    stale, reason = staleness()
    if stale:
        if auto_build:
            print(f"[sil] {reason} — rebuilding fh_sim...", flush=True)
            try:
                build()
            except (subprocess.CalledProcessError, FileNotFoundError) as e:
                raise ImportError(
                    f"[sil] auto-build failed ({e}). Build it manually:\n"
                    "  cd code/simulation/firmware_sil\n"
                    "  cmake -S . -B build -DPython_EXECUTABLE=$(which python)\n"
                    "  cmake --build build\n"
                    "(needs CMake >= 3.15 + a C++17 compiler + pybind11)"
                ) from e
        else:
            print(
                f"\n*** [sil] WARNING: {reason}. The .so may not reflect the current "
                "firmware. Rebuild with `cmake --build firmware_sil/build`, or unset "
                "FH_SIL_NO_BUILD to auto-rebuild. ***\n",
                file=sys.stderr,
                flush=True,
            )

    if str(_BUILD) not in sys.path:
        sys.path.insert(0, str(_BUILD))
    try:
        import fh_sim
    except ImportError as e:
        raise ImportError(
            "fh_sim (the compiled firmware) could not be imported. Build it:\n"
            "  cd code/simulation/firmware_sil\n"
            "  cmake -S . -B build -DPython_EXECUTABLE=$(which python)\n"
            "  cmake --build build\n"
            f"(underlying import error: {e})"
        ) from e
    return fh_sim
