"""
FaceHugger Quadruped Simulation

Modes: default stand, --walk, --trot.

Geometry sourcing (no duplication of constants in Python):
  - Mount XYZ, joint origins, joint axes, joint limits, link inertials
      -> parsed from facehugger.urdf via xml.etree (not pybullet.getJointInfo,
         because pybullet silently shifts link frames to COM which skews the
         parent-frame origins when inertial <origin> is non-zero).
  - Leg IDs, per-leg yaw_z, servo effort/velocity, joint limits (fallback)
      -> facehugger_config.yaml
  - Foot-tip offset in Link3 frame
      -> ASSUMPTION: lower-leg STL vertices live in the leg-assembly root
         frame (where the shoulder joint sits at origin). The foot tip is
         taken as the centroid of vertices at max +Y of leg_lower.stl. This
         falls back from a missing "FootTipPoint" construction point in
         fusion_export.json -- if one is added later, read it preferentially.
"""

import argparse
import sys

from .kinematics import build_config
from .runner import run_stand


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--walk", action="store_true")
    parser.add_argument("--trot", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--debug", action="store_true",
        help="add PyBullet debug sliders for live physics tuning (GUI only)")
    parser.add_argument(
        "--settle",
        type=float,
        default=0.5,
        help="Seconds to hold stance before the main loop "
        "begins (lets gravity resolve initial overlap). "
        "Default 0.5.",
    )
    parser.add_argument(
        "--clip",
        metavar="NAME",
        help="play a named animation clip from clips_all.h",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="replay the clip continuously (GUI only) to watch cumulative "
        "behaviour over time; physics state carries across loops",
    )
    parser.add_argument(
        "--float",
        dest="float_mode",
        action="store_true",
        help="no gravity, no floor, body pinned — watch the clip's pure joint "
        "geometry without the robot falling/slipping/collapsing",
    )
    parser.add_argument(
        "--monitor",
        action="store_true",
        help="print a torque + estimated-current status line periodically "
        "(per-leg angles, peak torque, total current [WARN >10A], [STALL] joints)",
    )
    parser.add_argument(
        "--log",
        action="store_true",
        help="record per-joint torque/current every step; on exit print a "
        "summary table and write sim_log.csv + sim_log.png (additive to --monitor)",
    )
    parser.add_argument(
        "--python-port",
        dest="python_port",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    args = parser.parse_args()

    if args.python_port:
        print(
            "error: --python-port was removed. The SIL (compiled firmware) is the "
            "only sim path. Build it with:\n"
            "  cd code/simulation/firmware_sil && cmake -S . -B build && cmake --build build",
            file=sys.stderr,
        )
        raise SystemExit(1)

    from firmware_sil import run_clip_sil as clip_fn  # noqa: N813
    from firmware_sil import run_gait_sil as gait_fn  # noqa: N813

    cfg = build_config()
    gui = not args.headless

    if args.clip:
        kwargs = dict(
            gui=gui,
            settle_s=args.settle,
            float_mode=args.float_mode,
            monitor=args.monitor,
            log=args.log,
        )
        clip_fn(cfg, args.clip, debug=args.debug, **kwargs)
    elif args.walk:
        gait_fn(
            cfg,
            "walk",
            gui=gui,
            settle_s=args.settle,
            float_mode=args.float_mode,
            monitor=args.monitor,
            log=args.log,
            debug=args.debug,
        )
    elif args.trot:
        gait_fn(
            cfg,
            "trot",
            gui=gui,
            settle_s=args.settle,
            float_mode=args.float_mode,
            monitor=args.monitor,
            log=args.log,
            debug=args.debug,
        )
    else:
        run_stand(
            cfg,
            gui=gui,
            settle_s=args.settle,
            float_mode=args.float_mode,
            monitor=args.monitor,
            log=args.log,
            debug=args.debug,
        )


if __name__ == "__main__":
    main()
