"""
FaceHugger Quadruped Robot - PyBullet Simulation

Run modes:
  default     : robot loads and holds a stable standing stance
  --walk      : static walk gait (one leg swings at a time, CoM always inside
                support triangle), with overlaid foot-trajectory debug lines.
  --trot      : dynamic trot gait (diagonal pairs FL+RR / FR+RL swing
                together, duty 0.5), same foot-trajectory debug overlay.
  --bound     : front pair then rear pair (FL+FR / RL+RR, duty 0.4). Higher
                step height; body pitches as the rear push-off launches it.
  --crab      : sideways gait driven by HIP pitch with shoulders splayed 45 deg.
  --terrain GAIT
              : spawn an obstacle course and run the named gait across it.
  --terrain-matrix
              : headless run of all three gaits; prints a survival table.
  --teleop [GAIT]
              : keyboard driver on flat ground.

Specs from project proposal:
  - 12-DOF: 4 legs x 3 joints (shoulder yaw, hip pitch, knee pitch)
  - Link lengths: L1=80mm, L2=75mm, L3=90mm
  - Servo: DSS-M15S, 1.47 N*m, 270 deg
  - Default stance: hip ~40 deg, knee ~-60 deg
"""

import argparse
import os

import pybullet as p

from constants import URDF_PATH
from gaits import GAITS, run_gait, run_stand
from terrain import run_terrain, run_terrain_matrix
from teleop import run_teleop


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--walk", action="store_true",
                        help="Run the static walk gait.")
    parser.add_argument("--trot", action="store_true",
                        help="Run the dynamic trot gait (diagonal pairs).")
    parser.add_argument("--bound", action="store_true",
                        help="Run the bound gait (front pair, then rear pair).")
    parser.add_argument("--crab", action="store_true",
                        help="Run the crab gait (sideways along body-X).")
    parser.add_argument("--terrain", choices=sorted(GAITS),
                        help="Run the named gait across the obstacle course.")
    parser.add_argument("--terrain-matrix", action="store_true",
                        help="Run all gaits over the course; print survival table.")
    parser.add_argument("--teleop", nargs="?", const="trot",
                        choices=sorted(GAITS),
                        help="Drive the robot with the keyboard "
                             "(Z/S throttle, Q/D turn, SPACE cycle gait, "
                             "R reset, ESC quit). Optional: starting gait.")
    parser.add_argument("--headless", action="store_true",
                        help="Run without GUI (useful for CI / quick checks).")
    args = parser.parse_args()

    if args.teleop:
        if args.headless:
            parser.error("--teleop requires GUI (drop --headless)")
        try:
            run_teleop(args.teleop, gui=True)
        except p.error as e:
            print(f"  [window closed: {e}]")
        return

    try:
        if args.terrain_matrix:
            report_path = os.path.join(os.path.dirname(URDF_PATH),
                                       "terrain_report.md")
            run_terrain_matrix(gui=not args.headless, save_path=report_path)
        elif args.terrain:
            duration = {"walk": 90.0, "trot": 60.0,
                        "bound": 60.0, "crab": 90.0}[args.terrain]
            run_terrain(args.terrain, gui=not args.headless, duration=duration)
        elif args.walk:
            run_gait("walk", gui=not args.headless)
        elif args.trot:
            run_gait("trot", gui=not args.headless)
        elif args.bound:
            run_gait("bound", gui=not args.headless)
        elif args.crab:
            run_gait("crab", gui=not args.headless)
        else:
            run_stand(gui=not args.headless)
    except p.error as e:
        print(f"  [window closed: {e}]")


if __name__ == "__main__":
    main()
