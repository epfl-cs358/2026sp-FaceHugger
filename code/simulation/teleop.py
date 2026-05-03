# Not yet integrated with the URDF pipeline — see SIM_PIPELINE.md.
# Inherited from origin/main during the feat/urdf-pipeline merge; this
# file's original body imported `STANCE`, `all_legs`, `apply_per_leg_pose`,
# `NEUTRAL_FOOT`, `splayed_foot_ik`, `pre_orient_splay` etc., none of
# which exist in this branch's cfg-driven kinematics. The original
# imports were left at the top of the file as a "placeholder" but they
# raise ImportError at module load, breaking even `python -c "import
# teleop"`. This stub replaces them with a clear NotImplementedError
# at call time so:
#   (a) `import teleop` succeeds (CLI dispatcher in simulate.py can
#       still defer the choice without crashing on startup);
#   (b) `run_teleop(...)` fails loud with a message pointing at the
#       port that needs to happen.
# To restore: re-implement on top of `kinematics.build_config()` /
# `cfg.leg_ik` / `cfg.leg_fk` and remove this stub.
"""Keyboard-driven teleop on flat ground (port to cfg-driven API pending)."""


def run_teleop(gait_name: str = "trot", gui: bool = True, duration: float = 600.0):
    raise NotImplementedError(
        "teleop.run_teleop is not yet ported to the cfg-driven URDF pipeline. "
        "The original implementation depended on theirs' module-state "
        "kinematics (LEG_INFO / NEUTRAL_FOOT / splayed_foot_ik / "
        "pre_orient_splay) which doesn't exist in this branch. To use "
        "--teleop, port this module on top of kinematics.build_config() / "
        "cfg.leg_ik. Tracked in code/simulation/docs/MERGE_AND_CONVENTION.md §8."
    )
