# Not yet integrated with the URDF pipeline — see SIM_PIPELINE.md.
# Inherited from origin/main during the feat/urdf-pipeline merge; this
# file's original body imported `STANCE`, `all_legs`, `apply_per_leg_pose`,
# `NEUTRAL_FOOT`, `splayed_foot_ik`, `pre_orient_splay`, etc., none of
# which exist in this branch's cfg-driven kinematics. The original
# imports were left at the top of the file as a "placeholder" but they
# raise ImportError at module load, breaking even `python -c "import
# terrain"`. This stub replaces them with clear NotImplementedErrors at
# call time so:
#   (a) `import terrain` succeeds (CLI dispatcher in simulate.py can
#       still defer the choice without crashing on startup);
#   (b) `run_terrain(...)` / `run_terrain_matrix(...)` fail loud with a
#       message pointing at the port that needs to happen.
# To restore: re-implement on top of `kinematics.build_config()` /
# `cfg.leg_ik` / `cfg.leg_fk` and remove this stub. The COURSE table
# below is preserved so the obstacle layout doesn't have to be
# rediscovered when the port lands.
"""Terrain obstacle course (port to cfg-driven API pending)."""

COURSE = [
    dict(name="UP_RAMP", y=0.20, max_tilt=25.0),
    dict(name="PLATEAU", y=0.45, max_tilt=20.0),
    dict(name="DOWN_RAMP", y=0.70, max_tilt=30.0),
    dict(name="BUMPS", y=0.95, max_tilt=30.0),
    dict(name="STEP_UP", y=1.40, max_tilt=45.0),
    dict(name="FINISH", y=1.70, max_tilt=25.0),
]


def _not_ported(fn_name: str):
    raise NotImplementedError(
        f"terrain.{fn_name} is not yet ported to the cfg-driven URDF pipeline. "
        "The original implementation depended on theirs' module-state "
        "kinematics (LEG_INFO / NEUTRAL_FOOT / splayed_foot_ik / "
        "pre_orient_splay) which doesn't exist in this branch. Port this "
        "module on top of kinematics.build_config() / cfg.leg_ik before "
        "wiring up --terrain or --terrain-matrix in simulate.py. Tracked "
        "in code/simulation/docs/MERGE_AND_CONVENTION.md §8."
    )


def run_terrain(
    gait_name: str,
    gui: bool = True,
    duration: float = 40.0,
    return_report: bool = False,
):
    _not_ported("run_terrain")


def run_terrain_matrix(gui: bool = False, save_path: str | None = None):
    _not_ported("run_terrain_matrix")
