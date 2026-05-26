"""Torque + estimated-current monitoring for the PyBullet sim (behind --monitor).

The position controller reports the torque it applied at each joint
(getJointState()[3]). From that we estimate per-servo current and the total
draw, and flag servos that are stalling or a total that exceeds the supply
budget — so you can see *why* a clip collapses (which joints saturate) without
hardware.

The pure helpers (estimate_current_a, total_current_a, format_status) take plain
numbers/dicts and are unit-tested without PyBullet; read_joint_torques /
read_joint_pos_deg do the PyBullet reads.
"""

import math

STALL_TORQUE_NM = (
    2.94  # 30 kgf·cm — leave effort_nm in yaml as-is, it's already correct
)
STALL_CURRENT_A = 2.0  # DSS-M15S (SER0044) stall current at 7 V
CURRENT_LIMIT_A = 10.0  # supply / multiplexer budget (12 servos × ~0.8 A realistic avg)
STALL_WARN_NM = 2.5  # ~85% of stall — flag a joint whose applied torque exceeds this

_LEGS = ("fr", "fl", "br", "bl")


def estimate_current_a(torque_nm: float) -> float:
    """Estimated servo current (A) for an applied torque, linear from the
    stall point: |torque| / (stall_torque / stall_current)."""
    return abs(torque_nm) * (STALL_CURRENT_A / STALL_TORQUE_NM)


def total_current_a(torques) -> float:
    """Sum of estimated current over an iterable of joint torques (N·m)."""
    return sum(estimate_current_a(t) for t in torques)


def format_status(elapsed_s: float, joint_torques: dict, joint_pos_deg=None) -> str:
    """Compact one-line status.

    joint_torques: {joint_name: torque_nm} for the 12 joints.
    joint_pos_deg: optional {joint_name: degrees} to include per-leg angles.

    Example:
      t= 2.3s | FR:sh=+1 th=-41 kn=-57 | ... | peak_τ=1.84N·m(fr_link2_joint)
      | est_I=6.2A | [STALL] fr_link2_joint
    Adds " [WARN >10A]" when the total estimated current exceeds the budget.
    """
    total_i = total_current_a(joint_torques.values())
    peak_name, peak_tau = max(
        joint_torques.items(), key=lambda kv: abs(kv[1]), default=("-", 0.0)
    )
    stalls = [n for n, t in joint_torques.items() if abs(t) > STALL_WARN_NM]

    parts = [f"t={elapsed_s:5.1f}s"]
    if joint_pos_deg:
        for leg in _LEGS:
            sh = joint_pos_deg.get(f"{leg}_link1_joint")
            th = joint_pos_deg.get(f"{leg}_link2_joint")
            kn = joint_pos_deg.get(f"{leg}_link3_joint")
            if sh is not None:
                parts.append(f"{leg.upper()}:sh={sh:+.0f} th={th:+.0f} kn={kn:+.0f}")
    parts.append(f"peak_τ={abs(peak_tau):.2f}N·m({peak_name})")
    warn = " [WARN >10A]" if total_i > CURRENT_LIMIT_A else ""
    parts.append(f"est_I={total_i:.1f}A{warn}")
    if stalls:
        parts.append("[STALL] " + ",".join(stalls))
    return " | ".join(parts)


def read_joint_torques(robot_id, joint_map) -> dict:
    """{joint_name: applied torque (N·m)} from getJointState()[3]."""
    import pybullet as p

    return {name: p.getJointState(robot_id, idx)[3] for name, idx in joint_map.items()}


def read_joint_pos_deg(robot_id, joint_map) -> dict:
    """{joint_name: current joint angle in degrees} from getJointState()[0]."""
    import pybullet as p

    return {
        name: math.degrees(p.getJointState(robot_id, idx)[0])
        for name, idx in joint_map.items()
    }


class SimLogger:
    """Per-step torque/current recorder for a sim run (behind --log).

    Purely additive to --monitor: where --monitor prints a periodic status
    line, --log records *every* step into in-memory arrays, then at the end of
    the run prints a per-joint summary and writes a CSV + a 3-panel plot. Uses
    the same torque->current model as the rest of this module
    (estimate_current_a). matplotlib/csv are imported lazily so importing
    sim_monitor stays dependency-light (same spirit as the lazy pybullet import).
    """

    def __init__(self, joint_names: list):
        self.joint_names = list(joint_names)
        self.t: list = []
        self.torque = {n: [] for n in self.joint_names}
        self.current = {n: [] for n in self.joint_names}
        self.total_current: list = []

    def record(self, t: float, torques: dict) -> None:
        """Append one timestep: time + per-joint torque & estimated current."""
        self.t.append(t)
        total = 0.0
        for n in self.joint_names:
            tau = torques.get(n, 0.0)
            amps = estimate_current_a(tau)
            self.torque[n].append(tau)
            self.current[n].append(amps)
            total += amps
        self.total_current.append(total)

    def summary(self) -> None:
        """Print a per-joint max/mean torque & current table for the run."""
        if not self.t:
            print("[log] no samples recorded")
            return
        span = self.t[-1] - self.t[0] if len(self.t) > 1 else 0.0
        print(f"\n=== sim log summary ({len(self.t)} samples, {span:.2f}s) ===")
        print(f"  {'joint':<16}{'max_τ':>9}{'mean_τ':>9}{'max_I':>9}{'mean_I':>9}")
        for n in self.joint_names:
            tq, cu = self.torque[n], self.current[n]
            max_tau = max((abs(x) for x in tq), default=0.0)
            mean_tau = sum(abs(x) for x in tq) / len(tq) if tq else 0.0
            max_i = max(cu, default=0.0)
            mean_i = sum(cu) / len(cu) if cu else 0.0
            print(f"  {n:<16}{max_tau:9.3f}{mean_tau:9.3f}{max_i:9.3f}{mean_i:9.3f}")
        peak_total = max(self.total_current, default=0.0)
        print(
            f"  peak total current: {peak_total:.2f} A (budget {CURRENT_LIMIT_A:.0f} A)"
        )

    def save_csv(self, path: str) -> None:
        """Write time, torque_<joint>..., current_<joint>..., total_current."""
        import csv

        header = (
            ["time"]
            + [f"torque_{n}" for n in self.joint_names]
            + [f"current_{n}" for n in self.joint_names]
            + ["total_current"]
        )
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            for k in range(len(self.t)):
                row = (
                    [self.t[k]]
                    + [self.torque[n][k] for n in self.joint_names]
                    + [self.current[n][k] for n in self.joint_names]
                    + [self.total_current[k]]
                )
                w.writerow(row)
        print(f"[log] wrote {path} ({len(self.t)} rows)")

    def plot(self, path: str = None) -> None:
        """3-subplot figure: per-joint torque, per-joint current, total current.

        Saves a PNG to `path`, or shows interactively when `path is None`.
        matplotlib is imported lazily; if it is unavailable the plot is skipped
        (the CSV/summary still ran).
        """
        try:
            import matplotlib

            if path is not None:
                matplotlib.use("Agg")  # headless-safe; no display needed to save
            import matplotlib.pyplot as plt
        except ImportError:
            print("[log] matplotlib not available — skipping plot")
            return
        if not self.t:
            print("[log] no samples to plot")
            return

        fig, (ax_tau, ax_cur, ax_tot) = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        for n in self.joint_names:
            ax_tau.plot(self.t, self.torque[n], lw=0.8, label=n)
            ax_cur.plot(self.t, self.current[n], lw=0.8, label=n)

        ax_tau.axhline(
            STALL_WARN_NM,
            color="r",
            ls="--",
            lw=1,
            label=f"stall warn {STALL_WARN_NM} N·m",
        )
        ax_tau.set_ylabel("torque (N·m)")
        ax_tau.set_title("Per-joint applied torque")
        ax_tau.legend(fontsize=6, ncol=4, loc="upper right")

        ax_cur.axhline(
            STALL_CURRENT_A,
            color="r",
            ls="--",
            lw=1,
            label=f"stall current {STALL_CURRENT_A} A",
        )
        ax_cur.set_ylabel("current (A)")
        ax_cur.set_title("Per-joint estimated current")
        ax_cur.legend(fontsize=6, ncol=4, loc="upper right")

        ax_tot.fill_between(self.t, self.total_current, color="C0", alpha=0.4)
        ax_tot.plot(self.t, self.total_current, color="C0", lw=1)
        ax_tot.axhline(
            CURRENT_LIMIT_A,
            color="r",
            ls="--",
            lw=1,
            label=f"budget {CURRENT_LIMIT_A} A",
        )
        ax_tot.set_ylabel("total current (A)")
        ax_tot.set_xlabel("time (s)")
        ax_tot.set_title("Total estimated current")
        ax_tot.legend(fontsize=7, loc="upper right")

        fig.tight_layout()
        if path is not None:
            fig.savefig(path, dpi=110)
            plt.close(fig)
            print(f"[log] wrote {path}")
        else:
            plt.show()
