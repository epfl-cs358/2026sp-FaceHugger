"""WebSocket robot-API server backed by the EXACT firmware (SIL).

Serves the API_SPEC v1.1 `T:` protocol and drives a PyBullet robot with the
compiled firmware code. The unmodified app — or the standalone control panel
(tools/robot_control_panel.html) — connects over WebSocket and drives the
simulated robot exactly as it would the real one (clip discovery, gaits, moves,
calibration, telemetry).

    python -m firmware_sil.ws_sim [--port 8081] [--host localhost] [--gui]

How faithful: the command routing IS the compiled firmware. Each inbound message
is handed verbatim to the firmware's own `network.cpp::handleParsedMessage`
(compiled into fh_sim), which parses the JSON with ArduinoJson and dispatches to
the real SpinalCord — so any change to the firmware's API handling reflects here
automatically, with no Python mirror to drift. The only Python parts are the
WebSocket transport and the periodic robot->dashboard telemetry (which the
firmware emits elsewhere, not in handleParsedMessage).

Port note: the real robot uses 81; that's privileged on macOS/Linux, so the sim
defaults to 8081. Point the panel/app at <host>:8081 for the sim, or the robot's
IP:81 for hardware.
"""

import argparse
import asyncio
import json
import time

import websockets

from firmware_sil.sil_bridge import load_fh_sim, servo_angles_to_joint_targets

CMD_TELEMETRY = 10
_STEP_HZ = 240


class RobotSim:
    """A PyBullet robot driven by the compiled firmware control code."""

    def __init__(self, gui=False):
        import pybullet as p

        from pybullet_sim.gaits import _connect_and_setup
        from pybullet_sim.kinematics import build_config

        self.p = p
        self.cfg = build_config()
        self.robot_id, self.joint_map = _connect_and_setup(self.cfg, gui)
        self.fc = load_fh_sim().FirmwareControl()

    def step(self, t_ms):
        self.fc.tick(t_ms)  # advance firmware time + run one control tick
        for name, rad in servo_angles_to_joint_targets(self.fc.servo_angles()).items():
            idx = self.joint_map.get(name)
            if idx is not None:
                self.p.setJointMotorControl2(
                    self.robot_id,
                    idx,
                    self.p.POSITION_CONTROL,
                    targetPosition=rad,
                    force=self.cfg.servo_force,
                    maxVelocity=self.cfg.servo_velocity,
                )
        self.p.stepSimulation()

    def handle(self, raw):
        """Route a raw JSON message through the EXACT firmware dispatch.

        Returns the firmware's reply text (e.g. T:8 clip list), or "" if none.
        """
        return self.fc.handle_message(raw)

    def telemetry(self):
        # Spec-shaped T:10 (robot -> dashboard). ToF/IMU aren't simulated -> null.
        return {
            "T": CMD_TELEMETRY,
            "s": self.fc.robot_state(),
            "g": self.fc.current_gait(),
            "pc": 0.0,
            "d": None,
            "a": None,
            "e": None,
            "servo_deg": [round(x) for x in self.fc.servo_angles()],
        }


async def _sim_loop(sim):
    dt = 1.0 / _STEP_HZ
    t0 = time.monotonic()
    while True:
        sim.step(int((time.monotonic() - t0) * 1000.0))
        await asyncio.sleep(dt)


async def _telemetry_loop(sim, clients, period=0.5):
    while True:
        await asyncio.sleep(period)
        if clients:
            msg = json.dumps(sim.telemetry())
            await asyncio.gather(
                *(c.send(msg) for c in list(clients)), return_exceptions=True
            )


async def serve(host="localhost", port=8081, gui=False):
    sim = RobotSim(gui=gui)
    clients = set()

    async def handler(ws):
        clients.add(ws)
        try:
            async for raw in ws:
                reply = sim.handle(raw)  # the compiled firmware dispatch
                if reply:
                    await ws.send(reply)
        except websockets.ConnectionClosed:
            pass
        finally:
            clients.discard(ws)

    async with websockets.serve(handler, host, port):
        print(
            f"[ws-sim] firmware-backed robot API on ws://{host}:{port}  "
            f"(real robot uses :81; point the panel/app here)"
        )
        await asyncio.gather(_sim_loop(sim), _telemetry_loop(sim, clients))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="localhost")
    ap.add_argument(
        "--port", type=int, default=8081, help="default 8081 (81 is privileged)"
    )
    ap.add_argument("--gui", action="store_true", help="show the PyBullet window")
    args = ap.parse_args()
    try:
        asyncio.run(serve(args.host, args.port, args.gui))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
