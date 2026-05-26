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

from firmware_sil.sil_bridge import (
    apply_torque_colors,
    build_telemetry_frame,
    load_fh_sim,
    parse_oor,
    servo_angles_to_joint_targets,
)

CMD_TELEMETRY = 10
_STEP_HZ = 240
_TELEM_EVERY = 12  # build a telemetry frame every 12th step → 240/12 = 20 Hz
_SSE_PORT = 8082


class RobotSim:
    """A PyBullet robot driven by the compiled firmware control code."""

    def __init__(self, gui=False):
        import pybullet as p

        from pybullet_sim.gaits import _connect_and_setup
        from pybullet_sim.kinematics import build_config

        self.p = p
        self.gui = gui
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

    def telemetry_frame(self, t_s):
        """Rich per-joint frame for the SSE telemetry panel (see sil_bridge).

        Drains the firmware's [OOR] serial lines accumulated since the last call so
        any pre-clamp request lands in pre_clamp_deg.
        """
        oor = parse_oor(self.fc.drain_serial())
        return build_telemetry_frame(
            self.fc, self.p, self.robot_id, self.joint_map, t_s, oor
        )


def _put_drop_stale(queue, item):
    """Push `item`, discarding any unconsumed frame so consumers see the latest."""
    if queue.full():
        try:
            queue.get_nowait()
        except asyncio.QueueEmpty:
            pass
    queue.put_nowait(item)


async def _sim_loop(sim, telem_queue=None):
    dt = 1.0 / _STEP_HZ
    t0 = time.monotonic()
    step = 0
    while True:
        now = time.monotonic() - t0
        sim.step(int(now * 1000.0))
        if step % _TELEM_EVERY == 0:
            if sim.gui:  # torque-tint the links in the PyBullet window
                apply_torque_colors(sim.p, sim.robot_id, sim.joint_map)
            if telem_queue is not None:
                _put_drop_stale(telem_queue, sim.telemetry_frame(now))
        step += 1
        await asyncio.sleep(dt)


async def _telemetry_loop(sim, clients, period=0.5):
    while True:
        await asyncio.sleep(period)
        if clients:
            msg = json.dumps(sim.telemetry())
            await asyncio.gather(
                *(c.send(msg) for c in list(clients)), return_exceptions=True
            )


def build_telemetry_app(telem_queue):
    """Starlette app serving the per-joint frames as Server-Sent Events.

    GET /telemetry streams `data: <frame-json>` at the rate the sim loop pushes
    (~20 Hz). CORS is wide open because a `file://` control panel is a cross-origin
    EventSource client. The queue is single-consumer (the local debug panel); a
    second client would split frames rather than each getting every frame.
    """
    from sse_starlette.sse import EventSourceResponse
    from starlette.applications import Starlette
    from starlette.middleware import Middleware
    from starlette.middleware.cors import CORSMiddleware
    from starlette.routing import Route

    async def telemetry(request):
        async def gen():
            while not await request.is_disconnected():
                frame = await telem_queue.get()
                yield {"data": json.dumps(frame)}

        return EventSourceResponse(gen())

    return Starlette(
        routes=[Route("/telemetry", telemetry)],
        middleware=[Middleware(CORSMiddleware, allow_origins=["*"])],
    )


async def serve(host="localhost", port=8081, gui=False, sse_port=_SSE_PORT):
    sim = RobotSim(gui=gui)
    clients = set()
    telem_queue = asyncio.Queue(maxsize=1)

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

    # SSE telemetry is best-effort: if sse-starlette/uvicorn aren't installed the
    # robot API still runs (telemetry is a debug aid, not part of the T: protocol).
    sse_task = None
    try:
        import uvicorn

        app = build_telemetry_app(telem_queue)
        sse_server = uvicorn.Server(
            uvicorn.Config(app, host=host, port=sse_port, log_level="warning")
        )
        sse_task = sse_server.serve()
    except ImportError:
        print(
            "[ws-sim] sse-starlette/uvicorn not installed — sim telemetry SSE "
            "disabled (pip/conda install them to enable the panel's telemetry table)"
        )

    async with websockets.serve(handler, host, port):
        print(
            f"[ws-sim] firmware-backed robot API on ws://{host}:{port}  "
            f"(real robot uses :81; point the panel/app here)"
        )
        if sse_task is not None:
            print(f"[ws-sim] sim telemetry (SSE) on http://{host}:{sse_port}/telemetry")
        coros = [_sim_loop(sim, telem_queue), _telemetry_loop(sim, clients)]
        if sse_task is not None:
            coros.append(sse_task)
        await asyncio.gather(*coros)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="localhost")
    ap.add_argument(
        "--port", type=int, default=8081, help="default 8081 (81 is privileged)"
    )
    ap.add_argument("--gui", action="store_true", help="show the PyBullet window")
    ap.add_argument(
        "--sse-port",
        type=int,
        default=_SSE_PORT,
        help=f"telemetry SSE port (default {_SSE_PORT})",
    )
    args = ap.parse_args()
    try:
        asyncio.run(serve(args.host, args.port, args.gui, args.sse_port))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
