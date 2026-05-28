"""End-to-end test of the firmware-backed WebSocket robot API (ws_sim).

A real WebSocket client connects to the server, which routes every message
through the compiled firmware's own command dispatch (network.cpp). Verifies
clip discovery (T:8 → the firmware's buildClipListJson) and that commands are
accepted without stalling the connection.

Skips if fh_sim isn't built or websockets/pybullet are unavailable.
"""

import asyncio
import json

import pytest

pytest.importorskip("pybullet")
pytest.importorskip("websockets")


def _skip_if_not_built():
    try:
        from firmware_sil.sil_bridge import staleness  # noqa: F401
        from firmware_sil.ws_sim import RobotSim  # noqa: F401
    except ImportError as e:
        pytest.skip(f"firmware_sil not importable: {e}")
    from firmware_sil.sil_bridge import _compiled_so

    if _compiled_so() is None:
        pytest.skip("fh_sim not built")


def test_ws_clip_discovery_and_commands():
    _skip_if_not_built()
    import websockets

    from firmware_sil.ws_sim import RobotSim

    async def run():
        sim = RobotSim(gui=False)

        async def handler(ws):
            async for raw in ws:
                reply = sim.handle(raw)
                if reply:
                    await ws.send(reply)

        try:
            async with websockets.serve(handler, "localhost", 0) as server:
                port = server.sockets[0].getsockname()[1]
                async with websockets.connect(f"ws://localhost:{port}") as ws:
                    # T:8 — clip discovery, answered by the firmware's own serializer
                    await ws.send(json.dumps({"T": 8}))
                    resp = json.loads(await asyncio.wait_for(ws.recv(), timeout=5))
                    assert "clips" in resp
                    # Don't pin a clip count or specific name: clips are authored
                    # content that comes and goes. Just check the discovery reply is
                    # a non-empty, well-formed list.
                    assert len(resp["clips"]) >= 1
                    assert all({"id", "name", "ms"} <= c.keys() for c in resp["clips"])

                    # A command with no reply (T:7 play clip) must not stall the
                    # socket: a following T:8 still answers. Play whatever clip 0 is.
                    await ws.send(json.dumps({"T": 7, "c": resp["clips"][0]["id"]}))
                    await ws.send(json.dumps({"T": 8}))
                    resp2 = json.loads(await asyncio.wait_for(ws.recv(), timeout=5))
                    assert "clips" in resp2
        finally:
            sim.p.disconnect()

    asyncio.run(run())
