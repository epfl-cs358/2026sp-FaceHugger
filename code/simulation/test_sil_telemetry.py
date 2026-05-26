"""Unit tests for the SIL telemetry frame builder (sil_bridge.build_telemetry_frame).

Exercises the pure servo->joint mapping with a faked PyBullet (so it runs fast and
headless); needs only the compiled fh_sim for the real firmware servo angles +
channel map. Verifies the schema the control panel consumes: per joint a servo-space
command, the same command in URDF-joint degrees, the actual measured joint angle, a
true tracking delta, torque/current, and the firmware's pre-clamp request.
"""

import asyncio
import json
import math

import pytest


def _skip_if_not_built():
    try:
        from firmware_sil.sil_bridge import _compiled_so, load_fh_sim  # noqa: F401
    except ImportError as e:
        pytest.skip(f"firmware_sil not importable: {e}")
    from firmware_sil.sil_bridge import _compiled_so

    if _compiled_so() is None:
        pytest.skip("fh_sim not built")


# Telemetry order the panel expects: FL, FR, BL, BR x shoulder, hip, knee.
_EXPECTED_NAMES = [
    f"{leg}_{j}" for leg in ("fl", "fr", "bl", "br") for j in ("sh", "hip", "knee")
]
_NAME_TO_URDF = {  # telemetry name -> URDF joint, link1=sh, link2=hip, link3=knee
    f"{leg}_{label}": f"{leg}_link{link}_joint"
    for leg in ("fl", "fr", "bl", "br")
    for label, link in (("sh", 1), ("hip", 2), ("knee", 3))
}


class _FakeP:
    """Minimal PyBullet stand-in: getJointState(body, idx) -> (pos, vel, rf, torque)."""

    def getJointState(self, body, idx):
        # Distinct, idx-dependent pos + torque so per-joint wiring is observable.
        return (0.02 * idx, 0.0, (0.0,) * 6, 0.10 * idx)


def test_build_telemetry_frame_schema_and_mapping():
    _skip_if_not_built()
    from firmware_sil.sil_bridge import (
        build_telemetry_frame,
        servo_angles_to_joint_targets,
    )
    from pybullet_sim import sim_monitor

    fc = (
        __import__("firmware_sil.sil_bridge", fromlist=["load_fh_sim"])
        .load_fh_sim()
        .FirmwareControl()
    )
    # Full joint map: each URDF joint -> a unique body index.
    joint_map = {urdf: i for i, urdf in enumerate(_NAME_TO_URDF.values())}
    p = _FakeP()

    frame = build_telemetry_frame(fc, p, robot_id=0, joint_map=joint_map, t_s=1.25)

    assert frame["t"] == 1.25
    joints = frame["joints"]
    assert [j["name"] for j in joints] == _EXPECTED_NAMES
    assert len(joints) == 12

    targets = servo_angles_to_joint_targets(fc.servo_angles())
    required = {
        "name",
        "commanded_servo_deg",
        "commanded_joint_deg",
        "actual_joint_deg",
        "delta_deg",
        "torque_nm",
        "current_a",
        "pre_clamp_deg",
    }
    for j in joints:
        assert required <= j.keys()
        # Servo space is the firmware's 0-180 output.
        assert 0.0 <= j["commanded_servo_deg"] <= 180.0
        urdf = _NAME_TO_URDF[j["name"]]
        idx = joint_map[urdf]
        # Frames are rounded for compact 20 Hz SSE: 2 dp angles, 4/3 dp tau/I.
        cmd_joint = math.degrees(targets[urdf])
        actual = math.degrees(0.02 * idx)
        # commanded_joint_deg = the same command expressed in URDF-joint degrees.
        assert j["commanded_joint_deg"] == round(cmd_joint, 2)
        # actual_joint_deg from the (fake) sim; delta is a true tracking error.
        assert j["actual_joint_deg"] == round(actual, 2)
        assert j["delta_deg"] == round(cmd_joint - actual, 2)
        torque = 0.10 * idx
        assert j["torque_nm"] == round(torque, 4)
        assert j["current_a"] == round(sim_monitor.estimate_current_a(torque), 3)
        # No [OOR] passed -> no pre-clamp request.
        assert j["pre_clamp_deg"] is None


def test_build_telemetry_frame_surfaces_pre_clamp():
    _skip_if_not_built()
    from firmware_sil.sil_bridge import build_telemetry_frame

    fc = (
        __import__("firmware_sil.sil_bridge", fromlist=["load_fh_sim"])
        .load_fh_sim()
        .FirmwareControl()
    )
    joint_map = {urdf: i for i, urdf in enumerate(_NAME_TO_URDF.values())}
    # Inject an out-of-range request on FR shoulder's PCA channel.
    channels = list(fc.servo_channels())
    fr_sh_channel = channels[0 * 3 + 0]  # LEG_FR=0, servo 0 (shoulder/link1)
    frame = build_telemetry_frame(
        fc, _FakeP(), 0, joint_map, t_s=0.0, oor={fr_sh_channel: 205.0}
    )
    by_name = {j["name"]: j for j in frame["joints"]}
    assert by_name["fr_sh"]["pre_clamp_deg"] == 205.0
    assert by_name["fr_hip"]["pre_clamp_deg"] is None


def test_put_drop_stale_keeps_only_latest():
    from firmware_sil.ws_sim import _put_drop_stale

    q = asyncio.Queue(maxsize=1)
    _put_drop_stale(q, {"t": 1})
    _put_drop_stale(q, {"t": 2})  # must evict the stale frame, not block
    assert q.get_nowait() == {"t": 2}
    assert q.empty()


def test_sse_streams_a_pushed_frame():
    """End-to-end: a frame pushed to the queue is delivered as SSE `data:`."""
    pytest.importorskip("sse_starlette")
    uvicorn = pytest.importorskip("uvicorn")

    from firmware_sil.ws_sim import build_telemetry_app

    frame = {"t": 0.5, "joints": [{"name": f"j{i}"} for i in range(12)]}

    async def run():
        queue = asyncio.Queue(maxsize=1)
        app = build_telemetry_app(queue)
        server = uvicorn.Server(
            uvicorn.Config(app, host="127.0.0.1", port=0, log_level="warning")
        )
        serve_task = asyncio.create_task(server.serve())
        try:
            while not server.started:  # wait for the socket to bind
                await asyncio.sleep(0.01)
            port = server.servers[0].sockets[0].getsockname()[1]

            await queue.put(frame)
            reader, writer = await asyncio.open_connection("127.0.0.1", port)
            writer.write(
                b"GET /telemetry HTTP/1.1\r\nHost: x\r\n"
                b"Accept: text/event-stream\r\n\r\n"
            )
            await writer.drain()

            payload = None
            while payload is None:  # skip status/headers/ping comments
                line = await asyncio.wait_for(reader.readline(), timeout=5)
                if line.startswith(b"data:"):
                    payload = json.loads(line[len(b"data:") :].strip())
            writer.close()
            return payload
        finally:
            server.should_exit = True
            await serve_task

    got = asyncio.run(run())
    assert got["t"] == 0.5
    assert len(got["joints"]) == 12
