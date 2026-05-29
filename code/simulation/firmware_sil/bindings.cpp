// firmware_sil/bindings.cpp
//
// pybind11 module `fh_sim` wrapping the EXACT firmware SpinalCord. This is the
// software-in-the-loop seam: Python feeds a command + a synthetic clock, the
// real firmware control code runs, and Python reads back the 12 servo angles
// the firmware computed (post-translateToServo, post-clamp, whole-degree — the
// same values the robot's servos would receive).
//
// The firmware sources (spinal_cord.cpp, servo.cpp, leg.cpp, motion_math.cpp,
// kinematics.cpp) are compiled UNCHANGED against firmware_sil/hal/ (see
// CMakeLists.txt). No Python re-port is in this path.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <string>
#include <vector>

#include "nervous_system/spinal_cord.h"   // the real firmware class
#include "nervous_system/clips_all.h"     // FH_CLIPS[] / FH_CLIP_COUNT
#include "shared/config.h"                // ADDR_SERVO_DRIVER
#include "brain/network.h"                // handleParsedMessage (real T: dispatch)
#include "brain/sensors.h"                // imuIsInverted accessor (SIL-backed)

namespace py = pybind11;

// THE robot. network.cpp dispatches commands to this exact global symbol
// (`extern SpinalCord spinalCord;`), so compiling network.cpp's handleParsedMessage
// and driving this object share one SpinalCord — the WebSocket command routing is
// the real firmware code, not a Python mirror.
SpinalCord spinalCord(ADDR_SERVO_DRIVER);

// Thin Python-facing wrapper over the global SpinalCord + the firmware's own
// JSON command dispatch.
class FirmwareControl {
   public:
    FirmwareControl() {
        fh_sim::clock_ms = 0;
        spinalCord.begin();  // sets each servo to its boot default angle
    }

    // Advance "firmware time" then run exactly one firmware control tick.
    // We replicate main.cpp::loop()'s auto-flip gate here — the SIL bypasses
    // loop() entirely (the WS server is Python-side), so without this the
    // IMU latch would be set but never forwarded to setInverted().
    void tick(uint32_t t_ms) {
        fh_sim::clock_ms = t_ms;
        spinalCord.tickAutoInvert(imuIsInverted());
        spinalCord.update();
    }

    void set_clock_ms(uint32_t t_ms) { fh_sim::clock_ms = t_ms; }

    void play_clip(uint8_t id) { spinalCord.playClip(id); }

    // Run the EXACT firmware command dispatch (network.cpp::handleParsedMessage)
    // on a raw JSON payload. Returns any text the firmware replied via sendTXT
    // (e.g. the T:8 clip list from the real buildClipListJson), else "".
    std::string handle_message(const std::string& json) {
        fh_sim::ws_last_txt.clear();
        std::string buf = json;  // mutable, NUL-terminated for ArduinoJson
        handleParsedMessage(0, reinterpret_cast<uint8_t*>(buf.data()));
        return fh_sim::ws_last_txt;
    }

    // --- direct command surface (used by tests; the WS server uses handle_message) ---
    void walk() { spinalCord.walk(); }
    void rest() { spinalCord.rest(); }
    void relax() { spinalCord.relax(); }
    void stand() { spinalCord.stand(); }
    void wall_flip() { spinalCord.wallFlip(); }
    void invert_robot() { spinalCord.invertRobot(); }
    void set_gait(int g) { spinalCord.setGait((GaitType)g); }
    int current_gait() const { return (int)spinalCord.currentGait(); }
    void process_command(const std::string& dir) { spinalCord.processCommand(String(dir)); }

    bool calibrate(int leg_id, int servo_id, int angle) {
        if (!isValidServoIndex(leg_id, servo_id)) return false;
        spinalCord.applyCalibration(LEG_SERVO_CHANNEL[leg_id][servo_id], angle);
        return true;
    }

    int clip_id_by_name(const std::string& name) const {
        for (uint8_t i = 0; i < FH_CLIP_COUNT; ++i)
            if (name == FH_CLIPS[i].name) return i;
        return -1;
    }

    std::vector<std::string> clip_names() const {
        std::vector<std::string> v;
        for (uint8_t i = 0; i < FH_CLIP_COUNT; ++i) v.emplace_back(FH_CLIPS[i].name);
        return v;
    }

    uint32_t clip_duration_ms(uint8_t id) const {
        return id < FH_CLIP_COUNT ? FH_CLIPS[id].duration_ms : 0;
    }

    // 12 servo angles in degrees, firmware LegId order:
    // FR(hip,thigh,knee), FL(...), BR(...), BL(...).
    std::array<float, 12> servo_angles() const {
        SpinalCord::Snapshot s = spinalCord.snapshot();
        std::array<float, 12> a{};
        for (int i = 0; i < 12; ++i) a[i] = s.servo_angles[i];
        return a;
    }

    uint8_t robot_state() const { return spinalCord.snapshot().robot_state; }

    // IMU SIL surface. The production firmware polls the MPU6050 in tickImu();
    // on the host the bridge computes the upside-down state from PyBullet's
    // body orientation (with the same hysteresis as the firmware) and writes
    // it here. imu_is_inverted() reads back through the firmware's own
    // sensors.h accessor so tests can assert on the same surface main.cpp
    // would see on hardware.
    void  set_imu_upside_down(bool v) { fh_sim::imu_upside_down = v; }
    void  set_imu_pitch_deg(float v)  { fh_sim::pitch_deg = v; }
    void  set_imu_roll_deg(float v)   { fh_sim::roll_deg = v; }
    bool  imu_is_inverted() const     { return imuIsInverted(); }
    // Read-back surface for Python telemetry — mirrors the same getters
    // main.cpp would call when building the T:10 frame on hardware.
    float imu_pitch_deg() const            { return imuPitchDeg(); }
    float imu_roll_deg() const             { return imuRollDeg(); }
    bool  is_auto_invert_enabled() const   { return spinalCord.isAutoInvertEnabled(); }

    // Drain the firmware's captured Serial output (complete lines since the last
    // call) and clear it. The firmware's own out-of-range guard prints
    // "[OOR] servo <ch> requested <deg>"; the SIL surfaces those by reading here.
    std::vector<std::string> drain_serial() {
        std::vector<std::string> out;
        out.swap(fh_sim::serial_lines);
        return out;
    }

    // PCA channel per servo, firmware order (FR,FL,RR,RL x hip,thigh,knee) — lets
    // the Python side map an [OOR] channel back to a joint.
    std::array<int, 12> servo_channels() const {
        std::array<int, 12> ch{};
        for (int leg = 0; leg < 4; ++leg)
            for (int j = 0; j < 3; ++j) ch[leg * 3 + j] = LEG_SERVO_CHANNEL[leg][j];
        return ch;
    }
};

PYBIND11_MODULE(fh_sim, m) {
    m.doc() = "FaceHugger firmware control, compiled for the host (SIL).";
    py::class_<FirmwareControl>(m, "FirmwareControl")
        .def(py::init<>())
        .def("tick", &FirmwareControl::tick, py::arg("t_ms"))
        .def("set_clock_ms", &FirmwareControl::set_clock_ms, py::arg("t_ms"))
        .def("play_clip", &FirmwareControl::play_clip, py::arg("id"))
        .def("handle_message", &FirmwareControl::handle_message, py::arg("json"))
        .def("walk", &FirmwareControl::walk)
        .def("rest", &FirmwareControl::rest)
        .def("relax", &FirmwareControl::relax)
        .def("stand", &FirmwareControl::stand)
        .def("wall_flip", &FirmwareControl::wall_flip)
        .def("invert_robot", &FirmwareControl::invert_robot)
        .def("set_gait", &FirmwareControl::set_gait, py::arg("g"))
        .def("current_gait", &FirmwareControl::current_gait)
        .def("process_command", &FirmwareControl::process_command, py::arg("dir"))
        .def("calibrate", &FirmwareControl::calibrate,
             py::arg("leg_id"), py::arg("servo_id"), py::arg("angle"))
        .def("clip_id_by_name", &FirmwareControl::clip_id_by_name, py::arg("name"))
        .def("clip_names", &FirmwareControl::clip_names)
        .def("clip_duration_ms", &FirmwareControl::clip_duration_ms, py::arg("id"))
        .def("servo_angles", &FirmwareControl::servo_angles)
        .def("robot_state", &FirmwareControl::robot_state)
        .def("drain_serial", &FirmwareControl::drain_serial)
        .def("servo_channels", &FirmwareControl::servo_channels)
        .def("set_imu_upside_down", &FirmwareControl::set_imu_upside_down, py::arg("v"))
        .def("set_imu_pitch_deg", &FirmwareControl::set_imu_pitch_deg, py::arg("v"))
        .def("set_imu_roll_deg", &FirmwareControl::set_imu_roll_deg, py::arg("v"))
        .def("imu_is_inverted", &FirmwareControl::imu_is_inverted)
        .def("imu_pitch_deg", &FirmwareControl::imu_pitch_deg)
        .def("imu_roll_deg", &FirmwareControl::imu_roll_deg)
        .def("is_auto_invert_enabled", &FirmwareControl::is_auto_invert_enabled);
}
