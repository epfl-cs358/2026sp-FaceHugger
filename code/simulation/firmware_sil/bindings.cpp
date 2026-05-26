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

namespace py = pybind11;

// Thin wrapper: owns one SpinalCord, drives it from an injected clock.
class FirmwareControl {
   public:
    FirmwareControl() : sc_(ADDR_SERVO_DRIVER) {
        fh_sim::clock_ms = 0;
        sc_.begin();  // sets each servo to its boot default angle
    }

    // Advance "firmware time" then run exactly one firmware control tick.
    void tick(uint32_t t_ms) {
        fh_sim::clock_ms = t_ms;
        sc_.update();
    }

    void set_clock_ms(uint32_t t_ms) { fh_sim::clock_ms = t_ms; }

    void play_clip(uint8_t id) { sc_.playClip(id); }

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
        SpinalCord::Snapshot s = sc_.snapshot();
        std::array<float, 12> a{};
        for (int i = 0; i < 12; ++i) a[i] = s.servo_angles[i];
        return a;
    }

    uint8_t robot_state() const { return sc_.snapshot().robot_state; }

   private:
    SpinalCord sc_;
};

PYBIND11_MODULE(fh_sim, m) {
    m.doc() = "FaceHugger firmware control, compiled for the host (SIL).";
    py::class_<FirmwareControl>(m, "FirmwareControl")
        .def(py::init<>())
        .def("tick", &FirmwareControl::tick, py::arg("t_ms"))
        .def("set_clock_ms", &FirmwareControl::set_clock_ms, py::arg("t_ms"))
        .def("play_clip", &FirmwareControl::play_clip, py::arg("id"))
        .def("clip_id_by_name", &FirmwareControl::clip_id_by_name, py::arg("name"))
        .def("clip_names", &FirmwareControl::clip_names)
        .def("clip_duration_ms", &FirmwareControl::clip_duration_ms, py::arg("id"))
        .def("servo_angles", &FirmwareControl::servo_angles)
        .def("robot_state", &FirmwareControl::robot_state);
}
