#include "csv_log.h"
#include <cstdio>

namespace csv_log {

static const char kHeader[] =
    "millis,robot_state,gait,is_moving,is_inverted,last_cmd_ms,"
    "target_x,target_y,target_yaw,active_x,active_y,active_yaw,"
    "fr_hip,fr_thigh,fr_knee,fl_hip,fl_thigh,fl_knee,"
    "br_hip,br_thigh,br_knee,bl_hip,bl_thigh,bl_knee\n";

const char* header() { return kHeader; }

std::size_t format_row(const State& s, char* out, std::size_t cap) {
    int n = std::snprintf(
        out, cap,
        "%lu,%u,%u,%u,%u,%lu,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
        (unsigned long)s.millis,
        (unsigned)s.robot_state, (unsigned)s.gait,
        (unsigned)s.is_moving, (unsigned)s.is_inverted,
        (unsigned long)s.last_cmd_ms,
        (double)s.target_x, (double)s.target_y, (double)s.target_yaw,
        (double)s.active_x, (double)s.active_y, (double)s.active_yaw,
        (double)s.servo_angles[0], (double)s.servo_angles[1], (double)s.servo_angles[2],
        (double)s.servo_angles[3], (double)s.servo_angles[4], (double)s.servo_angles[5],
        (double)s.servo_angles[6], (double)s.servo_angles[7], (double)s.servo_angles[8],
        (double)s.servo_angles[9], (double)s.servo_angles[10], (double)s.servo_angles[11]
    );
    if (n <= 0 || (std::size_t)n >= cap) return 0;
    return (std::size_t)n;
}

} // namespace csv_log
