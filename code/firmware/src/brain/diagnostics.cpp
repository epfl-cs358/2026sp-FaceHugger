#include "diagnostics.h"
#include "csv_log.h"

#include <Arduino.h>
#include <WebServer.h>

#include "../nervous_system/spinal_cord.h"

namespace {

constexpr std::size_t kBufBytes = 8 * 1024;  // 8 KB rolling buffer

WebServer                gServer(diagnostics::kHttpPort);
// gRing is mutated by both tick() (append) and handleLog/handleClear (read/clear),
// but both are driven from loop() in series — no concurrency to worry about.
csv_log::Ring<kBufBytes> gRing;
SpinalCord*              gSc           = nullptr;
unsigned long            gLastSampleMs = 0;

void handleLog() {
    gServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    gServer.send(200, "text/plain", "");
    gServer.sendContent(csv_log::header());

    const char* d1; std::size_t s1;
    const char* d2; std::size_t s2;
    gRing.view(d1, s1, d2, s2);
    if (s1) gServer.sendContent(d1, s1);
    if (s2) gServer.sendContent(d2, s2);
    gServer.sendContent("");  // end chunked
}

void handleClear() {
    gRing.clear();
    gServer.send(200, "text/plain", "OK\n");
}

void handleRoot() {
    gServer.send(200, "text/plain",
        "FaceHugger diagnostics\n"
        "  GET /log        — CSV header + rolling buffer\n"
        "  GET /log/clear  — clear the buffer\n");
}

} // namespace

namespace diagnostics {

void begin(SpinalCord& sc) {
    gSc = &sc;
    gServer.on("/",          HTTP_GET, handleRoot);
    gServer.on("/log",       HTTP_GET, handleLog);
    gServer.on("/log/clear", HTTP_GET, handleClear);
    gServer.begin();
}

void tick() {
    if (gSc == nullptr) return;
    const unsigned long now = millis();
    if (now - gLastSampleMs < kSampleIntervalMs) return;
    gLastSampleMs = now;

    SpinalCord::Snapshot snap = gSc->snapshot();
    csv_log::State s{};
    s.millis       = now;
    s.robot_state  = snap.robot_state;
    s.gait         = snap.gait;
    s.is_moving    = snap.is_moving ? 1 : 0;
    s.is_inverted  = snap.is_inverted ? 1 : 0;
    s.last_cmd_ms  = snap.last_command_ms;
    s.target_x = snap.target_x; s.target_y = snap.target_y; s.target_yaw = snap.target_yaw;
    s.active_x = snap.active_x; s.active_y = snap.active_y; s.active_yaw = snap.active_yaw;
    for (int i = 0; i < 12; ++i) s.servo_angles[i] = snap.servo_angles[i];

    char row[csv_log::kMaxRowBytes];
    std::size_t n = csv_log::format_row(s, row, sizeof(row));
    if (n > 0) gRing.append(row, n);
}

void handleHttp() {
    gServer.handleClient();
}

} // namespace diagnostics
