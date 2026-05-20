#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <cstddef>

class SpinalCord;

namespace diagnostics {

// === Tunables ============================================================
// Changing these requires a firmware rebuild + flash.

// How often a CSV row is appended. Lower = denser samples, shorter window.
constexpr unsigned long kSampleIntervalMs = 200;

// Rolling RAM buffer size in bytes (static BSS allocation). Rough window:
//   window_seconds ≈ (kBufBytes / row_bytes) × kSampleIntervalMs / 1000
// With defaults (8 KB, ~140 B/row, 200 ms) → ~11–12 s of rolling history.
// Must be ≥ csv_log::kMaxRowBytes (192) so a single row always fits.
constexpr std::size_t kBufBytes = 8 * 1024;

// HTTP port for the diagnostics server. Must NOT collide with the WebSocket
// on 81 (see code/firmware/src/brain/network.cpp).
constexpr int kHttpPort = 80;

// =========================================================================

// One-time setup. Starts the WebServer on kHttpPort and registers routes.
// Must be called AFTER initNetwork() so WiFi is up.
void begin(SpinalCord& sc);

// Rate-limited row sampler. Cheap — does nothing until the interval has elapsed.
// Call once per loop, AFTER spinalCord.update().
void tick();

// Pumps the HTTP server. Non-blocking. Call once per loop.
void handleHttp();

} // namespace diagnostics

#endif
