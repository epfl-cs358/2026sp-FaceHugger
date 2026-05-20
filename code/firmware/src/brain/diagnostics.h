#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

class SpinalCord;

namespace diagnostics {

// Sample interval and HTTP port — adjust here if needed.
constexpr unsigned long kSampleIntervalMs = 200;
constexpr int           kHttpPort         = 80;

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
