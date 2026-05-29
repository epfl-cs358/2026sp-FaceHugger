// firmware_sil/hal/WebSocketsServer.h
//
// Host stub for the links2004 <WebSocketsServer.h> so the firmware's network.cpp
// compiles unchanged. The real transport is the Python `websockets` server
// (ws_sim.py); here we only need the type surface network.cpp references, plus a
// capture of sendTXT() so the binding can return the firmware's reply (e.g. the
// T:8 clip list built by the real buildClipListJson()).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "WiFi.h"  // IPAddress

enum WStype_t {
    WStype_ERROR,
    WStype_DISCONNECTED,
    WStype_CONNECTED,
    WStype_TEXT,
    WStype_BIN,
    WStype_FRAGMENT_TEXT_START,
    WStype_FRAGMENT_BIN_START,
    WStype_FRAGMENT,
    WStype_FRAGMENT_FIN,
    WStype_PING,
    WStype_PONG,
};

typedef void (*WebSocketServerEvent)(uint8_t, WStype_t, uint8_t*, size_t);

namespace fh_sim {
// Last text the firmware "sent" via sendTXT — read back by the binding.
inline std::string ws_last_txt;
}  // namespace fh_sim

class WebSocketsServer {
   public:
    explicit WebSocketsServer(uint16_t) {}
    void begin() {}
    void onEvent(WebSocketServerEvent) {}
    void loop() {}
    void sendTXT(uint8_t, const char* payload) { fh_sim::ws_last_txt = payload; }
    void sendTXT(uint8_t, const std::string& payload) { fh_sim::ws_last_txt = payload; }
    // broadcastTXT — used by network.cpp's T:10 telemetry broadcast. The Python
    // ws_sim layer publishes telemetry differently; here we just capture the
    // payload (overwriting ws_last_txt) so the firmware's broadcast call
    // compiles and tests can introspect what it would have sent.
    void broadcastTXT(const char* payload, size_t n) {
        fh_sim::ws_last_txt.assign(payload, n);
    }
    void broadcastTXT(const char* payload) { fh_sim::ws_last_txt = payload; }
    void broadcastTXT(const std::string& payload) { fh_sim::ws_last_txt = payload; }
    IPAddress remoteIP(uint8_t) { return IPAddress(); }
};
