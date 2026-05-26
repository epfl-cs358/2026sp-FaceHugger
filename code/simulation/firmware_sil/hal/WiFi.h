// firmware_sil/hal/WiFi.h
//
// Host stub for <WiFi.h> so the firmware's network.cpp compiles (we compile its
// EXACT command dispatch; the real WiFi AP is irrelevant on the desktop). Just
// enough surface for initNetwork() to build — it is never called in the sim.
#pragma once

#include <cstdint>
#include <string>

class IPAddress {
   public:
    IPAddress() = default;
    std::string toString() const { return std::string("0.0.0.0"); }
};

enum WiFiModeStub { WIFI_OFF = 0, WIFI_STA = 1, WIFI_AP = 2, WIFI_AP_STA = 3 };

class _FhWiFiStub {
   public:
    void mode(int) {}
    bool softAP(const char*, const char*, int = 1, int = 0, int = 4) { return true; }
    IPAddress softAPIP() { return IPAddress(); }
};

inline _FhWiFiStub WiFi;
