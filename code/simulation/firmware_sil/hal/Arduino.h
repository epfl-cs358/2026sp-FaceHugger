// firmware_sil/hal/Arduino.h
//
// Minimal host shim for <Arduino.h>, placed FIRST on the include path so it
// shadows any real Arduino header when the EXACT firmware sources are compiled
// for the desktop (SIL). It provides only what the compiled control files
// actually use — verified surface: millis(), map(), constrain(), Serial.printf,
// and the String type (used by an uncalled command method). Zero firmware
// changes; the firmware .cpp files are compiled unmodified against this.
//
// millis() is the determinism hinge: it returns an INJECTED sim clock
// (fh_sim::clock_ms), set by the pybind11 bridge before each tick — never the
// host wall clock. (The SIL plan's D1 names ByteNana/ArduinoMock for a fuller
// Arduino surface; for the control files this hand-rolled shim is the smaller,
// network-free, deterministic equivalent.)
#pragma once

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace fh_sim {
// The single source of "firmware time". The bridge writes this each tick.
inline uint32_t clock_ms = 0;

// IMU state injected by the SIL bridge from PyBullet's body orientation. The
// bridge applies the same hysteresis as the firmware (flip>150°, clear<30°)
// before writing imu_upside_down; pitch_deg / roll_deg are the raw angles so
// the T:10 telemetry surface can show them. sensors_sil.cpp reads these and
// returns them through the firmware's sensors.h surface (imuIsInverted etc).
inline bool  imu_upside_down = false;
inline float pitch_deg       = 0.0f;
inline float roll_deg        = 0.0f;

// Captured Serial output, split into complete lines. The firmware's own
// Serial.printf warnings (e.g. "[OOR] servo N requested X") land here so the
// SIL can surface them via the binding — we capture the firmware's reports, we
// don't manufacture them.
inline std::vector<std::string> serial_lines;
inline std::string serial_partial;

inline void serial_write(const char* s) {
    for (; *s; ++s) {
        if (*s == '\n') {
            serial_lines.push_back(serial_partial);
            serial_partial.clear();
        } else {
            serial_partial += *s;
        }
    }
    if (serial_lines.size() > 8192)  // bound memory on long runs
        serial_lines.erase(serial_lines.begin(), serial_lines.begin() + 4096);
}
}  // namespace fh_sim

inline uint32_t millis() { return fh_sim::clock_ms; }
inline uint32_t micros() { return fh_sim::clock_ms * 1000u; }
inline void delay(uint32_t) {}
inline void delayMicroseconds(uint32_t) {}

// Arduino's integer map() — exact semantics (truncating long arithmetic), so
// servo.cpp's degree->pulse map matches the firmware bit-for-bit.
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Arduino constrain() is a macro.
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

// PROGMEM / F(): no-ops on the host (flash-string placement is an MCU concern).
// The firmware's OLED sprite tables (eye_sprites.h) and F("...") logs use these.
#ifndef PROGMEM
#define PROGMEM
#endif
#ifndef F
#define F(string_literal) (string_literal)
#endif

// Serial: format like the real Arduino Serial and capture into fh_sim::serial_lines
// (so the firmware's own warnings are observable), instead of going to a UART.
struct _FhFakeSerial {
    void begin(long) {}
    void printf(const char* fmt, ...) {
        char buf[512];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        fh_sim::serial_write(buf);
    }
    void print(const char* s) { fh_sim::serial_write(s); }
    template <typename T>
    void print(T) {}
    void println(const char* s) {
        fh_sim::serial_write(s);
        fh_sim::serial_write("\n");
    }
    template <typename T>
    void println(T) {
        fh_sim::serial_write("\n");
    }
    void println() { fh_sim::serial_write("\n"); }
};
inline _FhFakeSerial Serial;

// The firmware's command method takes an Arduino String; std::string supports
// the only operations it uses (==/!= against string literals).
using String = std::string;
