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
#include <cstdint>
#include <cstdio>
#include <string>

namespace fh_sim {
// The single source of "firmware time". The bridge writes this each tick.
inline uint32_t clock_ms = 0;
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

// Serial: discard everything. printf is variadic with a format string.
struct _FhFakeSerial {
    void begin(long) {}
    template <typename... A>
    void printf(const char*, A...) {}
    template <typename T>
    void print(T) {}
    template <typename T>
    void println(T) {}
    void println() {}
};
inline _FhFakeSerial Serial;

// The firmware's command method takes an Arduino String; std::string supports
// the only operations it uses (==/!= against string literals).
using String = std::string;
