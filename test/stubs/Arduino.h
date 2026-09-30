#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Host-side Arduino stub for native unit tests (no ESP32, no Arduino core).
//
// Provides just enough of the Arduino surface for the pure-logic firmware
// units to compile and run on a PC:
//   • fixed-width integer types
//   • a no-op `Serial` object (satisfies DBG_* macros in DebugConfig.h)
//   • millis()/micros()/delay() shims
//
// The firmware source is NOT modified for testing — this stub is the seam.
// ─────────────────────────────────────────────────────────────────────────────
#include <stdint.h>
#include <stddef.h>
#include <cstdarg>

// --- no-op Serial (DBG_CTRL/DBG_GPS/... expand to Serial.printf) --------------
struct HostSerial {
    void  begin(unsigned long = 0) {}
    // Accept any printf-style call; discard output to keep tests quiet.
    int   printf(const char*, ...) { return 0; }
    void  print(const char*)   {}
    void  print(long)          {}
    void  println(const char* = "") {}
    void  println(long)        {}
    void  write(const uint8_t*, size_t) {}
    int   available()          { return 0; }
    int   read()               { return -1; }
    explicit operator bool() const { return true; }
};
// [[maybe_unused]]: DBG_* macros that would reference this are compile-time
// gated (DebugConfig.h), so plenty of TUs never touch it — that's expected,
// not a real unused-variable bug, and must not warn under -Wall -Wextra.
[[maybe_unused]] static HostSerial Serial;

// --- ADC surface (for BatteryAdc host tests) ---------------------------------
// Declared here so driver .cpp compiles; DEFINED by the test TU (test_hw.cpp),
// which lets a test feed a controllable millivolt reading through the real code.
#define ADC_11db 3
void     analogSetPinAttenuation(uint8_t pin, int attenuation);
uint32_t analogReadMilliVolts(uint8_t pin);

// --- timing shims -------------------------------------------------------------
static inline unsigned long millis() { return 0UL; }
static inline unsigned long micros() { return 0UL; }
static inline void delay(unsigned long) {}
static inline void delayMicroseconds(unsigned int) {}
