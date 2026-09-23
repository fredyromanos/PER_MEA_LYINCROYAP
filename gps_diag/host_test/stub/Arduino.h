// Minimal Arduino stub so real firmware / library sources compile on a PC.
// The clock is simulated (fake_millis, advanced by delay()); serial ports record traffic.
#pragma once
#include <algorithm>
#include <cctype>
#include <climits>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

typedef uint8_t byte;
extern unsigned long fake_millis;
inline unsigned long millis() { return fake_millis; }
inline void delay(unsigned long ms) { fake_millis += ms; }

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI  6.283185307179586476925286766559
#define radians(deg) ((deg) * PI / 180.0)
#define degrees(rad) ((rad) * 180.0 / PI)
#define sq(x) ((x) * (x))
#define SERIAL_8N1 0x800001c

class String {
public:
    String(const char* s = "") : s_(s ? s : "") {}
    String(const std::string& s) : s_(s) {}
    String(int v) : s_(std::to_string(v)) {}
    String(unsigned v) : s_(std::to_string(v)) {}
    String(long v) : s_(std::to_string(v)) {}
    String(unsigned long v) : s_(std::to_string(v)) {}
    String(double v, int decimals = 2) {
        char b[64]; std::snprintf(b, sizeof b, "%.*f", decimals, v); s_ = b;
    }
    const char* c_str() const { return s_.c_str(); }
    String operator+(const String& o) const { return String(s_ + o.s_); }
    friend String operator+(const char* a, const String& b) { return String(std::string(a) + b.s_); }
private:
    std::string s_;
};

// Records everything written; feed() queues bytes to be read.
class HardwareSerial {
public:
    std::deque<uint8_t> rx;
    std::vector<uint8_t> tx;
    std::string text;                          // printed text (print/println/printf)
    bool begun = false;
    void begin(unsigned long, uint32_t = SERIAL_8N1, int = -1, int = -1) { begun = true; }
    int available() { return (int)rx.size(); }
    int read() { if (rx.empty()) return -1; int c = rx.front(); rx.pop_front(); return c; }
    size_t write(uint8_t c) { tx.push_back(c); return 1; }
    size_t write(const uint8_t* p, size_t n) { tx.insert(tx.end(), p, p + n); return n; }
    void feed(const std::string& s) { for (unsigned char c : s) rx.push_back(c); }
    void print(const char* s) { text += s; }
    void print(const String& s) { text += s.c_str(); }
    void print(double v, int d = 2) { text += String(v, d).c_str(); }
    void print(int v) { text += std::to_string(v); }
    void println() { text += "\n"; }
    void println(const char* s) { text += s; text += "\n"; }
    void println(const String& s) { text += s.c_str(); text += "\n"; }
    void println(double v, int d = 2) { text += String(v, d).c_str(); text += "\n"; }
    void println(int v) { text += std::to_string(v) + "\n"; }
    int printf(const char* fmt, ...) {
        char b[512]; va_list ap; va_start(ap, fmt);
        int n = std::vsnprintf(b, sizeof b, fmt, ap); va_end(ap); text += b; return n;
    }
};
extern HardwareSerial Serial;
extern HardwareSerial Serial1;
