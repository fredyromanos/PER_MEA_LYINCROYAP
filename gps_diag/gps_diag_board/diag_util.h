// Pure C++ helpers shared by the board sketch and the PC tests (no Arduino API).
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace diag {

inline void ubxChecksum(const uint8_t* p, size_t n, uint8_t& a, uint8_t& b) {
    a = b = 0;
    for (size_t i = 0; i < n; ++i) { a = (uint8_t)(a + p[i]); b = (uint8_t)(b + a); }
}

// Fills an 8-byte UBX-CFG-NAV5 poll message (checksum computed).
inline void buildNav5Poll(uint8_t out[8]) {
    const uint8_t head[8] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x00, 0x00};
    std::memcpy(out, head, 8);
    ubxChecksum(out + 2, 4, out[6], out[7]);
}

// Byte-wise UBX receiver; only extracts dynModel from a UBX-CFG-NAV5 answer.
struct UbxNav5Reader {
    int      dynModel = -1;        // -1 until a valid CFG-NAV5 answer is seen
    uint8_t  state = 0, cls = 0, id = 0, ckA = 0, ckB = 0;
    uint16_t len = 0, idx = 0;
    uint8_t  buf[64] = {};

    void feed(uint8_t c) {
        switch (state) {
        case 0: state = (c == 0xB5) ? 1 : 0; break;
        case 1: state = (c == 0x62) ? 2 : ((c == 0xB5) ? 1 : 0); break;
        case 2: cls = c; ckA = c; ckB = c; state = 3; break;
        case 3: id = c; ckA = (uint8_t)(ckA + c); ckB = (uint8_t)(ckB + ckA); state = 4; break;
        case 4: len = c; ckA = (uint8_t)(ckA + c); ckB = (uint8_t)(ckB + ckA); state = 5; break;
        case 5:
            len = (uint16_t)(len | (c << 8)); ckA = (uint8_t)(ckA + c); ckB = (uint8_t)(ckB + ckA);
            idx = 0;
            if (len > sizeof buf) { state = 0; break; }
            state = len ? 6 : 7;
            break;
        case 6:
            buf[idx++] = c; ckA = (uint8_t)(ckA + c); ckB = (uint8_t)(ckB + ckA);
            if (idx >= len) state = 7;
            break;
        case 7: state = (c == ckA) ? 8 : 0; break;
        case 8:
            if (c == ckB && cls == 0x06 && id == 0x24 && len == 36) dynModel = buf[2];
            state = 0;
            break;
        }
    }
};

inline int hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// True if `s` is "$...*HH" with a matching XOR checksum.
inline bool nmeaChecksumOk(const char* s) {
    if (!s || s[0] != '$') return false;
    uint8_t cs = 0;
    const char* p = s + 1;
    for (; *p && *p != '*'; ++p) cs ^= (uint8_t)*p;
    if (*p != '*') return false;
    const int hi = hexVal(p[1]), lo = p[1] ? hexVal(p[2]) : -1;
    if (hi < 0 || lo < 0) return false;
    return cs == (uint8_t)(hi * 16 + lo);
}

// Copies field `idx` (0 = "$GPRMC") of an NMEA line, stopping at '*'. Empty if absent.
inline void nmeaField(const char* s, int idx, char* out, size_t outSize) {
    int f = 0;
    size_t o = 0;
    for (const char* p = s; *p && *p != '*'; ++p) {
        if (*p == ',') { if (f == idx) break; ++f; continue; }
        if (f == idx && o + 1 < outSize) out[o++] = *p;
    }
    out[o] = '\0';
}

// Matches "$GPxxx," and "$GNxxx," (any talker "G?").
inline bool isType(const char* s, const char* type) {
    return s && s[0] == '$' && s[1] == 'G' && s[2] && std::strncmp(s + 3, type, 3) == 0 && s[6] == ',';
}

} // namespace diag
