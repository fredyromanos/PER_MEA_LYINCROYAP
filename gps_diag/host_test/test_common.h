// Shared helpers: pass/fail reporting with test IDs, NMEA builder with computed checksum.
#pragma once
#include <cstdio>
#include <string>
#include <cmath>

unsigned long fake_millis = 0;
static int g_pass = 0, g_fail = 0;

inline void check(bool ok, const char* id, const char* what) {
    std::printf("[%s] %-6s %s\n", ok ? "PASS" : "FAIL", id, what);
    ok ? ++g_pass : ++g_fail;
}
inline int finish() {
    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
inline std::string nmea(const std::string& body) {
    unsigned char cs = 0;
    for (char c : body) cs ^= (unsigned char)c;
    char tail[8];
    std::snprintf(tail, sizeof tail, "*%02X\r\n", cs);
    return "$" + body + tail;
}
// lat/lon in NMEA ddmm.mmmmm format for a decimal position.
inline std::string nmeaLat(double lat) {
    char b[32]; double a = std::fabs(lat); int d = (int)a;
    std::snprintf(b, sizeof b, "%02d%08.5f,%c", d, (a - d) * 60.0, lat < 0 ? 'S' : 'N'); return b;
}
inline std::string nmeaLon(double lon) {
    char b[32]; double a = std::fabs(lon); int d = (int)a;
    std::snprintf(b, sizeof b, "%03d%08.5f,%c", d, (a - d) * 60.0, lon < 0 ? 'W' : 'E'); return b;
}
inline std::string rmc(char status, double lat, double lon, const char* spdKn, const char* crs) {
    return nmea("GPRMC,101500.00," + std::string(1, status) + "," + nmeaLat(lat) + "," +
                nmeaLon(lon) + "," + spdKn + "," + crs + ",160926,,,A");
}
inline std::string gga(char quality, double lat, double lon, const char* sats) {
    return nmea("GPGGA,101500.00," + nmeaLat(lat) + "," + nmeaLon(lon) + "," +
                std::string(1, quality) + "," + sats + ",1.20,40.0,M,50.0,M,,");
}
