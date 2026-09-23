// Tests the parsing helpers used by the board sketch (same header, same code).
#include <cstdio>
#include <string>
#include "../gps_diag_board/diag_util.h"
#include "TinyGPS++.h"

unsigned long fake_millis = 0;
static int failures = 0, passes = 0;
static void check(bool ok, const char* name) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    ok ? ++passes : ++failures;
}
static std::string field(const char* s, int i) { char b[32]; diag::nmeaField(s, i, b, sizeof b); return b; }

int main() {
    using namespace diag;
    // Real sentences captured from our own board today.
    check(nmeaChecksumOk("$GPGLL,,,,,,V,N*64"), "checksum OK on captured $GPGLL,,,,,,V,N*64");
    check(nmeaChecksumOk("$GPGGA,,,,,,0,00,99.99,,,,,,*48"), "checksum OK on captured $GPGGA...*48");
    check(nmeaChecksumOk("$GPTXT,01,01,02,ANTSTATUS=OK*3B") ==
          ([]{ unsigned char c=0; for(const char*p="GPTXT,01,01,02,ANTSTATUS=OK";*p;++p)c^=*p; return c==0x3B; })(),
          "checksum agrees with independent XOR on ANTSTATUS=OK");
    check(!nmeaChecksumOk("$GPGLL,,,,,,V,N*65"), "wrong checksum rejected");
    check(!nmeaChecksumOk("$GPGLL,,,,,,V,N"),    "missing checksum rejected");
    check(!nmeaChecksumOk("$GPGLL,,,,,,V,N*6"),  "truncated checksum rejected");

    const char* rmcEmpty = "$GPRMC,101500.00,A,4821.62490,N,00433.99598,W,,,160926,,,A*00";
    check(field(rmcEmpty, 2) == "A", "RMC field 2 = status");
    check(field(rmcEmpty, 7) == "",  "RMC field 7 empty speed -> empty string");
    check(field(rmcEmpty, 8) == "",  "RMC field 8 empty course -> empty string");
    const char* rmcMove = "$GPRMC,101500.00,A,4821.62490,N,00433.99598,W,0.512,123.40,160926,,,A*00";
    check(field(rmcMove, 7) == "0.512" && field(rmcMove, 8) == "123.40", "RMC speed/course read correctly");
    check(field("$GPGGA,101500.00,4821.6,N,00433.9,W,1,07,1.20,40.0,M,50.0,M,,*00", 6) == "1" &&
          field("$GPGGA,101500.00,4821.6,N,00433.9,W,1,07,1.20,40.0,M,50.0,M,,*00", 7) == "07" &&
          field("$GPGGA,101500.00,4821.6,N,00433.9,W,1,07,1.20,40.0,M,50.0,M,,*00", 8) == "1.20",
          "GGA quality/sats/hdop read correctly");
    check(field("$GPRMC,1*7F", 9) == "", "field past end -> empty");
    char tiny[4]; nmeaField(rmcMove, 8, tiny, sizeof tiny);
    check(std::string(tiny) == "123", "long field truncated safely to buffer");

    check(isType("$GPRMC,x", "RMC") && isType("$GNRMC,x", "RMC"), "isType accepts GP and GN talkers");
    check(!isType("$GPRMCX,", "RMC") && !isType("$GPGGA,", "RMC") && !isType("$GP", "RMC"),
          "isType rejects wrong / short ids");

    uint8_t poll[8]; buildNav5Poll(poll);
    check(poll[6] == 0x2A && poll[7] == 0x84, "NAV5 poll checksum = 2A 84 (u-blox reference value)");

    // CFG-NAV5 answer: 36-byte payload, dynModel at offset 2.
    auto nav5 = [](uint8_t model, bool corrupt) {
        std::string m = {char(0xB5), char(0x62), 0x06, 0x24, 36, 0};
        std::string pl(36, '\0'); pl[0] = char(0xFF); pl[1] = char(0xFF); pl[2] = char(model); pl[3] = 3;
        m += pl;
        uint8_t a, b; ubxChecksum((const uint8_t*)m.data() + 2, m.size() - 2, a, b);
        m += char(a); m += char(corrupt ? b ^ 1 : b);
        return m;
    };
    { UbxNav5Reader r; for (char c : nav5(5, false)) r.feed((uint8_t)c);
      check(r.dynModel == 5, "UBX reader extracts dynModel=5 (sea)"); }
    { UbxNav5Reader r; for (char c : nav5(0, false)) r.feed((uint8_t)c);
      check(r.dynModel == 0, "UBX reader extracts dynModel=0 (portable)"); }
    { UbxNav5Reader r; for (char c : nav5(5, true)) r.feed((uint8_t)c);
      check(r.dynModel == -1, "corrupted UBX checksum ignored"); }
    { UbxNav5Reader r; std::string noise = "$GPGGA,,,,,,0,00,99.99,,,,,,*48\r\n";
      std::string s = noise + nav5(4, false) + noise; for (char c : s) r.feed((uint8_t)c);
      check(r.dynModel == 4, "UBX answer found inside NMEA stream"); }
    { UbxNav5Reader r; std::string ack = {char(0xB5), char(0x62), 0x05, 0x01, 2, 0, 0x06, 0x24};
      uint8_t a, b; ubxChecksum((const uint8_t*)ack.data() + 2, ack.size() - 2, a, b); ack += char(a); ack += char(b);
      for (char c : ack) r.feed((uint8_t)c);
      check(r.dynModel == -1, "ACK-ACK is not mistaken for a NAV5 answer"); }

    // Sketch prints the D line on '\n'. TinyGPSPlus must have committed by then.
    {
        TinyGPSPlus g; fake_millis = 5000;
        std::string body = "GPRMC,101500.00,A,4821.62490,N,00433.99598,W,1.0,90.0,160926,,,A";
        unsigned char cs = 0; for (char c : body) cs ^= c;
        char tail[8]; std::snprintf(tail, sizeof tail, "*%02X\r", cs);
        for (char c : "$" + body + tail) g.encode(c);   // everything up to and incl. '\r'
        check(g.location.isValid() && g.course.isUpdated(),
              "TinyGPSPlus committed at '\\r', before the sketch reacts to '\\n'");
    }

    std::printf("\n%d passed, %d failed\n", passes, failures);
    return failures ? 1 : 0;
}
