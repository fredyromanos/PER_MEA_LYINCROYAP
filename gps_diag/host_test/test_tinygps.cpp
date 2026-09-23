// Host-side proof of TinyGPSPlus behaviours the drone firmware relies on.
// Uses the REAL library source; only millis() is simulated.
#include <cstdio>
#include <string>
#include <new>
#include <cstring>
#include "TinyGPS++.h"

unsigned long fake_millis = 0;
static int failures = 0, passes = 0;

static void check(bool ok, const char* name) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    ok ? ++passes : ++failures;
}

// Build a sentence with a correct checksum, so no hand-computed checksum can be wrong.
static std::string nmea(const std::string& body) {
    unsigned char cs = 0;
    for (char c : body) cs ^= (unsigned char)c;
    char tail[8];
    std::snprintf(tail, sizeof tail, "*%02X\r\n", cs);
    return "$" + body + tail;
}

static void feed(TinyGPSPlus& g, const std::string& s) { for (char c : s) g.encode(c); }

// Real field layout: RMC,time,status,lat,N,lon,W,speedKnots,courseDeg,date,,,mode
static std::string rmc(char status, const char* spd, const char* crs) {
    return nmea(std::string("GPRMC,101500.00,") + status +
                ",4821.62490,N,00433.99598,W," + spd + "," + crs + ",160926,,,A");
}
static std::string gga(char quality, const char* sats) {
    return nmea(std::string("GPGGA,101500.00,4821.62490,N,00433.99598,W,") + quality +
                "," + sats + ",1.20,40.0,M,50.0,M,,");
}

int main() {
    // --- Checksum builder sanity: known-good NEO-6M sentence from our own test log ---
    check(nmea("GPGLL,,,,,,V,N") == "$GPGLL,,,,,,V,N*64\r\n",
          "checksum builder reproduces real captured sentence $GPGLL,,,,,,V,N*64");

    // --- 1. Before any fix, location is invalid ---
    {
        TinyGPSPlus g; fake_millis = 1000;
        feed(g, rmc('V', "", ""));
        check(!g.location.isValid(), "1. no fix yet -> location.isValid() == false");
    }

    // --- 2. Location stays valid after the fix is LOST (RMC status V) ---
    {
        TinyGPSPlus g; fake_millis = 1000;
        feed(g, rmc('A', "0.10", "45.00"));
        bool validAfterFix = g.location.isValid();
        fake_millis = 31000;                       // 30 s later, receiver lost the fix
        feed(g, rmc('V', "", ""));
        feed(g, gga('0', "00"));
        check(validAfterFix, "2a. RMC status A -> location valid");
        check(g.location.isValid(),
              "2b. after fix LOST (RMC V + GGA 0) location.isValid() is STILL true");
        check(g.location.age() == 30000,
              "2c. age() keeps growing (30000 ms) -> age is the only loss signal");
    }

    // --- 3. Empty course field keeps the PREVIOUS course and marks it valid ---
    {
        TinyGPSPlus g; fake_millis = 1000;
        feed(g, rmc('A', "5.00", "123.40"));
        double first = g.course.deg();
        fake_millis = 2000;
        feed(g, rmc('A', "", ""));                 // stopped: receiver blanks speed/course
        check(first > 123.39 && first < 123.41, "3a. moving: course = 123.4");
        check(g.course.isValid() && g.course.deg() > 123.39 && g.course.deg() < 123.41,
              "3b. stopped (empty field): course STILL 123.4 and valid (stale value)");
        check(g.course.age() == 0, "3c. course age reset to 0 -> looks perfectly fresh");
        check(g.speed.kmph() > 9.25 && g.speed.kmph() < 9.27,
              "3d. empty speed field keeps previous speed 5 kn = 9.26 km/h");
    }

    // --- 4. Empty course from boot: valid, but the value was never received ---
    // TinyGPSDecimal's constructor initialises `val` but NOT `newval`, so the first
    // commit with an empty field copies whatever memory held.
    // 4a mirrors the firmware: `static DroneApp app;` / global `GpsUart gps;`
    //    -> static storage is zeroed -> 0.0 (= fake north).
    {
        static TinyGPSPlus g; fake_millis = 1000;
        feed(g, rmc('A', "", ""));
        check(g.course.isValid() && g.course.deg() == 0.0 && g.speed.kmph() == 0.0,
              "4a. static object (as in firmware), never moved: course valid, 0.0 = fake north");
    }
    // 4b. Proof `newval` is not set by the constructor: pre-fill storage with 0x7F,
    //     construct in place, commit an empty course -> the 0x7F bytes come out as "valid".
    {
        static unsigned char mem[sizeof(TinyGPSPlus)];
        std::memset(mem, 0x7F, sizeof mem);
        TinyGPSPlus* g = new (mem) TinyGPSPlus(); fake_millis = 1000;
        feed(*g, rmc('A', "", ""));
        check(g->course.isValid() && g->course.value() == 0x7F7F7F7F,
              "4b. constructor leaves newval uninitialised: garbage committed as a VALID course");
        g->~TinyGPSPlus();
    }

    // --- 5. Satellites / HDOP from GGA commit even WITHOUT a fix ---
    {
        TinyGPSPlus g; fake_millis = 1000;
        feed(g, gga('0', "03"));
        check(g.satellites.isValid() && g.satellites.value() == 3 && !g.location.isValid(),
              "5. GGA quality 0: satellites valid (3) while location invalid");
    }

    // --- 6. isUpdated() distinguishes a new sample from a re-read ---
    {
        TinyGPSPlus g; fake_millis = 1000;
        feed(g, rmc('A', "5.00", "90.00"));
        bool u1 = g.course.isUpdated();
        (void)g.course.deg();                      // reading clears the flag
        bool u2 = g.course.isUpdated();
        check(u1 && !u2, "6. isUpdated(): true on new RMC, false after read (no new sample)");
    }

    // --- 7. Corrupted sentence is rejected ---
    {
        TinyGPSPlus g; fake_millis = 1000;
        std::string bad = rmc('A', "5.00", "90.00");
        bad[bad.size() - 4] = (bad[bad.size() - 4] == '0') ? '1' : '0';   // break checksum
        feed(g, bad);
        check(!g.location.isValid() && g.failedChecksum() == 1,
              "7. bad checksum -> not committed, failedChecksum() == 1");
    }

    std::printf("\n%d passed, %d failed\n", passes, failures);
    return failures ? 1 : 0;
}
