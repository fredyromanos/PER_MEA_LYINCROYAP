// ─────────────────────────────────────────────────────────────────────────────
// Hardware-input SEAM tests — validate the pure math inside a hardware driver
// by supplying the ADC primitives ourselves (the mock seam) and feeding a
// controllable reading through the REAL BatteryAdc::readVolts() code path.
//
// This is the template for host-testing any HW-coupled driver: stub the
// peripheral call, drive the real conversion logic, assert the contract.
// Covered: battery voltage divider R5=562k / R6=120k → ratio 5.6833×.
// ─────────────────────────────────────────────────────────────────────────────
#include <cstdio>
#include <cstdint>
#include <cmath>

#include "drivers/BatteryAdc.h"

// ── seam: definitions the driver links against on host ───────────────────────
static uint32_t g_fake_mV = 0;  // test-controlled ADC millivolt reading
void     analogSetPinAttenuation(uint8_t, int) {}
uint32_t analogReadMilliVolts(uint8_t)          { return g_fake_mV; }

// ── tiny harness ─────────────────────────────────────────────────────────────
static int g_pass = 0, g_fail = 0;
#define CHECK_NEAR(label, got, want, tol)                               \
    do {                                                                \
        double _g=(got), _w=(want), _t=(tol);                           \
        if (std::fabs(_g-_w) <= _t) { printf("  PASS  %s  (%.4f)\n", label, _g); g_pass++; } \
        else { printf("  FAIL  %s  got=%.4f want=%.4f  (line %d)\n", label,_g,_w,__LINE__); g_fail++; } \
    } while (0)

int main() {
    printf("=== SeaDrone HW-seam tests (BatteryAdc) ===\n\n── BatteryAdc::readVolts divider math\n");
    BatteryAdc bat;
    bat.begin();  // no-op on host

    const double ratio = (562000.0 + 120000.0) / 120000.0;  // 5.68333…

    g_fake_mV = 0;      CHECK_NEAR("0 mV → 0 V",            bat.readVolts(), 0.0, 1e-4);
    g_fake_mV = 1000;   CHECK_NEAR("1000 mV → ratio V",    bat.readVolts(), ratio, 1e-3);
    g_fake_mV = 1302;   CHECK_NEAR("1302 mV → ~7.40 V",    bat.readVolts(), 1.302 * ratio, 1e-3);
    g_fake_mV = 1232;   CHECK_NEAR("1232 mV → ~7.00 V",    bat.readVolts(), 7.00, 0.02);

    printf("\n=== Results: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail > 0) ? 1 : 0;
}
