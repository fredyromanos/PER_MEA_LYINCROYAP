// Ruled-out check R3: "Calcul dist IHM/test.cpp" (compiled unmodified, its main() renamed)
// converts to radians before the haversine formula, so the distance is correct.
#include "test_common.h"
#include <cmath>
#include <string>
using std::string;
double DMStoDecimals(const string& dms);
double decimalsToRad(double decimals);
double calculateDist(double lat1, double lon1, double lat2, double lon2);

int main() {
    const double lat = DMStoDecimals("48°32'55\"");
    check(std::fabs(lat - (48 + 32 / 60.0 + 55 / 3600.0)) < 1e-9, "R3a", "DMS 48d32'55\" parsed to 48.548611");
    // Brest (48.3904, -4.4861) -> Paris (48.8566, 2.3522): great-circle distance about 505 km.
    const double d = calculateDist(decimalsToRad(48.3904), decimalsToRad(-4.4861),
                                   decimalsToRad(48.8566), decimalsToRad(2.3522));
    char msg[120];
    std::snprintf(msg, sizeof msg, "Brest-Paris computed %.1f km (reference ~505 km): formula correct", d);
    check(std::fabs(d - 505.0) < 5.0, "R3b", msg);
    return finish();
}
