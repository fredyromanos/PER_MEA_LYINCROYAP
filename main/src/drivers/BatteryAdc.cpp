#include "BatteryAdc.h"
#include "../config/BoardConfig.h"
#include "../config/DebugConfig.h"

// R5=562kΩ (high side, battery to ADC pin), R6=120kΩ (low side to GND)
// V_bat = V_pin × (R5+R6)/R6
static constexpr float DIVIDER_RATIO = (562000.0f + 120000.0f) / 120000.0f;

void BatteryAdc::begin() {
    analogSetPinAttenuation(BoardConfig::BATTERY_ADC_PIN, ADC_11db);
    DBG_BAT("ADC init GPIO%d attn=11dB divider=%.2fx",
        BoardConfig::BATTERY_ADC_PIN, DIVIDER_RATIO);
}

float BatteryAdc::readVolts() {
    uint32_t vPin_mV = analogReadMilliVolts(BoardConfig::BATTERY_ADC_PIN);
    return (vPin_mV * DIVIDER_RATIO) / 1000.0f;
}
