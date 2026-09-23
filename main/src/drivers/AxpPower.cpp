#include "AxpPower.h"
#include "../config/BoardConfig.h"
#include "../config/DebugConfig.h"
#include <Wire.h>
#include <axp20x.h>

static AXP20X_Class axp;

bool AxpPower::begin() {
    DBG_AXP("Wire.begin SDA=%d SCL=%d", BoardConfig::I2C_SDA_PIN, BoardConfig::I2C_SCL_PIN);
    Wire.begin(BoardConfig::I2C_SDA_PIN, BoardConfig::I2C_SCL_PIN);
    Wire.setClock(400000);

    if (axp.begin(Wire, AXP192_SLAVE_ADDRESS) != AXP_PASS) {
        DBG_AXP("axp.begin FAIL — check I2C bus");
        Wire.end();
        return false;
    }
    DBG_AXP("axp.begin OK");

    // LoRa SX1276 — LDO2 at 3.3 V
    axp.setLDO2Voltage(3300);
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    DBG_AXP("LDO2 3300mV ON  (LoRa SX1276)");

    // GPS NEO-6M — LDO3 at 3.3 V
    axp.setLDO3Voltage(3300);
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
    DBG_AXP("LDO3 3300mV ON  (GPS NEO-6M)");

    // 3.3 V board rail — DCDC1
    axp.setDCDC1Voltage(3300);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    DBG_AXP("DCDC1 3300mV ON (3.3V rail)");

    // Enable battery voltage ADC for future monitoring
    axp.adc1Enable(AXP202_BATT_VOL_ADC1 | AXP202_BATT_CUR_ADC1, true);
    DBG_AXP("battery ADC enabled");

    return true;
}
