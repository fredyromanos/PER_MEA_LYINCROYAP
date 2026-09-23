#pragma once
#include <Arduino.h>

// Minimal subset of the firmware config/BoardConfig.h needed by the GPS test.
namespace BoardConfig {

// I2C — AXP192 power manager (T-Beam internal bus, GPIO21/22)
static constexpr uint8_t I2C_SDA_PIN = 21;
static constexpr uint8_t I2C_SCL_PIN = 22;

// GPS UART — Serial1
static constexpr uint8_t  GPS_RX_PIN    = 34;   // input-only GPIO
static constexpr uint8_t  GPS_TX_PIN    = 12;
static constexpr uint32_t GPS_BAUD_RATE = 9600;

} // namespace BoardConfig
