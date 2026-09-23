#pragma once
#include <Arduino.h>

namespace BoardConfig {

// I2C — AXP192 power manager (T-Beam internal bus, GPIO21/22)
// GPIO21 and GPIO22 remain dedicated to I2C; they are also wired to an
// expansion connector on the V2 PCB for future external I2C peripherals.
static constexpr uint8_t I2C_SDA_PIN = 21;
static constexpr uint8_t I2C_SCL_PIN = 22;

// RC receiver inputs — interrupt-driven PWM (Pro-Tronik R8X)
static constexpr uint8_t RX_CH2_PIN = 39;  // sail toggle       (GPIO39 SVN, input-only)
static constexpr uint8_t RX_CH3_PIN = 14;  // propulsion throttle
static constexpr uint8_t RX_CH4_PIN = 13;  // rotor winch
static constexpr uint8_t RX_CH5_PIN =  4;  // mode selector (3-pos switch)

// Actuator PWM outputs — MCPWM only, never LEDC (LEDC breaks RC interrupts)
static constexpr uint8_t SAIL_SERVO_PIN  =  2;  // PWM1
static constexpr uint8_t ROTOR_SERVO_PIN = 25;  // PWM2
static constexpr uint8_t ESC1_PIN        = 15;  // single ESC

// Battery ADC — external voltage divider (R2=562kΩ high-side, R1=120kΩ low-side)
// GPIO36 (SVP, ADC1_CH0, input-only) — no continuous drain unlike GPIO35.
static constexpr uint8_t BATTERY_ADC_PIN = 36;

// GPS UART — Serial1 (future use)
static constexpr uint8_t  GPS_RX_PIN    = 34;   // input-only GPIO
static constexpr uint8_t  GPS_TX_PIN    = 12;
static constexpr uint32_t GPS_BAUD_RATE = 9600;

// LoRa SPI — SX1276 on HSPI
static constexpr uint8_t LORA_SCK_PIN  =  5;
static constexpr uint8_t LORA_MISO_PIN = 19;
static constexpr uint8_t LORA_MOSI_PIN = 27;
static constexpr uint8_t LORA_CS_PIN   = 18;
static constexpr uint8_t LORA_RST_PIN  = 23;  // GPIO23 now free (CH6 removed)
static constexpr uint8_t LORA_IRQ_PIN  = 26;

// RC signal validation
static constexpr uint16_t RC_VALID_MIN_US = 800;
static constexpr uint16_t RC_VALID_MAX_US = 2200;
static constexpr uint32_t RC_TIMEOUT_US   = 100000;  // 100 ms without pulse = signal lost

// RC command range (Pro-Tronik PTR-6A)
static constexpr uint16_t RC_MIN_US     = 1000;
static constexpr uint16_t RC_MID_US     = 1500;
static constexpr uint16_t RC_MAX_US     = 2000;
static constexpr uint16_t RC_DEADBAND_US = 35;

// CH4 measured travel (rotor / safran stick) — measured on bench
// Center is ~1500 µs; full stick range is 1180–1790 µs.
static constexpr uint16_t CH4_MIN_US = 1180;
static constexpr uint16_t CH4_MAX_US = 1790;

// CH3 — propulsion BIDIRECTIONNELLE en mode manuel (manette à crans).
//   manche vers CH3_FULL_US (bas)  = +100 % marche AVANT
//   centre (CH3_CENTER_US)         =    0 % arrêt (zone morte)
//   manche vers CH3_ZERO_US (haut) = -100 % marche ARRIÈRE
static constexpr uint16_t CH3_FULL_US   = 1100;  // extrémité avant (+100 %)
static constexpr uint16_t CH3_ZERO_US   = 1990;  // extrémité arrière (-100 %)
static constexpr uint16_t CH3_CENTER_US = 1545;  // neutre = milieu de course — À VÉRIFIER au banc

// Mode selector thresholds — CH5 three-position switch (PTR-6A outputs ~1000/1500/2000 µs)
// ≤1250 → Automatic | 1400–1600 → Sail (inerte) | >1800 → Manual
static constexpr uint16_t CH5_AUTO_THRESHOLD      = 1250;  // at or below → Automatic
static constexpr uint16_t CH5_SAIL_LOW_US         = 1400;  // Sail band low
static constexpr uint16_t CH5_SAIL_HIGH_US        = 1600;  // Sail band high
static constexpr uint16_t CH5_PROP_THRESHOLD      = 1800;  // above → Manual

} // namespace BoardConfig
