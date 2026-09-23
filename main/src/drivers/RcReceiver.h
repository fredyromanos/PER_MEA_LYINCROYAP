#pragma once
#include "../core/Types.h"
#include <Arduino.h>  // pulls in FreeRTOS portMUX_TYPE and IRAM_ATTR for ESP32

// Reads 4 RC PWM channels via GPIO edge interrupts.
// Each channel value is a pulse width in microseconds (1000–2000).
// Returns 0 for any channel whose signal has been absent > RC_TIMEOUT_US.
class RcReceiver {
public:
    void    begin();
    RcFrame readFrame() const;

private:
    static constexpr uint8_t N = 4;  // CH2..CH5

    enum class Ch : uint8_t { CH2 = 0, CH3, CH4, CH5 };

    volatile uint32_t riseUs_[N]      = {};
    volatile uint16_t pulseUs_[N]     = {};
    volatile uint32_t lastValidUs_[N] = {};
    portMUX_TYPE      mux_            = portMUX_INITIALIZER_UNLOCKED;

    static RcReceiver* instance_;

    void     IRAM_ATTR handleEdge(uint8_t pin, Ch ch);
    uint16_t           getChannel(Ch ch) const;

    static void IRAM_ATTR isrCH2();
    static void IRAM_ATTR isrCH3();
    static void IRAM_ATTR isrCH4();
    static void IRAM_ATTR isrCH5();
};
