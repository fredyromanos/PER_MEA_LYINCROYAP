#include "RcReceiver.h"
#include "../config/BoardConfig.h"
#include "../config/DebugConfig.h"
#include "driver/gpio.h"

RcReceiver* RcReceiver::instance_ = nullptr;

static const uint8_t kPins[4] = {
    BoardConfig::RX_CH2_PIN,
    BoardConfig::RX_CH3_PIN,
    BoardConfig::RX_CH4_PIN,
    BoardConfig::RX_CH5_PIN,
};

void RcReceiver::begin() {
    instance_ = this;
    for (uint8_t i = 0; i < N; i++) {
        pinMode(kPins[i], INPUT);
        riseUs_[i] = lastValidUs_[i] = pulseUs_[i] = 0;
    }
    attachInterrupt(digitalPinToInterrupt(BoardConfig::RX_CH2_PIN), isrCH2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BoardConfig::RX_CH3_PIN), isrCH3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BoardConfig::RX_CH4_PIN), isrCH4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BoardConfig::RX_CH5_PIN), isrCH5, CHANGE);
    DBG_RC("ISRs attached: CH2=GPIO%d CH3=GPIO%d CH4=GPIO%d CH5=GPIO%d",
        BoardConfig::RX_CH2_PIN, BoardConfig::RX_CH3_PIN, BoardConfig::RX_CH4_PIN,
        BoardConfig::RX_CH5_PIN);
}

// NOTE: Serial must NOT be called from ISR context.
// handleEdge / isrCHx run in IRAM — no DBG here.
void IRAM_ATTR RcReceiver::handleEdge(uint8_t pin, Ch ch) {
    const uint32_t now = micros();
    const int      idx = static_cast<int>(ch);

    if (gpio_get_level(static_cast<gpio_num_t>(pin))) {
        portENTER_CRITICAL_ISR(&mux_);
        riseUs_[idx] = now;
        portEXIT_CRITICAL_ISR(&mux_);
    } else {
        uint32_t rise;
        portENTER_CRITICAL_ISR(&mux_);
        rise = riseUs_[idx];
        portEXIT_CRITICAL_ISR(&mux_);

        const uint32_t width = now - rise;
        if (width >= BoardConfig::RC_VALID_MIN_US && width <= BoardConfig::RC_VALID_MAX_US) {
            portENTER_CRITICAL_ISR(&mux_);
            pulseUs_[idx]    = static_cast<uint16_t>(width);
            lastValidUs_[idx] = now;
            portEXIT_CRITICAL_ISR(&mux_);
        }
    }
}

uint16_t RcReceiver::getChannel(Ch ch) const {
    const int      idx = static_cast<int>(ch);
    const uint32_t now = micros();
    uint16_t pw;
    uint32_t last;

    portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&mux_));
    pw   = pulseUs_[idx];
    last = lastValidUs_[idx];
    portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&mux_));

    if (last == 0) return 0;
    if (static_cast<uint32_t>(now - last) > BoardConfig::RC_TIMEOUT_US) return 0;
    return pw;
}

RcFrame RcReceiver::readFrame() const {
    RcFrame f;
    f.ch2 = getChannel(Ch::CH2);
    f.ch3 = getChannel(Ch::CH3);
    f.ch4 = getChannel(Ch::CH4);
    f.ch5 = getChannel(Ch::CH5);
    return f;
}

void IRAM_ATTR RcReceiver::isrCH2() { if (instance_) instance_->handleEdge(BoardConfig::RX_CH2_PIN, Ch::CH2); }
void IRAM_ATTR RcReceiver::isrCH3() { if (instance_) instance_->handleEdge(BoardConfig::RX_CH3_PIN, Ch::CH3); }
void IRAM_ATTR RcReceiver::isrCH4() { if (instance_) instance_->handleEdge(BoardConfig::RX_CH4_PIN, Ch::CH4); }
void IRAM_ATTR RcReceiver::isrCH5() { if (instance_) instance_->handleEdge(BoardConfig::RX_CH5_PIN, Ch::CH5); }
