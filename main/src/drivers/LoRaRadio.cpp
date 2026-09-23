#include "LoRaRadio.h"
#include "../config/BoardConfig.h"
#include "../config/DebugConfig.h"
#include <SPI.h>
#include <LoRa.h>

bool LoRaRadio::begin() {
    SPI.begin(BoardConfig::LORA_SCK_PIN,
              BoardConfig::LORA_MISO_PIN,
              BoardConfig::LORA_MOSI_PIN,
              BoardConfig::LORA_CS_PIN);
    LoRa.setSPI(SPI);
    LoRa.setPins(BoardConfig::LORA_CS_PIN, BoardConfig::LORA_RST_PIN, BoardConfig::LORA_IRQ_PIN);
    // First attempt — may fail if SX1276 is in a stuck TX state from a previous run
    ready_ = (LoRa.begin(433E6) == 1);
    if (!ready_) {
        // Force SX1276 out of stuck TX, then retry
        LoRa.sleep();
        delay(50);
        ready_ = (LoRa.begin(433E6) == 1);
        if (!ready_) {
            delay(100);
            LoRa.sleep();
            delay(50);
            ready_ = (LoRa.begin(433E6) == 1);
        }
    }
    DBG_RADIO("begin: %s (SCK=%d MISO=%d MOSI=%d CS=%d IRQ=%d)",
        ready_ ? "OK 433MHz" : "FAIL",
        BoardConfig::LORA_SCK_PIN, BoardConfig::LORA_MISO_PIN,
        BoardConfig::LORA_MOSI_PIN, BoardConfig::LORA_CS_PIN,
        BoardConfig::LORA_IRQ_PIN);
    return ready_;
}

bool LoRaRadio::send(const char* msg) {
    if (!ready_) return false;
    if (!LoRa.beginPacket()) {
        DBG_RADIO("TX skipped — still busy");
        return false;
    }
    DBG_RADIO("TX %u bytes", (unsigned)strlen(msg));
    LoRa.print(msg);
    LoRa.endPacket(true);  // async — returns immediately, radio transmits in background
    return true;
}

bool LoRaRadio::poll(char* buf, size_t maxLen) {
    if (!ready_) return false;
    static uint32_t pollCount = 0;
    if (++pollCount % 100 == 0) {
        DBG_RADIO("DIAG #%lu op=0x%02X irq=0x%02X",
            (unsigned long)pollCount,
            (unsigned)LoRa.debugOpMode(),
            (unsigned)LoRa.debugIrqFlags());
    }
    int sz = LoRa.parsePacket();
    if (sz > 0) {
        rxDetected_++;
        DBG_RADIO("RX pkt detected sz=%d rssi=%d", sz, LoRa.packetRssi());
    }
    if (sz <= 0) return false;
    size_t i = 0;
    while (LoRa.available() && i < maxLen - 1) {
        buf[i++] = (char)LoRa.read();
    }
    buf[i]    = '\0';
    lastRssi_ = LoRa.packetRssi();
    DBG_RADIO("RX read %u bytes", (unsigned)i);
    return i > 0;
}
