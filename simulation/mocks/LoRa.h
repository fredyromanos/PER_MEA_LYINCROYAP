#ifndef LORA_H
#define LORA_H

#include <string>

class LoRaClass {
public:
    int begin(long frequency) { return 1; }
    void beginPacket() {}
    void endPacket() {}
    void print(const char* data) {}
    void print(const std::string& data) {}
    int parsePacket() { return 0; }
    int available() { return 0; }
    std::string readString() { return ""; }
    void setPins(int cs, int rst, int irq) {}
    bool availableForWrite() { return true; }
};

extern LoRaClass LoRa;

#endif // LORA_H
