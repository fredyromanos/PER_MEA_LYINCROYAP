#include "Arduino.h"

// Initialize simulation time
unsigned long SimTime::currentTime = 0;

void SimTime::init() {
    currentTime = 0;
}

unsigned long SimTime::millis() {
    return currentTime;
}

void SimTime::delayMs(unsigned long ms) {
    currentTime += ms;
}

void SimTime::advanceTime(unsigned long ms) {
    currentTime += ms;
}

// Serial implementation
SerialMock Serial;

void SerialMock::begin(int baudRate) {
    std::cout << "[Serial] Initialized at " << baudRate << " baud" << std::endl;
}

void SerialMock::print(const char* str) {
    std::cout << str;
}

void SerialMock::print(const std::string& str) {
    std::cout << str;
}

void SerialMock::print(int value) {
    std::cout << value;
}

void SerialMock::print(float value) {
    std::cout << value;
}

void SerialMock::print(double value) {
    std::cout << value;
}

void SerialMock::println(const char* str) {
    std::cout << str << std::endl;
}

void SerialMock::println(const std::string& str) {
    std::cout << str << std::endl;
}

void SerialMock::println(int value) {
    std::cout << value << std::endl;
}

void SerialMock::println(float value) {
    std::cout << value << std::endl;
}

void SerialMock::println(double value) {
    std::cout << value << std::endl;
}

// GPIO Mocks
static int pinStates[40] = {0};

void pinMode(int pin, int mode) {
    // Mock implementation
}

void digitalWrite(int pin, int value) {
    if (pin >= 0 && pin < 40) {
        pinStates[pin] = value;
    }
}

int digitalRead(int pin) {
    if (pin >= 0 && pin < 40) {
        return pinStates[pin];
    }
    return 0;
}

unsigned long pulseIn(int pin, int level, unsigned long timeout) {
    // In simulation, return a default PWM pulse (1500 µs = center)
    return 1500;
}
