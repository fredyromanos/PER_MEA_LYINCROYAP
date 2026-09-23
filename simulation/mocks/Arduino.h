#ifndef ARDUINO_H
#define ARDUINO_H

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

// Types Arduino
typedef uint8_t byte;
typedef unsigned long ul;

// Simulation time management
class SimTime {
public:
    static void init();
    static unsigned long millis();
    static void delayMs(unsigned long ms);
    static void advanceTime(unsigned long ms);
    
private:
    static unsigned long currentTime;
};

// Mock Serial
class SerialMock {
public:
    void begin(int baudRate);
    void print(const char* str);
    void print(const std::string& str);
    void print(int value);
    void print(float value);
    void print(double value);
    void println(const char* str);
    void println(const std::string& str);
    void println(int value);
    void println(float value);
    void println(double value);
    int available() const { return 0; }
    std::string readString() { return ""; }
};

extern SerialMock Serial;

// Mock GPIO
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
unsigned long pulseIn(int pin, int level, unsigned long timeout);

// Constants
#define INPUT 0
#define OUTPUT 1
#define HIGH 1
#define LOW 0

// Utility macros
#define delay(ms) SimTime::delayMs(ms)
#define millis() SimTime::millis()
#define delayMicroseconds(us) SimTime::delayMs((us) / 1000)
#define micros() (SimTime::millis() * 1000)

#endif // ARDUINO_H
