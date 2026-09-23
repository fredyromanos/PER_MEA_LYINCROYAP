#ifndef SPI_H
#define SPI_H

class SPIClass {
public:
    void begin(int sck, int miso, int mosi, int cs) {}
};

extern SPIClass SPI;

#endif // SPI_H

// Dummy SPI implementation
SPIClass SPI;
