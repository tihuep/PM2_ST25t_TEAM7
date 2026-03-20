/**
 * @file WS2812SPI.h
 * @brief This file defines the WS2812SPI class for controlling WS2812 LED strips using SPI communication.
 * @author langhma2
 */

 #ifndef WS2812SPI_H
#define WS2812SPI_H

#include "mbed.h"

class WS2812SPI
{
public:
    WS2812SPI(PinName mosi, int num_leds);

    void setPixelColor(int index, uint8_t r, uint8_t g, uint8_t b);
    void clear();
    void show();
    void setBrightness(uint8_t brightness);

private:
    SPI spi;
    int numLEDs;
    uint8_t brightness;

    uint8_t* pixelBuffer;

    uint8_t encodeBit(bool bit);
};

#endif