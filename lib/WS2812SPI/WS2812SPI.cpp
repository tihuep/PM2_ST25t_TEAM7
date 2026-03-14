#include "WS2812SPI.h"

//Constructor
WS2812SPI::WS2812SPI(PinName mosi, int num_leds)
    : spi(mosi, NC, NC)
{
    numLEDs = num_leds;

    spi.format(8,0);
    spi.frequency(2400000); // 2.4 MHz
    brightness = 60; //0-255

    pixelBuffer = new uint8_t[numLEDs * 3];

    clear();
}

void WS2812SPI::setBrightness(uint8_t brightness)
{
    this->brightness = brightness;
}

void WS2812SPI::clear()
{
    for(int i = 0; i < numLEDs * 3; i++)
        pixelBuffer[i] = 0;
    WS2812SPI::setBrightness(60); // reset brightness to default when clearing
}

void WS2812SPI::setPixelColor(int index, uint8_t r, uint8_t g, uint8_t b)
{
    if(index >= numLEDs) return;

    r = (r * brightness) / 255;
    g = (g * brightness) / 255;
    b = (b * brightness) / 255;

    pixelBuffer[index*3 + 0] = g;
    pixelBuffer[index*3 + 1] = r;
    pixelBuffer[index*3 + 2] = b;
}

uint8_t WS2812SPI::encodeBit(bool bit)
{
    if(bit)
        return 0b110;  // WS2812 logical 1
    else
        return 0b100;  // WS2812 logical 0
}

void WS2812SPI::show()
{
    for(int i = 0; i < numLEDs * 3; i++)
    {
        uint8_t byte = pixelBuffer[i];

        for(int bit = 7; bit >= 0; bit--)
        {
            bool b = byte & (1 << bit);
            spi.write(encodeBit(b));
        }
    }

    // reset pulse (>50µs)
    thread_sleep_for(1);
}