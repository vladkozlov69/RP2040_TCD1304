#pragma once

#include <Arduino.h>

class Approximator
{
    private: 
        float a, b;
    public:
        Approximator(float x1, float y1, float x2, float y2)
        {
            this->update(x1, y1, x2, y2);
        }
        void update(float x1, float y1, float x2, float y2)
        {
            a = (y2 - y1) / (x2 - x1);
            b = y1 - a * x1;
        }
        float get(float x)
        {
            return a * x + b;
        }
        float getArgument(float y)
        {
            return (y - b) / a;
        }
};

static uint16_t CALIBRATION_BLUE_PIXEL = 2000;
static uint16_t CALIBRATION_GREEN_PIXEL = 1000;
static uint16_t CALIBRATION_RED_PIXEL = 2;
static const uint16_t CALIBRATION_BLUE_WAVELENGTH = 405;
static const uint16_t CALIBRATION_GREEN_WAVELENGTH = 532;
static const uint16_t CALIBRATION_RED_WAVELENGTH = 650;

struct TCD1304_SpectralResponse {
    int wl;
    float coef;
    Approximator * approx;
};

static TCD1304_SpectralResponse tcd1304SR[] = {
    {.wl = 380, .coef = 0.70, .approx = new Approximator(380, 0.70, 400, 0.80)},
    {.wl = 400, .coef = 0.80, .approx = new Approximator(400, 0.80, 450, 0.93)},
    {.wl = 450, .coef = 0.93, .approx = new Approximator(450, 0.93, 500, 0.98)},
    {.wl = 500, .coef = 0.98, .approx = new Approximator(500, 0.98, 550, 0.99)},
    {.wl = 550, .coef = 0.99, .approx = new Approximator(550, 0.99, 600, 0.97)},
    {.wl = 600, .coef = 0.97, .approx = new Approximator(600, 0.97, 650, 0.92)},
    {.wl = 650, .coef = 0.92, .approx = new Approximator(650, 0.92, 700, 0.80)},
    {.wl = 700, .coef = 0.80, .approx = new Approximator(700, 0.80, 750, 0.63)},
    {.wl = 750, .coef = 0.63, .approx = new Approximator(750, 0.63, 800, 0.45)},
    {.wl = 800, .coef = 0.45}
};
Approximator * redApproximator = NULL;
    // DEFAULT_CALIBRATION_RED_PIXEL, DEFAULT_CALIBRATION_RED_WAVELENGTH, 
    // DEFAULT_CALIBRATION_GREEN_PIXEL, DEFAULT_CALIBRATION_GREEN_WAVELENGTH);

Approximator * blueApproximator = NULL;
    // DEFAULT_CALIBRATION_GREEN_PIXEL, DEFAULT_CALIBRATION_GREEN_WAVELENGTH, 
    // DEFAULT_CALIBRATION_BLUE_PIXEL, DEFAULT_CALIBRATION_BLUE_WAVELENGTH);


float getWavelength(uint16_t pixel)
{
    if (pixel >= CALIBRATION_GREEN_PIXEL)
    {
        return blueApproximator->get(pixel);
    }
    else
    {
        return redApproximator->get(pixel);
    }
}

int getPixelForWavelength(float wavelength)
{
    if (wavelength <= CALIBRATION_GREEN_WAVELENGTH)
    {
        return blueApproximator->getArgument(wavelength);
    }
    else
    {
        return redApproximator->getArgument(wavelength);
    }
}

// TODO cache a, b in record
float getTCD1304Coef(int waveLength)
{
    for (int i = 0; i < 9; i++)
    {
        if (tcd1304SR[i].wl <= waveLength && tcd1304SR[i+1].wl > waveLength)
        {
            // SerialUSB.print(sr[i].wl);
            // SerialUSB.print(" <= ");
            // SerialUSB.print(waveLength);
            // SerialUSB.print(" < ");
            // SerialUSB.print(sr[i+1].wl);
            // SerialUSB.print("   ");
            // SerialUSB.println(y);
            return 1.0/tcd1304SR[i].approx->get(waveLength);
        }
    }
    
    return 0;
}
