#pragma once

#include <Arduino.h>

class Approximator
{
    private: 
        float a, b;
    public:
        Approximator(float x1, float y1, float x2, float y2)
        {
            a = (y2 - y1) / (x2 - x1);
            b = y1 - a * x1;
        }
        float get(float x)
        {
            return a * x + b;
        }
};

static const uint16_t DEFAULT_CALIBRATION_BLUE_PIXEL = 2410;
static const uint16_t DEFAULT_CALIBRATION_GREEN_PIXEL = 1980;
static const uint16_t DEFAULT_CALIBRATION_RED_PIXEL = 1441;
static const uint16_t DEFAULT_CALIBRATION_BLUE_WAVELENGTH = 445;
static const uint16_t DEFAULT_CALIBRATION_GREEN_WAVELENGTH = 532;
static const uint16_t DEFAULT_CALIBRATION_RED_WAVELENGTH = 650;

struct TCD1304_SpectralResponse {
    int wl;
    float coef;
};

static Approximator redApproximator(
    DEFAULT_CALIBRATION_RED_PIXEL, DEFAULT_CALIBRATION_RED_WAVELENGTH, 
    DEFAULT_CALIBRATION_GREEN_PIXEL, DEFAULT_CALIBRATION_GREEN_WAVELENGTH);

static Approximator blueApproximator(
    DEFAULT_CALIBRATION_GREEN_PIXEL, DEFAULT_CALIBRATION_GREEN_WAVELENGTH, 
    DEFAULT_CALIBRATION_BLUE_PIXEL, DEFAULT_CALIBRATION_BLUE_WAVELENGTH);


float getWavelength(uint16_t pixel)
{
    if (pixel >= DEFAULT_CALIBRATION_GREEN_PIXEL)
    {
        return blueApproximator.get(pixel);
    }
    else
    {
        return redApproximator.get(pixel);
    }
}

// TODO cache a, b in record
float getTCD1304Coef(TCD1304_SpectralResponse * sr, int srCount, int waveLength)
{
    for (int i = 0; i < srCount - 1; i++)
    {
        if (sr[i].wl <= waveLength && sr[i+1].wl > waveLength)
        {
            // SerialUSB.print(sr[i].wl);
            // SerialUSB.print(" <= ");
            // SerialUSB.print(waveLength);
            // SerialUSB.print(" < ");
            // SerialUSB.print(sr[i+1].wl);
            float a = 1.0 * (sr[i+1].coef - sr[i].coef) / (sr[i+1].wl - sr[i].wl);
            float b = sr[i].coef - a * sr[i].wl;
            float y = a * waveLength + b;
            // SerialUSB.print("   ");
            // SerialUSB.println(y);
            return 1.0/y;
        }
    }
    
    return 0;
}
