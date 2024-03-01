#pragma once

#include <Arduino.h>

static const uint16_t DEFAULT_CALIBRATION_BLUE_PIXEL = 2410;
static const uint16_t DEFAULT_CALIBRATION_GREEN_PIXEL = 1980;
static const uint16_t DEFAULT_CALIBRATION_RED_PIXEL = 1441;
static const uint16_t DEFAULT_CALIBRATION_BLUE_WAVELENGTH = 445;
static const uint16_t DEFAULT_CALIBRATION_GREEN_WAVELENGTH = 532;
static const uint16_t DEFAULT_CALIBRATION_RED_WAVELENGTH = 650;

struct SENSOR_CALIBRATION_POINT {
    uint16_t wavelength;
    uint16_t pixelNum;
};

struct SENSOR_CALIBRATION {
    SENSOR_CALIBRATION_POINT R;
    SENSOR_CALIBRATION_POINT G;
    SENSOR_CALIBRATION_POINT B;
};

struct TCD1304_SpectralResponse {
    int wl;
    float coef;
};

float getWavelengthByPoints(uint16_t pixel1, uint16_t wl1, uint16_t pixel2, uint16_t wl2, u_int16_t pixel)
{
    float a = 1.0 * (wl2 - wl1) / (pixel2 - pixel1);
    float b = wl1 - a * pixel1;

    return a * pixel + b;
}

float getWavelength(SENSOR_CALIBRATION &calibration, uint16_t pixel)
{
    if (pixel >= calibration.G.pixelNum)
    {
        return getWavelengthByPoints(calibration.B.pixelNum, calibration.B.wavelength, 
                             calibration.G.pixelNum, calibration.G.wavelength, pixel);
    }
    else
    {
        return getWavelengthByPoints(calibration.G.pixelNum, calibration.G.wavelength, 
                             calibration.R.pixelNum, calibration.R.wavelength, pixel);
    }
}

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

