#pragma once

#include <Arduino.h>

static const uint16_t DEFAULT_CALIBRATION_BLUE_PIXEL = 1000;
static const uint16_t DEFAULT_CALIBRATION_GREEN_PIXEL = 2000;
static const uint16_t DEFAULT_CALIBRATION_RED_PIXEL = 3000;
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

float getWavelengthByPoints(uint16_t pixel1, uint16_t wl1, uint16_t pixel2, uint16_t wl2, u_int16_t pixel)
{
    float a = 1.0 * (wl2 - wl1) / (pixel2 - pixel1);
    float b = wl1 - a * pixel1;

    return a * pixel + b;
}

float getWavelength(SENSOR_CALIBRATION &calibration, uint16_t pixel)
{
    if (pixel <= calibration.G.pixelNum)
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

