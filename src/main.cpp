/****************************************************************************************************************************
  RP2040 driver for TCD1304

  Written by Vladimir Kozlov https://github.com/vladkozlov69/RP2040_TCD1304
  Licensed under MIT license
*****************************************************************************************************************************/


#include <Arduino.h>
#include "pins_arduino.h"
#include "pinDefinitions.h"
#include "hardware/adc.h"
#include "RP2040_PWM.h"
#include "SpectralTool.h"
#include "Calibration.h"

#define PIXEL_COUNT 3691
#define MAX_EXPOSURE_TIME 800000UL
#define MAX_READ_CYCLE_COUNT 10000

#define CAPTURE_CHANNEL 0
#define CLOCK_DIV 96
#define LED_PIN LED_BUILTIN

#define ADC_FLAG_PIN    LED_PIN

#define BITSET_SH           gpio_put(SH_PIN, 1)
#define BITCLR_SH           gpio_put(SH_PIN, 0)
#define BITSET_ICG          gpio_put(ICG_PIN, 1)
#define BITCLR_ICG          gpio_put(ICG_PIN, 0)
#define BITSET_ADC_READ     gpio_put(ADC_FLAG_PIN, 1)
#define BITCLR_ADC_READ     gpio_put(ADC_FLAG_PIN, 0)
#define BITREAD_ADC_SYNC    gpio_get(ADC_SYNC_PIN)

RP2040_PWM * PWM_CLK;
RP2040_PWM * PWM_ADC_SYNC;

uint32_t buffer[PIXEL_COUNT];

int32_t readCycleCount = -3;
int32_t exposureTime = 100, readTime;
uint32_t adcFreq;
uint16_t lowestCCDVoltage;

uint32_t waitLoops;

uint32_t measureAdcSpeed();
void setupTimer(PinName pin, uint32_t ovfCounter);
void readCCD(void);

char buf[250];

Spectrum sp;
SpectralTool st;
RI ri;

SENSOR_CALIBRATION SensorCalibration = {
    .R = {
        .wavelength = DEFAULT_CALIBRATION_RED_WAVELENGTH,
        .pixelNum = DEFAULT_CALIBRATION_RED_PIXEL
    },
    .G = {
        .wavelength = DEFAULT_CALIBRATION_GREEN_WAVELENGTH,
        .pixelNum = DEFAULT_CALIBRATION_GREEN_PIXEL
    }, 
    .B = {
        .wavelength = DEFAULT_CALIBRATION_BLUE_WAVELENGTH,
        .pixelNum = DEFAULT_CALIBRATION_BLUE_PIXEL
    }
};

void setup() 
{
    SerialUSB.begin(115200);

    pinMode(SH_PIN, OUTPUT);
    pinMode(ICG_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);


    adc_gpio_init(ADC_PIN + CAPTURE_CHANNEL);
    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_set_clkdiv(CLOCK_DIV);

    sleep_ms(1000);

    // measure ADC speed
    adcFreq = measureAdcSpeed();

    PWM_CLK = new RP2040_PWM(CLK_PIN, adcFreq * 4, 50);
    PWM_ADC_SYNC = new RP2040_PWM(ADC_SYNC_PIN, adcFreq, 50);
    PWM_CLK->setPWM();
    PWM_ADC_SYNC->setPWM();
}

unsigned long copyTimer = 0;

constexpr int pixel_agg_power = 0;

constexpr int pixel_agg = 1 << pixel_agg_power;
constexpr int pixel_iter = PIXEL_COUNT / pixel_agg;

void loop() 
{
    if (millis() - copyTimer >= 1000)
    {
        unsigned long writeStart = micros();
        snprintf(buf, sizeof(buf), "#START adcF=%lu, exposureTime=%ld\r\n", adcFreq, exposureTime);
        SerialUSB.print(buf);

        sp.clear();
        
        for (int i = 0; i < pixel_iter; ++i)
        {
            int pixelNum = i * pixel_agg;
            long adcSum = 0;

            for (int j = 0; j < pixel_agg; j++)
            {
                adcSum = adcSum + buffer[pixelNum + j];
            }

            adcSum = adcSum >> pixel_agg_power;
            adcSum = adcSum / readCycleCount;
            SerialUSB.println(adcSum);
        }
        snprintf(buf, sizeof(buf), "#END pixels=%d, readTime=%ld, writeTime=%lu, waitLoops=%lu, readCycleCount=%ld\r\n", 
            pixel_iter, readTime, micros() - writeStart, waitLoops, readCycleCount);
        SerialUSB.print(buf);
        
        readCycleCount = 0;  // skip first
        memset(buffer, 0, sizeof(buffer));

        copyTimer = millis();
    }

    readCCD();

    if (lowestCCDVoltage < 1100) 
    {
        // decrease exposure
        exposureTime = max(exposureTime / 1.618, 10);
    } 
    else if (lowestCCDVoltage > 1500)
    {
        // increase exposure
        exposureTime = min(exposureTime * 1.618, MAX_EXPOSURE_TIME);
    }

    delayMicroseconds(max(exposureTime - readTime, 10L));
}

uint32_t readCCDInternal(int pixelsToRead, bool sync=false)
{
    lowestCCDVoltage = 4096;
    uint16_t readVal;
    uint32_t started = micros();
    waitLoops = 0;
    for (int x = 0; x < pixelsToRead; x++)
    {
        if (sync)
        {
            while (BITREAD_ADC_SYNC == 0)
            {
                waitLoops++;
            }  
        }
        
        readVal = adc_read();
        if (readCycleCount > 0 && readCycleCount <= MAX_READ_CYCLE_COUNT)
        {
            buffer[x] = buffer[x] + readVal;
        }
        if (readVal < lowestCCDVoltage) lowestCCDVoltage = readVal;
    }  

    if (readCycleCount <= MAX_READ_CYCLE_COUNT)
    {
        readCycleCount++;
    }
    return micros() - started;  
}

uint32_t measureAdcSpeed()
{
    return 1000000UL * 1000 / readCCDInternal(1000);
}

void readCCD(void)
{
    BITCLR_ICG;
    delayMicroseconds(1);
    BITSET_SH;  
    delayMicroseconds(5);
    BITCLR_SH;
    delayMicroseconds(15);
    BITSET_ICG;
    delayMicroseconds(1);

    BITSET_ADC_READ;
    readTime = readCCDInternal(PIXEL_COUNT, true);
    BITCLR_ADC_READ;
}




