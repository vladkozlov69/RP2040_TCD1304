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
#define MIN_EXPOSURE_TIME 10UL
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
int32_t exposureTime = MIN_EXPOSURE_TIME, readTime;
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



TCD1304_SpectralResponse tcd1304SR[] = {
    {.wl = 380, .coef = 0.70},
    {.wl = 400, .coef = 0.80},
    {.wl = 450, .coef = 0.93},
    {.wl = 500, .coef = 0.98},
    {.wl = 550, .coef = 0.99},
    {.wl = 600, .coef = 0.97},
    {.wl = 650, .coef = 0.92},
    {.wl = 700, .coef = 0.8},
    {.wl = 750, .coef = 0.63},
    {.wl = 800, .coef = 0.45}
};

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
    SerialUSB.begin(230400);

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
    if (readCycleCount >= 3)
    {
        unsigned long writeStart = micros();
        snprintf(buf, sizeof(buf), "#START adcF=%lu, lowestCCDVoltage=%d, exposureTime=%ld\r\n", 
            adcFreq, lowestCCDVoltage, exposureTime);
        SerialUSB.print(buf);

        // normalize values
        uint32_t maxVoltage = 0, maxVal = 0, minVal = 10000;
        size_t maxPos = 0, minPos = 0;
        for (int i = 0; i < PIXEL_COUNT; i++)
        {
            buffer[i] = buffer[i] / readCycleCount;
            if (buffer[i] > maxVoltage) 
            {
                maxVoltage = buffer[i];
            }
        }

        for (int i = 0; i < PIXEL_COUNT; i++)
        {
            buffer[i] = maxVoltage - buffer[i];

            if (buffer[i] > maxVal) 
            {
                maxVal = buffer[i];
                maxPos = i;
            }
            if (buffer[i] < minVal) 
            {
                minVal = buffer[i];
                minPos = i;
            }
        }

        sp.clear();
        int prevWavelength = 0;
        int countWavelength = 0;
        float sumPerWavelength = 0;
        
        for (int i = 0; i < PIXEL_COUNT; ++i)
        {
            // aggregate rounded wavelenghts as we have ~3..4 values per nm
            int waveLenght = (int) getWavelength(SensorCalibration, i);
            if (prevWavelength == waveLenght)
            {
                countWavelength++;
                sumPerWavelength += buffer[i];
            }
            else
            {
                if (prevWavelength > 0 && countWavelength > 0)
                {
                    if (prevWavelength >= 380 && prevWavelength <= 780)
                    {
                        float coef = getTCD1304Coef(tcd1304SR, sizeof(tcd1304SR)/sizeof(tcd1304SR[0]), prevWavelength);
                        // SerialUSB.print(prevWavelength);
                        // SerialUSB.print(" => ");
                        // SerialUSB.println(coef, 4);
                        sp.insert({prevWavelength, coef * sumPerWavelength/countWavelength});
                    }
                }
                prevWavelength = waveLenght;
                countWavelength = 1;
                sumPerWavelength = buffer[i];    
            }
        }

        if (prevWavelength >= 380 && prevWavelength <= 780)
        {
            float coef = getTCD1304Coef(tcd1304SR, sizeof(tcd1304SR)/sizeof(tcd1304SR[0]), prevWavelength);
            // SerialUSB.print(prevWavelength);
            // SerialUSB.print(" => ");
            // SerialUSB.println(coef, 4);
            sp.insert({prevWavelength, coef * sumPerWavelength/countWavelength});
        }

        // for (auto const& spElement : sp)
        // {
        //     SerialUSB.print(spElement.first);
        //     SerialUSB.print(",");
        //     SerialUSB.print(spElement.second, 4);
        //     SerialUSB.print(",");
        //     SerialUSB.println(sp1.find(spElement.first)->second, 4);
        // }

        snprintf(buf, sizeof(buf), "#END pixels=%d, readTime=%ld, writeTime=%lu, waitLoops=%lu, readCycleCount=%ld\r\n", 
            pixel_iter, readTime, micros() - writeStart, waitLoops, readCycleCount);
        SerialUSB.print(buf);
        
        readCycleCount = 0;  // skip first
        memset(buffer, 0, sizeof(buffer));

        // TODO here we have 'sp' populated with values so we can calculate CCT, CRI, Ri etc.
        Spectrum toProcess = st.transpose(sp);
        st.normalize(toProcess);

        for (auto const& spElement : toProcess)
        {
            SerialUSB.print(spElement.first);
            SerialUSB.print(",");
            SerialUSB.println(spElement.second, 4);
        }


        XY XYcoord = st.calcXY(toProcess);
        float CCT = st.calcCCT(XYcoord);

        SerialUSB.print("#REM CCT=");
        SerialUSB.println(CCT);

        float DUV = st.calcDUV(XYcoord);

        SerialUSB.print("#REM DUV=");
        SerialUSB.println(DUV, 6);

        memset(ri, 0, sizeof(ri));
        st.calcCRI(toProcess, ri);

        float Ra=0, Re=0;
        for (int q = 0; q < 15; q++) 
        {
            if (q < 8) 
            {
                Ra = Ra + ri[q];
            }
            Re = Re + ri[q];
        }
        Ra = Ra / 8;
        Re = Re / 15;

        snprintf(buf, sizeof(buf), "#REM CCT=%d Ra=%d Re=%d MIN=%lu@%d MAX=%lu@%d", 
            (int)CCT, (int)Ra, (int)Re, minVal, minPos, maxVal, maxPos);
        SerialUSB.println(buf);

        copyTimer = millis();
    }

    readCCD();

    if (lowestCCDVoltage < 1100) 
    {
        // reset exposure
        exposureTime = MIN_EXPOSURE_TIME; 
    } 
    else if (lowestCCDVoltage > 2200 && exposureTime < MAX_EXPOSURE_TIME)
    {
        // increase exposure   
        exposureTime = min(exposureTime * 4, MAX_EXPOSURE_TIME);
        SerialUSB.println("#REM Increase exposure x4");
    }
    else if (lowestCCDVoltage > 1600 && exposureTime < MAX_EXPOSURE_TIME)
    {
        exposureTime = min(exposureTime * 1600 / (2900 - lowestCCDVoltage), MAX_EXPOSURE_TIME);
        SerialUSB.println("#REM Increase exposure proportionally");
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




