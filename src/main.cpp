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
#define MIN_EXPOSURE_TIME 12L
#define MAX_EXPOSURE_TIME 2400L//(1000000L/44)
#define MAX_READ_CYCLE_COUNT 10000
#define MAX_CCD_ADC_VALUE 3000
#define IDEAL_CCD_ADC_VALUE 1600

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
#define BITREAD_SH_SYNC     gpio_get(SH_PIN)

RP2040_PWM * PWM_CLK;
RP2040_PWM * PWM_ADC_SYNC;

#ifdef USE_SH_PWM
RP2040_PWM * PWM_SH;
#endif

uint32_t buffer[PIXEL_COUNT];

int32_t exposureTime = MIN_EXPOSURE_TIME, readTime;
uint32_t adcFreq;
uint16_t lowestCCDVoltage;

uint32_t waitLoops;

uint32_t measureAdcSpeed();
void setupTimer(PinName pin, uint32_t ovfCounter);
void readCCD(void);
void processData();
bool dataReady;

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

    // pinMode(SH_PIN, OUTPUT);
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

#ifdef USE_SH_PWM
    PWM_SH = new RP2040_PWM(SH_PIN, 1000000.0/MIN_EXPOSURE_TIME, 50);
    PWM_SH->setPWM();
#endif
}

unsigned long copyTimer = 0;

constexpr int pixel_agg_power = 0;

constexpr int pixel_agg = 1 << pixel_agg_power;
constexpr int pixel_iter = PIXEL_COUNT / pixel_agg;

void loop() 
{
    processData();

    readCCD();

    dataReady = false;

    if (lowestCCDVoltage > 1400 && (lowestCCDVoltage < 2700 || exposureTime == MAX_EXPOSURE_TIME))
    {
        dataReady = true;
    }
    else 
    if (lowestCCDVoltage < 1100) 
    {
        // reset exposure
        SerialUSB.print("#REM lowestCCDVoltage=");
        SerialUSB.print(lowestCCDVoltage);
        SerialUSB.print(" Decrease exposure /4 from ");
        SerialUSB.println(exposureTime);
        exposureTime = MIN_EXPOSURE_TIME;
    } 
    else if (lowestCCDVoltage < 1600 && exposureTime > MIN_EXPOSURE_TIME) 
    {
        // reduce exposure
        SerialUSB.print("#REM lowestCCDVoltage="); 
        SerialUSB.print(lowestCCDVoltage);
        SerialUSB.print(" Decrease exposure proportionally from ");
        SerialUSB.print(exposureTime);
        float K = 1.0 * (MAX_CCD_ADC_VALUE - IDEAL_CCD_ADC_VALUE) / (MAX_CCD_ADC_VALUE - lowestCCDVoltage);
        SerialUSB.print(" Using K = ");
        SerialUSB.println(K);
        exposureTime = exposureTime * K;
    } 
    else if (lowestCCDVoltage > 2200 && exposureTime < MAX_EXPOSURE_TIME && exposureTime < MIN_EXPOSURE_TIME * 4)
    {
        // increase exposure   
        SerialUSB.print("#REM lowestCCDVoltage=");
        SerialUSB.print(lowestCCDVoltage);
        SerialUSB.print(" Increase exposure x4 from ");
        SerialUSB.println(exposureTime);
        exposureTime = exposureTime * 4;
        
    }
    else if (lowestCCDVoltage > 1600 && exposureTime < MAX_EXPOSURE_TIME )
    {
        SerialUSB.print("#REM lowestCCDVoltage="); 
        SerialUSB.print(lowestCCDVoltage);
        SerialUSB.print(" Increase exposure proportionally from ");
        SerialUSB.print(exposureTime);
        float K = 1.0 * (MAX_CCD_ADC_VALUE - IDEAL_CCD_ADC_VALUE) / (MAX_CCD_ADC_VALUE - lowestCCDVoltage);
        SerialUSB.print(" Using K = ");
        SerialUSB.println(K);
        exposureTime = exposureTime * K;
    }

    // if (exposureTime > readTime) exposureTime = exposureTime - readTime;
    if (exposureTime < MIN_EXPOSURE_TIME) exposureTime = MIN_EXPOSURE_TIME;
    if (exposureTime > MAX_EXPOSURE_TIME) exposureTime = MAX_EXPOSURE_TIME;


#ifdef USE_SH_PWM
    PWM_SH->setPWM(SH_PIN, 1000000.0/exposureTime, 50);
    SerialUSB.print("SH Freq ");
    SerialUSB.println(PWM_SH->getActualFreq());
#else
    delayMicroseconds(max(exposureTime - readTime, MIN_EXPOSURE_TIME)); 
#endif
}

void processData()
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
        sp.insert({prevWavelength, coef * sumPerWavelength/countWavelength});
    }

    snprintf(buf, sizeof(buf), "#END pixels=%d, readTime=%ld, writeTime=%lu, waitLoops=%lu\r\n", 
        pixel_iter, readTime, micros() - writeStart, waitLoops);
    SerialUSB.print(buf);
    
    memset(buffer, 0, sizeof(buffer));

    if (!dataReady) return;

    // TODO here we have 'sp' populated with values so we can calculate CCT, CRI, Ri etc.
    Spectrum toProcess = st.transpose(sp);
    st.normalize(toProcess);

    for (auto const& spElement : toProcess)
    {
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
        buffer[x] = buffer[x] + readVal;
        if (readVal < lowestCCDVoltage) lowestCCDVoltage = readVal;
    }  

    return micros() - started;  
}

uint32_t measureAdcSpeed()
{
    return 1000000UL * 1000 / readCCDInternal(1000);
}

void readCCD(void)
{
#ifdef USE_SH_PWM
    while (BITREAD_SH_SYNC == 1) waitLoops++; 
    while (BITREAD_SH_SYNC == 0) waitLoops++;
    delayMicroseconds(2);
    BITCLR_ICG;
    delayMicroseconds(15);
    while (BITREAD_SH_SYNC == 1) waitLoops++; 
    while (BITREAD_SH_SYNC == 0) waitLoops++;
    BITSET_ICG;
    delayMicroseconds(1);
#else
    if (exposureTime < readTime)
    {
        for (int i = 0; i < 200000 / exposureTime; i++)
        {
            BITSET_SH;  
            delayMicroseconds(5);
            BITCLR_SH;
            delayMicroseconds(exposureTime - 5);
        }
    }
    delayMicroseconds(2);
    BITCLR_ICG;
    delayMicroseconds(1);
    BITSET_SH;  
    delayMicroseconds(5);
    BITCLR_SH;
    delayMicroseconds(15);
    BITSET_ICG;
    delayMicroseconds(1);
#endif

    BITSET_ADC_READ;
    readTime = readCCDInternal(PIXEL_COUNT, true);
    BITCLR_ADC_READ;
}




