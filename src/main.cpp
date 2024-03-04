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
#define MIN_EXPOSURE_TIME 11L
#define MAX_EXPOSURE_TIME_PWM 133333L
#define MAX_EXPOSURE_TIME 800000L
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

void loop() 
{
    processData();
    // SerialUSB.println("#START");
    // for (int i = 0; i < PIXEL_COUNT / 8; i++)
    // {
    //     unsigned long val = 0;
    //     for (int j = 0; j < 8; j++)
    //     {
    //         val = val + buffer[i * 8 + j];
    //     }
    //     SerialUSB.println(val / 8.0, 4);
    // }
    // SerialUSB.println("#END");


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
    if (exposureTime <= MAX_EXPOSURE_TIME_PWM)
    {
        PWM_SH->enablePWM();
        PWM_SH->setPWM(SH_PIN, 1000000.0/exposureTime, 50);
        SerialUSB.print("#REM SH Freq ");
        SerialUSB.println(PWM_SH->getActualFreq());
    }
    else
    {
        PWM_SH->disablePWM();
        pinMode(SH_PIN, OUTPUT);
        delayMicroseconds(max(exposureTime - readTime, MIN_EXPOSURE_TIME));
    }
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
    
    // TODO find i_start and i_end for 380-780 nm and optimize iteration
    for (int i = getPixelForWavelength(800); i < getPixelForWavelength(360); i++)
    //for (int i = 0; i < PIXEL_COUNT; ++i)
    {
        // aggregate rounded wavelenghts as we have ~3..4 values per nm
        int waveLenght = (int) getWavelength(i) / 2;
        if (prevWavelength == waveLenght)
        {
            countWavelength++;
            sumPerWavelength += buffer[i];
        }
        else
        {
            if (prevWavelength > 0 && countWavelength > 0)
            {
                if (prevWavelength >= (380/2) && prevWavelength <= (780/2))
                {
                    float coef = getTCD1304Coef(prevWavelength*2);
                    sp.insert({prevWavelength*2, coef * sumPerWavelength/countWavelength});
                }
            }
            prevWavelength = waveLenght;
            countWavelength = 1;
            sumPerWavelength = buffer[i];    
        }
    }

    if (prevWavelength >= 380/2 && prevWavelength <= 780/2)
    {
        float coef = getTCD1304Coef(prevWavelength*2);
        sp.insert({prevWavelength*2, coef * sumPerWavelength/countWavelength});
    }

    snprintf(buf, sizeof(buf), "#END readTime=%ld, writeTime=%lu, waitLoops=%lu\r\n", 
        readTime, micros() - writeStart, waitLoops);
    SerialUSB.print(buf);
    
    memset(buffer, 0, sizeof(buffer));

    if (!dataReady) return;

    SerialUSB.print("#REM Orig SP:");
    SerialUSB.println(sp.size());

    // for (auto const& spElement : sp)
    // {
    //     SerialUSB.print("#REM ");
    //     SerialUSB.print(spElement.first);
    //     SerialUSB.print(" => ");
    //     SerialUSB.println(spElement.second);
    // }

    // // TODO here we have 'sp' populated with values so we can calculate CCT, CRI, Ri etc.
    // Spectrum toProcess = st.transpose(sp);
    st.normalize(sp);

    for (auto const& spElement : sp)
    {
        SerialUSB.println(spElement.second, 4);
    }

    XY XYcoord = st.calcXY(sp);
    float CCT = st.calcCCT(XYcoord);

    SerialUSB.print("#REM CCT=");
    SerialUSB.println(CCT);

    float DUV = st.calcDUV(XYcoord);

    SerialUSB.print("#REM DUV=");
    SerialUSB.println(DUV, 6);

    memset(ri, 0, sizeof(ri));
    st.calcCRI(sp, ri);

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
        buffer[x] = readVal;
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
    if (exposureTime <= MAX_EXPOSURE_TIME_PWM)
    {
        while (BITREAD_SH_SYNC == 1) waitLoops++; 
        while (BITREAD_SH_SYNC == 0) waitLoops++;
        delayMicroseconds(2);
        BITCLR_ICG;
        delayMicroseconds(15);
        while (BITREAD_SH_SYNC == 1) waitLoops++; 
        while (BITREAD_SH_SYNC == 0) waitLoops++;
        BITSET_ICG;
        delayMicroseconds(1);
    }
    else
    {
        delayMicroseconds(2);
        BITCLR_ICG;
        delayMicroseconds(1);
        BITSET_SH;  
        delayMicroseconds(5);
        BITCLR_SH;
        delayMicroseconds(15);
        BITSET_ICG;
        delayMicroseconds(1);
    }
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




