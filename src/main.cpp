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
#include "SettingsHelper.h"

#ifdef TCD1254
    #define PIXEL_COUNT 2547
    #define CLK_ADC_DIVIDER 2
    #define MAX_EXPOSURE_TIME 800000L
    #define IDEAL_CCD_ADC_VALUE 1000
    #define DATAREADY_MIN_CCD_VOLTAGE 1300
    #define DATAREADY_MAX_CCD_VOLTAGE 2100
    #define RESET_EXPOSURE_CCD_VOLTAGE 650
    #define REDUCE_EXPOSURE_CCD_VOLTAGE 1200
    #define INCREASE_PROP_EXPOSURE_CCD_VOLTAGE 1900
    #define INCREASE_X4_EXPOSURE_CCD_VOLTAGE 1400
#else
    #define PIXEL_COUNT 3691
    #define CLK_ADC_DIVIDER 4
    #define MAX_EXPOSURE_TIME 800000L
    #define IDEAL_CCD_ADC_VALUE 1600
    #define DATAREADY_MIN_CCD_VOLTAGE 1400
    #define DATAREADY_MAX_CCD_VOLTAGE 2500
    #define RESET_EXPOSURE_CCD_VOLTAGE 1100
    #define REDUCE_EXPOSURE_CCD_VOLTAGE 1600
    #define INCREASE_PROP_EXPOSURE_CCD_VOLTAGE 2200
    #define INCREASE_X4_EXPOSURE_CCD_VOLTAGE 1600
#endif

#define MIN_EXPOSURE_TIME 11L
#define MAX_EXPOSURE_TIME_PWM 133333L

#define MAX_CCD_ADC_VALUE 3000


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

int32_t exposureTime = 100, readTime;
uint32_t adcFreq;
uint16_t lowestCCDVoltage, highestCCDVoltage;


uint32_t waitLoops;

uint32_t measureAdcSpeed();
void setupTimer(PinName pin, uint32_t ovfCounter);
void readCCD(void);
void processData();
void readCalibration();
bool dataReady;

char buf[250];

Spectrum sp;
SpectralTool st;
RI ri;

LittleFS_MBED *myFS;
SettingsHelper sh;

enum DumpDataMode
{
    OFF,
    SPECTRUM,
    RAW
};

DumpDataMode dumpData = DumpDataMode::SPECTRUM;
boolean autoExposure = true;

#include "Display.h"

void setup() 
{
    SerialUSB.begin(230400);

    myFS = new LittleFS_MBED();

    if (!myFS->init())
    {
        SerialUSB.println("LITTLEFS Mount Failed");
    }

    readCalibration();
   

    SPI = MbedSPI(PICO_DEFAULT_SPI_RX_PIN, 
                PICO_DEFAULT_SPI_TX_PIN, 
                PICO_DEFAULT_SPI_SCK_PIN);
    SPI.begin();

    tft = new Adafruit_ILI9341(&SPI, DC_PIN, CS_PIN, RST_PIN);
    tft->begin();
    tft->invertDisplay(true);
    tft->setRotation(0);
    tft->fillScreen(ILI9341_BLACK);

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
    adcFreq = measureAdcSpeed() ;

    PWM_CLK = new RP2040_PWM(CLK_PIN, adcFreq * CLK_ADC_DIVIDER, 50);
    PWM_ADC_SYNC = new RP2040_PWM(ADC_SYNC_PIN, adcFreq, 50);
    PWM_CLK->setPWM();
    PWM_ADC_SYNC->setPWM();

#ifdef USE_SH_PWM
    PWM_SH = new RP2040_PWM(SH_PIN, 1000000.0/MIN_EXPOSURE_TIME, 50);
    PWM_SH->setPWM();
#endif
}

unsigned long screenTimer = 0;
int screenNum = 0;

void processConsoleInput();

void loop() 
{
    if (SerialUSB.available())
    {
        processConsoleInput();
    }

    processData();

    int avgCount = exposureTime <= 1000 
        ? 20 
        : exposureTime < 10000 
            ? 10
            : exposureTime < 100000 
                ? 5
                : 1;

    for (int i = 0; i < avgCount; i++)
    {
        readCCD();
    }

    if (avgCount > 1) 
    {
        for (size_t i = 0; i < PIXEL_COUNT; i++)
        {
            buffer[i] = buffer[i] / avgCount;
        }  
    }

    if (dumpData == DumpDataMode::RAW)
    {
        for (int i = 0; i < PIXEL_COUNT; i++)
        {
            SerialUSB.println(buffer[i]);
        }
    }

    dataReady = false;

    // TIN PINS = 2500
    // GOLD = 2700
    if (lowestCCDVoltage > DATAREADY_MIN_CCD_VOLTAGE && (lowestCCDVoltage < DATAREADY_MAX_CCD_VOLTAGE || exposureTime == MAX_EXPOSURE_TIME))
    {
        dataReady = true;
    }
    else if (autoExposure) 
    {
        if (lowestCCDVoltage < RESET_EXPOSURE_CCD_VOLTAGE) 
        {
            // reset exposure
            SerialUSB.print("#REM lowestCCDVoltage=");
            SerialUSB.print(lowestCCDVoltage);
            SerialUSB.print(" Decrease exposure /4 from ");
            SerialUSB.println(exposureTime);
            exposureTime = MIN_EXPOSURE_TIME;
        } 
        else if (lowestCCDVoltage < REDUCE_EXPOSURE_CCD_VOLTAGE && exposureTime > MIN_EXPOSURE_TIME) 
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
        else if (lowestCCDVoltage > INCREASE_X4_EXPOSURE_CCD_VOLTAGE && exposureTime < MAX_EXPOSURE_TIME && exposureTime < MIN_EXPOSURE_TIME * 4)
        {
            // increase exposure   
            SerialUSB.print("#REM lowestCCDVoltage=");
            SerialUSB.print(lowestCCDVoltage);
            SerialUSB.print(" Increase exposure x4 from ");
            SerialUSB.println(exposureTime);
            exposureTime = exposureTime * 4;
            
        }
        else if (lowestCCDVoltage > INCREASE_PROP_EXPOSURE_CCD_VOLTAGE && exposureTime < MAX_EXPOSURE_TIME )
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
    }

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
    snprintf(buf, sizeof(buf), "#START adcF=%lu, lowestCCDVoltage=%d, highestCCDVoltage=%d, exposureTime=%ld, C=%d|%d|%d\r\n", 
        adcFreq, lowestCCDVoltage, highestCCDVoltage, exposureTime, 
        CALIBRATION_BLUE_PIXEL, CALIBRATION_GREEN_PIXEL, CALIBRATION_RED_PIXEL);
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
    
    int pixel800 = getPixelForWavelength(800);
    int pixel360 = getPixelForWavelength(360);
    int startPixel = min(pixel360, pixel800);
    int endPixel = max(pixel360, pixel800);
    // SerialUSB.print("#REM startPixel:");
    // SerialUSB.print(startPixel);
    // SerialUSB.print(" endPixel:");
    // SerialUSB.println(endPixel);
    for (int i = startPixel; i < endPixel; i++)
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

    snprintf(buf, sizeof(buf), "#END readTime=%ld, writeTime=%lu, waitLoops=%lu, spLen=%d\r\n", 
        readTime, micros() - writeStart, waitLoops, sp.size());
    SerialUSB.print(buf);

    if (!dataReady) return;

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

    if (dumpData == DumpDataMode::SPECTRUM)
    {
        for (auto const& spElement : sp)
        {
            SerialUSB.println(spElement.second, 4);
        }
    }

    XY XYcoord = st.calcXY(sp);
    float CCT = st.calcCCT(XYcoord);

    if (CCT > 8000)
    {
        dataReady = false;
        return;
    }

    // SerialUSB.print("#REM CCT=");
    // SerialUSB.println(CCT);

    float DUV = st.calcDUV(XYcoord);

    // SerialUSB.print("#REM DUV=");
    // SerialUSB.println(DUV, 6);

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

    displayBasicInfo(Ra, Re, CCT, DUV);
    if (millis() - screenTimer >= 3000)
    {
        screenTimer = millis();
        if (screenNum == 0)
        {
            displayDetails1();
        }
        else
        {
            displayDetails2();
        }
        screenNum = (screenNum + 1) % 2;
    }
}

uint32_t readCCDInternal(int pixelsToRead, bool sync=false)
{
    highestCCDVoltage = 0;
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
        buffer[x] += readVal;
        if (readVal < lowestCCDVoltage) lowestCCDVoltage = readVal;
        if (readVal > highestCCDVoltage) highestCCDVoltage = readVal;
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

void updateCalibrationPoint(const char * param, int value)
{
    sh.begin(MBED_LITTLEFS_FILE_PREFIX "/calib.json", &SerialUSB);
    sh.putInt(param, value);
    sh.end();
    readCalibration();
}

void processConsoleInput()
{
    String input = SerialUSB.readString();
    SerialUSB.print("#REM processConsoleInput ");
    SerialUSB.println(input);
    if (input.startsWith("C532="))
    {
        int pos = input.substring(5).toInt();
        if (pos > 0) updateCalibrationPoint("C532", pos);
    }
    if (input.startsWith("C405="))
    {
        int pos = input.substring(5).toInt();
        if (pos > 0) updateCalibrationPoint("C405", pos);
    }
    if (input.startsWith("C650="))
    {
        int pos = input.substring(5).toInt();
        if (pos > 0) updateCalibrationPoint("C650", pos);
    }
    if (input.startsWith("EXP="))
    {
        int val = input.substring(4).toInt();
        if (val > 0) 
        {
            exposureTime = val;
            autoExposure = false;
        }
        else
        {
            autoExposure = true;
        }
    }
    if (input.startsWith("DUMP=S"))
    {
        dumpData = DumpDataMode::SPECTRUM;
    }
    if (input.startsWith("DUMP=N"))
    {
        dumpData = DumpDataMode::OFF;
    }
    if (input.startsWith("DUMP=R"))
    {
        dumpData = DumpDataMode::RAW;
    }
}

void readCalibration()
{
    sh.begin(MBED_LITTLEFS_FILE_PREFIX "/calib.json", &SerialUSB);
    CALIBRATION_BLUE_PIXEL = sh.getInt("C405", CALIBRATION_BLUE_PIXEL);
    CALIBRATION_GREEN_PIXEL = sh.getInt("C532", CALIBRATION_GREEN_PIXEL);
    CALIBRATION_RED_PIXEL = sh.getInt("C650", CALIBRATION_RED_PIXEL);
    sh.end();
    if (redApproximator)
    {
        redApproximator->update(
            CALIBRATION_RED_PIXEL, CALIBRATION_RED_WAVELENGTH, 
            CALIBRATION_GREEN_PIXEL, CALIBRATION_GREEN_WAVELENGTH);
    }
    else
    {
        redApproximator = new Approximator(
            CALIBRATION_RED_PIXEL, CALIBRATION_RED_WAVELENGTH, 
            CALIBRATION_GREEN_PIXEL, CALIBRATION_GREEN_WAVELENGTH);   
    } 
    if (blueApproximator)
    {
        blueApproximator->update(
            CALIBRATION_GREEN_PIXEL, CALIBRATION_GREEN_WAVELENGTH, 
            CALIBRATION_BLUE_PIXEL, CALIBRATION_BLUE_WAVELENGTH);
    }
    else
    {
        blueApproximator = new Approximator(
            CALIBRATION_GREEN_PIXEL, CALIBRATION_GREEN_WAVELENGTH, 
            CALIBRATION_BLUE_PIXEL, CALIBRATION_BLUE_WAVELENGTH);
    }
}



