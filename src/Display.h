#include "Adafruit_GFX.h"
#include "FreeSansBold7pt7b.h"
#include "FreeSansBold9pt7b.h"
#include "Fonts/FreeSansBold12pt7b.h"
#include "Fonts/FreeSansBold18pt7b.h"
#include "Adafruit_ILI9341.h"

#include <map>
#include <numeric>

Adafruit_ILI9341 * tft;

int16_t detailStartY = 0;

uint16_t R_COLORS[] = {
    tft->color565(242, 185, 158),
    tft->color565(206, 177, 82),
    tft->color565(128, 186, 76),
    tft->color565(0, 168, 166),
    tft->color565(0, 159, 222),
    tft->color565(0, 134, 205), 
    tft->color565(165, 148, 198),
    tft->color565(233, 155, 193),
    tft->color565(230, 0, 54),
    tft->color565(255, 255, 0),
    tft->color565(0, 137, 94),
    tft->color565(0, 60, 149),
    tft->color565(244, 232, 219),
    tft->color565(0, 96, 68),
    tft->color565(245, 204, 165)
};

void displayBasicInfo(float Ra, float Re, float CCT, float DUV)
{
    tft->fillRect(0, 0, tft->width() - 1, 28, ILI9341_BLACK);
    tft->setCursor(0, 20);
    tft->setFont(&FreeSansBold18pt7b);
    tft->setTextSize(1);
    // tft->setTextSize(3);
    tft->setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    tft->print("Ra=");
    tft->print(Ra, 1);
    tft->print("/");
    tft->println(Re, 1);
    tft->setFont();

    tft->fillRect(0, 32, tft->width() - 1, 36, ILI9341_BLACK);
    tft->setFont(&FreeSansBold9pt7b);
    tft->setTextSize(1);
    tft->setCursor(0, 46);
    tft->setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft->print("CCT=");
    tft->print(CCT, 0);

    // tft->setCursor(0, tft->getCursorY() + 5);
    tft->setTextColor(ILI9341_CYAN, ILI9341_BLACK);
    tft->print("  DUV=");
    tft->println(DUV, 4);

    tft->setTextSize(1);
    tft->setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft->setCursor(0, tft->getCursorY() - 3);
    tft->print("Exp,uS:");
    tft->print(exposureTime);
    tft->print(" LV:");
    tft->println(lowestCCDVoltage);
    detailStartY = tft->getCursorY();
    tft->setFont();
}

void displayDetails1()
{
    int minY = tft->getCursorY() - 6;
    tft->fillRect(0, minY - 8, 
                  tft->width() - 1, tft->height() - (minY - 8) - 1, 
                  ILI9341_BLACK);
    tft->setCursor(0, minY);

    tft->setFont(&FreeSansBold7pt7b);
    tft->setTextSize(1);

    int16_t x1, y1;
    uint16_t barStartX, barHeight;
    tft->getTextBounds("R99:99", tft->getCursorX(), tft->getCursorY(), &x1, &y1, &barStartX, &barHeight);
    barStartX = barStartX + 10;
    barHeight = barHeight - 1;
    int maxBarWidth = tft->width() - barStartX - 1;
    for (int q = 0; q < 15; q++) 
    {
        int16_t barStartY = tft->getCursorY() - 7;
        tft->setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        tft->print("R");
        tft->print(q + 1);
        tft->print("=");
        tft->println(ri[q], 0);
        tft->fillRect(barStartX, barStartY, maxBarWidth * ri[q] / 100, barHeight, R_COLORS[q]);
        tft->setCursor(0, tft->getCursorY() - 1);
    }
    tft->setFont();
}

void displayDetails2()
{
    // at this point "sp" has actual spectral power distribution
    int minY = tft->getCursorY() - 6;
    int maxY = tft->height() - 18;
    int barWidth = tft->width() / sp.size();
    int maxBarHeight = maxY - minY;

    tft->fillRect(0, minY - 4, 
                  tft->width() - 1, tft->height() - (minY - 4) - 1, 
                  ILI9341_BLACK);

    // find spectral peak
    float spPeak = std::accumulate(sp.begin(), sp.end(), 0,
                                   [](float accu,std::pair<const int, double> elem)
    {
        return (elem.second > accu) ? elem.second : accu;
    });

    // for (auto const& spElement : sp)
    // {
    //     spPeak = max(spPeak, spElement.second);
    // }

    int itemX = 20;
    // render spectral distribution
    for (auto const& spElement : sp)
    {
        RGB rgb = st.nmToRgb(spElement.first);
        uint16_t color = tft->color565(rgb.R, rgb.G, rgb.B);

        int itemHeight = maxBarHeight * spElement.second / spPeak;
        int itemY = minY + (maxBarHeight - itemHeight);
        tft->drawFastVLine(itemX, itemY, itemHeight, color);
        itemX++;
    }
}