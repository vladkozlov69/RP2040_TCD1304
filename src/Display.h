#include "Adafruit_GFX.h"
#include "FreeSansBold7pt7b.h"
#include "FreeSansBold9pt7b.h"
#include "Fonts/FreeSansBold12pt7b.h"
#include "Fonts/FreeSansBold18pt7b.h"
#include "Adafruit_ILI9341.h"

Adafruit_ILI9341 * tft;

int screenNum = 0;
int16_t detailStartY = 0;
