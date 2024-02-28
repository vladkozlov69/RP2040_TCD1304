#ifndef SETTINGSHELPER_H_
#define SETTINGSHELPER_H_

#pragma once

#include <Arduino.h>
#include "ArduinoJson.h"

#define LFS_MBED_RP2040_VERSION_MIN_TARGET      "LittleFS_Mbed_RP2040 v1.1.0"
#define LFS_MBED_RP2040_VERSION_MIN             1001000

#define _LFS_LOGLEVEL_          1
#define RP2040_FS_SIZE_KB       64

#define FORCE_REFORMAT          false

#include <LittleFS_Mbed_RP2040.h>

//LittleFS_MBED *myFS;

class SettingsHelper
{
private:
    DynamicJsonDocument * jsonDocument = NULL;
    boolean _dirty = false;
    Stream * _logger = NULL;
    String _path;
    FILE * _file = NULL;
public:
    bool begin(const char * path, Stream * debug = &Serial);
    void end();
    size_t putString(const char* key, const char* value);
    size_t getString(const char* key, char* value, int maxLen);
    String getString(const char* key, String defaultValue = String());
    size_t putFloat(const char* key, float_t value);
    float_t getFloat(const char* key, float_t defaultValue = NAN);
    size_t putBool(const char* key, bool value);
    bool getBool(const char* key, bool defaultValue = false);
    size_t putInt(const char* key, int value);
    int getInt(const char* key, int defaultValue = 0);
    bool clear();
};

#endif
