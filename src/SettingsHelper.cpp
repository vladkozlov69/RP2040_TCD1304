#include "SettingsHelper.h"

bool SettingsHelper::begin(const char * path, Stream * debug)
{
    _logger = debug;
    _dirty = false;
    _path = path;
    _file = fopen(path, "r");
    if (_file)
    {
        fclose(_file); 
    }
    else
    {
        clear();
    }

    jsonDocument = new DynamicJsonDocument(512);
    char buf[4096];
    FILE * f = fopen(_path.c_str(), "r");
    fread(buf, sizeof(buf), 1, f);
    fclose(f);
    DeserializationError error = deserializeJson(*jsonDocument, buf);
    if (_logger)
    {
        _logger->print("At SettingsHelper::begin, deserializeJson=");
        _logger->println(error.c_str());
    }

    return true;
}

void SettingsHelper::end()
{
    if (_dirty && jsonDocument)
    {

        if (jsonDocument->overflowed())
        {
          if (_logger)
          {
            _logger->println("Overflowed");
          }
        }

        String b2;
        serializeJsonPretty(*jsonDocument, b2);
        if (_logger)
        {
          _logger->println("SettingsHelper::end serializeJson2:");
          _logger->println(b2);
        }

        FILE *file = fopen(_path.c_str(), "w");
        fwrite((uint8_t *) b2.c_str(), 1, b2.length(), file);
        fclose(file);
        _dirty = false;
    }
    if (jsonDocument)
    {
      delete jsonDocument;
      jsonDocument = NULL;
    }
}

size_t SettingsHelper::putString(const char* key, const char* value)
{
    if (jsonDocument)
    {
      if (_logger)
      {
        String b1;
        serializeJsonPretty(*jsonDocument, b1);
        _logger->println("SettingsHelper::putString serializeJson1:");
        _logger->println(b1);
      }
      (*jsonDocument)[key].set(value);
      if (_logger)
      {
        String b2;
        serializeJsonPretty(*jsonDocument, b2);
        _logger->println("SettingsHelper::putString serializeJson2:");
        _logger->println(b2);
      }
    }

    _dirty = true;
    return strlen(value);
}

size_t SettingsHelper::getString(const char* key, char* value, int maxLen)
{
    if (jsonDocument->containsKey(key))
    {
        String val = (*jsonDocument)[key];
        strncpy(value, val.c_str(), maxLen);
        return strlen(value);
    }
    else
    {
        return 0;
    }
}

String SettingsHelper::getString(const char* key, String defaultValue)
{
    char buf[25];

    if (0 == getString(key, buf, 25))
    {
        memcpy(buf, defaultValue.c_str(), 25);
    }

    return String(buf);
}

size_t SettingsHelper::putFloat(const char* key, float_t value)
{
    if (jsonDocument)
    {
        (*jsonDocument)[key].set(value);
    }

    _dirty = true;
    return sizeof(float_t);
}

size_t SettingsHelper::putInt(const char* key, int value)
{
    if (jsonDocument)
    {
        (*jsonDocument)[key].set(value);
    }

    _dirty = true;
    return sizeof(int);
}

float_t SettingsHelper::getFloat(const char* key, float_t defaultValue)
{
    if (jsonDocument->containsKey(key))
    {
        return (*jsonDocument)[key];
    }

    return defaultValue;
}

int SettingsHelper::getInt(const char* key, int defaultValue)
{
    if (jsonDocument->containsKey(key))
    {
        return (*jsonDocument)[key];
    }

    return defaultValue;
}

size_t SettingsHelper::putBool(const char* key, bool value)
{
    if (jsonDocument)
    {
        (*jsonDocument)[key].set(value);
    }

    _dirty = true;
    return sizeof(bool);
}

bool SettingsHelper::getBool(const char* key, bool defaultValue)
{
    if (jsonDocument->containsKey(key))
    {
        return (*jsonDocument)[key];
    }

    return defaultValue;
}

bool SettingsHelper::clear()
{
    (*jsonDocument)["sign"] = "DEADBEEF";
    String b;
    serializeJsonPretty(*jsonDocument, b);
    if (_logger)
    {
      _logger->print("SettingsHelper::clear serializeJson:");
      _logger->println(b);
    }

    FILE *file = fopen(_path.c_str(), "w");

    if (file)
    {
        if (_logger)
        {
            _logger->println(" => Open OK");
        }
    }
    else
    {
        if (_logger)
        {
            _logger->println(" => Open Failed");
        }

        return false;
    }

    if (fwrite((uint8_t *) b.c_str(), 1, b.length(), file))
    {
        if (_logger)
        {
            _logger->println("* Writing OK");
        }
    }
    else
    {
        if (_logger)
        {
            _logger->println("* Writing failed");
        }
    }

    fclose(file);

    _dirty = false;
    return true;
}
