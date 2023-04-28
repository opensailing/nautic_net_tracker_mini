#include "config.h"

#ifndef DEBUG_H
#define DEBUG_H

#ifdef SERIAL_DEBUG
#define debugWait() \
    while (!Serial) \
    delay(10)
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugln2(x, y) Serial.println(x, y)
#else
#define debugWait()
#define debug(x)
#define debugln(x)
#define debugln2(x, y)
#endif

#endif