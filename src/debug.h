#include "config.h"

#ifndef DEBUG_H
#define DEBUG_H

#ifdef SERIAL_DEBUG

#ifdef WAIT_FOR_SERIAL
#define debugWait() \
    while (!Serial) \
    delay(10)
#else
#define debugWait()
#endif

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