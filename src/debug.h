#include "config.h"

#ifndef DEBUG_H
#define DEBUG_H

#ifdef SERIAL_DEBUG
#define debugWait() \
    while (!Serial) \
    delay(1)
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debugWait()
#define debug(x)
#define debugln(x)
#endif

#endif