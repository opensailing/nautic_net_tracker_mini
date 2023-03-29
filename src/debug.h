#ifndef DEBUG_H
#define DEBUG_H

#define SERIAL_DEBUG

#ifdef SERIAL_DEBUG
#define debugBegin(x) Serial.begin(x)
#define debugWait() \
    while (!Serial) \
    delay(1)
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debugBegin(x)
#define debugWait()
#define debug(x)
#define debugln(x)
#endif

#endif