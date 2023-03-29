#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>

namespace util
{
    volatile uint32_t get_hardware_id();
    void print_serial_number();
}

#endif
