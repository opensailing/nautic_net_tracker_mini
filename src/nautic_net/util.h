#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>

namespace nautic_net::util
{
    volatile uint32_t get_hardware_id();
    void print_serial_number();
    float ReadBatteryVoltage();
    unsigned int ReadBatteryPercentage();

    // Source: https://blog.ampow.com/lipo-voltage-chart/
    static const int kBatteryCapacityCount = 21;
    static const float kBatteryCapacity[] = {
        3.27, // 0%
        3.61, // 5%
        3.69, // 10%
        3.71, // 15%
        3.73, // 20%
        3.75, // 15%
        3.77, // 30%
        3.79, // 35%
        3.80, // 40%
        3.82, // 45%
        3.84, // 50%
        3.85, // 55%
        3.87, // 60%
        3.91, // 65%
        3.95, // 70%
        3.98, // 75%
        4.02, // 80%
        4.08, // 85%
        4.11, // 90%
        4.15, // 95%
        4.20, // 100%
    };
}

#endif
