#ifndef CONFIG_H
#define CONFIG_H

#include <set>
#include <Arduino.h>

// Uncomment to enable debug() and debugln() macros for printing to Serial
// #define SERIAL_DEBUG

namespace nautic_net::config
{
    // TDMA configuration
    static const int kCycleDurationSec = 10;  // sec, must divide evenly into 60
    static const int kSlotCount = 100;        // Total number of slots available
    static const int kReservedSlotCount = 20; // Number of slots reserved for rover discovery + configuration (kRoverDiscoverySlots + kRoverConfigurationSlots)
    static const std::set<int> kRoverDiscoverySlots = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90};
    static const std::set<int> kRoverConfigurationSlots = {1, 11, 21, 31, 41, 51, 61, 71, 81, 91};

    // Base station configuration
    static const unsigned int kMaxRoverCount = 8; // The number of supported rovers; must divide evenly into tdma::kRoverDataSlotCount

    // LoRa configuration
    static const uint8_t kLoraSF = 9;     // Spreading factor (7 through 12)
    static const long kLoraSBW = 500000;  // Hz (125000, 250000, or 500000)
    static const uint8_t kLoraPower = 20; // dBm (0 through 20)

    // Serial logging configuration
    static const bool kEnableBell = false;       // Print \a when receiving data
    static const bool kEnableIMULogging = false; // Output for Serial Studio
}

#endif