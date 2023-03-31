#ifndef CONFIG_H
#define CONFIG_H

#include <set>

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

    // Base configuration
    static const unsigned int kMaxRoverCount = 8; // The number of supported rovers; must divide evenly into tdma::kRoverDataSlotCount
}

#endif