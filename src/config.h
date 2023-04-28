#ifndef CONFIG_H
#define CONFIG_H

#include <set>
#include <Arduino.h>

#include "nautic_net/hw/radio.h"

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
    static const unsigned int kSlotCountPerTransmit = 1;

    // Base station configuration
    static const unsigned int kMaxRoverCount = 8; // The number of supported rovers; must divide evenly into tdma::kRoverDataSlotCount

    // LoRa configuration
    static const uint8_t kLoraPower = 20; // dBm (0 through 20)

    // The FIXED radio mode for rover discovery and configuration (slots 0 and 1)
    static const nautic_net::hw::radio::Config kLoraDefaultConfig = {
        .sbw = 500, // kHz (125, 250, or 500)
        .sf = 7     // Spreading factor (7 through 12)
    };

    // The CONFIGURABLE radio mode for rover data, which is handed out to the rovers from the base
    static const nautic_net::hw::radio::Config kLoraRoverDataConfig = {
        .sbw = 500, // kHz (125, 250, or 500)
        .sf = 9     // Spreading factor (7 through 12)
    };

    // Serial logging configuration
    static const bool kEnableBell = false;                   // Print \a when receiving data
    static const bool kEnableSerialStudioIMULogging = false; // Output for Serial Studio

    // Pins
    static const int kPinGPSPPS = A5;      // Hardwire
    static const int kPinBaseMode = A0;    // Jumper
    static const int kPinBattery = A7;     // Hardwired voltage divider (Vbat/2) on Feather M0
    static const int kPinCalibration = A1; // Calibration switch
}

#endif