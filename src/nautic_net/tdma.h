#ifndef TDMA_H
#define TDMA_H

#include <set>

#include "config.h"

namespace nautic_net::tdma
{
    // See config.h
    static const int kCycleDurationSec = config::kCycleDurationSec;   // sec, must divide evenly into 60
    static const int kSlotCount = config::kSlotCount;                 // Total number of slots available
    static const int kReservedSlotCount = config::kReservedSlotCount; // Number of slots reserved for rover discovery + configuration (kRoverDiscoverySlots + kRoverConfigurationSlots)
    static const std::set<int> kRoverDiscoverySlots = config::kRoverDiscoverySlots;
    static const std::set<int> kRoverConfigurationSlots = config::kRoverConfigurationSlots;
    static const unsigned int kMaxRoverCount = config::kMaxRoverCount;
    static const unsigned int kSlotCountPerTransmit = config::kSlotCountPerTransmit;

    // Derived
    static const int kRoverDataSlotCount = kSlotCount - kReservedSlotCount;                                           // Total number of slots reserved for rover data
    static const unsigned long kCycleDuration = kCycleDurationSec * 1000000;                                          // µs
    static const unsigned long kSlotDuration = kCycleDuration / kSlotCount;                                           // µs
    static const unsigned int kRoverSlotCount = tdma::kRoverDataSlotCount / (kMaxRoverCount * kSlotCountPerTransmit); // The number of TX slots allocated to each rover in one cycle
    static const unsigned int kRoverSlotInterval = tdma::kSlotCount / kRoverSlotCount;                                // The number of slots between subsequent TX for one rover

    enum class SlotType
    {
        kRoverDiscovery,
        kRoverConfiguration,
        kRoverData
    };

    struct Slot
    {
        int number;
        SlotType type;
    };

    class TDMA
    {
    public:
        static SlotType GetSlotType(int slot);

        TDMA();

        void SyncToGPS(int second);
        bool TryGetSlotTransition(tdma::Slot *slot);

        void ClearTxSlots();
        void EnableTxSlot(unsigned int slot);

    private:
        unsigned long synced_at_ = 0;
        int current_slot_number_ = 0;
        SlotType current_slot_type_ = SlotType::kRoverDiscovery;

        int GetSlotNumber(unsigned long synced_time);
    };
}
#endif
