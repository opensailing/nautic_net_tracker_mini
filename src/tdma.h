#ifndef TDMA_H
#define TDMA_H

#include <set>

namespace tdma
{
    // TDMA configuration (SEE ALSO: base.h)
    static const int kCycleDurationSec = 10;  // sec, must divide evenly into 60
    static const int kSlotCount = 100;        // Total number of slots available
    static const int kReservedSlotCount = 20; // Number of slots reserved for rover discovery + configuration (kRoverDiscoverySlots + kRoverConfigurationSlots)
    static const std::set<int> kRoverDiscoverySlots({0, 10, 20, 30, 40, 50, 60, 70, 80, 90});
    static const std::set<int> kRoverConfigurationSlots({1, 11, 21, 31, 41, 51, 61, 71, 81, 91});

    // Derived
    static const int kRoverDataSlotCount = kSlotCount - kReservedSlotCount;  // Number of slots reserved for rover data
    static const unsigned long kCycleDuration = kCycleDurationSec * 1000000; // µs
    static const unsigned long kSlotDuration = kCycleDuration / kSlotCount;  // µs

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
