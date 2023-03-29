#ifndef TDMA_H
#define TDMA_H

namespace tdma
{
    enum class SlotType
    {
        kNone = 0b00000000,
        kRoverDiscovery = 0b00000001,
        kRoverConfiguration = 0b00000010,
        kThisRoverData = 0b00000100,
        kOtherRoverData = 0b00001000,
    };

    class TDMA
    {
    public:
        TDMA();

        SlotType CurrentSlotType(int slot, bool my_slots[]);
        SlotType GetSlotTransition();
        void SyncToGPS();

        void ClearTxSlots();
        void EnableTxSlot(unsigned int slot);

    private:
        static const unsigned int kSlotCount = 100;
        static const unsigned long kSlotDuration = 100000; // µs

        unsigned long synced_at_ = 0;
        int current_slot_number_ = 0;
        SlotType current_slot_type_ = SlotType::kRoverDiscovery;

        bool tx_slots_[kSlotCount];

        int CurrentSlotNumber(unsigned long synced_time);
    };
}
#endif
