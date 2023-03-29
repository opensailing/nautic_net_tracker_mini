#ifndef TDMA_H
#define TDMA_H

namespace tdma
{
    static const unsigned int kSlotCount = 100;
    static const unsigned long kSlotDuration = 100000; // µs

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
        TDMA();

        void SyncToGPS();
        bool TryGetSlotTransition(tdma::Slot *slot);

        void ClearTxSlots();
        void EnableTxSlot(unsigned int slot);

    private:
        unsigned long synced_at_ = 0;
        int current_slot_number_ = 0;
        SlotType current_slot_type_ = SlotType::kRoverDiscovery;

        int GetSlotNumber(unsigned long synced_time);
        SlotType GetSlotType(int slot);
    };
}
#endif
