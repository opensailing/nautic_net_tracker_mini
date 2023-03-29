#include <Arduino.h>
#include "tdma.h"

tdma::TDMA::TDMA()
{
}

//
// TX SLOTS
//

void tdma::TDMA::ClearTxSlots()
{
    for (unsigned int i = 0; i < kSlotCount; i++)
    {
        tx_slots_[i] = false;
    }
}

void tdma::TDMA::EnableTxSlot(unsigned int slot)
{
    tx_slots_[slot] = true;
}

//
// SLOT DETERMINATION
//

void tdma::TDMA::SyncToGPS()
{
    synced_at_ = micros();
}

int tdma::TDMA::CurrentSlotNumber(unsigned long synced_time)
{
    return (synced_time / kSlotDuration) % kSlotCount;
}

tdma::SlotType tdma::TDMA::CurrentSlotType(int slot, bool my_slots[])
{
    if ((slot % 10) == 0)
    {
        return SlotType::kRoverDiscovery;
    }
    else if ((slot % 10) == 1)
    {
        return SlotType::kRoverConfiguration;
    }
    else if (my_slots[slot])
    {
        return SlotType::kThisRoverData;
    }
    else
    {
        return SlotType::kOtherRoverData;
    }
}

tdma::SlotType tdma::TDMA::GetSlotTransition()
{
    if (synced_at_ != 0)
    {
        unsigned long synced_time = micros() - synced_at_;
        int slot = CurrentSlotNumber(synced_time);

        // Only move forward to the next slot, never backwards to a previous one
        if (slot > current_slot_number_ || (slot == 0 && current_slot_number_ == (kSlotCount - 1)))
        {
            current_slot_number_ = slot;
            SlotType slot_type = CurrentSlotType(slot, tx_slots_);

            if (slot_type != current_slot_type_)
            {
                current_slot_type_ = slot_type;
                return slot_type;
            }
        }
    }

    return SlotType::kNone;
}
