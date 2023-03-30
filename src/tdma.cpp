#include <Arduino.h>
#include "tdma.h"

tdma::TDMA::TDMA()
{
}

void tdma::TDMA::SyncToGPS(int second)
{
    if (second != -1 && second % kCycleDurationSec == 0)
    {
        synced_at_ = micros();
    }
}

int tdma::TDMA::GetSlotNumber(unsigned long synced_time)
{
    return (synced_time / kSlotDuration) % kSlotCount;
}

tdma::SlotType tdma::TDMA::GetSlotType(int slot)
{
    if (kRoverDiscoverySlots.find(slot) != kRoverDiscoverySlots.end())
    {
        return SlotType::kRoverDiscovery;
    }
    else if (kRoverConfigurationSlots.find(slot) != kRoverConfigurationSlots.end())
    {
        return SlotType::kRoverConfiguration;
    }
    else
    {
        return SlotType::kRoverData;
    }
}

bool tdma::TDMA::TryGetSlotTransition(tdma::Slot *slot)
{
    if (synced_at_ != 0)
    {
        unsigned long synced_time = micros() - synced_at_;
        int slot_num = GetSlotNumber(synced_time);

        // Only move forward to the next slot, never backwards to a previous one
        if (slot_num > current_slot_number_ || (slot_num == 0 && current_slot_number_ == (kSlotCount - 1)))
        {
            SlotType slot_type = GetSlotType(slot_num);

            current_slot_number_ = slot_num;
            current_slot_type_ = slot_type;

            *slot = Slot{slot_num, slot_type};
            return true;
        }
    }

    return false;
}
