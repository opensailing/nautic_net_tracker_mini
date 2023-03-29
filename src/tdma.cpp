#include <Arduino.h>
#include "tdma.h"

int currentSlot(unsigned long syncedTime)
{
    return (syncedTime / SLOT_US) % NUM_SLOTS;
}

unsigned long syncNow(int seconds, unsigned long syncedTime)
{
    // Determine expected slot
    int slotsPerSecond = 1000 / SLOT_MS;
    int minuteSlot = seconds * slotsPerSecond;
    int slot = minuteSlot % NUM_SLOTS;

    // Determine the time of slot 0
    int offset = slot * SLOT_US;
    return syncedTime - offset;
}

SlotType currentSlotType(int slot, bool mySlots[])
{
    if ((slot % 10) == 0)
    {
        return SlotTypeRoverDiscovery;
    }
    else if ((slot % 10) == 1)
    {
        return SlotTypeRoverConfiguration;
    }
    else if (mySlots[slot])
    {
        return SlotTypeThisRoverData;
    }
    else
    {
        return SlotTypeOtherRoverData;
    }
}
