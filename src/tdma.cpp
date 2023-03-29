#include <Arduino.h>
#include "tdma.h"

namespace tdma
{
    SlotType _tdmaSlotType = SlotTypeRoverDiscovery;
    bool _activeSlots[NUM_SLOTS];
    unsigned long _tdmaSyncAt = 0;
    int _tdmaSlot = 0;

    void clearSlots()
    {
        for (unsigned int i = 0; i < NUM_SLOTS; i++)
        {
            _activeSlots[i] = false;
        }
    }

    int currentSlot(unsigned long syncedTime)
    {
        return (syncedTime / SLOT_US) % NUM_SLOTS;
    }

    void syncNow()
    {
        _tdmaSyncAt = micros();
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

    SlotType startOfSlot()
    {
        if (_tdmaSyncAt != 0)
        {
            unsigned long syncedTime = micros() - _tdmaSyncAt;
            int slot = currentSlot(syncedTime);

            // Only move forward to the next slot, never backwards to a previous one
            if (slot > _tdmaSlot || (slot == 0 && _tdmaSlot == (NUM_SLOTS - 1)))
            {
                _tdmaSlot = slot;
                SlotType slotType = currentSlotType(slot, _activeSlots);

                if (slotType != _tdmaSlotType)
                {
                    _tdmaSlotType = slotType;
                    return slotType;
                }
            }
        }

        return SlotTypeNone;
    }

    void enableSlot(unsigned int slot)
    {
        _activeSlots[slot] = true;
    }
}