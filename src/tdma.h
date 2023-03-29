#ifndef TDMA_H
#define TDMA_H

const unsigned int NUM_SLOTS = 100;
const unsigned int SLOT_MS = 100;
const unsigned long SLOT_US = 100000;

typedef enum
{
    SlotTypeNone = 0b00000000,
    SlotTypeRoverDiscovery = 0b00000001,
    SlotTypeRoverConfiguration = 0b00000010,
    SlotTypeThisRoverData = 0b00000100,
    SlotTypeOtherRoverData = 0b00001000,
} SlotType;

int currentSlot(unsigned long syncedTime);
SlotType currentSlotType(int slot, bool mySlots[]);
unsigned long syncNow(int seconds, unsigned long now);

#endif
