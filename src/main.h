#include "lora.pb.h"

void baseHandleRoverDiscovery(LoRaPacket packet);
void becomeBase();
void becomeRover();
volatile uint32_t getHardwareID();
void loraTxData();
void loraTxDiscovery();
void printChipId();
void roverHandleConfiguration(LoRaPacket packet);

enum class State
{
    kNoFix,
    kUnconfiguredRover,
    kConfiguredRover,
    kBaseStation
};
