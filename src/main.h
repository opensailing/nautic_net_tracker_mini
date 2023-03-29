#include "lora.pb.h"

void becomeBase();
void becomeRover();
void loraTxData();
void loraTxDiscovery();
void roverHandleConfiguration(LoRaPacket packet);

enum class State
{
    kNoFix,
    kUnconfiguredRover,
    kConfiguredRover,
    kBaseStation
};
