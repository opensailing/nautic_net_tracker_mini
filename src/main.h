#include "tdma.h"
#include "lora.pb.h"

void baseHandleRoverDiscovery(LoRaPacket packet);
void becomeBase();
void becomeRover();
void debugPacketType(LoRaPacket packet);
volatile uint32_t getHardwareID();
int loraTx(LoRaPacket packet);
void loraTxData();
void loraTxDiscovery();
void printChipId();
void radioSetup();
void radioRx();
void roverHandleConfiguration(LoRaPacket packet);

typedef enum
{
    StateNoFix,
    StateUnconfiguredRover,
    StateConfiguredRover,
    StateBase
} State;
