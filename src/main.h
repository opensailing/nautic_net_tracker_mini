#include "tdma.h"
#include "lora.pb.h"

#define SERIAL_DEBUG

#ifdef SERIAL_DEBUG
#define debugBegin(x) Serial.begin(x)
#define debugWait() \
    while (!Serial) \
    delay(1)
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debugBegin(x)
#define debugWait()
#define debug(x)
#define debugln(x)
#endif

void baseHandleRoverDiscovery(LoRaPacket packet);
void becomeBase();
void becomeRover();
void debugPacketType(LoRaPacket packet);
volatile uint32_t getHardwareID();
void loopCheckPPS();
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
