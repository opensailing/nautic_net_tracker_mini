#ifndef BASE_H
#define BASE_H

#include <map>

#include "config.h"
#include "lora_packet.pb.h"
#include "nautic_net/base/rover_info.h"
#include "nautic_net/hw/radio.h"
#include "nautic_net/tdma.h"

namespace nautic_net::base
{
    class Base
    {
    public:
        Base(nautic_net::hw::radio::Radio *radio);
        void HandlePacket(LoRaPacket packet, int rssi);
        void HandleSlot(tdma::Slot slot);
        void ResetConfiguration();

    private:
        nautic_net::hw::radio::Radio *radio_;
        unsigned int next_base_slot_ = 0;   // the next rover base slot to hand out upon discovery
        unsigned int rover_count_ = 0;      // number of discovered rovers
        unsigned int reset_sent_count_ = 0; // number of RoverReset packets that have been broadcast

        RoverInfo *rovers_[nautic_net::tdma::kMaxRoverCount];

        void DiscoverRover(LoRaPacket packet);
        void PrintRoverData(LoRaPacket packet, int rssi);
        bool TryPopConfigPacket(LoRaPacket *packet);
        int FindRoverIndex(unsigned int hardware_id);
    };
}

#endif