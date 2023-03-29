#ifndef BASE_H
#define BASE_H

#include "lora.pb.h"
#include "radio.h"
#include "tdma.h"

namespace base
{
    class Base
    {
    public:
        Base(radio::Radio *radio);
        void HandlePacket(LoRaPacket packet);
        void HandleSlot(tdma::Slot slot);

    private:
        int available_slot_ = 2;
        LoRaPacket config_queue_[20];
        int config_queue_length_ = 0;
        radio::Radio *radio_;

        void DiscoverRover(LoRaPacket packet);
        void PrintRoverData(LoRaPacket packet);
        bool TryPopConfigPacket(LoRaPacket *packet);
    };
}

#endif