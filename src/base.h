#ifndef BASE_H
#define BASE_H

#include "lora.pb.h"

namespace base
{
    class Base
    {
    public:
        Base();
        bool TryPopConfigPacket(LoRaPacket *packet);
        void DiscoverRover(LoRaPacket packet);

    private:
        int available_slot_ = 2;
        LoRaPacket config_queue_[20];
        int config_queue_length_ = 0;
    };
}

#endif