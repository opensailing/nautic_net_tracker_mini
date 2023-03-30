#ifndef BASE_H
#define BASE_H

#include <map>

#include "lora.pb.h"
#include "radio.h"
#include "tdma.h"

namespace base
{
    // Configuration (SEE ALSO: tdma.h)
    static const unsigned int kMaxRoverCount = 8; // The number of supported rovers; must divide evenly into tdma::kRoverDataSlotCount

    // Derived
    static const unsigned int kRoverSlotCount = tdma::kRoverDataSlotCount / kMaxRoverCount; // The number of TX slots allocated to each rover in one cycle
    static const unsigned int kRoverSlotInterval = tdma::kSlotCount / kRoverSlotCount;      // The number of slots between subsequent TX for one rover

    class Base
    {
    public:
        Base(radio::Radio *radio);
        void HandlePacket(LoRaPacket packet);
        void HandleSlot(tdma::Slot slot);

    private:
        int next_base_slot_ = 2;
        LoRaPacket config_queue_[20];
        int config_queue_length_ = 0;
        radio::Radio *radio_;
        std::map<int, int> rover_slots_;

        void DiscoverRover(LoRaPacket packet);
        void PrintRoverData(LoRaPacket packet);
        bool TryPopConfigPacket(LoRaPacket *packet);
    };
}

#endif