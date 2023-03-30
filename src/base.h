#ifndef BASE_H
#define BASE_H

#include <map>

#include "config.h"
#include "lora.pb.h"
#include "radio.h"
#include "tdma.h"

namespace base
{
    // See config.h
    static const unsigned int kMaxRoverCount = config::kMaxRoverCount;

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
        int next_base_slot_ = 0;
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