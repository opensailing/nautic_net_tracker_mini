#include "base.h"

base::Base::Base()
{
}

void base::Base::DiscoverRover(LoRaPacket packet)
{
    // Build the config packet
    RoverConfiguration config;
    config.slots_count = 10;
    for (int i = 0; i < 10; i++)
    {
        config.slots[i] = available_slot_ + (10 * i);
    }

    LoRaPacket config_packet;
    config_packet.hardwareID = packet.hardwareID;
    config_packet.payload.roverConfiguration = config;
    config_packet.which_payload = LoRaPacket_roverConfiguration_tag;

    // Enqueue for TX later
    config_queue_[config_queue_length_] = config_packet;
    config_queue_length_++;

    // Determine the next set of slots to hand out (2 through 9, inclusive)
    if (available_slot_ == 9)
    {
        available_slot_ = 2;
    }
    else
    {
        available_slot_++;
    }
}

bool base::Base::TryPopConfigPacket(LoRaPacket *packet)
{
    if (config_queue_length_ == 0)
    {
        return false;
    }

    // Pop off the head of the queue
    packet = &config_queue_[0];

    // Shift the rest of the queue down
    config_queue_length_--;
    for (int i = 0; i < config_queue_length_; i++)
    {
        config_queue_[i] = config_queue_[i + 1];
    }

    return true;
}