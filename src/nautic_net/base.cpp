#include "base.h"
#include "debug.h"
#include "nautic_net/tdma.h"

using namespace nautic_net::base;

Base::Base(nautic_net::hw::radio::Radio *radio) : radio_(radio)
{
}

void Base::DiscoverRover(LoRaPacket packet)
{
    int base_slot;

    if (rover_slots_[packet.hardwareID] == 0)
    {
        debug("Found a new rover: ");
        debugln(packet.hardwareID);

        base_slot = next_base_slot_;
        rover_slots_[packet.hardwareID] = base_slot;

        // Determine the next set of slots to hand out
        while (true)
        {
            next_base_slot_ = (next_base_slot_ + 1) % kRoverSlotInterval;

            if (tdma::TDMA::GetSlotType(next_base_slot_) == tdma::SlotType::kRoverData)
            {
                break;
            }
        }
    }
    else
    {
        debug("Rediscovered existing rover: ");
        debugln(packet.hardwareID);
        base_slot = rover_slots_[packet.hardwareID];
    }

    // Build the config packet
    RoverConfiguration config;
    config.slots_count = kRoverSlotCount;
    for (unsigned int i = 0; i < kRoverSlotCount; i++)
    {
        config.slots[i] = base_slot + (kRoverSlotInterval * i);
    }

    LoRaPacket config_packet;
    config_packet.hardwareID = packet.hardwareID;
    config_packet.payload.roverConfiguration = config;
    config_packet.which_payload = LoRaPacket_roverConfiguration_tag;

    // Enqueue for TX later
    config_queue_[config_queue_length_] = config_packet;
    config_queue_length_++;
}

bool Base::TryPopConfigPacket(LoRaPacket *packet)
{
    if (config_queue_length_ == 0)
    {
        return false;
    }

    // Pop off the head of the queue
    *packet = config_queue_[0];

    // Shift the rest of the queue down
    config_queue_length_--;
    for (int i = 0; i < config_queue_length_; i++)
    {
        config_queue_[i] = config_queue_[i + 1];
    }

    return true;
}

void Base::HandleSlot(tdma::Slot slot)
{
    if (slot.type == tdma::SlotType::kRoverConfiguration)
    {
        LoRaPacket config_packet;
        if (TryPopConfigPacket(&config_packet))
        {
            radio_->Send(config_packet);
        }
    }
}

void Base::HandlePacket(LoRaPacket packet)
{
    if (packet.which_payload == LoRaPacket_roverDiscovery_tag)
    {
        DiscoverRover(packet);
    }
    else if (packet.which_payload == LoRaPacket_roverData_tag)
    {
        PrintRoverData(packet);
    }
}

void Base::PrintRoverData(LoRaPacket packet)
{
    Serial.print("DATA: ");
    Serial.print(packet.hardwareID, 16);
    Serial.print(",");
    Serial.print(packet.payload.roverData.latitude, 8);
    Serial.print(",");
    Serial.print(packet.payload.roverData.longitude, 8);
    Serial.print(",");
    Serial.print(packet.payload.roverData.heading);
    Serial.print(",");
    Serial.println(packet.payload.roverData.heel);
}