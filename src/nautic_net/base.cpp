#include "base.h"
#include "debug.h"
#include "nautic_net/tdma.h"

using namespace nautic_net::base;

Base::Base(nautic_net::hw::radio::Radio *radio) : radio_(radio)
{
}

void Base::DiscoverRover(LoRaPacket packet)
{
    int rover_index = FindRoverIndex(packet.hardware_id);

    if (rover_index == -1)
    {
        debug("Found a new rover: ");
        debugln2(packet.hardware_id, 16);

        // Determine the next set of slots to hand out
        while (true)
        {
            next_base_slot_ = (next_base_slot_ + tdma::kSlotCountPerTransmit) % tdma::kRoverSlotInterval;

            if (tdma::TDMA::GetSlotType(next_base_slot_) == tdma::SlotType::kRoverData)
            {
                break;
            }
        }

        // Wrap around if we have too many rovers
        rover_index = rover_count_ % tdma::kMaxRoverCount;
        rover_count_ = min(rover_count_ + 1, tdma::kMaxRoverCount);

        // TODO: Deallocate a RoverInfo if it already exists at rover_index?

        rovers_[rover_index] = new RoverInfo(packet.hardware_id, packet.serial_number);
        rovers_[rover_index]->radio_config_ = config::kLoraRoverDataConfig;

        for (unsigned int i = 0; i < tdma::kRoverSlotCount; i++)
        {
            rovers_[rover_index]->slots_[i] = next_base_slot_ + (tdma::kRoverSlotInterval * i);
        }
    }
    else
    {
        debug("Rediscovered existing rover: ");
        debugln2(packet.hardware_id, 16);
    }

    // Enqueue for TX later
    rovers_[rover_index]->is_configured_ = false;
}

bool Base::TryPopConfigPacket(LoRaPacket *packet)
{
    for (unsigned int i = 0; i < rover_count_; i++)
    {
        RoverInfo *rover_info = rovers_[i];
        if (!rover_info->is_configured_)
        {
            debug("Sending config to rover ");
            debugln2(rover_info->hardware_id_, 16);

            RoverConfiguration config;
            config.sf = rover_info->radio_config_.sf;
            config.sbw = rover_info->radio_config_.sbw;
            config.slots_count = tdma::kRoverSlotCount;
            for (unsigned int i = 0; i < tdma::kRoverSlotCount; i++)
            {
                config.slots[i] = rover_info->slots_[i];
            }

            packet->hardware_id = rover_info->hardware_id_;
            packet->payload.rover_configuration = config;
            packet->which_payload = LoRaPacket_rover_configuration_tag;

            return true;
        }
    }

    return false;
}

void Base::HandleSlot(tdma::Slot slot)
{
    if (slot.type == tdma::SlotType::kRoverData)
    {
        radio_->Configure(config::kLoraRoverDataConfig);
    }
    else
    {
        radio_->Configure(config::kLoraDefaultConfig);
    }

    if (slot.type == tdma::SlotType::kRoverConfiguration)
    {
        LoRaPacket config_packet;
        if (TryPopConfigPacket(&config_packet))
        {
            radio_->Send(config_packet);
        }
    }
}

void Base::HandlePacket(LoRaPacket packet, int rssi)
{
    int rover_index = FindRoverIndex(packet.hardware_id);
    if (rover_index != -1 && !rovers_[rover_index]->is_configured_ && packet.which_payload == LoRaPacket_rover_data_tag)
    {
        debugln("Got data; rover was successfully configured");
        rovers_[rover_index]->is_configured_ = true;
    }

    if (packet.which_payload == LoRaPacket_rover_discovery_tag)
    {
        DiscoverRover(packet);
    }
    else if (packet.which_payload == LoRaPacket_rover_data_tag)
    {
        PrintRoverData(packet, rssi);
    }
}

void Base::PrintRoverData(LoRaPacket packet, int rssi)
{
    if (config::kEnableBell)
    {
        Serial.print('\a');
    }
    Serial.print("BOAT,");
    Serial.print(rssi);
    Serial.print(',');
    Serial.print(packet.hardware_id, 16);
    Serial.print(',');
    Serial.print(packet.payload.rover_data.latitude, 8);
    Serial.print(',');
    Serial.print(packet.payload.rover_data.longitude, 8);
    Serial.print(',');
    Serial.print(packet.payload.rover_data.heading);
    Serial.print(',');
    Serial.print(packet.payload.rover_data.heel);
    Serial.print(',');
    Serial.print(packet.payload.rover_data.sog);
    Serial.print(',');
    Serial.print(packet.payload.rover_data.cog);
    Serial.print(',');
    Serial.print(packet.payload.rover_data.battery);
    Serial.print(',');
    Serial.println(packet.serial_number);
}

int Base::FindRoverIndex(unsigned int hardware_id)
{
    for (unsigned int i = 0; i < rover_count_; i++)
    {
        if (rovers_[i]->hardware_id_ == hardware_id)
        {
            return i;
        }
    }

    return -1;
}