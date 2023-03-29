#include <Arduino.h>

#include "debug.h"
#include "rover.h"
#include "util.h"

rover::Rover::Rover(radio::Radio radio, gps::GPS gps) : radio_(radio), gps_(gps)
{
}

void rover::Rover::HandleSlot(tdma::Slot slot)
{
    if (state_ == RoverState::kUnconfigured && slot.type == tdma::SlotType::kRoverDiscovery)
    {
        // Add 50ms random delay to prevent 100% collisions
        delay(random(50));
        SendDiscovery();
    }
    else if (state_ == RoverState::kConfigured && IsMyTransmitSlot(slot))
    {
        SendData();
    }
}

void rover::Rover::SendDiscovery()
{
    RoverDiscovery discovery;
    discovery.dummy_field = 0;

    LoRaPacket packet;
    packet.hardwareID = util::get_hardware_id();
    packet.payload.roverDiscovery = discovery;
    packet.which_payload = LoRaPacket_roverDiscovery_tag;

    radio_.Send(packet);
}

void rover::Rover::SendData()
{
    RoverData data;
    data.heading = random(360);
    data.heel = random(200) - 100;
    data.latitude = gps_.gps_.latitudeDegrees;
    data.longitude = gps_.gps_.longitudeDegrees;

    LoRaPacket packet;
    packet.hardwareID = util::get_hardware_id();
    packet.payload.roverData = data;
    packet.which_payload = LoRaPacket_roverData_tag;

    radio_.Send(packet);
}

void rover::Rover::HandlePacket(LoRaPacket packet)
{
    // Ignore configs destined for other rovers
    if (packet.which_payload == LoRaPacket_roverConfiguration_tag && packet.hardwareID == util::get_hardware_id())
    {
        Configure(packet);
    }
}

void rover::Rover::Configure(LoRaPacket packet)
{
    ResetConfiguration();

    for (unsigned int i = 0; i < packet.payload.roverConfiguration.slots_count; i++)
    {
        tx_slots_[packet.payload.roverConfiguration.slots[i]] = true;
    }

    state_ = RoverState::kConfigured;
}

void rover::Rover::ResetConfiguration()
{
    for (unsigned int i = 0; i < tdma::kSlotCount; i++)
    {
        tx_slots_[i] = false;
    }

    state_ = RoverState::kUnconfigured;
}

bool rover::Rover::IsMyTransmitSlot(tdma::Slot slot)
{
    return (slot.type == tdma::SlotType::kRoverData && tx_slots_[slot.number]);
}
