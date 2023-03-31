#include <Arduino.h>

#include "debug.h"
#include "nautic_net/rover.h"
#include "nautic_net/util.h"

using namespace nautic_net::rover;

Rover::Rover(nautic_net::hw::radio::Radio *radio, nautic_net::hw::gps::GPS *gps, nautic_net::hw::imu::IMU *imu) : radio_(radio), gps_(gps), imu_(imu)
{
}

void Rover::Setup()
{
    // Nothing right now
}

void Rover::Loop()
{
    // Nothing right now
}

void Rover::HandleSlot(tdma::Slot slot)
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

void Rover::SendDiscovery()
{
    RoverDiscovery discovery;
    discovery.dummy_field = 0;

    LoRaPacket packet;
    packet.hardwareID = util::get_hardware_id();
    packet.payload.roverDiscovery = discovery;
    packet.which_payload = LoRaPacket_roverDiscovery_tag;

    radio_->Send(packet);
}

void Rover::SendData()
{
    // Negative integers are not efficient to encode in protobuf, so let's avoid -90° through 0° by normalizing
    // the heel angle such that 0° is full counter-clockwise deflection (laying flat to the left), and 180°
    // (integer value 1800) is full clockwise deflection (laying flat to the right). If the boat goes beyond these
    // angles, we have bigger problems than optimizing dwell time.
    int encoded_heel_angle = (int)((imu_->heel_angle_deg_ + 90.0) * 10);

    RoverData data;
    data.heading = (int)imu_->compass_angle_deg_;
    data.heel = encoded_heel_angle;
    data.latitude = gps_->gps_.latitudeDegrees;
    data.longitude = gps_->gps_.longitudeDegrees;

    LoRaPacket packet;
    packet.hardwareID = util::get_hardware_id();
    packet.payload.roverData = data;
    packet.which_payload = LoRaPacket_roverData_tag;

    radio_->Send(packet);
}

void Rover::HandlePacket(LoRaPacket packet)
{
    // Ignore configs destined for other rovers
    if (packet.which_payload == LoRaPacket_roverConfiguration_tag && packet.hardwareID == util::get_hardware_id())
    {
        Configure(packet);
    }
}

void Rover::Configure(LoRaPacket packet)
{
    ResetConfiguration();

    for (unsigned int i = 0; i < packet.payload.roverConfiguration.slots_count; i++)
    {
        tx_slots_[packet.payload.roverConfiguration.slots[i]] = true;
    }

    state_ = RoverState::kConfigured;
}

void Rover::ResetConfiguration()
{
    for (unsigned int i = 0; i < tdma::kSlotCount; i++)
    {
        tx_slots_[i] = false;
    }

    state_ = RoverState::kUnconfigured;
}

bool Rover::IsMyTransmitSlot(tdma::Slot slot)
{
    return (slot.type == tdma::SlotType::kRoverData && tx_slots_[slot.number]);
}
