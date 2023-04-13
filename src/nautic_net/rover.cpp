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
    cal_switch_state_ = HIGH;
}

void Rover::Loop()
{
    // Debounce calibration switch
    int cal_reading = digitalRead(config::kPinCalibration);

    if (cal_reading != last_cal_reading_)
    {
        last_cal_debounce_time_ = millis();
    }

    if ((millis() - last_cal_debounce_time_) > 50 && cal_reading != cal_switch_state_)
    {
        cal_switch_state_ = cal_reading;

        if (cal_switch_state_ == LOW)
        {
            imu_->BeginCompassCalibration();
        }
        else
        {
            imu_->FinishCompassCalibration();
        }
    }

    last_cal_reading_ = cal_reading;
}

void Rover::HandleSlot(tdma::Slot slot)
{
    // Change radio parameters depending on the slot type
    if (IsMyTransmitSlot(slot))
    {
        radio_->Configure(radio_config_);
    }
    else
    {
        radio_->Configure(config::kLoraDefaultConfig);
    }

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
    uint32_t encoded_heel_angle = (int)((imu_->heel_angle_deg_ + 90.0) * 10);
    encoded_heel_angle = min(max(encoded_heel_angle, 0U), 1800U);

    // These values are fixed-point integers with 0.1 precision
    uint32_t encoded_cog = (uint32_t)(gps_->gps_.angle * 10);
    uint32_t encoded_sog = (uint32_t)(gps_->gps_.speed * 10);

    RoverData data;
    data.heading = (uint32_t)imu_->compass_angle_deg_;
    data.heel = encoded_heel_angle;
    data.latitude = gps_->gps_.latitudeDegrees;
    data.longitude = gps_->gps_.longitudeDegrees;
    data.cog = encoded_cog;
    data.sog = encoded_sog;

    // Occasionally include battery voltage (a value of 0 takes up no extra bytes)
    send_counter_++;
    if ((send_counter_ % 10) == 0)
    {
        data.battery = util::ReadBatteryPercentage();
    }
    else
    {
        data.battery = 0;
    }

    LoRaPacket packet;
    packet.hardwareID = util::get_hardware_id();
    packet.payload.roverData = data;
    packet.which_payload = LoRaPacket_roverData_tag;

    radio_->Send(packet);
}

void Rover::HandlePacket(LoRaPacket packet, int rssi)
{
    // Ignore configs destined for other rovers
    if (packet.which_payload == LoRaPacket_roverConfiguration_tag && packet.hardwareID == util::get_hardware_id())
    {
        Configure(packet);
    }
}

void Rover::Configure(LoRaPacket packet)
{
    RoverConfiguration configPayload = packet.payload.roverConfiguration;

    ResetConfiguration();

    debugln("Got rover configuration: ");
    debug(" - SBW: ");
    debugln(configPayload.sbw);
    debug(" - SF: ");
    debugln(configPayload.sf);

    for (unsigned int i = 0; i < configPayload.slots_count; i++)
    {
        debug(" - TX slot: ");
        debugln(configPayload.slots[i]);

        tx_slots_[configPayload.slots[i]] = true;
    }

    radio_config_.sbw = configPayload.sbw;
    radio_config_.sf = configPayload.sf;

    state_ = RoverState::kConfigured;
}

void Rover::ResetConfiguration()
{
    for (unsigned int i = 0; i < tdma::kSlotCount; i++)
    {
        tx_slots_[i] = false;
    }

    radio_config_ = config::kLoraDefaultConfig;

    state_ = RoverState::kUnconfigured;
}

bool Rover::IsMyTransmitSlot(tdma::Slot slot)
{
    return (slot.type == tdma::SlotType::kRoverData && tx_slots_[slot.number]);
}
