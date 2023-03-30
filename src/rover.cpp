#include <Arduino.h>

#include "debug.h"
#include "rover.h"
#include "util.h"

rover::Rover::Rover(radio::Radio *radio, gps::GPS *gps) : radio_(radio), gps_(gps)
{
}

void rover::Rover::Setup()
{
    //
    // Initialize accelerometer
    // https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/arduino
    //
    accel_.begin_I2C();
    accel_.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    accel_.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
    accel_.setGyroDataRate(LSM6DS_RATE_SHUTDOWN);
    accel_.configInt1(false, false, true); // accelerometer DRDY on INT1
    accel_.configInt2(false, true, false); // gyro DRDY on INT2

    //
    // Initialize magnetometer
    // https://learn.adafruit.com/lis3mdl-triple-axis-magnetometer/arduino
    //
    magnet_.begin_I2C();
    magnet_.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    magnet_.setDataRate(LIS3MDL_DATARATE_10_HZ);
    magnet_.setRange(LIS3MDL_RANGE_4_GAUSS);
    magnet_.setIntThreshold(500);
    magnet_.configInterrupt(false, false, true, // enable z axis
                            true,               // polarity
                            false,              // don't latch
                            true);              // enabled!
}

void rover::Rover::Loop()
{
    if (accel_.accelerationAvailable())
    {
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;
        accel_.getEvent(&accel, &gyro, &temp);

        // X is towards the sky, Y is towards port, Z is towards the bow
        float angle_rad = atan2(accel.acceleration.y, accel.acceleration.x);
        float angle_deg = angle_rad * 180 / PI;

        heel_angle_deg_ -= heel_angle_deg_ / kHeelAveraging;
        heel_angle_deg_ += angle_deg / kHeelAveraging;
    }

    if (magnet_.magneticFieldAvailable())
    {
        sensors_event_t event;
        magnet_.getEvent(&event);

        // X is towards the sky, Y is towards port, Z is towards the bow
        float compass_rad = atan2(event.magnetic.y, event.magnetic.z);
        float compass_deg = compass_rad * 180 / PI + 180.0;

        compass_angle_deg_ -= compass_angle_deg_ / kCompassAveraging;
        compass_angle_deg_ += compass_deg / kCompassAveraging;

        debug("Compass: ");
        debug((int)compass_angle_deg_);
        debugln("°");
    }
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

    radio_->Send(packet);
}

void rover::Rover::SendData()
{
    // Negative integers are not efficient to encode in protobuf, so let's avoid -90° through 0° by normalizing
    // the heel angle such that 0° is full counter-clockwise deflection (laying flat to the left), and 180°
    // (integer value 1800) is full clockwise deflection (laying flat to the right). If the boat goes beyond these
    // angles, we have bigger problems than optimizing dwell time.
    int encoded_heel_angle = (int)((heel_angle_deg_ + 90.0) * 10);

    RoverData data;
    data.heading = random(360);
    data.heel = encoded_heel_angle;
    data.latitude = gps_->gps_.latitudeDegrees;
    data.longitude = gps_->gps_.longitudeDegrees;

    LoRaPacket packet;
    packet.hardwareID = util::get_hardware_id();
    packet.payload.roverData = data;
    packet.which_payload = LoRaPacket_roverData_tag;

    radio_->Send(packet);
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
