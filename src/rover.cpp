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
    accel_.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    accel_.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
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
    static float prev_millis;

    if (accel_.accelerationAvailable() && accel_.gyroscopeAvailable())
    {
        sensors_event_t accel_event;
        sensors_event_t gyro_event;
        sensors_event_t temp_event;
        accel_.getEvent(&accel_event, &gyro_event, &temp_event);

        float accel_x = ROVER_NORMAL_X(accel_event.acceleration); // towards bow
        float accel_y = ROVER_NORMAL_Y(accel_event.acceleration); // towards port
        float accel_z = ROVER_NORMAL_Z(accel_event.acceleration); // towards sky

        // Use the gyro to estimate short-term changes
        float gyro_x = ROVER_NORMAL_X(gyro_event.gyro); // towards bow
        float gyro_y = ROVER_NORMAL_Y(gyro_event.gyro); // towards port

        // Gyro reading is rad/sec, so we need to know how much time has elapsed since the last measurement
        float dt = (millis() - prev_millis) / 1000.0;
        prev_millis = millis();

        // Calculate instantaneous pitch and roll
        float pitch_m = -atan2(accel_x, accel_z); // pitch
        float roll_m = -atan2(accel_y, accel_z);  // roll

        // Smooth the data by taking the gyro into account
        pitch_ = (pitch_ + gyro_y * dt) * .95 + pitch_m * .05; // damped pitch
        roll_ = (roll_ - gyro_x * dt) * .95 + roll_m * .05;    // damped roll

        float heel_angle_deg = -roll_ * kRadToDeg;

        heel_angle_deg_ -= heel_angle_deg_ / kHeelAveraging;
        heel_angle_deg_ += heel_angle_deg / kHeelAveraging;
    }

    if (magnet_.magneticFieldAvailable())
    {
        sensors_event_t event;
        magnet_.getEvent(&event);

        // Normalize physical measurements to expected coordinate system
        float raw_mag_x = ROVER_NORMAL_X(event.magnetic);
        float raw_mag_y = ROVER_NORMAL_Y(event.magnetic);
        float raw_mag_z = ROVER_NORMAL_Z(event.magnetic);

        // Calculate calibrated values
        float mag_x = raw_mag_x + compass_x_calibration_;
        float mag_y = raw_mag_y + compass_y_calibration_;
        float mag_z = raw_mag_z + compass_z_calibration_;

        // Tilt compensation
        float mag_x_compensated = mag_x * cos(pitch_) - mag_y * sin(roll_) * sin(pitch_) + mag_z * cos(roll_) * sin(pitch_);
        float mag_y_compensated = mag_y * cos(roll_) + mag_z * sin(roll_);

        // Finally calculate compass angle
        float compass_rad = atan2(mag_y_compensated, mag_x_compensated);
        float compass_deg = compass_rad * kRadToDeg;

        // TODO: Compass smoothing based on gyro

        if (is_calibrating_compass_)
        {
            compass_cal_x_min_ = min(compass_cal_x_min_, mag_x);
            compass_cal_x_max_ = max(compass_cal_x_max_, mag_x);
            compass_cal_y_min_ = min(compass_cal_y_min_, mag_y);
            compass_cal_y_max_ = max(compass_cal_y_max_, mag_y);
            compass_cal_z_min_ = min(compass_cal_z_min_, mag_y);
            compass_cal_z_max_ = max(compass_cal_z_max_, mag_y);
        }

        Serial.print("/*");
        Serial.print(pitch_ * 180.0 / PI);
        Serial.print(",");
        Serial.print(roll_ * 180.0 / PI);
        Serial.print(",");
        Serial.print(mag_x_compensated);
        Serial.print(",");
        Serial.print(mag_y_compensated);
        Serial.print(",");
        Serial.print(compass_deg < 0 ? compass_deg + 360.0 : compass_deg);
        Serial.print(",");
        Serial.print(mag_x);
        Serial.print(",");
        Serial.print(mag_y);
        Serial.print(",");
        Serial.print(mag_z);
        Serial.println("*/");
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

void rover::Rover::BeginCompassCalibration()
{
    is_calibrating_compass_ = true;
    compass_x_calibration_ = 0.0;
    compass_y_calibration_ = 0.0;
    compass_z_calibration_ = 0.0;
    compass_cal_x_min_ = INFINITY;
    compass_cal_x_max_ = -INFINITY;
    compass_cal_y_min_ = INFINITY;
    compass_cal_y_max_ = -INFINITY;
    compass_cal_z_min_ = INFINITY;
    compass_cal_z_max_ = -INFINITY;
}

//
// Implementing this algorithm:
// https://www.fierceelectronics.com/components/compensating-for-tilt-hard-iron-and-soft-iron-effects
//
void rover::Rover::FinishCompassCalibration()
{
    if (compass_cal_y_min_ == INFINITY)
    {
        is_calibrating_compass_ = false;
        is_compass_calibrated_ = false;
    }
    else
    {
        // Make these negative so it is simple addition to apply them
        compass_x_calibration_ = -(compass_cal_x_min_ + compass_cal_x_max_) / 2.0; // alpha
        compass_y_calibration_ = -(compass_cal_y_min_ + compass_cal_y_max_) / 2.0; // beta
        compass_z_calibration_ = -(compass_cal_z_min_ + compass_cal_z_max_) / 2.0;
        is_calibrating_compass_ = false;
        is_compass_calibrated_ = true;
    }

    // TODO: Save calibration in EEPROM
}