#ifndef ROVER_H
#define ROVER_H

#include <vector>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>

#include "gps.h"
#include "lora.pb.h"
#include "radio.h"
#include "tdma.h"

// Measured:   X is towards the sky, Y is towards starbord, Z is towards the bow
// Normalized: X is towards the bow, Y is towards port, Z is towards the sky
#define ROVER_NORMAL_X(value) value.z
#define ROVER_NORMAL_Y(value) -value.y
#define ROVER_NORMAL_Z(value) value.x

namespace rover
{
    enum class RoverState
    {
        kUnconfigured,
        kConfigured
    };

    static const float kRadToDeg = 180 / PI;

    class Rover
    {
    public:
        Rover(radio::Radio *radio, gps::GPS *gps);
        void Setup();
        void Loop();
        void HandlePacket(LoRaPacket packet);
        void HandleSlot(tdma::Slot slot);
        void ResetConfiguration();
        void BeginCompassCalibration();
        void FinishCompassCalibration();

    private:
        radio::Radio *radio_;
        gps::GPS *gps_;
        RoverState state_;
        Adafruit_ISM330DHCX accel_;       // Accelerometer/gyro
        Adafruit_LIS3MDL magnet_;         // Magnetometer
        bool tx_slots_[tdma::kSlotCount]; // Which slots this rover is configured to TX during
        float heel_angle_deg_;            // Latest measurement from accelerometer
        float compass_angle_deg_;         // Latest measurement from magnetometer
        bool is_calibrating_compass_;
        bool is_compass_calibrated_ = false;

        float compass_cal_x_min_ = INFINITY;
        float compass_cal_x_max_ = -INFINITY;
        float compass_cal_y_min_ = INFINITY;
        float compass_cal_y_max_ = -INFINITY;
        float compass_cal_z_min_ = INFINITY;
        float compass_cal_z_max_ = -INFINITY;

        float compass_x_calibration_;
        float compass_y_calibration_;
        float compass_z_calibration_;
        float pitch_; // theta
        float roll_;  // phi

        static const int kHeelAveraging = 6;     // Determined experimentally based on what "looks right"
        static const int kCompassAveraging = 10; // Determined experimentally based on what "looks right"

        void SendDiscovery();
        void SendData();
        void Configure(LoRaPacket packet);
        bool IsMyTransmitSlot(tdma::Slot slot);
    };
}

#endif