#ifndef ROVER_H
#define ROVER_H

#include <vector>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>

#include "gps.h"
#include "lora.pb.h"
#include "radio.h"
#include "tdma.h"

namespace rover
{
    enum class RoverState
    {
        kUnconfigured,
        kConfigured
    };

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
        sensors_event_t latest_sensor_;
        bool is_compass_calibrated_ = false;

        float compass_cal_y_min_ = INFINITY;
        float compass_cal_y_max_ = -INFINITY;
        float compass_cal_z_min_ = INFINITY;
        float compass_cal_z_max_ = -INFINITY;

        float compass_y_offset_;
        float compass_z_offset_;

        static const int kHeelAveraging = 6;     // Determined experimentally based on what "looks right"
        static const int kCompassAveraging = 10; // Determined experimentally based on what "looks right"

        void SendDiscovery();
        void SendData();
        void Configure(LoRaPacket packet);
        bool IsMyTransmitSlot(tdma::Slot slot);
    };
}

#endif