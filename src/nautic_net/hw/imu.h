#ifndef IMU_H
#define IMU_H

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>

// Measured:   X is towards the sky, Y is towards starbord, Z is towards the bow
// Normalized: X is towards the bow, Y is towards port, Z is towards the sky
#define ROVER_NORMAL_X(value) value.z
#define ROVER_NORMAL_Y(value) -value.y
#define ROVER_NORMAL_Z(value) value.x

namespace nautic_net::hw::imu
{
    class IMU
    {
    public:
        IMU();
        void Setup();
        void Loop();
        void BeginCompassCalibration();
        void FinishCompassCalibration();

        float heel_angle_deg_;    // Latest measurement
        float compass_angle_deg_; // Latest measurement

    private:
        static constexpr float kRadToDeg = 180 / PI;

        Adafruit_LSM6DSOX accel_; // Accelerometer/gyro
        Adafruit_LIS3MDL magnet_; // Magnetometer

        bool is_calibrating_compass_;
        bool is_compass_calibrated_ = false;
        bool successful_init_ = false;

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
    };
}

#endif