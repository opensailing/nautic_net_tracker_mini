#include "imu.h"
#include "config.h"

using namespace nautic_net::hw::imu;

IMU::IMU()
{
}

void IMU::Setup()
{
    // In the base station, the IMU board may be omitted
    successful_init_ = accel_.begin_I2C() && magnet_.begin_I2C();
    if (!successful_init_)
    {
        return;
    }

    //
    // Configure accelerometer
    // https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/arduino
    //
    accel_.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    accel_.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
    accel_.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    accel_.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
    accel_.configInt1(false, false, true); // accelerometer DRDY on INT1
    accel_.configInt2(false, true, false); // gyro DRDY on INT2

    //
    // Configure magnetometer
    // https://learn.adafruit.com/lis3mdl-triple-axis-magnetometer/arduino
    //
    magnet_.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    magnet_.setDataRate(LIS3MDL_DATARATE_10_HZ);
    magnet_.setRange(LIS3MDL_RANGE_4_GAUSS);
    magnet_.setIntThreshold(500);
    magnet_.configInterrupt(false, false, true, // enable z axis
                            true,               // polarity
                            false,              // don't latch
                            true);              // enabled!
}

void IMU::Loop()
{
    if (!successful_init_)
    {
        return;
    }

    static float prev_millis;

    if (accel_.accelerationAvailable() && accel_.gyroscopeAvailable() && magnet_.magneticFieldAvailable())
    {
        //
        // Gyro & accelerometer
        //
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
        float pitch_m = -atan2(accel_x, accel_z); // pitch (theta)
        float roll_m = -atan2(accel_y, accel_z);  // roll (phi)

        // Smooth the data by taking the gyro into account
        pitch_ = (pitch_ + gyro_y * dt) * .95 + pitch_m * .05; // damped pitch (theta)
        roll_ = (roll_ - gyro_x * dt) * .95 + roll_m * .05;    // damped roll (phi)

        float heel_angle_deg = -roll_ * kRadToDeg;

        heel_angle_deg_ -= heel_angle_deg_ / kHeelAveraging;
        heel_angle_deg_ += heel_angle_deg / kHeelAveraging;

        //
        // Magnetometer
        //
        sensors_event_t magnet_event;
        magnet_.getEvent(&magnet_event);

        // Normalize physical measurements to expected coordinate system
        float raw_mag_x = ROVER_NORMAL_X(magnet_event.magnetic);
        float raw_mag_y = ROVER_NORMAL_Y(magnet_event.magnetic);
        float raw_mag_z = ROVER_NORMAL_Z(magnet_event.magnetic);

        if (is_calibrating_compass_)
        {
            compass_cal_x_min_ = min(compass_cal_x_min_, raw_mag_x);
            compass_cal_x_max_ = max(compass_cal_x_max_, raw_mag_x);
            compass_cal_y_min_ = min(compass_cal_y_min_, raw_mag_y);
            compass_cal_y_max_ = max(compass_cal_y_max_, raw_mag_y);
            compass_cal_z_min_ = min(compass_cal_z_min_, raw_mag_y);
            compass_cal_z_max_ = max(compass_cal_z_max_, raw_mag_y);
        }

        // Generate calibrated values
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

        if (nautic_net::config::kEnableIMULogging)
        {
            // Serial.print(mag_x_compensated);
            // Serial.print(',');
            // Serial.print(mag_y_compensated);
            // Serial.print(',');
            // Serial.println(mag_z);

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
}

void IMU::BeginCompassCalibration()
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
void IMU::FinishCompassCalibration()
{
    if (compass_cal_x_min_ == INFINITY)
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

        // TODO: Hook up I2C EEPROM
        // TODO: Save calibration in EEPROM

        is_calibrating_compass_ = false;
        is_compass_calibrated_ = true;
    }
}