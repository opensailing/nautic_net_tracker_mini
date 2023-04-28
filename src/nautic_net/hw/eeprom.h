#ifndef EEPROM_H
#define EEPROM_H
#include <Adafruit_EEPROM_I2C.h>

namespace nautic_net::hw::eeprom
{
    typedef struct
    {
        unsigned int version;
        float x;
        float y;
        float z;
    } CompassCalibration;

    class EEPROM
    {
    public:
        EEPROM();
        void Setup();
        void Reset();

        void WriteSerialNumber(uint32_t number);
        uint32_t ReadSerialNumber();

        void WriteCompassCalibration(CompassCalibration cal);
        CompassCalibration ReadCompassCalibration();

        uint32_t serial_number_;
        CompassCalibration compass_calibration_;

    private:
        static const uint8_t kI2CAddress = 0x50;
        static const unsigned int kFirstByte = 0x00;
        static const unsigned int kAddressSerialNumber = 0x01;
        static const unsigned int kAddressCompassCalibration = kAddressSerialNumber + 4;
        // NOTE: When adding the next address, leave some EXTRA bytes after kAddressCompassCalibration, in case it grows

        bool initialized_;
        Adafruit_EEPROM_I2C eeprom_;
    };
}

#endif