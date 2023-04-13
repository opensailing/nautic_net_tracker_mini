#ifndef EEPROM_H
#define EEPROM_H
#include <Adafruit_EEPROM_I2C.h>

namespace nautic_net::hw::eeprom
{
    class EEPROM
    {
    public:
        EEPROM();
        void Setup();
        void WriteSerialNumber(uint32_t number);
        uint32_t ReadSerialNumber();

    private:
        static const uint8_t kI2CAddress = 0x50;
        static const unsigned int kAddressSerialNumber = 0x00;

        bool initialized_;

        Adafruit_EEPROM_I2C eeprom_;
        uint32_t serial_number_;
    };
}

#endif