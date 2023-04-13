#include "eeprom.h"

using namespace nautic_net::hw::eeprom;

EEPROM::EEPROM()
{
}

void EEPROM::Setup()
{
    if (eeprom_.begin(kI2CAddress))
    {
        initialized_ = true;
        ReadSerialNumber();
    }
}

uint32_t EEPROM::ReadSerialNumber()
{
    if (!initialized_)
    {
        return 0;
    }

    uint8_t buffer[4];
    uint32_t result;

    eeprom_.read(kAddressSerialNumber, buffer, 4);
    memcpy((void *)&result, buffer, 4);

    serial_number_ = result;
    return result;
}

void EEPROM::WriteSerialNumber(uint32_t number)
{
    if (!initialized_)
    {
        return;
    }

    uint8_t buffer[4];

    memcpy(buffer, (void *)&number, 4);
    eeprom_.write(kAddressSerialNumber, buffer, 4);
}