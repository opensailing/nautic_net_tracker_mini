#include "eeprom.h"
#include "debug.h"

using namespace nautic_net::hw::eeprom;

EEPROM::EEPROM()
{
}

void EEPROM::Setup()
{
    debugln("Beginning EEPROM setup");

    if (eeprom_.begin(kI2CAddress))
    {
        debugln("Connected to EEPROM");
        // EEPROM is initialized with 0xFF in every address, so let's put in some default values
        // if it hasn't been initialized yet
        if (eeprom_.read(kFirstByte) == 0xFF)
        {
            Reset();
        }
        initialized_ = true;
        ReadSerialNumber();
    }
    else
    {
        debugln("Failed to connect to EEPROM");
    }
    debugln("EEPROM setup complete");
}

uint32_t EEPROM::ReadSerialNumber()
{
    if (!initialized_)
    {
        return 0;
    }

    uint8_t buffer[sizeof(uint32_t)];
    uint32_t result;

    eeprom_.read(kAddressSerialNumber, buffer, sizeof(uint32_t));
    memcpy((void *)&result, buffer, sizeof(uint32_t));

    serial_number_ = result;
    return result;
}

void EEPROM::WriteSerialNumber(uint32_t number)
{
    if (!initialized_)
    {
        return;
    }

    uint8_t buffer[sizeof(uint32_t)];

    memcpy(buffer, (void *)&number, sizeof(uint32_t));
    eeprom_.write(kAddressSerialNumber, buffer, sizeof(uint32_t));
}

CompassCalibration EEPROM::ReadCompassCalibration()
{
    if (!initialized_)
    {
        return {};
    }

    uint8_t buffer[sizeof(CompassCalibration)];
    CompassCalibration result;

    eeprom_.read(kAddressCompassCalibration, buffer, sizeof(CompassCalibration));
    memcpy((void *)&result, buffer, sizeof(CompassCalibration));

    compass_calibration_ = result;
    return result;
}

void EEPROM::WriteCompassCalibration(CompassCalibration cal)
{
    if (!initialized_)
    {
        return;
    }

    uint8_t buffer[sizeof(CompassCalibration)];

    memcpy(buffer, (void *)&cal, sizeof(CompassCalibration));
    eeprom_.write(kAddressCompassCalibration, buffer, sizeof(CompassCalibration));
}

void EEPROM::Reset()
{
    eeprom_.write(kFirstByte, 0);
    WriteSerialNumber(0);
    WriteCompassCalibration({});
}