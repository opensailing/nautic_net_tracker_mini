#include "debug.h"
#include "gps.h"

using namespace nautic_net::hw::gps;

GPS::GPS(Uart *serial, int pps_pin) : gps_(Adafruit_GPS(serial)), pps_pin_(pps_pin)
{
}

void GPS::Setup()
{
    // PPS input
    pinMode(pps_pin_, INPUT);

    gps_.begin(9600);
    gps_.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    gps_.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}

void GPS::WaitForFix()
{
    debugln("Waiting for GPS fix...");
    digitalWrite(LED_BUILTIN, HIGH);

    while (!gps_.fix)
    {
        Read();
    }

    debugln("Got GPS fix");
    digitalWrite(LED_BUILTIN, LOW);
}

void GPS::Read()
{
    gps_.read();
    if (gps_.newNMEAreceived())
    {
        if (gps_.parse(gps_.lastNMEA()))
        {
            if (gps_.fix)
            {
                gps_seconds_ = gps_.seconds;
            }
        }
    }
}

int GPS::GetSyncedSecond()
{
    int pps = digitalRead(pps_pin_);
    if (prev_pps_ != pps)
    {
        prev_pps_ = pps;

        if (pps == HIGH && gps_seconds_ != -1)
        {
            return (gps_seconds_ + 1) % 60;
        }
    }

    return -1;
}