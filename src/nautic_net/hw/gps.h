#include <Adafruit_GPS.h>
#include <Arduino.h>

#ifndef GPS_H
#define GPS_H

namespace nautic_net::hw::gps
{
    class GPS
    {
    public:
        GPS(Uart *serial, int pps_pin);

        Adafruit_GPS gps_;

        void Read();
        void Setup();
        void WaitForFix();
        int GetSyncedSecond();

    private:
        int pps_pin_;
        int gps_seconds_ = -1;
        int prev_pps_ = LOW;
    };
}
#endif