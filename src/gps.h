#include <Adafruit_GPS.h>
#include <Arduino.h>

#ifndef GPS_H
#define GPS_H

namespace gps
{
    class GPS
    {
    public:
        GPS(Uart *serial, int pps_pin);

        void Read();
        void Setup();
        void WaitForFix();
        int GetSyncedSecond();

    private:
        Adafruit_GPS gps_;

        int pps_pin_;
        int gps_seconds_ = -1;
        int prev_pps_ = LOW;
    };
}
#endif