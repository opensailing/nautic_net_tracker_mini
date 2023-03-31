#ifndef ROVER_H
#define ROVER_H

#include "gps.h"
#include "imu.h"
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
        Rover(radio::Radio *radio, gps::GPS *gps, nautic_net::IMU *imu);
        void Setup();
        void Loop();
        void HandlePacket(LoRaPacket packet);
        void HandleSlot(tdma::Slot slot);
        void ResetConfiguration();

    private:
        radio::Radio *radio_;
        gps::GPS *gps_;
        nautic_net::IMU *imu_;
        RoverState state_;

        bool tx_slots_[tdma::kSlotCount]; // Which slots this rover is configured to TX during

        void SendDiscovery();
        void SendData();
        void Configure(LoRaPacket packet);
        bool IsMyTransmitSlot(tdma::Slot slot);
    };
}

#endif