#ifndef ROVER_H
#define ROVER_H

#include "lora.pb.h"
#include "nautic_net/hw/gps.h"
#include "nautic_net/hw/imu.h"
#include "nautic_net/hw/radio.h"
#include "nautic_net/tdma.h"

namespace nautic_net::rover
{
    enum class RoverState
    {
        kUnconfigured,
        kConfigured
    };

    class Rover
    {
    public:
        Rover(nautic_net::hw::radio::Radio *radio, nautic_net::hw::gps::GPS *gps, nautic_net::hw::imu::IMU *imu);
        void Setup();
        void Loop();
        void HandlePacket(LoRaPacket packet, int rssi);
        void HandleSlot(tdma::Slot slot);
        void ResetConfiguration();

    private:
        nautic_net::hw::radio::Radio *radio_;
        nautic_net::hw::gps::GPS *gps_;
        nautic_net::hw::imu::IMU *imu_;
        RoverState state_;

        bool tx_slots_[tdma::kSlotCount]; // Which slots this rover is configured to TX during

        void SendDiscovery();
        void SendData();
        void Configure(LoRaPacket packet);
        bool IsMyTransmitSlot(tdma::Slot slot);
    };
}

#endif