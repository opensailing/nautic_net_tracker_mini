#ifndef ROVER_INFO_H
#define ROVER_INFO_H

#include "config.h"
#include "nautic_net/hw/radio.h"
#include "nautic_net/tdma.h"

namespace nautic_net::base
{
    class RoverInfo
    {
    public:
        unsigned int hardware_id_;
        unsigned int serial_number_;
        bool is_configured_;
        int slots_[nautic_net::tdma::kRoverSlotCount];
        nautic_net::hw::radio::Config radio_config_;

        RoverInfo(unsigned int hardware_id, unsigned int serial_number);
        void Configure(int slots[], nautic_net::hw::radio::Config radio_config);
    };
}

#endif
