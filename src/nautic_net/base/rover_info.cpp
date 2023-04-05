#include "rover_info.h"

nautic_net::base::RoverInfo::RoverInfo(int hardware_id) : hardware_id_(hardware_id)
{
}

void nautic_net::base::RoverInfo::Configure(int slots[], nautic_net::hw::radio::Config radio_config)
{
    for (unsigned int i = 0; i < nautic_net::tdma::kRoverSlotCount; i++)
    {
        slots_[i] = slots[i];
    }

    radio_config_ = radio_config;
    is_configured_ = false;
}
