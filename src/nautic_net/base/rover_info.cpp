#include "rover_info.h"

nautic_net::base::RoverInfo::RoverInfo(unsigned int hardware_id, unsigned int serial_number) : hardware_id_(hardware_id), serial_number_(serial_number)
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
