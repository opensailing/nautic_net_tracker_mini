#include "debug.h"
#include "util.h"

volatile uint32_t nautic_net::util::get_hardware_id()
{
    return *(volatile uint32_t *)0x0080A00C;
}

void nautic_net::util::print_serial_number()
{
    // Source: https://microchip.my.site.com/s/article/Reading-unique-serial-number-on-SAM-D20-SAM-D21-SAM-R21-devices
    //
    // Some examples:
    // 866b43ee 50304c4b 552e3120 ff07270a
    // 3848db81 50304c4b 552e3120 ff073132
    volatile uint32_t val1, val2, val3, val4;
    volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
    val1 = *ptr1;
    volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
    val2 = *ptr;
    ptr++;
    val3 = *ptr;
    ptr++;
    val4 = *ptr;

    debug("chip id: 0x");
    char buf[33];
    sprintf(buf, "%8x%8x%8x%8x", (unsigned int)val1, (unsigned int)val2, (unsigned int)val3, (unsigned int)val4);
    debugln(buf);
}

float nautic_net::util::read_battery()
{
    float measured_vbat = analogRead(nautic_net::config::kPinBattery);
    measured_vbat *= 2;    // we divided by 2, so multiply back
    measured_vbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measured_vbat /= 1024; // convert to voltage
    return measured_vbat;
}