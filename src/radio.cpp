#include "debug.h"
#include "radio.h"

RH_RF95 kRF95(RFM95_CS, RFM95_INT);

radio::Radio::Radio()
{
}

void radio::Radio::Setup()
{

    pinMode(RFM95_RST, OUTPUT);

    // Manually reset radio
    digitalWrite(RFM95_RST, HIGH);
    delay(100);
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!kRF95.init())
    {
        debugln("LoRa radio init failed");
        while (1)
            ;
    }
    debugln("LoRa radio init OK!");

    if (!kRF95.setFrequency(RF95_FREQ))
    {
        debugln("setFrequency failed");
        while (1)
            ;
    }

    debug("Set Freq to: ");
    debugln(RF95_FREQ);

    kRF95.setTxPower(RF95_POWER, false);
    kRF95.setSignalBandwidth(RF95_SBW);
    kRF95.setSpreadingFactor(RF95_SF);
}

size_t radio::Radio::Send(LoRaPacket packet)
{
    uint8_t buffer[RH_RF95_MAX_MESSAGE_LEN];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    pb_encode(&stream, LoRaPacket_fields, &packet);

    digitalWrite(LED_BUILTIN, HIGH);
    unsigned long started_at = millis();
    kRF95.send((uint8_t *)buffer, stream.bytes_written);
    kRF95.waitPacketSent();
    long airtime = millis() - started_at;
    digitalWrite(LED_BUILTIN, LOW);

    debug("TX   -> ");
    debug(stream.bytes_written);
    debug(" (");
    debug(airtime);
    debug("ms): ");
    DebugPacketType(packet);

    return stream.bytes_written;
}

bool radio::Radio::TryReceive(LoRaPacket *rx_packet)
{
    if (kRF95.available())
    {
        // Should be a message for us now
        uint8_t buffer[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t length = sizeof(buffer);

        if (kRF95.recv(buffer, &length))
        {
            LoRaPacket packet;
            pb_istream_t stream = pb_istream_from_buffer(buffer, length);
            pb_decode(&stream, LoRaPacket_fields, &packet);

            debug("RX <-   ");
            debug(length);
            debug(": ");
            DebugPacketType(packet);

            rx_packet = &packet;
            return true;
        }
    }

    return false;
}

void radio::Radio::DebugPacketType(LoRaPacket packet)
{
    switch (packet.which_payload)
    {
    case LoRaPacket_roverDiscovery_tag:
        debugln("RoverDiscovery");
        break;
    case LoRaPacket_roverData_tag:
        debugln("RoverData");
        break;
    case LoRaPacket_roverConfiguration_tag:
        debugln("RoverConfiguration");
        break;
    default:
        debugln("Unknown");
        break;
    }
}
