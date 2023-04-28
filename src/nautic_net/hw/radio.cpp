#include "debug.h"
#include "radio.h"

using namespace nautic_net::hw::radio;

RH_RF95 kRF95(RFM95_CS, RFM95_INT);

Radio::Radio()
{
}

void Radio::Setup()
{
    debugln("Beginning radio setup");
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

    kRF95.setTxPower(config::kLoraPower, false);

    Configure(config::kLoraDefaultConfig);

    debugln("Radio setup complete");
}

void Radio::Configure(Config config)
{
    if (config.sbw != current_config_.sbw)
    {
        kRF95.setSignalBandwidth(config.sbw * 1000);
    }

    if (config.sf != current_config_.sf)
    {
        kRF95.setSpreadingFactor(config.sf);
    }

    current_config_ = config;
}

size_t Radio::Send(LoRaPacket packet)
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

bool Radio::TryReceive(LoRaPacket *rx_packet, int *rssi)
{
    if (kRF95.available())
    {
        // Should be a message for us now

        uint8_t buffer[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t length = sizeof(buffer);

        if (kRF95.recv(buffer, &length))
        {
            pb_istream_t stream = pb_istream_from_buffer(buffer, length);
            pb_decode(&stream, LoRaPacket_fields, rx_packet);
            *rssi = kRF95.lastRssi();

            // Print packet as hexadecimal, for consumption by nautic_net_device
            Serial.print("LORA,");
            Serial.print(*rssi);
            Serial.print(',');
            for (int i = 0; i < length; i++)
            {
                // Print leading "0" if necessary
                if (buffer[i] < 16)
                {
                    Serial.print("0");
                }
                Serial.print(buffer[i], 16);
            }
            Serial.println();

            debug("RX <-   ");
            debug(length);
            debug(" (");
            debug(*rssi);
            debug(" dBm): ");
            DebugPacketType(*rx_packet);
            return true;
        }
    }

    return false;
}

void Radio::DebugPacketType(LoRaPacket packet)
{
    switch (packet.which_payload)
    {
    case LoRaPacket_rover_discovery_tag:
        debugln("RoverDiscovery");
        break;
    case LoRaPacket_rover_data_tag:
        debugln("RoverData");
        break;
    case LoRaPacket_rover_configuration_tag:
        debugln("RoverConfiguration");
        break;
    default:
        debugln("Unknown");
        break;
    }
}
