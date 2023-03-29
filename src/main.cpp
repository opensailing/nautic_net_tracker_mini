#include <Adafruit_BusIO_Register.h>
#include <Adafruit_GPS.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Wire.h>

#include "main.h"
#include "tdma.h"
#include "lora.pb.h"

tdma::TDMA kTDMA;

//
// State machine
//
State _state = StateNoFix;

//
// Base station
//
int _baseAvailableSlot = 2;
LoRaPacket _baseConfigQueue[20];
int _baseConfigQueueLength = 0;

//
// GPS
//
#define GPSSerial Serial1
const byte PIN_PPS = A5;

Adafruit_GPS _gps(&GPSSerial);
int _gpsSeconds = -1;
int _prevPPS = LOW;
unsigned long _ppsAt = 0;

//
// Magnetometer and IMU
//
Adafruit_LIS3MDL lis3mdl;
Adafruit_ISM330DHCX ism330dhcx;

//
// Radio
// https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/using-the-rfm-9x-radio
//
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define RF95_SF 7
#define RF95_SBW 500000
#define RF95_POWER 20
RH_RF95 _rf95(RFM95_CS, RFM95_INT);

void setup()
{
  // A0 is disconnected, so we can seed with random noise
  randomSeed(analogRead(0));

  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Serial debug
  debugBegin(115200);

  // Radio
  radioSetup();

  // GPS
  gpsSetup();
  gpsWaitForFix();

  // Transition to new state
  _state = StateUnconfiguredRover;
}

void loop()
{
  //
  // GPS time sync
  //
  loopCheckPPS();
  gpsRead();

  //
  // TDMA slot determination
  //
  tdma::SlotType newSlotType = kTDMA.GetSlotTransition();
  switch (newSlotType)
  {
  case tdma::SlotType::kRoverDiscovery:
    if (_state == StateUnconfiguredRover)
    {
      // Delay 0-80 ms and then attempt to TX discovery
      delay(random(80));
      loraTxDiscovery();
    }
    break;

  case tdma::SlotType::kThisRoverData:
    if (_state == StateConfiguredRover)
    {
      loraTxData();
    }
    break;

  case tdma::SlotType::kRoverConfiguration:
    if (_state == StateBase && _baseConfigQueueLength > 0)
    {
      // Pop off the head of the queue
      loraTx(_baseConfigQueue[0]);

      // Shift the rest of the queue down
      _baseConfigQueueLength--;
      for (int i = 0; i < _baseConfigQueueLength; i++)
      {
        _baseConfigQueue[i] = _baseConfigQueue[i + 1];
      }
    }
    break;

  default:
    break;
  }

  //
  // Radio RX
  //
  radioRx();

  //
  // Serial commands
  //
  if (Serial.available())
  {
    char byte = Serial.read();
    if (byte == 'b')
    {
      becomeBase();
    }
    else if (byte == 'r')
    {
      becomeRover();
    }
  }
}

void loopCheckPPS()
{
  int pps = digitalRead(PIN_PPS);
  if (_prevPPS != pps)
  {
    if (pps == HIGH && _gpsSeconds != -1)
    {
      _ppsAt = micros();

      int currentGPSSeconds = (_gpsSeconds + 1) % 60;
      if (currentGPSSeconds % 10 == 0)
      {
        kTDMA.SyncToGPS();
        debugln("--- SYNC ---");
      }

      // Turn on LED when PPS pulse occurs
      digitalWrite(LED_BUILTIN, HIGH);
    }

    _prevPPS = pps;
  }

  // Turn off LED 100ms after PPS pulse
  if (micros() - _ppsAt > 100000)
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void printChipId()
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
};

volatile uint32_t getHardwareID()
{
  return *(volatile uint32_t *)0x0080A00C;
}

int loraTx(LoRaPacket packet)
{
  uint8_t buffer[RH_RF95_MAX_MESSAGE_LEN];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  pb_encode(&stream, LoRaPacket_fields, &packet);

  unsigned long startedAt = millis();
  _rf95.send((uint8_t *)buffer, stream.bytes_written);
  _rf95.waitPacketSent();
  long airtime = millis() - startedAt;

  debug("TX   -> ");
  debug(stream.bytes_written);
  debug(" (");
  debug(airtime);
  debug("ms): ");
  debugPacketType(packet);

  return stream.bytes_written;
}

void loraTxDiscovery()
{
  RoverDiscovery discovery;

  LoRaPacket packet;
  packet.hardwareID = getHardwareID();
  packet.payload.roverDiscovery = discovery;
  packet.which_payload = LoRaPacket_roverDiscovery_tag;

  loraTx(packet);
}

void loraTxData()
{
  RoverData data;
  data.heading = random(360);
  data.heel = random(200) - 100;
  data.latitude = 40.0;
  data.longitude = -70.0;

  LoRaPacket packet;
  packet.hardwareID = getHardwareID();
  packet.payload.roverData = data;
  packet.which_payload = LoRaPacket_roverData_tag;

  loraTx(packet);
}

void radioSetup()
{
  pinMode(RFM95_RST, OUTPUT);

  // Manually reset radio
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!_rf95.init())
  {
    debugln("LoRa radio init failed");
    while (1)
      ;
  }
  debugln("LoRa radio init OK!");

  if (!_rf95.setFrequency(RF95_FREQ))
  {
    debugln("setFrequency failed");
    while (1)
      ;
  }

  debug("Set Freq to: ");
  debugln(RF95_FREQ);

  _rf95.setTxPower(RF95_POWER, false);
  _rf95.setSignalBandwidth(RF95_SBW);
  _rf95.setSpreadingFactor(RF95_SF);
}

void gpsSetup()
{
  // PPS input
  pinMode(PIN_PPS, INPUT);

  _gps.begin(9600);
  _gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  _gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}

void gpsWaitForFix()
{
  digitalWrite(LED_BUILTIN, HIGH);

  while (!_gps.fix)
  {
    gpsRead();
  }

  digitalWrite(LED_BUILTIN, LOW);
}

void gpsRead()
{
  _gps.read();
  if (_gps.newNMEAreceived())
  {
    if (_gps.parse(_gps.lastNMEA()))
    {
      if (_gps.fix)
      {
        _gpsSeconds = _gps.seconds;
      }
    }
  }
}

void becomeBase()
{
  debugln("--- BASE ---");
  _state = StateBase;
  kTDMA.ClearTxSlots();
}

void becomeRover()
{
  debugln("--- ROVER ---");
  _state = StateUnconfiguredRover;
  kTDMA.ClearTxSlots();
}

void baseHandleRoverDiscovery(LoRaPacket packet)
{
  // Build the config packet
  RoverConfiguration config;
  config.slots_count = 10;
  for (int i = 0; i < 10; i++)
  {
    config.slots[i] = _baseAvailableSlot + (10 * i);
  }

  LoRaPacket configPacket;
  configPacket.hardwareID = packet.hardwareID;
  configPacket.payload.roverConfiguration = config;
  configPacket.which_payload = LoRaPacket_roverConfiguration_tag;

  // Enqueue for TX later
  _baseConfigQueue[_baseConfigQueueLength] = configPacket;
  _baseConfigQueueLength++;

  // Determine the next set of slots to hand out (2 through 9, inclusive)
  if (_baseAvailableSlot == 9)
  {
    _baseAvailableSlot = 2;
  }
  else
  {
    _baseAvailableSlot++;
  }
}

void roverHandleConfiguration(LoRaPacket packet)
{
  // Ignore configs destined for other rovers
  if (packet.hardwareID != getHardwareID())
  {
    return;
  }

  kTDMA.ClearTxSlots();

  for (unsigned int i = 0; i < packet.payload.roverConfiguration.slots_count; i++)
  {
    kTDMA.EnableTxSlot(packet.payload.roverConfiguration.slots[i]);
  }

  _state = StateConfiguredRover;
}

void radioRx()
{
  if (_rf95.available())
  {
    // Should be a message for us now
    uint8_t buffer[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t length = sizeof(buffer);

    if (_rf95.recv(buffer, &length))
    {
      LoRaPacket packet;
      pb_istream_t stream = pb_istream_from_buffer(buffer, length);
      pb_decode(&stream, LoRaPacket_fields, &packet);

      debug("RX <-   ");
      debug(length);
      debug(": ");
      debugPacketType(packet);

      if (_state == StateBase && packet.which_payload == LoRaPacket_roverDiscovery_tag)
      {
        baseHandleRoverDiscovery(packet);
      }

      if (_state == StateUnconfiguredRover && packet.which_payload == LoRaPacket_roverConfiguration_tag)
      {
        roverHandleConfiguration(packet);
      }
    }
  }
}

void debugPacketType(LoRaPacket packet)
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
