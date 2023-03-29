#include <Adafruit_BusIO_Register.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "debug.h"
#include "gps.h"
#include "lora.pb.h"
#include "main.h"
#include "radio.h"
#include "tdma.h"

tdma::TDMA kTDMA;

//
// State machine
//
State kState = StateNoFix;

//
// Base station
//
int _baseAvailableSlot = 2;
LoRaPacket _baseConfigQueue[20];
int _baseConfigQueueLength = 0;

//
// GPS
//
gps::GPS kGPS(&Serial1, A5);

//
// Magnetometer and IMU
//
Adafruit_LIS3MDL lis3mdl;
Adafruit_ISM330DHCX ism330dhcx;

//
// Radio
// https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/using-the-rfm-9x-radio
//
radio::Radio kRadio;

void setup()
{
  // A0 is disconnected, so we can seed with random noise
  randomSeed(analogRead(0));

  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Serial debug
  debugBegin(115200);
  // debugWait();

  // Radio
  kRadio.Setup();

  // GPS
  kGPS.Setup();
  kGPS.WaitForFix();

  // Transition to new state
  kState = StateUnconfiguredRover;
}

void loop()
{
  //
  // GPS time sync
  //
  int second = kGPS.GetSyncedSecond();
  if (second != -1 && second % 10 == 0)
  {
    kTDMA.SyncToGPS();

    debug("--- SYNC ");
    debug(second);
    debugln(" ---");
  }
  kGPS.Read();

  //
  // TDMA slot determination
  //
  tdma::SlotType newSlotType = kTDMA.GetSlotTransition();
  switch (newSlotType)
  {
  case tdma::SlotType::kRoverDiscovery:
    if (kState == StateUnconfiguredRover)
    {
      // Delay 0-80 ms and then attempt to TX discovery
      delay(random(80));
      loraTxDiscovery();
    }
    break;

  case tdma::SlotType::kThisRoverData:
    if (kState == StateConfiguredRover)
    {
      loraTxData();
    }
    break;

  case tdma::SlotType::kRoverConfiguration:
    if (kState == StateBase && _baseConfigQueueLength > 0)
    {
      // Pop off the head of the queue
      kRadio.Send(_baseConfigQueue[0]);

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
  LoRaPacket rx_packet;
  if (kRadio.TryReceive(&rx_packet))
  {
    if (kState == StateBase && rx_packet.which_payload == LoRaPacket_roverDiscovery_tag)
    {
      baseHandleRoverDiscovery(rx_packet);
    }

    if (kState == StateUnconfiguredRover && rx_packet.which_payload == LoRaPacket_roverConfiguration_tag)
    {
      roverHandleConfiguration(rx_packet);
    }
  }

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

void loraTxDiscovery()
{
  RoverDiscovery discovery;

  LoRaPacket packet;
  packet.hardwareID = getHardwareID();
  packet.payload.roverDiscovery = discovery;
  packet.which_payload = LoRaPacket_roverDiscovery_tag;

  kRadio.Send(packet);
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

  kRadio.Send(packet);
}

void becomeBase()
{
  debugln("--- BASE ---");
  kState = StateBase;
  kTDMA.ClearTxSlots();
}

void becomeRover()
{
  debugln("--- ROVER ---");
  kState = StateUnconfiguredRover;
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

  kState = StateConfiguredRover;
}
