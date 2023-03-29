#include <Adafruit_BusIO_Register.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "base.h"
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
State kState = State::kNoFix;

//
// Base station
//
base::Base kBase;

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
  kState = State::kUnconfiguredRover;
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
  }
  kGPS.Read();

  //
  // TDMA slot determination
  //
  tdma::SlotType newSlotType = kTDMA.GetSlotTransition();
  switch (newSlotType)
  {
  case tdma::SlotType::kRoverDiscovery:
    if (kState == State::kUnconfiguredRover)
    {
      // Delay 0-80 ms and then attempt to TX discovery
      delay(random(80));
      loraTxDiscovery();
    }
    break;

  case tdma::SlotType::kThisRoverData:
    if (kState == State::kConfiguredRover)
    {
      loraTxData();
    }
    break;

  case tdma::SlotType::kRoverConfiguration:
    LoRaPacket config_packet;
    if (kState == State::kBaseStation && kBase.TryPopConfigPacket(&config_packet))
    {
      kRadio.Send(config_packet);
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
    if (kState == State::kBaseStation && rx_packet.which_payload == LoRaPacket_roverDiscovery_tag)
    {
      kBase.DiscoverRover(rx_packet);
    }

    if (kState == State::kUnconfiguredRover && rx_packet.which_payload == LoRaPacket_roverConfiguration_tag)
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
  debugln("--- BASE STATION ---");
  kState = State::kBaseStation;
  kTDMA.ClearTxSlots();
}

void becomeRover()
{
  debugln("--- ROVER ---");
  kState = State::kUnconfiguredRover;
  kTDMA.ClearTxSlots();
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

  kState = State::kConfiguredRover;
}
