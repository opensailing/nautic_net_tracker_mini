#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>

#include "debug.h"
#include "lora.pb.h"
#include "main.h"
#include "nautic_net/base.h"
#include "nautic_net/hw/gps.h"
#include "nautic_net/hw/imu.h"
#include "nautic_net/hw/radio.h"
#include "nautic_net/rover.h"
#include "nautic_net/tdma.h"
#include "nautic_net/util.h"

using namespace nautic_net;

Mode kMode = Mode::kRover;

hw::radio::Radio kRadio;
hw::imu::IMU kIMU;
hw::gps::GPS kGPS(&Serial1, config::kPinGPSPPS);
rover::Rover kRover(&kRadio, &kGPS, &kIMU);
base::Base kBase(&kRadio);
tdma::TDMA kTDMA;

void setup()
{
  // A0 is disconnected, so we can seed with random noise
  randomSeed(analogRead(0));

  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Determine initial mode - jumper A0 to ground to configure as base station
  pinMode(config::kPinBaseMode, INPUT_PULLUP);
  if (digitalRead(config::kPinBaseMode) == HIGH)
  {
    kMode = Mode::kRover;
  }
  else
  {
    kMode = Mode::kBase;
  }

  // Serial debug
  debugBegin(115200);
  // debugWait();

  // IMU
  kIMU.Setup();

  // Radio
  kRadio.Setup();

  // Rover
  kRover.Setup();

  // GPS
  kGPS.Setup();
  kGPS.WaitForFix();
}

void loop()
{
  //
  // Sync TDMA at the top of every 10th second
  //
  int second = kGPS.GetSyncedSecond();
  kTDMA.SyncToGPS(second);
  kGPS.Read();

  //
  // Give the IMU a chance
  //
  kIMU.Loop();

  //
  // Handle slot transitions
  //
  tdma::Slot newSlot;
  if (kTDMA.TryGetSlotTransition(&newSlot))
  {
    switch (kMode)
    {
    case Mode::kRover:
      kRover.HandleSlot(newSlot);
      break;

    case Mode::kBase:
      kBase.HandleSlot(newSlot);
      break;
    }
  }

  //
  // Handle received packets
  //
  LoRaPacket rx_packet;
  int rssi;
  if (kRadio.TryReceive(&rx_packet, &rssi))
  {
    switch (kMode)
    {
    case Mode::kRover:
      kRover.HandlePacket(rx_packet, rssi);
      break;

    case Mode::kBase:
      kBase.HandlePacket(rx_packet, rssi);
      break;
    }
  }

  //
  // Serial commands
  //
  if (Serial.available())
  {
    char byte = Serial.read();
    switch (byte)
    {
    case 'b':
      debugln("--- BASE STATION ---");
      kMode = Mode::kBase;
      break;

    case 'r':
      debugln("--- ROVER ---");
      kMode = Mode::kRover;
      kRover.ResetConfiguration();
      break;

    case 'c':
      Serial.println("--- BEGIN COMPASS CALIBRATION ---");
      kIMU.BeginCompassCalibration();
      break;

    case 'f':
      Serial.println("--- END COMPASS CALIBRATION ---");
      kIMU.FinishCompassCalibration();
      break;
    }
  }

  //
  // Give some processor time
  //
  switch (kMode)
  {
  case Mode::kRover:
    kRover.Loop();
    break;

  case Mode::kBase:
    // nothing yet
    break;
  }
}
