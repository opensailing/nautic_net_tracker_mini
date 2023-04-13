#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>

#include "debug.h"
#include "lora_packet.pb.h"
#include "main.h"
#include "nautic_net/base.h"
#include "nautic_net/hw/eeprom.h"
#include "nautic_net/hw/gps.h"
#include "nautic_net/hw/imu.h"
#include "nautic_net/hw/radio.h"
#include "nautic_net/rover.h"
#include "nautic_net/tdma.h"
#include "nautic_net/util.h"

using namespace nautic_net;

Mode kMode = Mode::kRover;

hw::eeprom::EEPROM kEEPROM;
hw::radio::Radio kRadio;
hw::imu::IMU kIMU;
hw::gps::GPS kGPS(&Serial1, config::kPinGPSPPS);
rover::Rover kRover(&kRadio, &kGPS, &kIMU);
base::Base kBase(&kRadio);
tdma::TDMA kTDMA;

static const int kSerialBufferSize = 128;
char serial_buffer_[kSerialBufferSize];
uint16_t serial_buffer_index_;

void setup()
{
  // A0 is disconnected, so we can seed with random noise
  randomSeed(analogRead(0));

  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Calibration switch
  pinMode(config::kPinCalibration, INPUT_PULLUP);

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

  // Configure hardware
  kEEPROM.Setup();
  kIMU.Setup();
  kRadio.Setup();
  kRover.Setup();
  kGPS.Setup();

  // Don't continue until GPS has a fix, because we need accurate timing
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
    serial_buffer_[serial_buffer_index_] = byte;

    if (byte == '\n')
    {
      // Replace \n with null terminator
      serial_buffer_[serial_buffer_index_] = 0;

      switch (serial_buffer_[0])
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

      case 's':
        Serial.println("--- STATUS ---");
        Serial.print("Battery: ");
        Serial.println(util::read_battery(), 2);
        break;

      case 'c':
        kIMU.BeginCompassCalibration();
        break;

      case 'f':
        kIMU.FinishCompassCalibration();
        break;

      case 'w':
        // Convert "w12345" to an integer and persist to EEPROM
        {
          uint32_t new_serial_number_ = (uint32_t)atoi(serial_buffer_ + 1);
          kEEPROM.WriteSerialNumber(new_serial_number_);
          Serial.println(new_serial_number_);
          break;
        }

      case 'x':
        Serial.println(kEEPROM.ReadSerialNumber());
        break;
      }

      serial_buffer_index_ = 0;
    }
    else
    {
      serial_buffer_index_ = (serial_buffer_index_ + 1) % kSerialBufferSize;
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
