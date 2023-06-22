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
hw::imu::IMU kIMU(&kEEPROM);
hw::gps::GPS kGPS(&Serial1, config::kPinGPSPPS);
rover::Rover kRover(&kRadio, &kGPS, &kIMU, &kEEPROM);
base::Base kBase(&kRadio);
tdma::TDMA kTDMA;

static const int kSerialBufferSize = 128;
char serial_buffer_[kSerialBufferSize];
uint16_t serial_buffer_index_;
// bool is_serial_connected_;

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

  // Serial and debug
  Serial.begin(115200);
  debugWait();

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
  // if (Serial && !is_serial_connected_)
  // {
  //   delay(50);
  //   PrintNarwin();
  //   PrintEEPROM();
  //   PrintStatus();
  //   is_serial_connected_ = true;
  // }
  // else if (!Serial && is_serial_connected_)
  // {
  //   is_serial_connected_ = false;
  // }

  if (Serial.available())
  {
    char byte = Serial.read();
    serial_buffer_[serial_buffer_index_] = byte;

    // Use line feed (LF) as the line separator
    if (byte == '\n')
    {
      // Replace \n with null terminator
      serial_buffer_[serial_buffer_index_] = 0;

      // If a CRLF ("\r\n") was used as the line terminator, replace \r with null terminator, too
      if (serial_buffer_index_ >= 1 && serial_buffer_[serial_buffer_index_ - 1] == '\r')
      {
        serial_buffer_[serial_buffer_index_ - 1] = 0;
      }

      switch (serial_buffer_[0])
      {
      case 'b': // Convert to base station mode
        debugln("--- BASE STATION ---");
        kMode = Mode::kBase;
        kBase.ResetConfiguration();
        break;

      case 'c': // Begin compass cal
        kIMU.BeginCompassCalibration();
        break;

      case 'e': // Read EEPROM
        PrintEEPROM();
        break;

      case 'f': // Finish compass cal
        kIMU.FinishCompassCalibration();
        break;

      case 'r': // Convert to rover mode (and reset any existing rover config)
        debugln("--- ROVER ---");
        kMode = Mode::kRover;
        kRover.ResetConfiguration();
        break;

      case 's': // Read or write serial number
      {
        if (serial_buffer_index_ == 1)
        {
          Serial.print("Serial number: ");
          Serial.println(kEEPROM.ReadSerialNumber());
        }
        else
        {
          // Convert "s12345" to an integer
          uint32_t new_serial_number_ = (uint32_t)atoi(serial_buffer_ + 1);

          kEEPROM.WriteSerialNumber(new_serial_number_);
          Serial.print("New serial number: ");
          Serial.println(new_serial_number_);
        }
        break;
      }

      case 'z': // Reset EEPROM to default values
        kEEPROM.Reset();
        Serial.println("Reset EEPROM to default values");
        break;

      case '?': // Print general info
        PrintStatus();
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

void PrintEEPROM()
{
  Serial.print("Serial number: ");
  Serial.println(kEEPROM.ReadSerialNumber());

  nautic_net::hw::eeprom::CompassCalibration cal = kEEPROM.ReadCompassCalibration();
  Serial.print("Compass cal X: ");
  Serial.println(cal.x);

  Serial.print("Compass cal Y: ");
  Serial.println(cal.y);

  Serial.print("Compass cal Z: ");
  Serial.println(cal.z);
}

void PrintStatus()
{
  Serial.println("--- STATUS ---");

  Serial.print("Firmware: ");
  Serial.println(config::kFirmwareVersion);

  Serial.print("Battery: ");
  Serial.print(util::ReadBatteryVoltage(), 2);
  Serial.print("V (");
  Serial.print(util::ReadBatteryPercentage());
  Serial.println("%)");

  Serial.print("Mode: ");
  if (kMode == Mode::kBase)
  {
    Serial.println("Base Station");
  }
  else
  {
    Serial.println("Rover");
  }
}

void PrintNarwin()
{
  Serial.println("                                       7D");
  Serial.println("                                    .7DD");
  Serial.println("                                  DDD7");
  Serial.println("                              . DDDD");
  Serial.println("                      DDDDD:. DDDD?");
  Serial.println("                  DDDDDDDDDDDDDDD.");
  Serial.println("                 DDDDDDDDDDDDDDDD");
  Serial.println("                DDDDDDD.?DDDDDDDDD");
  Serial.println("               7DDDDDD   DDDDDDDDD");
  Serial.println("               DDDDDDDDDDDDDDDDDDD");
  Serial.println("               DDDDDDDDDDDDDDDDDDD");
  Serial.println("               DDDDDDDDDDDDDDDDDDD");
  Serial.println("               DDDDDDDDDDDDDDDDDDD");
  Serial.println("               DDDDDDDDDDDDDDDDDDD");
  Serial.println("               DDDDDD    DDDDDDDDD");
  Serial.println("D   .DD       DDDDD     DDDDDDDDDD");
  Serial.println("DDDDDD?       DDDD      DDDDDDDDDD");
  Serial.println("DDDDD        .DDDD     DDDDDDDDDDD");
  Serial.println("  DDDD      DDDDD     DDDDDDDDDND");
  Serial.println("   DDDDDDDDDDDD     DDDDDDDDDDDD  ~~~~~~~");
  Serial.println("    DDDDDDDDDDD  .DDDDDDDdDDDD      ~~~~~~~~~~~~~");
  Serial.println("~~~~~ DDDDDDDDDDDDDDDDDDDDDD  ~~~~          ~~~~~~~~~~~");
  Serial.println("       dDDDDDDDDDDDDDDDD  ~~~~~~~~~~~~~~~~~~~~~~~~~~");
  Serial.println("           ~~~~~~~~~~~~~~~~~~~");
  Serial.println("~~~~~~~~~~~~~~~~          ~~~~~~~~~~~~");
}