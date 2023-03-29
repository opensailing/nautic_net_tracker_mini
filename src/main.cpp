#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>

#include "base.h"
#include "debug.h"
#include "gps.h"
#include "lora.pb.h"
#include "main.h"
#include "radio.h"
#include "rover.h"
#include "tdma.h"
#include "util.h"

Mode kMode = Mode::kRover;

radio::Radio kRadio;
gps::GPS kGPS(&Serial1, A5);
rover::Rover kRover(&kRadio, &kGPS);
base::Base kBase(&kRadio);
tdma::TDMA kTDMA;

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
}

void loop()
{
  //
  // Sync TDMA at the top of every 10th second
  //
  int second = kGPS.GetSyncedSecond();
  if (second != -1 && second % 10 == 0)
  {
    kTDMA.SyncToGPS();
  }
  kGPS.Read();

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
  if (kRadio.TryReceive(&rx_packet))
  {
    switch (kMode)
    {
    case Mode::kRover:
      kRover.HandlePacket(rx_packet);
      break;

    case Mode::kBase:
      kBase.HandlePacket(rx_packet);
      break;
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
      debugln("--- BASE STATION ---");
      kMode = Mode::kBase;
    }
    else if (byte == 'r')
    {
      debugln("--- ROVER ---");
      kMode = Mode::kRover;
      kRover.ResetConfiguration();
    }
  }
}
