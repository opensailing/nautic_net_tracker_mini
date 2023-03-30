# NauticNet Tracker Mini

This is [PlatformIO](https://platformio.org/) project for the Adafruit Feather M0 platform.

## Hardware

- [Adafruit Feather M0 with RFM95](https://www.adafruit.com/product/3178)
- [Adafruit Ultimate GPS FeatherWing](https://www.adafruit.com/product/3133)
- [Adafruit LSM6DSOX + LIS3MDL FeatherWing](https://www.adafruit.com/product/4565)

### Additional Wiring

- A connection must be made between the GPS `PPS` output and pin `A5`.

## Serial commands

- `r` puts the unit into Rover mode (default)
- `b` puts the unit into Base Station mode

## Development

1. Install the [PlatformIO IDE extension for VS Code](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
2. Clone and open the repo.
3. Wait for dependencies to install.
4. Build and upload.
