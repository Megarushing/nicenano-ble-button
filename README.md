# BLE Button (nice!nano v2)

Simple Arduino code for a NRF52840 using BTHome and a GPIO on/off characteristic (P0.17), with the onboard LED as an indicator.

## Overview
- Advertises BTHome v2 service data (battery %, GPIO switch state, USB power present)
- Exposes Nordic LED Button Service (LBS) characteristics for GPIO on/off control
- Targets nice!nano v2 (nRF52840) with main GPIO on P0.17 and built-in LED on P0.15 as a mirror indicator

## Arduino IDE setup (required)
The project uses the Adafruit nRF52 Arduino core with a Nice!Nano v2 fork. Follow these steps exactly.

### BSP Installation
There are two methods that you can use to install this BSP. We highly recommend the first option unless you wish to participate in active development of this codebase via GitHub.

#### Recommended: Adafruit nRF52 BSP via the Arduino Board Manager
1. Download and install the Arduino IDE (at least v1.6.12).
2. Start the Arduino IDE.
3. Go into Preferences.
4. Add `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json` as an "Additional Board Manager URL".
5. Restart the Arduino IDE.
6. Open the Boards Manager from the Tools -> Board menu and install "Adafruit nRF52 by Adafruit".
7. Once the BSP is installed, select "Adafruit Feather nRF52840 Express" from the Tools -> Board menu, which will update your system config to use the right compiler and settings for the nRF52.

#### Required for nice!nano v2 board: Adafruit nRF52 BSP via git
1. Install BSP via Board Manager as above to install compiler & tools.
2. Delete the core folder `nrf52` installed by Board Manager in Arduino15, depending on your OS. It could be:
   macOS: `~/Library/Arduino15/packages/adafruit/hardware/nrf52`
   Linux: `~/.arduino15/packages/adafruit/hardware/nrf52`
   Windows: `%APPDATA%\Local\Arduino15\packages\adafruit\hardware\nrf52`
3. `cd <SKETCHBOOK>`, where `<SKETCHBOOK>` is your Arduino Sketch folder:
   macOS: `~/Documents/Arduino`
   Linux: `~/Arduino`
   Windows: `~/Documents/Arduino`
4. Create a folder named `hardware/adafruit`, if it does not exist, and change directories to it.
5. Clone this repo & its submodules:

   ```bash
   git clone https://github.com/selimmeric/Adafruit_nRF52_Arduino_Nice-NanoV2
   cd Adafruit_nRF52_Arduino_Nice-NanoV2
   git submodule update --init
   ```

6. Restart the Arduino IDE.
7. Once the BSP is installed, select "nice!nano v2" from the Tools -> Board menu, which will update your system config to use the right compiler and settings for the nRF52.

### Adafruit's nrfutil tools
`adafruit-nrfutil` (derived from Nordic `pc-nrfutil`) is needed to upload sketches via serial port.

- For Windows and macOS, pre-built executable binaries are included in the BSP at `tools/adafruit-nrfutil/`. It should work out of the box.
- Linux users need to run the following command to install it from PyPI:

```bash
pip3 install adafruit-nrfutil --user
```

### Drivers
- SiLabs CP2104 driver is required for USB to Serial when using with Feather nRF52832.

## Build
1. Open `ble-button.ino` in the Arduino IDE.
2. Select the board and port.
3. Verify.
4. Upload.

## Notes
- Set `DEBUG_LOG` to 0 for battery saving/production.
