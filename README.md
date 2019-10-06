# Marlin 3D Printer Firmware

[![Build Status](https://travis-ci.org/MarlinFirmware/Marlin.svg?branch=bugfix-2.0.x)](https://travis-ci.org/MarlinFirmware/Marlin)
![GitHub](https://img.shields.io/github/license/marlinfirmware/marlin.svg)
![GitHub contributors](https://img.shields.io/github/contributors/marlinfirmware/marlin.svg)
![GitHub Release Date](https://img.shields.io/github/release-date/marlinfirmware/marlin.svg)

<img align="top" width=175 src="buildroot/share/pixmaps/logo/marlin-250.png" />

## This is a fork of the official Marlin code.
Original code location: https://github.com/MarlinFirmware/Marlin.git 

This repo is a port of Marlin to STM32 F4 MCU. I'm using Nucleo-F446RE as a test platform. I have an addon board for the NUCLEO that converts it to a 3D printer controller in similar fashion how RAMPS addon works with an arduino board. Open source schematics for the addon board: https://github.com/ghent360/PrntrBoard.git. Both the board and this firmware are functional.

I was able to print a few object with this code, it is not guaranteed bug free, but it is comparable to the official Marlin bugfix-2.0.x branch in terms of what works and what does not.

To compile I'm using Arduino 1.8.9 with the official STM32 support library from this link https://github.com/stm32duino/Arduino_Core_STM32

You can compile the code with the Arduino IDE or Visual Studio Code with an Arduino extension installed.

Branches:
- bugfix-2.0.x - this branch tracks Marlin's bugfix-2.0.x. Designed to work on Nucleo-F446RE dev board from STMicro with PRNTRboard with TMC2130 drivers.
- f407 - this branch is based on my bugfix-2.0.x with modifications so it works on my Nucleo-F407 board with PRNTRboard with TMC2130 driver.
- f407-2660 - this is based on f407 with configuration changes for PRNTRboard with TMC2660 drivers
- sldrbot - this branch is based on bugfix-2.0.x designed to work on Nucleo-F446RE with PRNTRboard with TMC2130 drivers. Changes to the configuration are made to support my soldering machine robot.
- sldrbot-407 - this branch is based on sldrbot, but designed to work on Nucleo-F407 with PRNTRBoard with TMC2130 drivers.
- sldrbot-407-2660 - this branch is based on sldrbot-407, but changes are made to support TMC2660 drivers.

## ~Happy hacking

The following is the original Marlin README text:
================================

Additional documentation can be found at the [Marlin Home Page](http://marlinfw.org/).
Please test this firmware and let us know if it misbehaves in any way. Volunteers are standing by!

## Marlin 2.0 Bugfix Branch

__Not for production use. Use with caution!__

Marlin 2.0 takes this popular RepRap firmware to the next level by adding support for much faster 32-bit and ARM-based boards while improving support for 8-bit AVR boards. Read about Marlin's decision to use a "Hardware Abstraction Layer" below.

This branch is for patches to the latest 2.0.x release version. Periodically this branch will form the basis for the next minor 2.0.x release.

Download earlier versions of Marlin on the [Releases page](https://github.com/MarlinFirmware/Marlin/releases).

## Building Marlin 2.0

To build Marlin 2.0 you'll need [Arduino IDE 1.8.8 or newer](https://www.arduino.cc/en/main/software) or [PlatformIO](http://docs.platformio.org/en/latest/ide.html#platformio-ide). We've posted detailed instructions on [Building Marlin with Arduino](http://marlinfw.org/docs/basics/install_arduino.html) and [Building Marlin with PlatformIO for ReArm](http://marlinfw.org/docs/basics/install_rearm.html) (which applies well to other 32-bit boards).

## Hardware Abstraction Layer (HAL)

Marlin 2.0 introduces a layer of abstraction so that all the existing high-level code can be built for 32-bit platforms while still retaining full 8-bit AVR compatibility. Retaining AVR compatibility and a single code-base is important to us, because we want to make sure that features and patches get as much testing and attention as possible, and that all platforms always benefit from the latest improvements.

### Current HALs

  name|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Arduino AVR](https://www.arduino.cc/)|ATmega, ATTiny, etc.|16-20MHz|64-256k|2-16k|5V|no
  [Teensy++ 2.0](http://www.microchip.com/wwwproducts/en/AT90USB1286)|[AT90USB1286](http://www.microchip.com/wwwproducts/en/AT90USB1286)|16MHz|128k|8k|5V|no
  [Arduino STM32](https://github.com/rogerclarkmelbourne/Arduino_STM32)|[STM32F1](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html) ARM-Cortex M3|72MHz|256-512k|48-64k|3.3V|no
  [Due](https://www.arduino.cc/en/Guide/ArduinoDue), [RAMPS-FD](http://www.reprap.org/wiki/RAMPS-FD), etc.|[SAM3X8E ARM-Cortex M3](http://www.microchip.com/wwwproducts/en/ATsam3x8e)|84MHz|512k|64+32k|3.3V|no
  [Re-ARM](https://www.kickstarter.com/projects/1245051645/re-arm-for-ramps-simple-32-bit-upgrade)|[LPC1768 ARM-Cortex M3](http://www.nxp.com/products/microcontrollers-and-processors/arm-based-processors-and-mcus/lpc-cortex-m-mcus/lpc1700-cortex-m3/512kb-flash-64kb-sram-ethernet-usb-lqfp100-package:LPC1768FBD100)|100MHz|512k|32+16+16k|3.3-5V|no
  [MKS SBASE](http://forums.reprap.org/read.php?13,499322)|LPC1768 ARM-Cortex M3|100MHz|512k|32+16+16k|3.3-5V|no
  [Azteeg X5 GT](https://www.panucatt.com/azteeg_X5_GT_reprap_3d_printer_controller_p/ax5gt.htm)|LPC1769 ARM-Cortex M3|120MHz|512k|32+16+16k|3.3-5V|no
  [Selena Compact](https://github.com/Ales2-k/Selena)|LPC1768 ARM-Cortex M3|100MHz|512k|32+16+16k|3.3-5V|no
  [Teensy 3.5](https://www.pjrc.com/store/teensy35.html)|ARM-Cortex M4|120MHz|512k|192k|3.3-5V|yes
  [Teensy 3.6](https://www.pjrc.com/store/teensy36.html)|ARM-Cortex M4|180MHz|1M|256k|3.3V|yes
  [PRNTRboard/NUCLEO-F446](https://github.com/ghent360/PrntrBoard.git)|[STM32F446RE ARM-Cortex M4](https://www.st.com/en/microcontrollers/stm32f446.html)|180MHz|512k|112+16k|3.3V|yes
  [PRNTRboard/NUCLEO-F407](https://github.com/ghent360/PrntrBoard.git)|[STM32F407VE ARM-Cortex M4](https://www.st.com/content/st_com/en/products/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus/stm32-high-performance-mcus/stm32f4-series/stm32f407-417/stm32f407ve.html)|168MHz|512k|192+4k|3.3V|yes

### HALs in Development

  name|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [STEVAL-3DP001V1](http://www.st.com/en/evaluation-tools/steval-3dp001v1.html)|[STM32F401VE Arm-Cortex M4](http://www.st.com/en/microcontrollers/stm32f401ve.html)|84MHz|512k|64+32k|3.3-5V|yes
  [Smoothieboard](http://reprap.org/wiki/Smoothieboard)|LPC1769 ARM-Cortex M3|120MHz|512k|64k|3.3-5V|no
  [Adafruit Grand Central M4](https://www.adafruit.com/product/4064)|[SAMD51P20A ARM-Cortex M4](https://www.microchip.com/wwwproducts/en/ATSAMD51P20A)|120MHz|1M|256k|3.3V|yes

## Submitting Patches

Proposed patches should be submitted as a Pull Request against the ([bugfix-2.0.x](https://github.com/MarlinFirmware/Marlin/tree/bugfix-2.0.x)) branch.

- This branch is for fixing bugs and integrating any new features for the duration of the Marlin 2.0.x life-cycle.
- Follow the [Coding Standards](http://marlinfw.org/docs/development/coding_standards.html) to gain points with the maintainers.
- Please submit your questions and concerns to the [Issue Queue](https://github.com/MarlinFirmware/Marlin/issues).

### [RepRap.org Wiki Page](http://reprap.org/wiki/Marlin)

## Credits

The current Marlin dev team consists of:

 - Scott Lahteine [[@thinkyhead](https://github.com/thinkyhead)] - USA &nbsp; [![Flattr Scott](http://api.flattr.com/button/flattr-badge-large.png)](https://flattr.com/submit/auto?user_id=thinkyhead&url=https://github.com/MarlinFirmware/Marlin&title=Marlin&language=&tags=github&category=software)
 - Roxanne Neufeld [[@Roxy-3D](https://github.com/Roxy-3D)] - USA
 - Bob Kuhn [[@Bob-the-Kuhn](https://github.com/Bob-the-Kuhn)] - USA
 - Chris Pepper [[@p3p](https://github.com/p3p)] - UK
 - João Brazio [[@jbrazio](https://github.com/jbrazio)] - Portugal
 - Erik van der Zalm [[@ErikZalm](https://github.com/ErikZalm)] - Netherlands &nbsp; [![Flattr Erik](http://api.flattr.com/button/flattr-badge-large.png)](https://flattr.com/submit/auto?user_id=ErikZalm&url=https://github.com/MarlinFirmware/Marlin&title=Marlin&language=&tags=github&category=software)

## License

Marlin is published under the [GPL license](/LICENSE) because we believe in open development. The GPL comes with both rights and obligations. Whether you use Marlin firmware as the driver for your open or closed-source product, you must keep Marlin open, and you must provide your compatible Marlin source code to end users upon request. The most straightforward way to comply with the Marlin license is to make a fork of Marlin on Github, perform your modifications, and direct users to your modified fork.

While we can't prevent the use of this code in products (3D printers, CNC, etc.) that are closed source or crippled by a patent, we would prefer that you choose another firmware or, better yet, make your own.
