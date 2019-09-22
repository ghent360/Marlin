#!/bin/sh
ulimit -n 2048
PLATFORM="STM32:stm32:Nucleo_64:pnum=NUCLEO_F446RE"
DBG=
while [ ! -z "$1" ]; do
  if [ "$1" = "-4" ]; then
    PLATFORM="STM32:stm32:3dprinter:pnum=PRNTR_F407_V1,xserial=generic,usb=CDCgen,xusb=FS"
  fi
  if [ "$1" = "-d" ]; then
    DBG=",opt=ogstd"
  fi
  shift
done
#arduino-cli compile --fqbn STM32:stm32:Nucleo_64:pnum=NUCLEO_F446RE Marlin
#arduino-cli compile --fqbn STM32:stm32:3dprinter:pnum=PRNTR_F407_V1,xserial=generic,usb=CDCgen,xusb=FS,opt=ogstd,rtlib=nano Marlin
echo arduino-cli compile --fqbn ${PLATFORM}${DBG},rtlib=nano Marlin
arduino-cli compile --fqbn ${PLATFORM}${DBG},rtlib=nano Marlin
