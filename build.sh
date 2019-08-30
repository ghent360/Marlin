#!/bin/sh
ulimit -n 2048
#arduino-cli compile --fqbn STM32:stm32:Nucleo_64:pnum=NUCLEO_F446RE Marlin
arduino-cli compile --fqbn STM32:stm32:3dprinter:pnum=PRNTR_F407_V1,xserial=generic,usb=CDCgen,xusb=FS,rtlib=nano Marlin
