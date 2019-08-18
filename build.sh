#!/bin/sh
ulimit -n 2048
exec arduino-cli compile --fqbn STM32:stm32:Nucleo_64:pnum=NUCLEO_F446RE Marlin
