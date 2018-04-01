#!/bin/sh
if [ ! -d /tmp/arduino_cache_506886 ]; then
	mkdir /tmp/arduino_cache_506886
fi
if [ ! -d /tmp/arduino_cache_506886 ]; then
	mkdir /tmp/arduino_cache_506886
fi
/home/vne/arduino-1.8.5/arduino-builder \
	-compile \
	-logger=machine \
	-hardware /home/vne/arduino-1.8.5/hardware \
	-hardware /home/vne/.arduino15/packages \
	-tools /home/vne/arduino-1.8.5/tools-builder \
	-tools /home/vne/arduino-1.8.5/hardware/tools/avr \
	-tools /home/vne/.arduino15/packages \
	-built-in-libraries /home/vne/arduino-1.8.5/libraries \
	-libraries /home/vne/Arduino/libraries \
	-fqbn=STM32:stm32:Nucleo_64:pnum=NUCLEO_F446RE,upload_method=STLink,xserial=generic,usb=none,opt=o3lto \
	-ide-version=10805 \
	-build-path /tmp/arduino_build_624108 \
	-warnings=none \
	-build-cache /tmp/arduino_cache_506886 \
	-prefs=build.warn_data_percentage=75 \
	-prefs=runtime.tools.CMSIS.path=/home/vne/.arduino15/packages/STM32/tools/CMSIS/5.3.0 \
	-prefs=runtime.tools.arm-none-eabi-gcc.path=/home/vne/.arduino15/packages/STM32/tools/arm-none-eabi-gcc/6-2017-q2-update \
	-prefs=runtime.tools.STM32Tools.path=/home/vne/.arduino15/packages/STM32/tools/STM32Tools/1.0.2 \
	-verbose \
	/home/vne/projects/marlin/PrntrBoardV1/Marlin/Marlin/Marlin.ino


#/home/vne/arduino-1.8.5/arduino-builder \
#	-compile \
#	-logger=machine \
#	-hardware /home/vne/arduino-1.8.5/hardware \
#	-hardware /home/vne/.arduino15/packages \
#	-tools /home/vne/arduino-1.8.5/tools-builder \
#	-tools /home/vne/arduino-1.8.5/hardware/tools/avr \
#	-tools /home/vne/.arduino15/packages \
#	-built-in-libraries /home/vne/arduino-1.8.5/libraries \
#	-libraries /home/vne/Arduino/libraries \
#	-fqbn=STM32:stm32:Nucleo_64:pnum=NUCLEO_F446RE,upload_method=STLink,xserial=generic,usb=none,opt=osstd \
#	-ide-version=10805 \
#	-build-path /tmp/arduino_build_624108 \
#	-warnings=none \
#	-build-cache /tmp/arduino_cache_506886 \
#	-prefs=build.warn_data_percentage=75 \
#	-prefs=runtime.tools.CMSIS.path=/home/vne/.arduino15/packages/STM32/tools/CMSIS/5.3.0 \
#	-prefs=runtime.tools.arm-none-eabi-gcc.path=/home/vne/.arduino15/packages/STM32/tools/arm-none-eabi-gcc/6-2017-q2-update \
#	-prefs=runtime.tools.STM32Tools.path=/home/vne/.arduino15/packages/STM32/tools/STM32Tools/1.0.2 \
#	-verbose \
#	/home/vne/projects/marlin/PrntrBoardV1/Marlin/Marlin/Marlin.ino

