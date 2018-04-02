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

#/home/vne/.arduino15/packages/STM32/tools/arm-none-eabi-gcc/6-2017-q2-update/bin/arm-none-eabi-g++ \
#	-mcpu=cortex-m4 \
#	-mfpu=fpv4-sp-d16 \
#	-mfloat-abi=hard \
#	-mthumb \
#	@/tmp/arduino_build_624108/sketch/build_opt.h \
#	-c \
#	-O3 \
#	-flto \
#	-w \
#	-std=gnu++14 \
#	-ffunction-sections \
#	-fdata-sections \
#	-nostdlib \
#	-fno-threadsafe-statics \
#	--param max-inline-insns-single=500 \
#	-fno-rtti \
#	-fno-exceptions \
#	-MMD \
#	-I/home/vne/projects/marlin/PrntrBoardV1/Marlin/Marlin \
#	-I/home/vne/.arduino15/packages/STM32/hardware/stm32/1.2.0/cores/arduino/avr \
#	-I/home/vne/.arduino15/packages/STM32/hardware/stm32/1.2.0/cores/arduino/stm32 \
#	-I/home/vne/.arduino15/packages/STM32/hardware/stm32/1.2.0/system/Drivers/STM32F4xx_HAL_Driver/Inc/ \
#	-I/home/vne/.arduino15/packages/STM32/hardware/stm32/1.2.0/system/Drivers/STM32F4xx_HAL_Driver/Src/ \
#	-I/home/vne/.arduino15/packages/STM32/hardware/stm32/1.2.0/system/STM32F4xx/ \
#	-I/home/vne/.arduino15/packages/STM32/hardware/stm32/1.2.0/variants/NUCLEO_F446RE/usb \
#	-I/home/vne/.arduino15/packages/STM32/hardware/stm32/1.2.0/system/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
#	-I/home/vne/.arduino15/packages/STM32/hardware/stm32/1.2.0/system/Middlewares/ST/STM32_USB_Device_Library/Core/Src \
#	-I/home/vne/.arduino15/packages/STM32/tools/CMSIS/5.3.0/CMSIS/Core/Include/ \
#	-I/home/vne/.arduino15/packages/STM32/hardware/stm32/1.2.0/system/Drivers/CMSIS/Device/ST/STM32F4xx/Include/ \
#	-I/home/vne/.arduino15/packages/STM32/hardware/stm32/1.2.0/system/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/ \
#	-I/home/vne/.arduino15/packages/STM32/hardware/stm32/1.2.0/cores/arduino \
#	-I/home/vne/.arduino15/packages/STM32/hardware/stm32/1.2.0/variants/NUCLEO_F446RE \
#	-I/home/vne/.arduino15/packages/STM32/hardware/stm32/1.2.0/libraries/SPI/src \
#	-DSTM32F4xx \
#	-DARDUINO=10805 \
#	-DARDUINO_NUCLEO_F446RE \
#	-DARDUINO_ARCH_STM32  \
#	-DSTM32F446xx   \
#	/tmp/arduino_build_624108/sketch/src/Marlin.cpp \
#	-o /tmp/arduino_build_624108/sketch/src/Marlin.cpp.o
