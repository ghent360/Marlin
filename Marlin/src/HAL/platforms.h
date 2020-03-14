/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#define HAL_ID_AVR 1
#define HAL_ID_DUE 2
#define HAL_ID_TEENSY31_32 3
#define HAL_ID_TEENSY35_36 4
#define HAL_ID_LPC1768 5
#define HAL_ID_STM32F1 6
#define HAL_ID_STM32F4 7
#define HAL_ID_STM32F7 8
#define HAL_ID_STM32 9
#define HAL_ID_ESP32 10
#define HAL_ID_LINUX 11
#define HAL_ID_STM32_F4_F7 12
#define HAL_ID_SAMD51 13

#define XSTR(V...) #V

#ifdef __AVR__
  #define HAL_PATH(PATH, NAME) XSTR(PATH/AVR/NAME)
  #define HAL_PLATFORM_ID HAL_ID_AVR
#elif defined(ARDUINO_ARCH_SAM)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/DUE/NAME)
  #define HAL_PLATFORM_ID HAL_ID_DUE
#elif defined(__MK20DX256__)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/TEENSY31_32/NAME)
  #define HAL_PLATFORM_ID HAL_ID_TEENSY31_32
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/TEENSY35_36/NAME)
  #define HAL_PLATFORM_ID HAL_ID_TEENSY35_36
#elif defined(TARGET_LPC1768)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/LPC1768/NAME)
  #define HAL_PLATFORM_ID HAL_ID_LPC1768
#elif defined(__STM32F1__) || defined(TARGET_STM32F1)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/STM32F1/NAME)
  #define HAL_PLATFORM_ID HAL_ID_STM32F1
#elif /*defined(STM32GENERIC) &&*/ defined(STM32F4) || defined(STM32F4xx) || defined(STM32F7)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/STM32_F4_F7/NAME)
  #define HAL_PLATFORM_ID HAL_ID_STM32_F4_F7
#elif defined(ARDUINO_ARCH_STM32)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/STM32/NAME)
  #define HAL_PLATFORM_ID HAL_ID_STM32
#elif defined(ARDUINO_ARCH_ESP32)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/ESP32/NAME)
  #define HAL_PLATFORM_ID HAL_ID_ESP32
#elif defined(__PLAT_LINUX__)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/LINUX/NAME)
  #define HAL_PLATFORM_ID HAL_ID_LINUX
#elif defined(__SAMD51__)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/SAMD51/NAME)
  #define HAL_PLATFORM_ID HAL_ID_SAMD51
#else
  #error "Unsupported Platform!"
#endif
