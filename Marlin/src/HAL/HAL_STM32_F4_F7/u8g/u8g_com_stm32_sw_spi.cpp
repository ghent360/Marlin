/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#include "../HAL.h"
#if (HAL_PLATFORM_ID == HAL_ID_STM32_F4_F7)

#include "../../../inc/MarlinConfig.h"

#if HAS_GRAPHICAL_LCD && ENABLED(FORCE_SOFT_SPI)

#include "../HAL.h"
#include <U8glib.h>

// The delay between bits needs to be adjusted to the chip spec
// Current settings tested with FYSETC MINI uc1701 chip and CPU 168MHz
static inline void stm32_Delay(uint8_t cycles) {
  while(cycles--)
    __NOP();
}

static inline void swSpiTransfer_mode_0(
  GPIO_TypeDef *mosiPort, uint32_t mosiPin,
  GPIO_TypeDef *sckPort, uint32_t sckPin,
  uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    if (b & 0x80) {
      LL_GPIO_SetOutputPin(mosiPort, mosiPin);
    } else {
      LL_GPIO_ResetOutputPin(mosiPort, mosiPin);
    }
    stm32_Delay(5);
    LL_GPIO_SetOutputPin(sckPort, sckPin);
    stm32_Delay(5);
    LL_GPIO_ResetOutputPin(sckPort, sckPin);
    stm32_Delay(1);
    b <<= 1;
  }
}

static inline void swSpiTransfer_mode_3(
  GPIO_TypeDef *mosiPort, uint32_t mosiPin,
  GPIO_TypeDef *sckPort, uint32_t sckPin,
  uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    LL_GPIO_ResetOutputPin(sckPort, sckPin);
    stm32_Delay(5);
    if (b & 0x80) {
      LL_GPIO_SetOutputPin(mosiPort, mosiPin);
    } else {
      LL_GPIO_ResetOutputPin(mosiPort, mosiPin);
    }
    stm32_Delay(3);
    LL_GPIO_SetOutputPin(sckPort, sckPin);
    stm32_Delay(1);
    b <<= 1;
  }
}

static void u8g_sw_spi_HAL_STM32F1_shift_out(
  GPIO_TypeDef *mosiPort, uint32_t mosiPin,
  GPIO_TypeDef *sckPort, uint32_t sckPin,
  uint8_t val) {
  #if ENABLED(FYSETC_MINI_12864)
    swSpiTransfer_mode_3(mosiPort, mosiPin, sckPort, sckPin, val);
  #else
    swSpiTransfer_mode_0(mosiPort, mosiPin, sckPort, sckPin, val);
  #endif
}

uint8_t u8g_com_HAL_stm32f4_sw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
  switch (msg) {
    case U8G_COM_MSG_INIT:
      u8g_com_arduino_assign_pin_output_high(u8g);
      u8g_com_arduino_digital_write(u8g, U8G_PI_SCK, LOW);
      u8g_com_arduino_digital_write(u8g, U8G_PI_MOSI, LOW);
      break;

    case U8G_COM_MSG_STOP:
      break;

    case U8G_COM_MSG_RESET:
      if ( u8g->pin_list[U8G_PI_RESET] != U8G_PIN_NONE )
        u8g_com_arduino_digital_write(u8g, U8G_PI_RESET, arg_val);
      break;

    case U8G_COM_MSG_CHIP_SELECT:
      #if ENABLED(FYSETC_MINI_12864) // This LCD SPI is running mode 3 while SD card is running mode 0
        if (arg_val) {               // SCK idle state needs to be set to the proper idle state before
                                     // the next chip select goes active
          u8g_com_arduino_digital_write(u8g, U8G_PI_SCK, HIGH);// Set SCK to mode 3 idle state before CS goes active
          u8g_com_arduino_digital_write(u8g, U8G_PI_CS, LOW);
        } else {
          u8g_com_arduino_digital_write(u8g, U8G_PI_CS, HIGH);
          u8g_com_arduino_digital_write(u8g, U8G_PI_SCK, LOW); // Set SCK to mode 0 idle state after CS goes inactive
        }
      #else
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS, !arg_val);
      #endif
      break;

    case U8G_COM_MSG_WRITE_BYTE: {
      GPIO_TypeDef *mosiPort = digitalPinToPort(u8g->pin_list[U8G_PI_MOSI]);
      uint32_t mosiPin = STM_LL_GPIO_PIN(digitalPinToPinName(u8g->pin_list[U8G_PI_MOSI]));
      GPIO_TypeDef *sckPort = digitalPinToPort(u8g->pin_list[U8G_PI_SCK]);
      uint32_t sckPin = STM_LL_GPIO_PIN(digitalPinToPinName(u8g->pin_list[U8G_PI_SCK]));

      u8g_sw_spi_HAL_STM32F1_shift_out(mosiPort, mosiPin, sckPort, sckPin, arg_val);
      }
      break;

    case U8G_COM_MSG_WRITE_SEQ: {
      uint8_t *ptr = (uint8_t *)arg_ptr;
      GPIO_TypeDef *mosiPort = digitalPinToPort(u8g->pin_list[U8G_PI_MOSI]);
      uint32_t mosiPin = STM_LL_GPIO_PIN(digitalPinToPinName(u8g->pin_list[U8G_PI_MOSI]));
      GPIO_TypeDef *sckPort = digitalPinToPort(u8g->pin_list[U8G_PI_SCK]);
      uint32_t sckPin = STM_LL_GPIO_PIN(digitalPinToPinName(u8g->pin_list[U8G_PI_SCK]));

      while (arg_val > 0) {
        u8g_sw_spi_HAL_STM32F1_shift_out(mosiPort, mosiPin, sckPort, sckPin, *ptr++);
        arg_val--;
      }
    } break;

    case U8G_COM_MSG_WRITE_SEQ_P: {
      uint8_t *ptr = (uint8_t *)arg_ptr;
      GPIO_TypeDef *mosiPort = digitalPinToPort(u8g->pin_list[U8G_PI_MOSI]);
      uint32_t mosiPin = STM_LL_GPIO_PIN(digitalPinToPinName(u8g->pin_list[U8G_PI_MOSI]));
      GPIO_TypeDef *sckPort = digitalPinToPort(u8g->pin_list[U8G_PI_SCK]);
      uint32_t sckPin = STM_LL_GPIO_PIN(digitalPinToPinName(u8g->pin_list[U8G_PI_SCK]));
      while (arg_val > 0) {
        u8g_sw_spi_HAL_STM32F1_shift_out(mosiPort, mosiPin, sckPort, sckPin, u8g_pgm_read(ptr));
        ptr++;
        arg_val--;
      }
    } break;

    case U8G_COM_MSG_ADDRESS: /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
      u8g_com_arduino_digital_write(u8g, U8G_PI_A0, arg_val);
      break;
  }
  return 1;
}
#endif // HAS_GRAPHICAL_LCD
#endif // PLATFORM_ID
