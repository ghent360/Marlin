/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#if !(defined(NUM_DIGITAL_PINS) || defined(BOARD_NR_GPIO_PINS))
  #error "M43 not supported for this board"
#endif

// Strange - STM32F4 comes to HAL_STM32 rather than HAL_STM32F4 for these files
#ifdef STM32F4
  #ifdef NUM_DIGITAL_PINS            // Only in ST's Arduino core (STM32duino, STM32Core)
    #include "pinsDebug_STM32duino.h"
  #elif defined(BOARD_NR_GPIO_PINS)  // Only in STM32GENERIC (Maple)
    #include "pinsDebug_STM32GENERIC.h"
  #else
    #error "M43 not supported for this board"
  #endif
#endif

uint8_t get_pin_mode(const pin_t Ard_num) {
  const PinName dp = digitalPinToPinName(Ard_num);
  uint32_t ll_pin  = STM_LL_GPIO_PIN(dp);
  GPIO_TypeDef *port = get_GPIO_Port(STM_PORT(dp));
  uint32_t mode = LL_GPIO_GetPinMode(port, ll_pin);
  switch (mode) {
    case LL_GPIO_MODE_ANALOG: return MODE_PIN_ANALOG;
    case LL_GPIO_MODE_INPUT: return MODE_PIN_INPUT;
    case LL_GPIO_MODE_OUTPUT: return MODE_PIN_OUTPUT;
    case LL_GPIO_MODE_ALTERNATE: return MODE_PIN_ALT;
    TERN_(STM32F1xx, case LL_GPIO_MODE_FLOATING:)
    default: return 0;
  }
}

bool GET_PINMODE(const pin_t Ard_num) {
  const uint8_t pin_mode = get_pin_mode(Ard_num);
  return pin_mode == MODE_PIN_OUTPUT || pin_mode == MODE_PIN_ALT;  // assume all alt definitions are PWM
}

int8_t digital_pin_to_analog_pin(pin_t Ard_num) {
  Ard_num -= NUM_ANALOG_FIRST;
  return (Ard_num >= 0 && Ard_num < NUM_ANALOG_INPUTS) ? Ard_num : -1;
}

bool IS_ANALOG(const pin_t Ard_num) {
  return get_pin_mode(Ard_num) == MODE_PIN_ANALOG;
}

bool is_digital(const pin_t x) {
  const uint8_t pin_mode = get_pin_mode(pin_array[x].pin);
  return pin_mode == MODE_PIN_INPUT || pin_mode == MODE_PIN_OUTPUT;
}

void port_print(const pin_t Ard_num) {
  char buffer[16];
  pin_t Index;
  for (Index = 0; Index < NUMBER_PINS_TOTAL; Index++)
    if (Ard_num == GET_PIN_MAP_PIN_M43(Index)) break;

  const char * ppa = pin_xref[Index].Port_pin_alpha;
  sprintf_P(buffer, PSTR("%s"), ppa);
  SERIAL_ECHO(buffer);
  if (ppa[3] == '\0') SERIAL_CHAR(' ');

  // print analog pin number
  const int8_t Port_pin = digital_pin_to_analog_pin(Ard_num);
  if (Port_pin >= 0) {
    sprintf_P(buffer, PSTR(" (A%d) "), Port_pin);
    SERIAL_ECHO(buffer);
    if (Port_pin < 10) SERIAL_CHAR(' ');
  }
  else
    SERIAL_ECHO_SP(7);

  // Print number to be used with M42
  sprintf_P(buffer, PSTR(" M42 P%d "), Ard_num);
  SERIAL_ECHO(buffer);
  if (Ard_num < 10) SERIAL_CHAR(' ');
  if (Ard_num < 100) SERIAL_CHAR(' ');
}

bool pwm_status(const pin_t Ard_num) {
  return get_pin_mode(Ard_num) == MODE_PIN_ALT;
}

void pwm_details(const pin_t Ard_num) {
  #ifndef STM32F1xx
    if (pwm_status(Ard_num)) {
      uint32_t alt_all = 0;
      const PinName dp = digitalPinToPinName(Ard_num);
      pin_t pin_number = uint8_t(PIN_NUM(dp));
      const bool over_7 = pin_number >= 8;
      const uint8_t ind = over_7 ? 1 : 0;
      switch (PORT_ALPHA(dp)) {  // get alt function
        case 'A' : alt_all = GPIOA->AFR[ind]; break;
        case 'B' : alt_all = GPIOB->AFR[ind]; break;
        case 'C' : alt_all = GPIOC->AFR[ind]; break;
        case 'D' : alt_all = GPIOD->AFR[ind]; break;
        #ifdef PE_0
          case 'E' : alt_all = GPIOE->AFR[ind]; break;
        #elif defined (PF_0)
          case 'F' : alt_all = GPIOF->AFR[ind]; break;
        #elif defined (PG_0)
          case 'G' : alt_all = GPIOG->AFR[ind]; break;
        #elif defined (PH_0)
          case 'H' : alt_all = GPIOH->AFR[ind]; break;
        #elif defined (PI_0)
          case 'I' : alt_all = GPIOI->AFR[ind]; break;
        #elif defined (PJ_0)
          case 'J' : alt_all = GPIOJ->AFR[ind]; break;
        #elif defined (PK_0)
          case 'K' : alt_all = GPIOK->AFR[ind]; break;
        #elif defined (PL_0)
          case 'L' : alt_all = GPIOL->AFR[ind]; break;
        #endif
      }
      if (over_7) pin_number -= 8;

      uint8_t alt_func = (alt_all >> (4 * pin_number)) & 0x0F;
      SERIAL_ECHOPAIR("Alt Function: ", alt_func);
      if (alt_func < 10) SERIAL_CHAR(' ');
      SERIAL_ECHOPGM(" - ");
      switch (alt_func) {
        case  0 : SERIAL_ECHOPGM("system (misc. I/O)"); break;
        case  1 : SERIAL_ECHOPGM("TIM1/TIM2 (probably PWM)"); break;
        case  2 : SERIAL_ECHOPGM("TIM3..5 (probably PWM)"); break;
        case  3 : SERIAL_ECHOPGM("TIM8..11 (probably PWM)"); break;
        case  4 : SERIAL_ECHOPGM("I2C1..3"); break;
        case  5 : SERIAL_ECHOPGM("SPI1/SPI2"); break;
        case  6 : SERIAL_ECHOPGM("SPI3"); break;
        case  7 : SERIAL_ECHOPGM("USART1..3"); break;
        case  8 : SERIAL_ECHOPGM("USART4..6"); break;
        case  9 : SERIAL_ECHOPGM("CAN1/CAN2, TIM12..14  (probably PWM)"); break;
        case 10 : SERIAL_ECHOPGM("OTG"); break;
        case 11 : SERIAL_ECHOPGM("ETH"); break;
        case 12 : SERIAL_ECHOPGM("FSMC, SDIO, OTG"); break;
        case 13 : SERIAL_ECHOPGM("DCMI"); break;
        case 14 : SERIAL_ECHOPGM("unused (shouldn't see this)"); break;
        case 15 : SERIAL_ECHOPGM("EVENTOUT"); break;
      }
    }
  #else
    // TODO: F1 doesn't support changing pins function, so we need to check the function of the PIN and if it's enabled
  #endif
} // pwm_details
