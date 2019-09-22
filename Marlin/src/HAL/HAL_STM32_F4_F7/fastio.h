/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2017 Victor Perez
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

#include <stdint.h>
/**
 * Fast I/O interfaces for STM32F4/7
 * These use GPIO functions instead of Direct Port Manipulation, as on AVR.
 */

#define PWM_PIN(p) true
#define USEABLE_HARDWARE_PWM(p) PWM_PIN(p)
#ifndef PWM
  #define PWM OUTPUT
#endif

#define USE_FAST_IO

#ifdef USE_FAST_IO

#define FAST_IO_PIN(port, pin_no) (0x7000 | (port << 4) | (pin_no & 0xf))
#define IS_FAST_IO_PIN(pin) (((pin) & 0xf000) == 0x7000)
#define GET_PORT(fio_pin) PortName((fio_pin >> 4) & 0xf)
#define GET_PIN_IDX(fio_pin) (fio_pin & 0xf)

struct FastIOPin {
    const uint16_t pin_;
    const uint8_t port_num_;

    /*constexpr FastIOPin(uint32_t fast_io_pin)
        : port_addr_(AHB1PERIPH_BASE + GET_PORT(fast_io_pin) * 0x400),
          pin_(1 << GET_PIN_IDX(fast_io_pin)),
          port_num_(GET_PORT(fast_io_pin)) {}*/
    constexpr FastIOPin(PortName port, uint8_t pin)
        : pin_(1 << pin),
          port_num_(port) {}

    void set_mode(uint32_t ulMode) const {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.Pin = pin_;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

        switch ( ulMode ) {
        case INPUT:
            GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
            GPIO_InitStructure.Pull = GPIO_NOPULL;
            break;
        case INPUT_PULLUP:
            GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
            GPIO_InitStructure.Pull = GPIO_PULLUP;
            break;
        case INPUT_PULLDOWN:
            GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
            GPIO_InitStructure.Pull = GPIO_PULLDOWN;
            break;
        case OUTPUT:
            GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStructure.Pull = GPIO_NOPULL;
            break;
        default:
            return;
        }
        set_GPIO_Port_Clock(port_num_);
        HAL_GPIO_Init(portAddr(), &GPIO_InitStructure);
    }

    GPIO_TypeDef* portAddr() const {
      return reinterpret_cast<GPIO_TypeDef*>(AHB1PERIPH_BASE + port_num_ * 0x400);
    }

    void set() const {
        portAddr()->BSRR = pin_;
    }

    void reset() const {
        portAddr()->BSRR = (uint32_t)pin_ << 16;
    }

    void toggle() const {
        portAddr()->ODR ^= pin_;
    }

    void write(uint8_t val) const {
        if (val) {
            set();
        } else {
            reset();
        }
    }

    uint8_t read() const {
        return (portAddr()->IDR & pin_) ? 1 : 0;
    }

    bool isValid() const {
        return pin_ != 0;
    }

    uint16_t toArduinoPin() const {
        if (!isValid()) return (uint16_t)-1;
        return to_arduino_pin_[port_num_][pin_];
    }

    static constexpr FastIOPin fromArduinoPin(const uint16_t pin) {
#ifdef PA0
        if (pin == PA0) return FastIOPin(PortA, 0);
#endif
#ifdef PA1
        if (pin == PA1) return FastIOPin(PortA, 1);
#endif
#ifdef PA2
        if (pin == PA2) return FastIOPin(PortA, 2);
#endif
#ifdef PA3
        if (pin == PA3) return FastIOPin(PortA, 3);
#endif
#ifdef PA4
        if (pin == PA4) return FastIOPin(PortA, 4);
#endif
#ifdef PA5
        if (pin == PA5) return FastIOPin(PortA, 5);
#endif
#ifdef PA6
        if (pin == PA6) return FastIOPin(PortA, 6);
#endif
#ifdef PA7
        if (pin == PA7) return FastIOPin(PortA, 7);
#endif
#ifdef PA8
        if (pin == PA8) return FastIOPin(PortA, 8);
#endif
#ifdef PA9
        if (pin == PA9) return FastIOPin(PortA, 9);
#endif
#ifdef PA10
        if (pin == PA10) return FastIOPin(PortA, 10);
#endif
#ifdef PA11
        if (pin == PA11) return FastIOPin(PortA, 11);
#endif
#ifdef PA12
        if (pin == PA12) return FastIOPin(PortA, 12);
#endif
#ifdef PA13
        if (pin == PA13) return FastIOPin(PortA, 13);
#endif
#ifdef PA14
        if (pin == PA14) return FastIOPin(PortA, 14);
#endif
#ifdef PA15
        if (pin == PA15) return FastIOPin(PortA, 15);
#endif
#ifdef PB0
        if (pin == PB0) return FastIOPin(PortB, 0);
#endif
#ifdef PB1
        if (pin == PB1) return FastIOPin(PortB, 1);
#endif
#ifdef PB2
        if (pin == PB2) return FastIOPin(PortB, 2);
#endif
#ifdef PB3
        if (pin == PB3) return FastIOPin(PortB, 3);
#endif
#ifdef PB4
        if (pin == PB4) return FastIOPin(PortB, 4);
#endif
#ifdef PB5
        if (pin == PB5) return FastIOPin(PortB, 5);
#endif
#ifdef PB6
        if (pin == PB6) return FastIOPin(PortB, 6);
#endif
#ifdef PB7
        if (pin == PB7) return FastIOPin(PortB, 7);
#endif
#ifdef PB8
        if (pin == PB8) return FastIOPin(PortB, 8);
#endif
#ifdef PB9
        if (pin == PB9) return FastIOPin(PortB, 9);
#endif
#ifdef PB10
        if (pin == PB10) return FastIOPin(PortB, 10);
#endif
#ifdef PB11
        if (pin == PB11) return FastIOPin(PortB, 11);
#endif
#ifdef PB12
        if (pin == PB12) return FastIOPin(PortB, 12);
#endif
#ifdef PB13
        if (pin == PB13) return FastIOPin(PortB, 13);
#endif
#ifdef PB14
        if (pin == PB14) return FastIOPin(PortB, 14);
#endif
#ifdef PB15
        if (pin == PB15) return FastIOPin(PortB, 15);
#endif
#ifdef PC0
        if (pin == PC0) return FastIOPin(PortC, 0);
#endif
#ifdef PC1
        if (pin == PC1) return FastIOPin(PortC, 1);
#endif
#ifdef PC2
        if (pin == PC2) return FastIOPin(PortC, 2);
#endif
#ifdef PC3
        if (pin == PC3) return FastIOPin(PortC, 3);
#endif
#ifdef PC4
        if (pin == PC4) return FastIOPin(PortC, 4);
#endif
#ifdef PC5
        if (pin == PC5) return FastIOPin(PortC, 5);
#endif
#ifdef PC6
        if (pin == PC6) return FastIOPin(PortC, 6);
#endif
#ifdef PC7
        if (pin == PC7) return FastIOPin(PortC, 7);
#endif
#ifdef PC8
        if (pin == PC8) return FastIOPin(PortC, 8);
#endif
#ifdef PC9
        if (pin == PC9) return FastIOPin(PortC, 9);
#endif
#ifdef PC10
        if (pin == PC10) return FastIOPin(PortC, 10);
#endif
#ifdef PC11
        if (pin == PC11) return FastIOPin(PortC, 11);
#endif
#ifdef PC12
        if (pin == PC12) return FastIOPin(PortC, 12);
#endif
#ifdef PC13
        if (pin == PC13) return FastIOPin(PortC, 13);
#endif
#ifdef PC14
        if (pin == PC14) return FastIOPin(PortC, 14);
#endif
#ifdef PC15
        if (pin == PC15) return FastIOPin(PortC, 15);
#endif
#ifdef PD0
        if (pin == PD0) return FastIOPin(PortD, 0);
#endif
#ifdef PD1
        if (pin == PD1) return FastIOPin(PortD, 1);
#endif
#ifdef PD2
        if (pin == PD2) return FastIOPin(PortD, 2);
#endif
#ifdef PD3
        if (pin == PD3) return FastIOPin(PortD, 3);
#endif
#ifdef PD4
        if (pin == PD4) return FastIOPin(PortD, 4);
#endif
#ifdef PD5
        if (pin == PD5) return FastIOPin(PortD, 5);
#endif
#ifdef PD6
        if (pin == PD6) return FastIOPin(PortD, 6);
#endif
#ifdef PD7
        if (pin == PD7) return FastIOPin(PortD, 7);
#endif
#ifdef PD8
        if (pin == PD8) return FastIOPin(PortD, 8);
#endif
#ifdef PD9
        if (pin == PD9) return FastIOPin(PortD, 9);
#endif
#ifdef PD10
        if (pin == PD10) return FastIOPin(PortD, 10);
#endif
#ifdef PD11
        if (pin == PD11) return FastIOPin(PortD, 11);
#endif
#ifdef PD12
        if (pin == PD12) return FastIOPin(PortD, 12);
#endif
#ifdef PD13
        if (pin == PD13) return FastIOPin(PortD, 13);
#endif
#ifdef PD14
        if (pin == PD14) return FastIOPin(PortD, 14);
#endif
#ifdef PD15
        if (pin == PD15) return FastIOPin(PortD, 15);
#endif
#ifdef PE0
        if (pin == PE0) return FastIOPin(PortE, 0);
#endif
#ifdef PE1
        if (pin == PE1) return FastIOPin(PortE, 1);
#endif
#ifdef PE2
        if (pin == PE2) return FastIOPin(PortE, 2);
#endif
#ifdef PE3
        if (pin == PE3) return FastIOPin(PortE, 3);
#endif
#ifdef PE4
        if (pin == PE4) return FastIOPin(PortE, 4);
#endif
#ifdef PE5
        if (pin == PE5) return FastIOPin(PortE, 5);
#endif
#ifdef PE6
        if (pin == PE6) return FastIOPin(PortE, 6);
#endif
#ifdef PE7
        if (pin == PE7) return FastIOPin(PortE, 7);
#endif
#ifdef PE8
        if (pin == PE8) return FastIOPin(PortE, 8);
#endif
#ifdef PE9
        if (pin == PE9) return FastIOPin(PortE, 9);
#endif
#ifdef PE10
        if (pin == PE10) return FastIOPin(PortE, 10);
#endif
#ifdef PE11
        if (pin == PE11) return FastIOPin(PortE, 11);
#endif
#ifdef PE12
        if (pin == PE12) return FastIOPin(PortE, 12);
#endif
#ifdef PE13
        if (pin == PE13) return FastIOPin(PortE, 13);
#endif
#ifdef PE14
        if (pin == PE14) return FastIOPin(PortE, 14);
#endif
#ifdef PE15
        if (pin == PE15) return FastIOPin(PortE, 15);
#endif
#ifdef PF0
        if (pin == PF0) return FastIOPin(PortF, 0);
#endif
#ifdef PF1
        if (pin == PF1) return FastIOPin(PortF, 1);
#endif
#ifdef PF2
        if (pin == PF2) return FastIOPin(PortF, 2);
#endif
#ifdef PF3
        if (pin == PF3) return FastIOPin(PortF, 3);
#endif
#ifdef PF4
        if (pin == PF4) return FastIOPin(PortF, 4);
#endif
#ifdef PF5
        if (pin == PF5) return FastIOPin(PortF, 5);
#endif
#ifdef PF6
        if (pin == PF6) return FastIOPin(PortF, 6);
#endif
#ifdef PF7
        if (pin == PF7) return FastIOPin(PortF, 7);
#endif
#ifdef PF8
        if (pin == PF8) return FastIOPin(PortF, 8);
#endif
#ifdef PF9
        if (pin == PF9) return FastIOPin(PortF, 9);
#endif
#ifdef PF10
        if (pin == PF10) return FastIOPin(PortF, 10);
#endif
#ifdef PF11
        if (pin == PF11) return FastIOPin(PortF, 11);
#endif
#ifdef PF12
        if (pin == PF12) return FastIOPin(PortF, 12);
#endif
#ifdef PF13
        if (pin == PF13) return FastIOPin(PortF, 13);
#endif
#ifdef PF14
        if (pin == PF14) return FastIOPin(PortF, 14);
#endif
#ifdef PF15
        if (pin == PF15) return FastIOPin(PortF, 15);
#endif
#ifdef PG0
        if (pin == PG0) return FastIOPin(PortG, 0);
#endif
#ifdef PG1
        if (pin == PG1) return FastIOPin(PortG, 1);
#endif
#ifdef PG2
        if (pin == PG2) return FastIOPin(PortG, 2);
#endif
#ifdef PG3
        if (pin == PG3) return FastIOPin(PortG, 3);
#endif
#ifdef PG4
        if (pin == PG4) return FastIOPin(PortG, 4);
#endif
#ifdef PG5
        if (pin == PG5) return FastIOPin(PortG, 5);
#endif
#ifdef PG6
        if (pin == PG6) return FastIOPin(PortG, 6);
#endif
#ifdef PG7
        if (pin == PG7) return FastIOPin(PortG, 7);
#endif
#ifdef PG8
        if (pin == PG8) return FastIOPin(PortG, 8);
#endif
#ifdef PG9
        if (pin == PG9) return FastIOPin(PortG, 9);
#endif
#ifdef PG10
        if (pin == PG10) return FastIOPin(PortG, 10);
#endif
#ifdef PG11
        if (pin == PG11) return FastIOPin(PortG, 11);
#endif
#ifdef PG12
        if (pin == PG12) return FastIOPin(PortG, 12);
#endif
#ifdef PG13
        if (pin == PG13) return FastIOPin(PortG, 13);
#endif
#ifdef PG14
        if (pin == PG14) return FastIOPin(PortG, 14);
#endif
#ifdef PG15
        if (pin == PG15) return FastIOPin(PortG, 15);
#endif
#ifdef PH0
        if (pin == PH0) return FastIOPin(PortH, 0);
#endif
#ifdef PH1
        if (pin == PH1) return FastIOPin(PortH, 1);
#endif
#ifdef PH2
        if (pin == PH2) return FastIOPin(PortH, 2);
#endif
#ifdef PH3
        if (pin == PH3) return FastIOPin(PortH, 3);
#endif
#ifdef PH4
        if (pin == PH4) return FastIOPin(PortH, 4);
#endif
#ifdef PH5
        if (pin == PH5) return FastIOPin(PortH, 5);
#endif
#ifdef PH6
        if (pin == PH6) return FastIOPin(PortH, 6);
#endif
#ifdef PH7
        if (pin == PH7) return FastIOPin(PortH, 7);
#endif
#ifdef PH8
        if (pin == PH8) return FastIOPin(PortH, 8);
#endif
#ifdef PH9
        if (pin == PH9) return FastIOPin(PortH, 9);
#endif
#ifdef PH10
        if (pin == PH10) return FastIOPin(PortH, 10);
#endif
#ifdef PH11
        if (pin == PH11) return FastIOPin(PortH, 11);
#endif
#ifdef PH12
        if (pin == PH12) return FastIOPin(PortH, 12);
#endif
#ifdef PH13
        if (pin == PH13) return FastIOPin(PortH, 13);
#endif
#ifdef PH14
        if (pin == PH14) return FastIOPin(PortH, 14);
#endif
#ifdef PH15
        if (pin == PH15) return FastIOPin(PortH, 15);
#endif
#ifdef PI0
        if (pin == PI0) return FastIOPin(PortI, 0);
#endif
#ifdef PI1
        if (pin == PI1) return FastIOPin(PortI, 1);
#endif
#ifdef PI2
        if (pin == PI2) return FastIOPin(PortI, 2);
#endif
#ifdef PI3
        if (pin == PI3) return FastIOPin(PortI, 3);
#endif
#ifdef PI4
        if (pin == PI4) return FastIOPin(PortI, 4);
#endif
#ifdef PI5
        if (pin == PI5) return FastIOPin(PortI, 5);
#endif
#ifdef PI6
        if (pin == PI6) return FastIOPin(PortI, 6);
#endif
#ifdef PI7
        if (pin == PI7) return FastIOPin(PortI, 7);
#endif
#ifdef PI8
        if (pin == PI8) return FastIOPin(PortI, 8);
#endif
#ifdef PI9
        if (pin == PI9) return FastIOPin(PortI, 9);
#endif
#ifdef PI10
        if (pin == PI10) return FastIOPin(PortI, 10);
#endif
#ifdef PI11
        if (pin == PI11) return FastIOPin(PortI, 11);
#endif
#ifdef PI12
        if (pin == PI12) return FastIOPin(PortI, 12);
#endif
#ifdef PI13
        if (pin == PI13) return FastIOPin(PortI, 13);
#endif
#ifdef PI14
        if (pin == PI14) return FastIOPin(PortI, 14);
#endif
#ifdef PI15
        if (pin == PI15) return FastIOPin(PortI, 15);
#endif
#ifdef PJ0
        if (pin == PJ0) return FastIOPin(PortJ, 0);
#endif
#ifdef PJ1
        if (pin == PJ1) return FastIOPin(PortJ, 1);
#endif
#ifdef PJ2
        if (pin == PJ2) return FastIOPin(PortJ, 2);
#endif
#ifdef PJ3
        if (pin == PJ3) return FastIOPin(PortJ, 3);
#endif
#ifdef PJ4
        if (pin == PJ4) return FastIOPin(PortJ, 4);
#endif
#ifdef PJ5
        if (pin == PJ5) return FastIOPin(PortJ, 5);
#endif
#ifdef PJ6
        if (pin == PJ6) return FastIOPin(PortJ, 6);
#endif
#ifdef PJ7
        if (pin == PJ7) return FastIOPin(PortJ, 7);
#endif
#ifdef PJ8
        if (pin == PJ8) return FastIOPin(PortJ, 8);
#endif
#ifdef PJ9
        if (pin == PJ9) return FastIOPin(PortJ, 9);
#endif
#ifdef PJ10
        if (pin == PJ10) return FastIOPin(PortJ, 10);
#endif
#ifdef PJ11
        if (pin == PJ11) return FastIOPin(PortJ, 11);
#endif
#ifdef PJ12
        if (pin == PJ12) return FastIOPin(PortJ, 12);
#endif
#ifdef PJ13
        if (pin == PJ13) return FastIOPin(PortJ, 13);
#endif
#ifdef PJ14
        if (pin == PJ14) return FastIOPin(PortJ, 14);
#endif
#ifdef PJ15
        if (pin == PJ15) return FastIOPin(PortJ, 15);
#endif
#ifdef PK0
        if (pin == PK0) return FastIOPin(PortK, 0);
#endif
#ifdef PK1
        if (pin == PK1) return FastIOPin(PortK, 1);
#endif
#ifdef PK2
        if (pin == PK2) return FastIOPin(PortK, 2);
#endif
#ifdef PK3
        if (pin == PK3) return FastIOPin(PortK, 3);
#endif
#ifdef PK4
        if (pin == PK4) return FastIOPin(PortK, 4);
#endif
#ifdef PK5
        if (pin == PK5) return FastIOPin(PortK, 5);
#endif
#ifdef PK6
        if (pin == PK6) return FastIOPin(PortK, 6);
#endif
#ifdef PK7
        if (pin == PK7) return FastIOPin(PortK, 7);
#endif
#ifdef PK8
        if (pin == PK8) return FastIOPin(PortK, 8);
#endif
#ifdef PK9
        if (pin == PK9) return FastIOPin(PortK, 9);
#endif
#ifdef PK10
        if (pin == PK10) return FastIOPin(PortK, 10);
#endif
#ifdef PK11
        if (pin == PK11) return FastIOPin(PortK, 11);
#endif
#ifdef PK12
        if (pin == PK12) return FastIOPin(PortK, 12);
#endif
#ifdef PK13
        if (pin == PK13) return FastIOPin(PortK, 13);
#endif
#ifdef PK14
        if (pin == PK14) return FastIOPin(PortK, 14);
#endif
#ifdef PK15
        if (pin == PK15) return FastIOPin(PortK, 15);
#endif
        return FastIOPin(PortA, (uint8_t)-1);
    }
private:
    static constexpr uint16_t to_arduino_pin_[PortZ][16] = {
    // PORTA
    {
#ifdef PA0
        PA0,
#else
        (uint16_t)-1,
#endif
#ifdef PA1
        PA1,
#else
        (uint16_t)-1,
#endif
#ifdef PA2
        PA2,
#else
        (uint16_t)-1,
#endif
#ifdef PA3
        PA3,
#else
        (uint16_t)-1,
#endif
#ifdef PA4
        PA4,
#else
        (uint16_t)-1,
#endif
#ifdef PA5
        PA5,
#else
        (uint16_t)-1,
#endif
#ifdef PA6
        PA6,
#else
        (uint16_t)-1,
#endif
#ifdef PA7
        PA7,
#else
        (uint16_t)-1,
#endif
#ifdef PA8
        PA8,
#else
        (uint16_t)-1,
#endif
#ifdef PA9
        PA9,
#else
        (uint16_t)-1,
#endif
#ifdef PA10
        PA10,
#else
        (uint16_t)-1,
#endif
#ifdef PA11
        PA11,
#else
        (uint16_t)-1,
#endif
#ifdef PA12
        PA12,
#else
        (uint16_t)-1,
#endif
#ifdef PA13
        PA13,
#else
        (uint16_t)-1,
#endif
#ifdef PA14
        PA14,
#else
        (uint16_t)-1,
#endif
#ifdef PA15
        PA15,
#else
        (uint16_t)-1,
#endif
    },

#ifdef GPIOB_BASE
    // PORTB
    {
#ifdef PB0
        PB0,
#else
        (uint16_t)-1,
#endif
#ifdef PB1
        PB1,
#else
        (uint16_t)-1,
#endif
#ifdef PB2
        PB2,
#else
        (uint16_t)-1,
#endif
#ifdef PB3
        PB3,
#else
        (uint16_t)-1,
#endif
#ifdef PB4
        PB4,
#else
        (uint16_t)-1,
#endif
#ifdef PB5
        PB5,
#else
        (uint16_t)-1,
#endif
#ifdef PB6
        PB6,
#else
        (uint16_t)-1,
#endif
#ifdef PB7
        PB7,
#else
        (uint16_t)-1,
#endif
#ifdef PB8
        PB8,
#else
        (uint16_t)-1,
#endif
#ifdef PB9
        PB9,
#else
        (uint16_t)-1,
#endif
#ifdef PB10
        PB10,
#else
        (uint16_t)-1,
#endif
#ifdef PB11
        PB11,
#else
        (uint16_t)-1,
#endif
#ifdef PB12
        PB12,
#else
        (uint16_t)-1,
#endif
#ifdef PB13
        PB13,
#else
        (uint16_t)-1,
#endif
#ifdef PB14
        PB14,
#else
        (uint16_t)-1,
#endif
#ifdef PB15
        PB15,
#else
        (uint16_t)-1,
#endif
    },
#endif // GPIOB_BASE


#ifdef GPIOC_BASE
    // PORTC
    {
#ifdef PC0
        PC0,
#else
        (uint16_t)-1,
#endif
#ifdef PC1
        PC1,
#else
        (uint16_t)-1,
#endif
#ifdef PC2
        PC2,
#else
        (uint16_t)-1,
#endif
#ifdef PC3
        PC3,
#else
        (uint16_t)-1,
#endif
#ifdef PC4
        PC4,
#else
        (uint16_t)-1,
#endif
#ifdef PC5
        PC5,
#else
        (uint16_t)-1,
#endif
#ifdef PC6
        PC6,
#else
        (uint16_t)-1,
#endif
#ifdef PC7
        PC7,
#else
        (uint16_t)-1,
#endif
#ifdef PC8
        PC8,
#else
        (uint16_t)-1,
#endif
#ifdef PC9
        PC9,
#else
        (uint16_t)-1,
#endif
#ifdef PC10
        PC10,
#else
        (uint16_t)-1,
#endif
#ifdef PC11
        PC11,
#else
        (uint16_t)-1,
#endif
#ifdef PC12
        PC12,
#else
        (uint16_t)-1,
#endif
#ifdef PC13
        PC13,
#else
        (uint16_t)-1,
#endif
#ifdef PC14
        PC14,
#else
        (uint16_t)-1,
#endif
#ifdef PC15
        PC15,
#else
        (uint16_t)-1,
#endif
    },
#endif // GPIOC_BASE

#ifdef GPIOD_BASE
    // PORTD
    {
#ifdef PD0
        PD0,
#else
        (uint16_t)-1,
#endif
#ifdef PD1
        PD1,
#else
        (uint16_t)-1,
#endif
#ifdef PD2
        PD2,
#else
        (uint16_t)-1,
#endif
#ifdef PD3
        PD3,
#else
        (uint16_t)-1,
#endif
#ifdef PD4
        PD4,
#else
        (uint16_t)-1,
#endif
#ifdef PD5
        PD5,
#else
        (uint16_t)-1,
#endif
#ifdef PD6
        PD6,
#else
        (uint16_t)-1,
#endif
#ifdef PD7
        PD7,
#else
        (uint16_t)-1,
#endif
#ifdef PD8
        PD8,
#else
        (uint16_t)-1,
#endif
#ifdef PD9
        PD9,
#else
        (uint16_t)-1,
#endif
#ifdef PD10
        PD10,
#else
        (uint16_t)-1,
#endif
#ifdef PD11
        PD11,
#else
        (uint16_t)-1,
#endif
#ifdef PD12
        PD12,
#else
        (uint16_t)-1,
#endif
#ifdef PD13
        PD13,
#else
        (uint16_t)-1,
#endif
#ifdef PD14
        PD14,
#else
        (uint16_t)-1,
#endif
#ifdef PD15
        PD15,
#else
        (uint16_t)-1,
#endif
    },
#endif // GPIOD_BASE

#ifdef GPIOE_BASE
    // PORTE
    {
#ifdef PE0
        PE0,
#else
        (uint16_t)-1,
#endif
#ifdef PE1
        PE1,
#else
        (uint16_t)-1,
#endif
#ifdef PE2
        PE2,
#else
        (uint16_t)-1,
#endif
#ifdef PE3
        PE3,
#else
        (uint16_t)-1,
#endif
#ifdef PE4
        PE4,
#else
        (uint16_t)-1,
#endif
#ifdef PE5
        PE5,
#else
        (uint16_t)-1,
#endif
#ifdef PE6
        PE6,
#else
        (uint16_t)-1,
#endif
#ifdef PE7
        PE7,
#else
        (uint16_t)-1,
#endif
#ifdef PE8
        PE8,
#else
        (uint16_t)-1,
#endif
#ifdef PE9
        PE9,
#else
        (uint16_t)-1,
#endif
#ifdef PE10
        PE10,
#else
        (uint16_t)-1,
#endif
#ifdef PE11
        PE11,
#else
        (uint16_t)-1,
#endif
#ifdef PE12
        PE12,
#else
        (uint16_t)-1,
#endif
#ifdef PE13
        PE13,
#else
        (uint16_t)-1,
#endif
#ifdef PE14
        PE14,
#else
        (uint16_t)-1,
#endif
#ifdef PE15
        PE15,
#else
        (uint16_t)-1,
#endif
    },
#endif // GPIOE_BASE

#ifdef GPIOF_BASE
    // PORTF
    {
#ifdef PF0
        PF0,
#else
        (uint16_t)-1,
#endif
#ifdef PF1
        PF1,
#else
        (uint16_t)-1,
#endif
#ifdef PF2
        PF2,
#else
        (uint16_t)-1,
#endif
#ifdef PF3
        PF3,
#else
        (uint16_t)-1,
#endif
#ifdef PF4
        PF4,
#else
        (uint16_t)-1,
#endif
#ifdef PF5
        PF5,
#else
        (uint16_t)-1,
#endif
#ifdef PF6
        PF6,
#else
        (uint16_t)-1,
#endif
#ifdef PF7
        PF7,
#else
        (uint16_t)-1,
#endif
#ifdef PF8
        PF8,
#else
        (uint16_t)-1,
#endif
#ifdef PF9
        PF9,
#else
        (uint16_t)-1,
#endif
#ifdef PF10
        PF10,
#else
        (uint16_t)-1,
#endif
#ifdef PF11
        PF11,
#else
        (uint16_t)-1,
#endif
#ifdef PF12
        PF12,
#else
        (uint16_t)-1,
#endif
#ifdef PF13
        PF13,
#else
        (uint16_t)-1,
#endif
#ifdef PF14
        PF14,
#else
        (uint16_t)-1,
#endif
#ifdef PF15
        PF15,
#else
        (uint16_t)-1,
#endif
    },
#endif // GPIOF_BASE

#ifdef GPIOG_BASE
    // PORTG
    {
#ifdef PG0
        PG0,
#else
        (uint16_t)-1,
#endif
#ifdef PG1
        PG1,
#else
        (uint16_t)-1,
#endif
#ifdef PG2
        PG2,
#else
        (uint16_t)-1,
#endif
#ifdef PG3
        PG3,
#else
        (uint16_t)-1,
#endif
#ifdef PG4
        PG4,
#else
        (uint16_t)-1,
#endif
#ifdef PG5
        PG5,
#else
        (uint16_t)-1,
#endif
#ifdef PG6
        PG6,
#else
        (uint16_t)-1,
#endif
#ifdef PG7
        PG7,
#else
        (uint16_t)-1,
#endif
#ifdef PG8
        PG8,
#else
        (uint16_t)-1,
#endif
#ifdef PG9
        PG9,
#else
        (uint16_t)-1,
#endif
#ifdef PG10
        PG10,
#else
        (uint16_t)-1,
#endif
#ifdef PG11
        PG11,
#else
        (uint16_t)-1,
#endif
#ifdef PG12
        PG12,
#else
        (uint16_t)-1,
#endif
#ifdef PG13
        PG13,
#else
        (uint16_t)-1,
#endif
#ifdef PG14
        PG14,
#else
        (uint16_t)-1,
#endif
#ifdef PG15
        PG15,
#else
        (uint16_t)-1,
#endif
    },
#endif // GPIOG_BASE

#ifdef GPIOH_BASE
    // PORTH
    {
#ifdef PH0
        PH0,
#else
        (uint16_t)-1,
#endif
#ifdef PH1
        PH1,
#else
        (uint16_t)-1,
#endif
#ifdef PH2
        PH2,
#else
        (uint16_t)-1,
#endif
#ifdef PH3
        PH3,
#else
        (uint16_t)-1,
#endif
#ifdef PH4
        PH4,
#else
        (uint16_t)-1,
#endif
#ifdef PH5
        PH5,
#else
        (uint16_t)-1,
#endif
#ifdef PH6
        PH6,
#else
        (uint16_t)-1,
#endif
#ifdef PH7
        PH7,
#else
        (uint16_t)-1,
#endif
#ifdef PH8
        PH8,
#else
        (uint16_t)-1,
#endif
#ifdef PH9
        PH9,
#else
        (uint16_t)-1,
#endif
#ifdef PH10
        PH10,
#else
        (uint16_t)-1,
#endif
#ifdef PH11
        PH11,
#else
        (uint16_t)-1,
#endif
#ifdef PH12
        PH12,
#else
        (uint16_t)-1,
#endif
#ifdef PH13
        PH13,
#else
        (uint16_t)-1,
#endif
#ifdef PH14
        PH14,
#else
        (uint16_t)-1,
#endif
#ifdef PH15
        PH15,
#else
        (uint16_t)-1,
#endif
    },
#endif // GPIOH_BASE

#ifdef GPIOI_BASE
    // PORTI
    {
#ifdef PI0
        PI0,
#else
        (uint16_t)-1,
#endif
#ifdef PI1
        PI1,
#else
        (uint16_t)-1,
#endif
#ifdef PI2
        PI2,
#else
        (uint16_t)-1,
#endif
#ifdef PI3
        PI3,
#else
        (uint16_t)-1,
#endif
#ifdef PI4
        PI4,
#else
        (uint16_t)-1,
#endif
#ifdef PI5
        PI5,
#else
        (uint16_t)-1,
#endif
#ifdef PI6
        PI6,
#else
        (uint16_t)-1,
#endif
#ifdef PI7
        PI7,
#else
        (uint16_t)-1,
#endif
#ifdef PI8
        PI8,
#else
        (uint16_t)-1,
#endif
#ifdef PI9
        PI9,
#else
        (uint16_t)-1,
#endif
#ifdef PI10
        PI10,
#else
        (uint16_t)-1,
#endif
#ifdef PI11
        PI11,
#else
        (uint16_t)-1,
#endif
#ifdef PI12
        PI12,
#else
        (uint16_t)-1,
#endif
#ifdef PI13
        PI13,
#else
        (uint16_t)-1,
#endif
#ifdef PI14
        PI14,
#else
        (uint16_t)-1,
#endif
#ifdef PI15
        PI15,
#else
        (uint16_t)-1,
#endif
    },
#endif // GPIOI_BASE

#ifdef GPIOJ_BASE
    // PORTJ
    {
#ifdef PJ0
        PJ0,
#else
        (uint16_t)-1,
#endif
#ifdef PJ1
        PJ1,
#else
        (uint16_t)-1,
#endif
#ifdef PJ2
        PJ2,
#else
        (uint16_t)-1,
#endif
#ifdef PJ3
        PJ3,
#else
        (uint16_t)-1,
#endif
#ifdef PJ4
        PJ4,
#else
        (uint16_t)-1,
#endif
#ifdef PJ5
        PJ5,
#else
        (uint16_t)-1,
#endif
#ifdef PJ6
        PJ6,
#else
        (uint16_t)-1,
#endif
#ifdef PJ7
        PJ7,
#else
        (uint16_t)-1,
#endif
#ifdef PJ8
        PJ8,
#else
        (uint16_t)-1,
#endif
#ifdef PJ9
        PJ9,
#else
        (uint16_t)-1,
#endif
#ifdef PJ10
        PJ10,
#else
        (uint16_t)-1,
#endif
#ifdef PJ11
        PJ11,
#else
        (uint16_t)-1,
#endif
#ifdef PJ12
        PJ12,
#else
        (uint16_t)-1,
#endif
#ifdef PJ13
        PJ13,
#else
        (uint16_t)-1,
#endif
#ifdef PJ14
        PJ14,
#else
        (uint16_t)-1,
#endif
#ifdef PJ15
        PJ15,
#else
        (uint16_t)-1,
#endif
    },
#endif // GPIOJ_BASE

#ifdef GPIOK_BASE
    // PORTK
    {
#ifdef PK0
        PK0,
#else
        (uint16_t)-1,
#endif
#ifdef PK1
        PK1,
#else
        (uint16_t)-1,
#endif
#ifdef PK2
        PK2,
#else
        (uint16_t)-1,
#endif
#ifdef PK3
        PK3,
#else
        (uint16_t)-1,
#endif
#ifdef PK4
        PK4,
#else
        (uint16_t)-1,
#endif
#ifdef PK5
        PK5,
#else
        (uint16_t)-1,
#endif
#ifdef PK6
        PK6,
#else
        (uint16_t)-1,
#endif
#ifdef PK7
        PK7,
#else
        (uint16_t)-1,
#endif
#ifdef PK8
        PK8,
#else
        (uint16_t)-1,
#endif
#ifdef PK9
        PK9,
#else
        (uint16_t)-1,
#endif
#ifdef PK10
        PK10,
#else
        (uint16_t)-1,
#endif
#ifdef PK11
        PK11,
#else
        (uint16_t)-1,
#endif
#ifdef PK12
        PK12,
#else
        (uint16_t)-1,
#endif
#ifdef PK13
        PK13,
#else
        (uint16_t)-1,
#endif
#ifdef PK14
        PK14,
#else
        (uint16_t)-1,
#endif
#ifdef PK15
        PK15,
#else
        (uint16_t)-1,
#endif
    },
#endif // GPIOK_BASE

    };
};

template <uint16_t IO>
constexpr inline FastIOPin makeFastIO() {
  static_assert(IS_FAST_IO_PIN(IO), "Requires FAST_PIN.");
  return FastIOPin(GET_PORT(IO), GET_PIN_IDX(IO));
}

constexpr inline uint16_t makeSlowIO(uint16_t IO) {
  if (IS_FAST_IO_PIN(IO)) {
    return FastIOPin(GET_PORT(IO), GET_PIN_IDX(IO)).toArduinoPin();
  }
  return IO;
}
#endif // USE_FAST_IO

#define FAST_PIN(port_letter, pin_no) FAST_IO_PIN(Port##port_letter, pin_no)
#define SLOW_PIN(port_letter, pin_no) P##port_letter##pin_no

#ifdef USE_FAST_IO
#define IO_PIN(prt, no) FAST_PIN(prt, no)
#define ARDUINO_PIN(IO) makeSlowIO(IO)
#else
#define IO_PIN(prt, no) SLOW_PIN(prt, no)
#define ARDUINO_PIN(IO) (IO)
#endif


#ifdef USE_FAST_IO
#define READ(IO)                makeFastIO<IO>().read()
#define WRITE(IO,V)             makeFastIO<IO>().write(V)
#else
#define READ(IO)                digitalRead(IO)
#define WRITE(IO,V)             digitalWrite(IO,V)
#endif

//#define _GET_MODE(IO)

#ifdef USE_FAST_IO
#define _SET_MODE(IO, M)        makeFastIO<IO>().set_mode(M)
#else
#define _SET_MODE(IO,M)         pinMode(IO, M)
#endif
#define _SET_OUTPUT(IO)         _SET_MODE(IO, OUTPUT)

#define OUT_WRITE(IO,V)         WRITE(IO,V)

#define SET_INPUT(IO)           _SET_MODE(IO, INPUT)                              /*!< Input Floating Mode                   */
#define SET_INPUT_PULLUP(IO)    _SET_MODE(IO, INPUT_PULLUP)                       /*!< Input with Pull-up activation         */
#define SET_INPUT_PULLDOWN(IO)  _SET_MODE(IO, INPUT_PULLDOWN)                     /*!< Input with Pull-down activation       */
#define SET_OUTPUT(IO)          _SET_MODE(IO, OUTPUT)

#define SET_PWM(IO)             pinMode(ARDUINO_PIN(IO), OUTPUT)

#ifdef USE_FAST_IO
#define TOGGLE(IO)              makeFastIO<IO>().toggle()
#else
#define TOGGLE(IO)              OUT_WRITE(IO, !READ(IO))
#endif

#define IS_INPUT(IO)
#define IS_OUTPUT(IO)

#define PWM_PIN(P)              true

// digitalRead/Write wrappers
#define extDigitalRead(IO)    digitalRead(ARDUINO_PIN(IO))
#define extDigitalWrite(IO,V) digitalWrite(ARDUINO_PIN(IO), V)

