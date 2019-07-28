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

/**
 * Fast I/O interfaces for STM32F4/7
 * These use GPIO functions instead of Direct Port Manipulation, as on AVR.
 */

#undef _BV
#define _BV(b) (1 << (b))

#define PWM_PIN(p) true
#define USEABLE_HARDWARE_PWM(p) PWM_PIN(p)

#define USE_FAST_IO

#ifdef USE_FAST_IO
enum PortNumber {
    PORTA = 0,
    PORTB = 1,
    PORTC = 2,
    PORTD = 3,
    PORTE = 4,
    PORTF = 5,
    PORTG = 6,
    PORTH = 7,
    PORTI = 8,
    PORTJ = 9,
    PORTK = 10,
};

#define FAST_IO_PIN(port, pin_no) (0x7000 | ((port) << 4) | (pin_no & 0xf))
#define IS_FAST_IO_PIN(pin) (((pin) & 0xf000) == 0x7000)
#define GET_PORT(fio_pin) ((fio_pin >> 4) & 0xf)
#define GET_PIN_IDX(fio_pin) (fio_pin & 0xf)

struct FastIOPin {
    const intptr_t port_addr_;
    const uint16_t pin_;
    const uint16_t port_num_;

    constexpr FastIOPin(uint32_t fast_io_pin)
        : port_addr_(AHB1PERIPH_BASE + GET_PORT(fast_io_pin) * 0x400),
          pin_(1 << GET_PIN_IDX(fast_io_pin)),
          port_num_(GET_PORT(fast_io_pin)) {}

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
        HAL_GPIO_Init(
            reinterpret_cast<GPIO_TypeDef*>(port_addr_),
            &GPIO_InitStructure);
    }

    void set() const {
        volatile GPIO_TypeDef* gpio_port = reinterpret_cast<GPIO_TypeDef*>(port_addr_);
        gpio_port->BSRR = pin_;
    }

    void reset() const {
        volatile GPIO_TypeDef* const gpio_port = reinterpret_cast<GPIO_TypeDef*>(port_addr_);
        gpio_port->BSRR = pin_ << 16;
    }

    void toggle() const {
        volatile GPIO_TypeDef* const gpio_port = reinterpret_cast<GPIO_TypeDef*>(port_addr_);
        gpio_port->ODR ^= pin_;
    }

    void write(uint8_t val) const {
        if (val) {
            set();
        } else {
            reset();
        }
    }

    uint8_t read() const {
        volatile  GPIO_TypeDef* gpio_port = reinterpret_cast<GPIO_TypeDef*>(port_addr_);
        return (gpio_port->IDR & pin_) ? 1 : 0;
    }
};
#endif

#define FAST_PIN(port_letter, pin_no) FAST_IO_PIN(PORT##port_letter, pin_no)
#define SLOW_PIN(port_letter, pin_no) P##port_letter##pin_no

#ifdef USE_FAST_IO
#define IO_PIN(prt, no) FAST_PIN(prt, no)
#else
#define IO_PIN(prt, no) SLOW_PIN(prt, no)
#endif

#ifdef USE_FAST_IO
#define READ(IO)                FastIOPin(IO).read()
#define WRITE(IO,V)             FastIOPin(IO).write(V)
#else
#define READ(IO)                digitalRead(IO)
#define WRITE(IO,V)             digitalWrite(IO,V)
#endif

//#define _GET_MODE(IO)

#ifdef USE_FAST_IO
#define _SET_MODE(IO, M)        FastIOPin(IO).set_mode(M)
#else
#define _SET_MODE(IO,M)         pinMode(IO, M)
#endif
#define _SET_OUTPUT(IO)         _SET_MODE(IO, OUTPUT)

#define OUT_WRITE(IO,V)         WRITE(IO,V)

#define SET_INPUT(IO)           _SET_MODE(IO, INPUT)                              /*!< Input Floating Mode                   */
#define SET_INPUT_PULLUP(IO)    _SET_MODE(IO, INPUT_PULLUP)                       /*!< Input with Pull-up activation         */
#define SET_INPUT_PULLDOWN(IO)  _SET_MODE(IO, INPUT_PULLDOWN)                     /*!< Input with Pull-down activation       */
#define SET_OUTPUT(IO)          _SET_MODE(IO, OUTPUT)
#define SET_PWM(IO)             SET_OUTPUT(IO)

#ifdef USE_FAST_IO
#define TOGGLE(IO)              FastIOPin(IO).toggle()
#else
#define TOGGLE(IO)              OUT_WRITE(IO, !READ(IO))
#endif

#define IS_INPUT(IO)
#define IS_OUTPUT(IO)

#define PWM_PIN(P)              true

// digitalRead/Write wrappers
#define extDigitalRead(IO)    digitalRead(IO)
#define extDigitalWrite(IO,V) digitalWrite(IO,V)

#ifdef REDEFINE_PIN_NAMES
//
// Pins Definitions
//
#define PORTA 0
#define PORTB 1
#define PORTC 2
#define PORTD 3
#define PORTE 4

#define _STM32_PIN(P,PN) ((PORT##P * 16) + PN)

#define PA0  _STM32_PIN(A,  0)
#define PA1  _STM32_PIN(A,  1)
#define PA2  _STM32_PIN(A,  2)
#define PA3  _STM32_PIN(A,  3)
#define PA4  _STM32_PIN(A,  4)
#define PA5  _STM32_PIN(A,  5)
#define PA6  _STM32_PIN(A,  6)
#define PA7  _STM32_PIN(A,  7)
#define PA8  _STM32_PIN(A,  8)
#define PA9  _STM32_PIN(A,  9)
#define PA10 _STM32_PIN(A, 10)
#define PA11 _STM32_PIN(A, 11)
#define PA12 _STM32_PIN(A, 12)
#define PA13 _STM32_PIN(A, 13)
#define PA14 _STM32_PIN(A, 14)
#define PA15 _STM32_PIN(A, 15)

#define PB0  _STM32_PIN(B,  0)
#define PB1  _STM32_PIN(B,  1)
#define PB2  _STM32_PIN(B,  2)
#define PB3  _STM32_PIN(B,  3)
#define PB4  _STM32_PIN(B,  4)
#define PB5  _STM32_PIN(B,  5)
#define PB6  _STM32_PIN(B,  6)
#define PB7  _STM32_PIN(B,  7)
#define PB8  _STM32_PIN(B,  8)
#define PB9  _STM32_PIN(B,  9)
#define PB10 _STM32_PIN(B, 10)
#define PB11 _STM32_PIN(B, 11)
#define PB12 _STM32_PIN(B, 12)
#define PB13 _STM32_PIN(B, 13)
#define PB14 _STM32_PIN(B, 14)
#define PB15 _STM32_PIN(B, 15)

#define PC0  _STM32_PIN(C,  0)
#define PC1  _STM32_PIN(C,  1)
#define PC2  _STM32_PIN(C,  2)
#define PC3  _STM32_PIN(C,  3)
#define PC4  _STM32_PIN(C,  4)
#define PC5  _STM32_PIN(C,  5)
#define PC6  _STM32_PIN(C,  6)
#define PC7  _STM32_PIN(C,  7)
#define PC8  _STM32_PIN(C,  8)
#define PC9  _STM32_PIN(C,  9)
#define PC10 _STM32_PIN(C, 10)
#define PC11 _STM32_PIN(C, 11)
#define PC12 _STM32_PIN(C, 12)
#define PC13 _STM32_PIN(C, 13)
#define PC14 _STM32_PIN(C, 14)
#define PC15 _STM32_PIN(C, 15)

#define PD0  _STM32_PIN(D,  0)
#define PD1  _STM32_PIN(D,  1)
#define PD2  _STM32_PIN(D,  2)
#define PD3  _STM32_PIN(D,  3)
#define PD4  _STM32_PIN(D,  4)
#define PD5  _STM32_PIN(D,  5)
#define PD6  _STM32_PIN(D,  6)
#define PD7  _STM32_PIN(D,  7)
#define PD8  _STM32_PIN(D,  8)
#define PD9  _STM32_PIN(D,  9)
#define PD10 _STM32_PIN(D, 10)
#define PD11 _STM32_PIN(D, 11)
#define PD12 _STM32_PIN(D, 12)
#define PD13 _STM32_PIN(D, 13)
#define PD14 _STM32_PIN(D, 14)
#define PD15 _STM32_PIN(D, 15)

#define PE0  _STM32_PIN(E,  0)
#define PE1  _STM32_PIN(E,  1)
#define PE2  _STM32_PIN(E,  2)
#define PE3  _STM32_PIN(E,  3)
#define PE4  _STM32_PIN(E,  4)
#define PE5  _STM32_PIN(E,  5)
#define PE6  _STM32_PIN(E,  6)
#define PE7  _STM32_PIN(E,  7)
#define PE8  _STM32_PIN(E,  8)
#define PE9  _STM32_PIN(E,  9)
#define PE10 _STM32_PIN(E, 10)
#define PE11 _STM32_PIN(E, 11)
#define PE12 _STM32_PIN(E, 12)
#define PE13 _STM32_PIN(E, 13)
#define PE14 _STM32_PIN(E, 14)
#define PE15 _STM32_PIN(E, 15)

#ifdef STM32F7
  #define PORTF 5
  #define PORTG 6

  #define PF0  _STM32_PIN(F,  0)
  #define PF1  _STM32_PIN(F,  1)
  #define PF2  _STM32_PIN(F,  2)
  #define PF3  _STM32_PIN(F,  3)
  #define PF4  _STM32_PIN(F,  4)
  #define PF5  _STM32_PIN(F,  5)
  #define PF6  _STM32_PIN(F,  6)
  #define PF7  _STM32_PIN(F,  7)
  #define PF8  _STM32_PIN(F,  8)
  #define PF9  _STM32_PIN(F,  9)
  #define PF10 _STM32_PIN(F, 10)
  #define PF11 _STM32_PIN(F, 11)
  #define PF12 _STM32_PIN(F, 12)
  #define PF13 _STM32_PIN(F, 13)
  #define PF14 _STM32_PIN(F, 14)
  #define PF15 _STM32_PIN(F, 15)

  #define PG0  _STM32_PIN(G,  0)
  #define PG1  _STM32_PIN(G,  1)
  #define PG2  _STM32_PIN(G,  2)
  #define PG3  _STM32_PIN(G,  3)
  #define PG4  _STM32_PIN(G,  4)
  #define PG5  _STM32_PIN(G,  5)
  #define PG6  _STM32_PIN(G,  6)
  #define PG7  _STM32_PIN(G,  7)
  #define PG8  _STM32_PIN(G,  8)
  #define PG9  _STM32_PIN(G,  9)
  #define PG10 _STM32_PIN(G, 10)
  #define PG11 _STM32_PIN(G, 11)
  #define PG12 _STM32_PIN(G, 12)
  #define PG13 _STM32_PIN(G, 13)
  #define PG14 _STM32_PIN(G, 14)
  #define PG15 _STM32_PIN(G, 15)
#endif // STM32F7

#endif // REDEFINE_PIN_NAMES
