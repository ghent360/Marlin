/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2017 Victor Perez
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

/**
 * Fast I/O interfaces for STM32F4
 * These use GPIO functions instead of Direct Port Manipulation, as on AVR.
 */

#ifndef _FASTIO_STM32F4_H
#define _FASTIO_STM32F4_H

#define _BV(b) (1 << (b))

#define USEABLE_HARDWARE_PWM(p) true
#define USE_FAST_IO

#ifdef USE_FAST_IO
enum PinNumber {
    Pin0  = 0x0001,
    Pin1  = 0x0002,
    Pin2  = 0x0004,
    Pin3  = 0x0008,
    Pin4  = 0x0010,
    Pin5  = 0x0020,
    Pin6  = 0x0040,
    Pin7  = 0x0080,
    Pin8  = 0x0100,
    Pin9  = 0x0200,
    Pin10 = 0x0400,
    Pin11 = 0x0800,
    Pin12 = 0x1000,
    Pin13 = 0x2000,
    Pin14 = 0x4000,
    Pin15 = 0x8000,
};

struct FastIOPin {
    const intptr_t port_;
    const uint16_t pin_;

    constexpr FastIOPin(uint32_t port, PinNumber pin)
        : port_(port),
          pin_((uint16_t)pin) {}

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
        HAL_GPIO_Init(reinterpret_cast<GPIO_TypeDef*>(port_), &GPIO_InitStructure);
    }

    void set() const {
        volatile GPIO_TypeDef* gpio_port = reinterpret_cast<GPIO_TypeDef*>(port_);
        gpio_port->BSRR = pin_;
    }

    void reset() const {
        volatile GPIO_TypeDef* const gpio_port = reinterpret_cast<GPIO_TypeDef*>(port_);
        gpio_port->BSRR = pin_ << 16;
    }

    void toggle() const {
        volatile GPIO_TypeDef* const gpio_port = reinterpret_cast<GPIO_TypeDef*>(port_);
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
        volatile  GPIO_TypeDef* gpio_port = reinterpret_cast<GPIO_TypeDef*>(port_);
        return (gpio_port->IDR & pin_) ? 1 : 0;
    }
};
#endif

#ifdef USE_FAST_IO
#define READ(IO)                IO.read()
#define WRITE(IO,V)             IO.write(V)
#else
#define READ(IO)                digitalRead(IO)
#define WRITE(IO,V)             digitalWrite(IO,V)
#endif
#define WRITE_VAR(IO,V)         WRITE(IO,V)

//#define _GET_MODE(IO)

#ifdef USE_FAST_IO
#define _SET_MODE(IO, M)        IO.set_mode(M)
#define _SET_OUTPUT(IO)         IO.set_mode(OUTPUT)
#else
#define _SET_MODE(IO,M)         pinMode(IO, M)
#define _SET_OUTPUT(IO)         pinMode(IO, OUTPUT)
#endif

#define OUT_WRITE(IO,V)         WRITE(IO,V)

#define SET_INPUT(IO)           _SET_MODE(IO, INPUT)                              /*!< Input Floating Mode                   */
#define SET_INPUT_PULLUP(IO)    _SET_MODE(IO, INPUT_PULLUP)                       /*!< Input with Pull-up activation         */
#define SET_INPUT_PULLDOWN(IO)  _SET_MODE(IO, INPUT_PULLDOWN)                     /*!< Input with Pull-down activation       */
#define SET_OUTPUT(IO)          _SET_MODE(IO, OUTPUT)

#ifdef USE_FAST_IO
#define TOGGLE(IO)              IO.toggle()
#else
#define TOGGLE(IO)              OUT_WRITE(IO, !READ(IO))
#endif

//#define GET_INPUT(IO)
//#define GET_OUTPUT(IO)
//#define GET_TIMER(IO)

#ifdef USE_FAST_IO
constexpr FastIOPin GPIOA_0(GPIOA_BASE, Pin0);
constexpr FastIOPin GPIOA_1(GPIOA_BASE, Pin1);
constexpr FastIOPin GPIOA_2(GPIOA_BASE, Pin2);
constexpr FastIOPin GPIOA_3(GPIOA_BASE, Pin3);
constexpr FastIOPin GPIOA_4(GPIOA_BASE, Pin4);
constexpr FastIOPin GPIOA_5(GPIOA_BASE, Pin5);
constexpr FastIOPin GPIOA_6(GPIOA_BASE, Pin6);
constexpr FastIOPin GPIOA_7(GPIOA_BASE, Pin7);
constexpr FastIOPin GPIOA_8(GPIOA_BASE, Pin8);
constexpr FastIOPin GPIOA_9(GPIOA_BASE, Pin9);
constexpr FastIOPin GPIOA_10(GPIOA_BASE, Pin10);
constexpr FastIOPin GPIOA_11(GPIOA_BASE, Pin11);
constexpr FastIOPin GPIOA_12(GPIOA_BASE, Pin12);
constexpr FastIOPin GPIOA_13(GPIOA_BASE, Pin13);
constexpr FastIOPin GPIOA_14(GPIOA_BASE, Pin14);
constexpr FastIOPin GPIOA_15(GPIOA_BASE, Pin15);

constexpr FastIOPin GPIOB_0(GPIOB_BASE, Pin0);
constexpr FastIOPin GPIOB_1(GPIOB_BASE, Pin1);
constexpr FastIOPin GPIOB_2(GPIOB_BASE, Pin2);
constexpr FastIOPin GPIOB_3(GPIOB_BASE, Pin3);
constexpr FastIOPin GPIOB_4(GPIOB_BASE, Pin4);
constexpr FastIOPin GPIOB_5(GPIOB_BASE, Pin5);
constexpr FastIOPin GPIOB_6(GPIOB_BASE, Pin6);
constexpr FastIOPin GPIOB_7(GPIOB_BASE, Pin7);
constexpr FastIOPin GPIOB_8(GPIOB_BASE, Pin8);
constexpr FastIOPin GPIOB_9(GPIOB_BASE, Pin9);
constexpr FastIOPin GPIOB_10(GPIOB_BASE, Pin10);
constexpr FastIOPin GPIOB_11(GPIOB_BASE, Pin11);
constexpr FastIOPin GPIOB_12(GPIOB_BASE, Pin12);
constexpr FastIOPin GPIOB_13(GPIOB_BASE, Pin13);
constexpr FastIOPin GPIOB_14(GPIOB_BASE, Pin14);
constexpr FastIOPin GPIOB_15(GPIOB_BASE, Pin15);

#ifdef GPIOC_BASE
constexpr FastIOPin GPIOC_0(GPIOC_BASE, Pin0);
constexpr FastIOPin GPIOC_1(GPIOC_BASE, Pin1);
constexpr FastIOPin GPIOC_2(GPIOC_BASE, Pin2);
constexpr FastIOPin GPIOC_3(GPIOC_BASE, Pin3);
constexpr FastIOPin GPIOC_4(GPIOC_BASE, Pin4);
constexpr FastIOPin GPIOC_5(GPIOC_BASE, Pin5);
constexpr FastIOPin GPIOC_6(GPIOC_BASE, Pin6);
constexpr FastIOPin GPIOC_7(GPIOC_BASE, Pin7);
constexpr FastIOPin GPIOC_8(GPIOC_BASE, Pin8);
constexpr FastIOPin GPIOC_9(GPIOC_BASE, Pin9);
constexpr FastIOPin GPIOC_10(GPIOC_BASE, Pin10);
constexpr FastIOPin GPIOC_11(GPIOC_BASE, Pin11);
constexpr FastIOPin GPIOC_12(GPIOC_BASE, Pin12);
constexpr FastIOPin GPIOC_13(GPIOC_BASE, Pin13);
constexpr FastIOPin GPIOC_14(GPIOC_BASE, Pin14);
constexpr FastIOPin GPIOC_15(GPIOC_BASE, Pin15);
#endif // GPIOC_BASE

#ifdef GPIOD_BASE
constexpr FastIOPin GPIOD_0(GPIOD_BASE, Pin0);
constexpr FastIOPin GPIOD_1(GPIOD_BASE, Pin1);
constexpr FastIOPin GPIOD_2(GPIOD_BASE, Pin2);
constexpr FastIOPin GPIOD_3(GPIOD_BASE, Pin3);
constexpr FastIOPin GPIOD_4(GPIOD_BASE, Pin4);
constexpr FastIOPin GPIOD_5(GPIOD_BASE, Pin5);
constexpr FastIOPin GPIOD_6(GPIOD_BASE, Pin6);
constexpr FastIOPin GPIOD_7(GPIOD_BASE, Pin7);
constexpr FastIOPin GPIOD_8(GPIOD_BASE, Pin8);
constexpr FastIOPin GPIOD_9(GPIOD_BASE, Pin9);
constexpr FastIOPin GPIOD_10(GPIOD_BASE, Pin10);
constexpr FastIOPin GPIOD_11(GPIOD_BASE, Pin11);
constexpr FastIOPin GPIOD_12(GPIOD_BASE, Pin12);
constexpr FastIOPin GPIOD_13(GPIOD_BASE, Pin13);
constexpr FastIOPin GPIOD_14(GPIOD_BASE, Pin14);
constexpr FastIOPin GPIOD_15(GPIOD_BASE, Pin15);
#endif // GPIOD_BASE

#ifdef GPIOE_BASE
constexpr FastIOPin GPIOE_0(GPIOE_BASE, Pin0);
constexpr FastIOPin GPIOE_1(GPIOE_BASE, Pin1);
constexpr FastIOPin GPIOE_2(GPIOE_BASE, Pin2);
constexpr FastIOPin GPIOE_3(GPIOE_BASE, Pin3);
constexpr FastIOPin GPIOE_4(GPIOE_BASE, Pin4);
constexpr FastIOPin GPIOE_5(GPIOE_BASE, Pin5);
constexpr FastIOPin GPIOE_6(GPIOE_BASE, Pin6);
constexpr FastIOPin GPIOE_7(GPIOE_BASE, Pin7);
constexpr FastIOPin GPIOE_8(GPIOE_BASE, Pin8);
constexpr FastIOPin GPIOE_9(GPIOE_BASE, Pin9);
constexpr FastIOPin GPIOE_10(GPIOE_BASE, Pin10);
constexpr FastIOPin GPIOE_11(GPIOE_BASE, Pin11);
constexpr FastIOPin GPIOE_12(GPIOE_BASE, Pin12);
constexpr FastIOPin GPIOE_13(GPIOE_BASE, Pin13);
constexpr FastIOPin GPIOE_14(GPIOE_BASE, Pin14);
constexpr FastIOPin GPIOE_15(GPIOE_BASE, Pin15);
#endif // GPIOE_BASE
#else
#define GPIOA_0     PA0
#define GPIOA_1     PA1
#define GPIOA_2     PA2
#define GPIOA_3     PA3
#define GPIOA_4     PA4
#define GPIOA_5     PA5
#define GPIOA_6     PA6
#define GPIOA_7     PA7
#define GPIOA_8     PA8
#define GPIOA_9     PA9
#define GPIOA_10    PA10
#define GPIOA_11    PA11
#define GPIOA_12    PA12
#define GPIOA_13    PA13
#define GPIOA_14    PA14
#define GPIOA_15    PA15

#define GPIOB_0     PB0
#define GPIOB_1     PB1
#define GPIOB_2     PB2
#define GPIOB_3     PB3
#define GPIOB_4     PB4
#define GPIOB_5     PB5
#define GPIOB_6     PB6
#define GPIOB_7     PB7
#define GPIOB_8     PB8
#define GPIOB_9     PB9
#define GPIOB_10    PB10
#define GPIOB_11    PB11
#define GPIOB_12    PB12
#define GPIOB_13    PB13
#define GPIOB_14    PB14
#define GPIOB_15    PB15

#define GPIOC_0     PC0
#define GPIOC_1     PC1
#define GPIOC_2     PC2
#define GPIOC_3     PC3
#define GPIOC_4     PC4
#define GPIOC_5     PC5
#define GPIOC_6     PC6
#define GPIOC_7     PC7
#define GPIOC_8     PC8
#define GPIOC_9     PC9
#define GPIOC_10    PC10
#define GPIOC_11    PC11
#define GPIOC_12    PC12
#define GPIOC_13    PC13
#define GPIOC_14    PC14
#define GPIOC_15    PC15

#define GPIOD_0     PD0
#define GPIOD_1     PD1
#define GPIOD_2     PD2
#define GPIOD_3     PD3
#define GPIOD_4     PD4
#define GPIOD_5     PD5
#define GPIOD_6     PD6
#define GPIOD_7     PD7
#define GPIOD_8     PD8
#define GPIOD_9     PD9
#define GPIOD_10    PD10
#define GPIOD_11    PD11
#define GPIOD_12    PD12
#define GPIOD_13    PD13
#define GPIOD_14    PD14
#define GPIOD_15    PD15

#define GPIOE_0     PE0
#define GPIOE_1     PE1
#define GPIOE_2     PE2
#define GPIOE_3     PE3
#define GPIOE_4     PE4
#define GPIOE_5     PE5
#define GPIOE_6     PE6
#define GPIOE_7     PE7
#define GPIOE_8     PE8
#define GPIOE_9     PE9
#define GPIOE_10    PE10
#define GPIOE_11    PE11
#define GPIOE_12    PE12
#define GPIOE_13    PE13
#define GPIOE_14    PE14
#define GPIOE_15    PE15
#endif

#ifdef REDEFINE_PIN_NAMES
#define PORTA 0
#define PORTB 1
#define PORTC 2
#define PORTD 3
#define PORTE 4

#define _STM32_PIN(_PORT,_PIN) ((PORT##_PORT * 16) + _PIN)

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
#endif // REDEFINE_PIN_NAMES

#endif // _FASTIO_STM32F4_H
