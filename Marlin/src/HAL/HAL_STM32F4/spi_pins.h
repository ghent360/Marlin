/**
* Marlin 3D Printer Firmware
* Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#ifndef SPI_PINS_H_
#define SPI_PINS_H_

#define PORTA 0
#define PORTB 1
#define PORTC 2
#define PORTD 3
#define PORTE 4
#define PORTF 5
#define PORTG 6

#define _STM32_PIN(_PORT,_PIN) ((_PORT * 16) + _PIN)

/**
 * Define SPI Pins: SCK, MISO, MOSI, SS
 *
 */
//#define SCK_PIN   _STM32_PIN(PORTA, 5)
//#define MISO_PIN  _STM32_PIN(PORTA, 6)
//#define MOSI_PIN  _STM32_PIN(PORTA, 7)
//#define SS_PIN    _STM32_PIN(PORTA, 8)

#ifndef SCK_PIN
    #define SCK_PIN   PA5
#endif
#ifndef MISO_PIN
    #define MISO_PIN  PA6
#endif
#ifndef MOSI_PIN
    #define MOSI_PIN  PA7
#endif
#ifndef SS_PIN
    #define SS_PIN    PA8
#endif

#endif // SPI_PINS_H_
