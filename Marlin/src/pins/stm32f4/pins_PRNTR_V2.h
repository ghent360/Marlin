/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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

#if !defined(__STM32F4__) && !defined(STM32F4xx)
  #error "Oops!  Make sure you have an STM32F4 board selected from the 'Tools -> Boards' menu."
#endif

/**
 * Venelin Efremov: PRNTRborad V2 is using STM32F407VE MCU
 * More details at https://github.com/ghent360/PrntrBoardV2
 */

#define DEFAULT_MACHINE_NAME "STM32F4"
#define BOARD_NAME "Marlin for PrntrBoard V2"
#define DEFAULT_WEBSITE_URL "https://blog.pcbxprt.com/index.php/category/prntrboard/"

#if HOTENDS > 3 || E_STEPPERS > 3
  #error "PRNTR Board supports up to 2 hotends / E-steppers."
#endif

// Ignore temp readings during develpment.
#define BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE

#define NUM_SERIAL 2
/*
 USART1(PA9, PA10)
 USART2(PD5, PD6)
 */
#define ADC0        SLOW_PIN(A, 0)
#define ADC1        SLOW_PIN(A, 1)
#define ADC2        SLOW_PIN(A, 2)
#define ADC3        SLOW_PIN(A, 3)
#define ADC4        SLOW_PIN(A, 4)
#define ADC5        SLOW_PIN(A, 5)

#define PWM1        IO_PIN(A, 8)
#define PWM2        IO_PIN(A, 15)
#define PWM3        IO_PIN(B, 4)
#define PWM4        IO_PIN(B, 8)
#define PWM5        SLOW_PIN(B, 9)
#define PWM6        SLOW_PIN(C, 7)
#define PWM7        SLOW_PIN(E, 5)
#define PWM8        SLOW_PIN(E, 6)
#define PWM9        SLOW_PIN(D, 15)

//
// Steppers
//
#define STEPPER_ENABLE_PIN IO_PIN(C, 0)

#define ST1_CS      SLOW_PIN(B, 2)
#define ST2_CS      SLOW_PIN(C, 13)
#define ST3_CS      SLOW_PIN(E, 4)
#define ST4_CS      SLOW_PIN(E, 3)
#define ST5_CS      SLOW_PIN(E, 2)
#define ST6_CS      SLOW_PIN(E, 1)

#define ST1_DIR     IO_PIN(D, 14)
#define ST2_DIR     IO_PIN(D, 13)
#define ST3_DIR     IO_PIN(D, 12)
#define ST4_DIR     IO_PIN(D, 11)
#define ST5_DIR     IO_PIN(D, 10)
#define ST6_DIR     IO_PIN(D, 9)

#define ST1_STEP    IO_PIN(E, 0)
#define ST2_STEP    IO_PIN(B, 5)
#define ST3_STEP    IO_PIN(D, 7)
#define ST4_STEP    IO_PIN(D, 4)
#define ST5_STEP    IO_PIN(D, 1)
#define ST6_STEP    IO_PIN(D, 0)

// Swapped E0 and X motor pins for a moment.
#define TMC2209_INVERT_DIAG
#define X_STEP_PIN         ST1_STEP
#define X_DIR_PIN          ST1_DIR
#define X_ENABLE_PIN       STEPPER_ENABLE_PIN
#define X_CS_PIN           ST1_CS
#define X_MIN_PIN          IO_PIN(C, 4)
#define X_MAX_PIN          IO_PIN(B, 1)

#define Y_STEP_PIN         ST2_STEP
#define Y_DIR_PIN          ST2_DIR
#define Y_ENABLE_PIN       STEPPER_ENABLE_PIN
#define Y_CS_PIN           ST2_CS
#define Y_MIN_PIN          IO_PIN(C, 5)
#define Y_MAX_PIN          IO_PIN(C, 14)

#define Z_STEP_PIN         ST3_STEP
#define Z_DIR_PIN          ST3_DIR
#define Z_ENABLE_PIN       STEPPER_ENABLE_PIN
#define Z_CS_PIN           ST3_CS
#define Z_MIN_PIN          IO_PIN(B, 0)
#define Z_MAX_PIN          IO_PIN(E, 7)

#define Y2_STEP_PIN        -1
#define Y2_DIR_PIN         -1
#define Y2_ENABLE_PIN      -1

#define Z2_STEP_PIN        -1
#define Z2_DIR_PIN         -1
#define Z2_ENABLE_PIN      -1

#define E0_STEP_PIN        ST4_STEP
#define E0_DIR_PIN         ST4_DIR
#define E0_ENABLE_PIN      STEPPER_ENABLE_PIN
#define E0_CS_PIN          ST4_CS

#define E1_STEP_PIN        ST5_STEP
#define E1_DIR_PIN         ST5_DIR
#define E1_ENABLE_PIN      STEPPER_ENABLE_PIN
#define E1_CS_PIN          ST5_CS

#define E2_STEP_PIN        ST6_STEP
#define E2_DIR_PIN         ST6_DIR
#define E2_ENABLE_PIN      STEPPER_ENABLE_PIN
#define E2_CS_PIN          ST6_CS

//
// Heaters / Fans
//
#define HEATER_0_PIN       PWM2    // EXTRUDER 0
#define HEATER_1_PIN       PWM3    // EXTRUDER 1
#define HEATER_2_PIN       PWM4    // EXTRUDER 2

#define HEATER_BED_PIN     PWM1    // BED

#ifndef FAN_PIN
#define FAN_PIN            PWM5  // E0 Part
#endif
#define FAN1_PIN           PWM6  // E1 Part
#define FAN2_PIN           PWM7  // E0 Cool / TC1
//#define FAN3_PIN           PWM8  // E1 Cool / TC2

#define SERVO0_PIN         PWM9
#define SERVO1_PIN         PWM8

#define UART_ST14          SLOW_PIN(B, 10)  // UART ST1-4
#define UART_ST56          SLOW_PIN(C, 6)   // UART ST5-6

#if HAS_TMC_UART
#define X_HARDWARE_SERIAL   Serial_ST14
#define Y_HARDWARE_SERIAL   Serial_ST14
#define Z_HARDWARE_SERIAL   Serial_ST14
#define E0_HARDWARE_SERIAL  Serial_ST14
#define E1_HARDWARE_SERIAL  Serial_ST56
#define E2_HARDWARE_SERIAL  Serial_ST56
#endif

//
// Temperature Sensors
//
#define TEMP_BED_PIN       ADC0
#define TEMP_0_PIN         ADC1
#define TEMP_1_PIN         ADC2
#define TEMP_2_PIN         ADC3
#define TEMP_3_PIN         ADC4
#define TEMP_4_PIN         ADC5

// Laser control
#if ENABLED(SPINDLE_LASER_ENABLE)
#error PRNTRboard does not support SPINDLE_LASER
#endif

// Extruder filament end detectors
#define U_MIN_PIN          IO_PIN(E, 8)
#define V_MIN_PIN          IO_PIN(E, 9)
#define W_MIN_PIN          -1

#define FIL_RUNOUT_PIN     U_MIN_PIN
#define FIL_RUNOUT2_PIN    V_MIN_PIN

#define STEPPER_SPI_MOSI   SLOW_PIN(A, 7)
#define STEPPER_SPI_MISO   SLOW_PIN(A, 6)
#define STEPPER_SPI_SCK    SLOW_PIN(B, 3)

#define EXT_SPI_MOSI   SLOW_PIN(C, 3)
#define EXT_SPI_MISO   SLOW_PIN(C, 2)
#define EXT_SPI_SCK    SLOW_PIN(B, 13)

#define MOSI_PIN STEPPER_SPI_MOSI
#define MISO_PIN STEPPER_SPI_MISO
#define SCK_PIN  STEPPER_SPI_SCK

#define SPI2_MOSI_PIN EXT_SPI_MOSI
#define SPI2_MISO_PIN EXT_SPI_MISO
#define SPI2_SCK_PIN  EXT_SPI_SCK

#define LCD_SPI_INSTANCE SPI_2
#define LCD_SPI_INSTANCE_SETTINGS spiConfig2

// Prevent the default SS_PIN definition
#define SS_PIN -1

#define I2C_EEPROM
#define EEPROM_DEVICE_ADDRESS 0x57

#define SD_DETECT_PIN       IO_PIN(D, 3)
//if ENABLED(SD_DETECT_INVERTED)
//  #error "SD_DETECT_INVERTED must be disabled for the PRNTR_V2 board."
//#endif


/**
 *               _____                                            _____
 * (KILL)   PD8 | · · | GND                                   5V | · · | GND
 * (RESET)      | · · | NC   (SD_DETECT)           (LCD_D7) PE12 | · · | PE13 (LCD_D6)
 *  (MOSI)  PC3 | · · | PB14 (BTN_EN2)             (LCD_D5) PE14 | · · | PE15 (LCD_D4)
 * (SD_SS)  PD3 | · · | PB15 (BTN_EN1)             (LCD_RS) PE11 | · · | PE10 (LCD_E)
 *   (SCK) PB13 | · · | PC2  (MISO)               (BTN_ENC) PB12 | · · | PB11 (BEEPER)
 *               -----                                            -----
 *               EXP2                                             EXP1
 */

#define EXP1_BEEPER  IO_PIN(B, 11)
#define EXP1_BTN_ENC IO_PIN(B, 12)
#define EXP1_LCD_D4  SLOW_PIN(E, 15)
#define EXP1_LCD_D5  SLOW_PIN(E, 14)
#define EXP1_LCD_D6  SLOW_PIN(E, 13)
#define EXP1_LCD_D7  SLOW_PIN(E, 12)
#define EXP1_LCD_RS  SLOW_PIN(E, 11)
#define EXP1_LCD_E   SLOW_PIN(E, 10)

#define EXP2_KILL      IO_PIN(D, 8)
#define EXP2_MOSI      SLOW_PIN(C, 3)
#define EXP2_MISO      SLOW_PIN(C, 2)
#define EXP2_SCK       SLOW_PIN(B, 13)
#define EXP2_SD_SS     SLOW_PIN(D, 3)
#define EXP2_SD_DETECT -1
#define EXP2_BTN_B     IO_PIN(B, 14)
#define EXP2_BTN_A     IO_PIN(B, 15)

#if HAS_SPI_LCD
  #define BEEPER_PIN       EXP1_BEEPER   // (37) not 5V tolerant
  #define BTN_ENC          EXP1_BTN_ENC   // (58) open-drain

  #if ENABLED(CR10_STOCKDISPLAY)
    #define LCD_PINS_RS    EXP1_LCD_D6

    #define BTN_EN1        EXP1_LCD_E
    #define BTN_EN2        EXP1_LCD_D4

    #define LCD_PINS_ENABLE EXP1_LCD_D7
    #define LCD_PINS_D4    EXP1_LCD_D5

  #else
    #define LCD_PINS_RS    EXP1_LCD_RS

    #define BTN_EN1        EXP2_BTN_A
    #define BTN_EN2        EXP2_BTN_B

    #define LCD_PINS_ENABLE EXP1_LCD_E
    #define LCD_PINS_D4    EXP1_LCD_D4

    #define LCD_SDSS       EXP2_SD_SS
    //#define SD_DETECT_PIN  EXP2_SD_DETECT

    #if ENABLED(FYSETC_MINI_12864)
      #define DOGLCD_CS    EXP1_LCD_E
      #define DOGLCD_A0    EXP1_LCD_RS
      #define DOGLCD_SCK   EXP2_SCK
      #define DOGLCD_MOSI  EXP2_MOSI

      #define LCD_BACKLIGHT_PIN -1

      //#define FORCE_SOFT_SPI      // Use this if default of hardware SPI causes display problems
                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0

      #define LCD_RESET_PIN EXP1_LCD_D4 // Must be high or open for LCD to operate normally.

      #if EITHER(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
        #ifndef RGB_LED_R_PIN
          #define RGB_LED_R_PIN EXP1_LCD_D5
        #endif
        #ifndef RGB_LED_G_PIN
          #define RGB_LED_G_PIN EXP1_LCD_D6
        #endif
        #ifndef RGB_LED_B_PIN
          #define RGB_LED_B_PIN EXP1_LCD_D7
        #endif
      #elif ENABLED(FYSETC_MINI_12864_2_1)
        #define NEOPIXEL_PIN    EXP1_LCD_D5
      #endif

    #else // !FYSETC_MINI_12864

      #if ENABLED(MKS_MINI_12864)
        #define DOGLCD_CS  EXP1_LCD_D5
        #define DOGLCD_A0  EXP1_LCD_D6
      #endif

      #if ENABLED(ULTIPANEL)
        #define LCD_PINS_D5 EXP1_LCD_D5
        #define LCD_PINS_D6 EXP1_LCD_D6
        #define LCD_PINS_D7 EXP1_LCD_D7
      #endif

    #endif // !FYSETC_MINI_12864

  #endif

#endif // HAS_SPI_LCD
