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

#define PRNTR_BOARD_REV 3
#define NUCLEO_BOARD_REV 3

/**
 * 21017 Venelin Efremov Marlin for stm32f4 test
 */

#define DEFAULT_MACHINE_NAME "STM32F4"
#if PRNTR_BOARD_REV < 3
#define BOARD_NAME "Marlin for PrntrBoard V1(F407)"
#else
#define BOARD_NAME "Marlin for PrntrBoard V1.3(F407)"
#endif
#define DEFAULT_WEBSITE_URL "https://blog.pcbxprt.com/index.php/category/prntrboard/"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "PRNTR Board supports up to 2 hotends / E-steppers."
#endif

// Ignore temp readings during develpment.
#define BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE

#define NUM_SERIAL 2

//
// Steppers
//
#define STEPPER_ENABLE_PIN IO_PIN(C, 1)

// Swapped E0 and X motor pins for a moment.
#define X_STEP_PIN         IO_PIN(E, 12)
#define X_DIR_PIN          IO_PIN(C, 4)
#define X_ENABLE_PIN       STEPPER_ENABLE_PIN
#define X_MIN_PIN          IO_PIN(D, 0)
#define X_MAX_PIN          IO_PIN(D, 4)

#define Y_STEP_PIN         IO_PIN(C, 5)
#define Y_DIR_PIN          IO_PIN(D, 13)
#define Y_ENABLE_PIN       STEPPER_ENABLE_PIN
#define Y_MIN_PIN          IO_PIN(D, 1)
#define Y_MAX_PIN          IO_PIN(D, 7)

#define Z_STEP_PIN         IO_PIN(B, 13)
#define Z_DIR_PIN          IO_PIN(D, 12)
#define Z_ENABLE_PIN       STEPPER_ENABLE_PIN
#define Z_MIN_PIN          IO_PIN(D, 3)
#define Z_MAX_PIN          IO_PIN(D, 8)

#define Y2_STEP_PIN        -1
#define Y2_DIR_PIN         -1
#define Y2_ENABLE_PIN      -1

#define Z2_STEP_PIN        -1
#define Z2_DIR_PIN         -1
#define Z2_ENABLE_PIN      -1

#define E0_STEP_PIN        IO_PIN(C, 7)
#define E0_DIR_PIN         IO_PIN(D, 14)
#define E0_ENABLE_PIN      STEPPER_ENABLE_PIN

#define E1_STEP_PIN        IO_PIN(E, 15)
#define E1_DIR_PIN         IO_PIN(B, 0)
#define E1_ENABLE_PIN      STEPPER_ENABLE_PIN

#define E2_STEP_PIN        -1
#define E2_DIR_PIN         -1
#define E2_ENABLE_PIN      -1

#define X_CS_PIN           SLOW_PIN(D, 9)
#define Y_CS_PIN           SLOW_PIN(D, 10)
#define Z_CS_PIN           SLOW_PIN(D, 11)
#if PRNTR_BOARD_REV < 3
#define E0_CS_PIN          SLOW_PIN(B, 8)
#define E1_CS_PIN          SLOW_PIN(B, 9)
#else
#define E0_CS_PIN          SLOW_PIN(B, 1)
#define E1_CS_PIN          SLOW_PIN(D, 15)
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN       IO_PIN(B, 6)   // EXTRUDER 0
#define HEATER_1_PIN       IO_PIN(B, 7)   // EXTRUDER 1
//#define HEATER_2_PIN       -1

#define HEATER_BED_PIN     IO_PIN(B, 11) // BED
//#define HEATER_BED2_PIN    -1    // BED2
//#define HEATER_BED3_PIN    -1    // BED3

#ifndef FAN_PIN
#define FAN_PIN            SLOW_PIN(B, 14)  // E0 Part
#endif
#define FAN1_PIN           SLOW_PIN(B, 15)  // E1 Part
#define FAN2_PIN           SLOW_PIN(C, 6)   // E0 Cool / TC1
#define FAN3_PIN           SLOW_PIN(B, 12)  // E1 Cool / TC2

//#define FAN_SOFT_PWM

//
// Temperature Sensors
//
#define TEMP_BED_PIN       SLOW_PIN(A, 2)
#define TEMP_0_PIN         SLOW_PIN(A, 0)
#define TEMP_1_PIN         SLOW_PIN(A, 1)
#define TEMP_2_PIN         -1

// Laser control
#if ENABLED(SPINDLE_LASER_ENABLE)
#error PRNTRboard does not support SPINDLE_LASER
#endif

// Extruder filament end detectors
#define U_MIN_PIN          IO_PIN(C, 0)
#define V_MIN_PIN          IO_PIN(E, 13)
#define W_MIN_PIN          -1

#define FIL_RUNOUT_PIN     U_MIN_PIN
#define FIL_RUNOUT2_PIN    V_MIN_PIN

#define STEPPER_SPI_MOSI   SLOW_PIN(B, 5)
#define STEPPER_SPI_MISO   SLOW_PIN(B, 4)
#define STEPPER_SPI_SCK    SLOW_PIN(B, 3)

#define MOSI_PIN STEPPER_SPI_MOSI
#define MISO_PIN STEPPER_SPI_MISO
#define SCK_PIN  STEPPER_SPI_SCK

#define Z_MIN_PROBE_PIN  IO_PIN(C, 13)

// Prevent the default SS_PIN definition
#define SS_PIN -1

// 16Mbit SPI FLASH
#define SPI_FLASH
#define SPI_FLASH_CS_PIN    SLOW_PIN(E, 8)
// Emulate 4k EEPROM emulation
#define E2END (0xfff - 8)

#define SD_DETECT_PIN       PE7
#if ENABLED(SD_DETECT_INVERTED)
  #error "SD_DETECT_INVERTED must be disabled for the PRNTR_F407 board."
#endif


/**
 * For NUCLEO-F407 rev 1-3
 *               _____                                            _____
 *           NC | · · | GND                                   5V | · · | GND
 * (RESET)  PE6 | · · | PE3  (SD_DETECT)           (LCD_D7)  PA8 | · · | PA7  (LCD_D6)
 *  (MOSI)  PC3 | · · | PE1  (BTN_EN2)             (LCD_D5)  PA6 | · · | PA3  (LCD_D4)
 * (SD_SS)  PE7 | · · | PE2  (BTN_EN1)             (LCD_RS)  PE4 | · · | PA15 (LCD_EN)
 *   (SCK) PB10 | · · | PC2  (MISO)               (BTN_ENC)  PE0 | · · | PE5  (BEEPER)
 *               -----                                            -----
 *               EXP2                                             EXP1
 *
 *
 * For NUCLEO-F407 rev 4+
 *               _____                                            _____
 *           NC | · · | GND                                   5V | · · | GND
 * (RESET)  PE6 | · · | PE3  (SD_DETECT)           (LCD_D7)  PA8 | · · | PA7  (LCD_D6)
 *  (MOSI)  PC3 | · · | PE10 (BTN_EN2)             (LCD_D5)  PA6 | · · | PA3  (LCD_D4)
 * (SD_SS)  PE7 | · · | PE9  (BTN_EN1)             (LCD_RS)  PE4 | · · | PE14 (LCD_EN)
 *   (SCK) PB10 | · · | PC2  (MISO)               (BTN_ENC) PC14 | · · | PE5  (BEEPER)
 *               -----                                            -----
 *               EXP2                                             EXP1
 */

#define EXP1_BEEPER  IO_PIN(E, 5)
#if NUCLEO_BOARD_REV <=3
#define EXP1_LCD_EN  SLOW_PIN(A, 15)
#else
#define EXP1_LCD_EN  SLOW_PIN(E, 14)
#endif
#define EXP1_LCD_D4  SLOW_PIN(A, 3)
#define EXP1_LCD_D5  SLOW_PIN(A, 6)
#define EXP1_LCD_D6  SLOW_PIN(A, 7)
#define EXP1_LCD_D7  SLOW_PIN(A, 8)
#define EXP1_LCD_RS  SLOW_PIN(E, 4)
#if NUCLEO_BOARD_REV <=3
  #define EXP1_BTN_ENC IO_PIN(E, 0)
#else
  #define EXP1_BTN_ENC IO_PIN(C, 14)
#endif

#define EXP2_SD_RST    IO_PIN(E, 6)
#define EXP2_MOSI      SLOW_PIN(C, 3)
#define EXP2_SD_SS     SLOW_PIN(E, 7)
#define EXP2_SCK       SLOW_PIN(B, 10)
#define EXP2_SD_DETECT IO_PIN(E, 3)
#if NUCLEO_BOARD_REV <=3
  #define EXP2_BTN_B     IO_PIN(E, 1)
  #define EXP2_BTN_A     IO_PIN(E, 2)
#else
  #define EXP2_BTN_B     IO_PIN(E, 10)
  #define EXP2_BTN_A     IO_PIN(E, 9)
#endif
#define EXP2_MISO      SLOW_PIN(C, 2)

#if HAS_SPI_LCD
  #define BEEPER_PIN       EXP1_BEEPER   // (37) not 5V tolerant
  #define BTN_ENC          EXP1_BTN_ENC   // (58) open-drain

  #if ENABLED(CR10_STOCKDISPLAY)
    #define LCD_PINS_RS    EXP1_LCD_D6

    #define BTN_EN1        EXP1_LCD_EN
    #define BTN_EN2        EXP1_LCD_D4

    #define LCD_PINS_ENABLE EXP1_LCD_D7
    #define LCD_PINS_D4    EXP1_LCD_D5

  #else
    #define LCD_PINS_RS    EXP1_LCD_RS

    #define BTN_EN1        EXP2_BTN_A
    #define BTN_EN2        EXP2_BTN_B

    #define LCD_PINS_ENABLE EXP1_LCD_EN
    #define LCD_PINS_D4    EXP1_LCD_D4

    #define LCD_SDSS       EXP2_SD_SS
    #define SD_DETECT_PIN  EXP2_SD_DETECT

    #if ENABLED(FYSETC_MINI_12864)
      #define DOGLCD_CS    EXP1_LCD_EN
      #define DOGLCD_A0    EXP1_LCD_RS
      #define DOGLCD_SCK   EXP2_SCK
      #define DOGLCD_MOSI  EXP2_MOSI

      #define LCD_BACKLIGHT_PIN -1

      #define FORCE_SOFT_SPI      // Use this if default of hardware SPI causes display problems
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
