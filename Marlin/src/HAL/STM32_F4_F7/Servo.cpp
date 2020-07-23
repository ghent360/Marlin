/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#include "../HAL.h"
#if (HAL_PLATFORM_ID == HAL_ID_STM32_F4_F7)

#include "../../inc/MarlinConfig.h"

#if HAS_SERVOS

#include "Servo.h"

int8_t libServo::attach(const int pin) {
  return super::attach(pin);
}

int8_t libServo::attach(const int pin, const int min, const int max) {
  this->pin_ = pin;
  return super::attach(pin, min, max);
}


#define CHECK_PIN_(IDX) if (pin_ == SERVO##IDX##_PIN) servoIndex = IDX;
#define SERVO_ENABLED(IDX) defined(SERVO##IDX##_PIN) && (SERVO##IDX##_PIN >= 0)
#if SERVO_ENABLED(0)
#define CHECK_PIN_0 CHECK_PIN_(0)
#else
#define CHECK_PIN_0 ;
#endif
#if SERVO_ENABLED(1)
#define CHECK_PIN_1 CHECK_PIN_(1)
#else
#define CHECK_PIN_1 ;
#endif
#if SERVO_ENABLED(2)
#define CHECK_PIN_2 CHECK_PIN_(2)
#else
#define CHECK_PIN_2 ;
#endif
#if SERVO_ENABLED(3)
#define CHECK_PIN_3 CHECK_PIN_(3)
#else
#define CHECK_PIN_3 ;
#endif
#if SERVO_ENABLED(4)
#define CHECK_PIN_4 CHECK_PIN_(4)
#else
#define CHECK_PIN_4 ;
#endif
#if SERVO_ENABLED(5)
#define CHECK_PIN_5 CHECK_PIN_(5)
#else
#define CHECK_PIN_5 ;
#endif
#if SERVO_ENABLED(6)
#define CHECK_PIN_6 CHECK_PIN_(6)
#else
#define CHECK_PIN_6 ;
#endif
#if SERVO_ENABLED(7)
#define CHECK_PIN_7 CHECK_PIN_(7)
#else
#define CHECK_PIN_7 ;
#endif
#if SERVO_ENABLED(8)
#define CHECK_PIN_8 CHECK_PIN_(8)
#else
#define CHECK_PIN_8 ;
#endif

void libServo::move(const int value) {
  constexpr uint16_t servo_delay[] = SERVO_DELAY;
  static_assert(COUNT(servo_delay) == NUM_SERVOS, "SERVO_DELAY must be an array NUM_SERVOS long.");
  if (!attached()) attach(pin_);
  write(value);
  uint8_t servoIndex = NUM_SERVOS + 1;
  CHECK_PIN_0
  CHECK_PIN_1
  CHECK_PIN_2
  CHECK_PIN_3
  CHECK_PIN_4
  CHECK_PIN_5
  CHECK_PIN_6
  CHECK_PIN_7
  CHECK_PIN_8
  if (servoIndex < NUM_SERVOS) {
    safe_delay(servo_delay[servoIndex]);
  }
  TERN_(DEACTIVATE_SERVOS_AFTER_MOVE, detach());
}

#endif // HAS_SERVOS
#endif // (HAL_PLATFORM_ID == HAL_ID_STM32_F4_F7)
