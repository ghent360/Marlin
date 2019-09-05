/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
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
#include <HardwareTimer.h>

// ------------------------
// Defines
// ------------------------

#define FORCE_INLINE __attribute__((always_inline)) inline

#define hal_timer_t uint32_t  // TODO: One is 16-bit, one 32-bit - does this need to be checked?
#define HAL_TIMER_TYPE_MAX 0xFFFF

#define HAL_TIMER_RATE         (HAL_RCC_GetSysClockFreq() / 2)  // frequency of timer peripherals

#define STEP_TIMER_NUM 0  // index of timer to use for stepper
#define TEMP_TIMER_NUM 1  // index of timer to use for temperature
#define PULSE_TIMER_NUM STEP_TIMER_NUM

//#define TEMP_TIMER_PRESCALE     1000 // prescaler for setting Temp timer, 72Khz
#define TEMP_TIMER_FREQUENCY    1000 // temperature interrupt frequency

#define STEPPER_TIMER_PRESCALE HAL_stepper_timer_prescale()
#define STEPPER_TIMER_RATE     HAL_stepper_timer_rate()
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs

//#define PULSE_TIMER_RATE       STEPPER_TIMER_RATE   // frequency of pulse timer
#define PULSE_TIMER_PRESCALE   STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US STEPPER_TIMER_TICKS_PER_US

#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt(STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_disable_interrupt(STEP_TIMER_NUM)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(STEP_TIMER_NUM)

#define ENABLE_TEMPERATURE_INTERRUPT() HAL_timer_enable_interrupt(TEMP_TIMER_NUM)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt(TEMP_TIMER_NUM)

#define HAL_STEP_TIMER_ISR() void TC5_Handler(stm32_timer_t htim)
#define HAL_TEMP_TIMER_ISR() void TC7_Handler(stm32_timer_t htim)


// ------------------------
// Types
// ------------------------
typedef HardwareTimer* stm32_timer_t;

// ------------------------
// Public Variables
// ------------------------

extern stm32_timer_t TimerHandle[];

// ------------------------
// Public functions
// ------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);
void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);
bool HAL_timer_interrupt_enabled(const uint8_t timer_num);

FORCE_INLINE static uint32_t HAL_timer_get_count(const uint8_t timer_num) {
  return TimerHandle[timer_num]->getCount(TICK_FORMAT);
}

FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timer_num, const uint32_t compare) {
  TimerHandle[timer_num]->setOverflow(compare + 1, TICK_FORMAT);
}

FORCE_INLINE static hal_timer_t HAL_timer_get_compare(const uint8_t timer_num) {
  return TimerHandle[timer_num]->getOverflow(TICK_FORMAT);
}

FORCE_INLINE static uint32_t HAL_stepper_timer_rate() {
  return TimerHandle[STEP_TIMER_NUM]->getTimerClkFreq()
    / TimerHandle[STEP_TIMER_NUM]->getPrescaleFactor();
}

FORCE_INLINE static uint32_t HAL_stepper_timer_prescale() {
  return TimerHandle[STEP_TIMER_NUM]->getPrescaleFactor();
}

#define HAL_timer_isr_prologue(TIMER_NUM)
#define HAL_timer_isr_epilogue(TIMER_NUM)
