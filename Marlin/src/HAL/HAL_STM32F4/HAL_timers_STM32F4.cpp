/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
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

#ifdef STM32F4xx

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "HAL.h"

#include "HAL_timers_STM32F4.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#define NUM_HARDWARE_TIMERS 2

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

static stimer_t timerConfig[NUM_HARDWARE_TIMERS];

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

static bool timersInitialised[NUM_HARDWARE_TIMERS] = {false};

#define MAX_RELOAD 0xffff

static void HAL_TimerInit(stimer_t* dev, uint32_t microseconds) {
  uint32_t cycles_per_us = getTimerClkFreq(dev->timer) / 1000000;
  uint32_t period_cyc = (microseconds * cycles_per_us);
  uint16_t prescaler = (uint16_t)(period_cyc / MAX_RELOAD + 1);
  uint16_t overflow = (uint16_t)((period_cyc + (prescaler / 2)) / prescaler);
  
  TimerHandleInit(dev, overflow, prescaler);
}

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  if (!timersInitialised[timer_num]) {
    switch (timer_num) {
    case STEP_TIMER_NUM:
      timerConfig[0].timer = TIM5;
      attachIntHandle(&timerConfig[0], STEP_Timer_Handler);
      HAL_NVIC_SetPriority((IRQn_Type)getTimerIrq(TIM5), 1, 0);
      HAL_TimerInit(&timerConfig[0], 1000000 / frequency);
      break;
    case TEMP_TIMER_NUM:
      timerConfig[1].timer = TIM7;
      attachIntHandle(&timerConfig[1], TEMP_Timer_Handler);
      HAL_NVIC_SetPriority((IRQn_Type)getTimerIrq(TIM7), 2, 0);
      HAL_TimerInit(&timerConfig[1], 1000000 / frequency);
      break;
    }
    timersInitialised[timer_num] = true;
  }
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
  case STEP_TIMER_NUM:
    attachIntHandle(&timerConfig[0], STEP_Timer_Handler);
    break;
  case TEMP_TIMER_NUM:
    attachIntHandle(&timerConfig[1], TEMP_Timer_Handler);
    break;
  }
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  attachIntHandle(&timerConfig[timer_num], NULL);
}

void HAL_timer_set_compare(const uint8_t timer_num, const uint32_t compare) {
  __HAL_TIM_SET_AUTORELOAD(&timerConfig[timer_num].handle, compare);
}

hal_timer_t HAL_timer_get_compare(const uint8_t timer_num) {
  __HAL_TIM_GET_AUTORELOAD(&timerConfig[timer_num].handle);
}

uint32_t HAL_timer_get_count(const uint8_t timer_num) {
  return getTimerCounter(&timerConfig[timer_num]);
}

void HAL_timer_restrain(const uint8_t timer_num, const uint16_t interval_ticks) {
  const hal_timer_t mincmp = HAL_timer_get_count(timer_num) + interval_ticks;
  if (HAL_timer_get_compare(timer_num) < mincmp) {
    HAL_timer_set_compare(timer_num, mincmp);
  }
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  return &timerConfig[timer_num].irqHandle != NULL;
}

#endif // STM32F4xx
