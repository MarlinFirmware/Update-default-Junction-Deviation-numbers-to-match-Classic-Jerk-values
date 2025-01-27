/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once
#include <stdint.h>
#include <Timer0.h>
#include "sysclock.h"

//
// Timer Types
//
typedef Timer0 *timer_channel_t;
typedef uint16_t hal_timer_t;
#define HAL_TIMER_TYPE_MAX 0xFFFF

//
// Timer instances
//
extern Timer0 temp_timer;
extern Timer0 step_timer;

//
// Timer Configurations
//

/**
 * HAL_TIMER_RATE must be known at compile time since it's used to calculate
 * STEPPER_TIMER_RATE, which is used in 'constexpr' calculations.
 * On the HC32F460 the timer rate depends on PCLK1, which is derived from the
 * system clock configured at runtime.
 * Thus we use the 'F_PCLK1' constant defined in 'sysclock.h'.
 *
 * See https://github.com/MarlinFirmware/Marlin/pull/27099 for more information.
 *
 * NOTE: If the 'constexpr' requirement is ever lifted, TIMER0_BASE_FREQUENCY could
 * be used instead. Tho this would probably not make any noticable difference.
 */
#define HAL_TIMER_RATE F_PCLK1

// Temperature timer
#define TEMP_TIMER_NUM        (&temp_timer)
#define TEMP_TIMER_PRIORITY   DDL_IRQ_PRIORITY_02
#define TEMP_TIMER_PRESCALE     16UL          // 12.5MHz
#define TEMP_TIMER_RATE       1000            // 1kHz
#define TEMP_TIMER_FREQUENCY  TEMP_TIMER_RATE // 1kHz also

// Stepper timer
#define STEP_TIMER_NUM         (&step_timer)
#define STEP_TIMER_PRIORITY    DDL_IRQ_PRIORITY_00  // Top priority, nothing else uses it
#define STEPPER_TIMER_PRESCALE  16UL                // 12.5MHz

#define STEPPER_TIMER_RATE (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)  // 50MHz / 16 = 3.125MHz
#define STEPPER_TIMER_TICKS_PER_US (STEPPER_TIMER_RATE / 1000000UL)   // Integer 3

// Pulse timer (== stepper timer)
#define PULSE_TIMER_NUM           STEP_TIMER_NUM
#define PULSE_TIMER_PRESCALE      STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US  STEPPER_TIMER_TICKS_PER_US

//
// Channel aliases
//
#define MF_TIMER_TEMP   TEMP_TIMER_NUM
#define MF_TIMER_STEP   STEP_TIMER_NUM
#define MF_TIMER_PULSE  PULSE_TIMER_NUM

//
// HAL functions
//
void HAL_timer_start(const timer_channel_t timer_num, const uint32_t frequency);

// Inlined since they are somewhat critical
#define MARLIN_HAL_TIMER_INLINE_ATTR __attribute__((always_inline)) inline

MARLIN_HAL_TIMER_INLINE_ATTR void HAL_timer_enable_interrupt(const timer_channel_t timer_num) {
  timer_num->resume();
}

MARLIN_HAL_TIMER_INLINE_ATTR void HAL_timer_disable_interrupt(const timer_channel_t timer_num) {
  timer_num->pause();
}

MARLIN_HAL_TIMER_INLINE_ATTR bool HAL_timer_interrupt_enabled(const timer_channel_t timer_num) {
  return timer_num->isPaused();
}

MARLIN_HAL_TIMER_INLINE_ATTR void HAL_timer_set_compare(const timer_channel_t timer_num, const hal_timer_t compare) {
  timer_num->setCompareValue(compare);
}

MARLIN_HAL_TIMER_INLINE_ATTR hal_timer_t HAL_timer_get_count(const timer_channel_t timer_num) {
  return timer_num->getCount();
}

MARLIN_HAL_TIMER_INLINE_ATTR void HAL_timer_isr_prologue(const timer_channel_t timer_num) {
  timer_num->clearInterruptFlag();
}

MARLIN_HAL_TIMER_INLINE_ATTR void HAL_timer_isr_epilogue(const timer_channel_t timer_num) {}

//
// HAL function aliases
//
#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt(STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_disable_interrupt(STEP_TIMER_NUM)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(STEP_TIMER_NUM)

#define ENABLE_TEMPERATURE_INTERRUPT() HAL_timer_enable_interrupt(TEMP_TIMER_NUM)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt(TEMP_TIMER_NUM);

//
// HAL ISR callbacks
//
void Step_Handler();
void Temp_Handler();

#ifndef HAL_STEP_TIMER_ISR
#define HAL_STEP_TIMER_ISR() void Step_Handler()
#endif
#ifndef HAL_TEMP_TIMER_ISR
#define HAL_TEMP_TIMER_ISR() void Temp_Handler()
#endif
