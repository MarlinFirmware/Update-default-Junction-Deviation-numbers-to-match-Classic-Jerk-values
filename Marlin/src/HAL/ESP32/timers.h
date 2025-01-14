/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
#pragma once

#include <stdint.h>
#include <driver/timer.h>

// ------------------------
// Defines
// ------------------------
#define FORCE_INLINE __attribute__((always_inline)) inline

typedef uint64_t hal_timer_t;
#define HAL_TIMER_TYPE_MAX 0xFFFFFFFFFFFFFFFFULL

#ifndef MF_TIMER_STEP
  #define MF_TIMER_STEP         0  // Timer Index for Stepper
#endif
#ifndef MF_TIMER_PULSE
  #define MF_TIMER_PULSE        MF_TIMER_STEP
#endif
#ifndef MF_TIMER_TEMP
  #define MF_TIMER_TEMP         1  // Timer Index for Temperature
#endif
#ifndef MF_TIMER_PWM
  #define MF_TIMER_PWM          2  // index of timer to use for PWM outputs
#endif
#ifndef MF_TIMER_TONE
  #define MF_TIMER_TONE         3  // index of timer for beeper tones
#endif

#define HAL_TIMER_RATE APB_CLK_FREQ // frequency of timer peripherals

#if ENABLED(I2S_STEPPER_STREAM)
  #define STEPPER_TIMER_PRESCALE     1
  #define STEPPER_TIMER_RATE         250000                           // 250khz, 4µs pulses of i2s word clock
#else
  #define STEPPER_TIMER_PRESCALE     40
  #define STEPPER_TIMER_RATE         ((HAL_TIMER_RATE) / (STEPPER_TIMER_PRESCALE)) // frequency of stepper timer, 2MHz
#endif
#define STEPPER_TIMER_TICKS_PER_US   ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs

#define STEP_TIMER_MIN_INTERVAL   8 // minimum time in µs between stepper interrupts

#define TONE_TIMER_PRESCALE    1000 // Arbitrary value, no idea what i'm doing here

#define TEMP_TIMER_PRESCALE    1000 // prescaler for setting Temp timer, 72Khz
#define TEMP_TIMER_FREQUENCY   1000 // temperature interrupt frequency

#define PWM_TIMER_PRESCALE       10
#if ENABLED(FAST_PWM_FAN)
  #define PWM_TIMER_FREQUENCY  FAST_PWM_FAN_FREQUENCY
#else
  #define PWM_TIMER_FREQUENCY  (50*128) // 50Hz and 7bit resolution
#endif
#define MAX_PWM_PINS             32 // Number of PWM pin-slots

#define PULSE_TIMER_RATE         STEPPER_TIMER_RATE   // frequency of pulse timer
#define PULSE_TIMER_PRESCALE     STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US STEPPER_TIMER_TICKS_PER_US

#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt(MF_TIMER_STEP)
#define DISABLE_STEPPER_DRIVER_INTERRUPT()  HAL_timer_disable_interrupt(MF_TIMER_STEP)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(MF_TIMER_STEP)

#define ENABLE_TEMPERATURE_INTERRUPT()  HAL_timer_enable_interrupt(MF_TIMER_TEMP)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt(MF_TIMER_TEMP)

#ifndef HAL_TEMP_TIMER_ISR
  #define HAL_TEMP_TIMER_ISR() extern "C" void tempTC_Handler()
#endif
#ifndef HAL_STEP_TIMER_ISR
  #define HAL_STEP_TIMER_ISR() extern "C" void stepTC_Handler()
#endif
#ifndef HAL_PWM_TIMER_ISR
  #define HAL_PWM_TIMER_ISR() extern "C" void pwmTC_Handler()
#endif
#ifndef HAL_TONE_TIMER_ISR
  #define HAL_TONE_TIMER_ISR() extern "C" void toneTC_Handler()
#endif

extern "C" {
  void tempTC_Handler();
  void stepTC_Handler();
  void pwmTC_Handler();
  void toneTC_Handler();
}

// ------------------------
// Types
// ------------------------

typedef struct {
  timer_group_t  group;
  timer_idx_t    idx;
  uint32_t       divider;
  void           (*fn)();
} tTimerConfig;

// ------------------------
// Public Variables
// ------------------------

extern const tTimerConfig timer_config[];

// ------------------------
// Public functions
// ------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);
void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t count);
hal_timer_t HAL_timer_get_compare(const uint8_t timer_num);
hal_timer_t HAL_timer_get_count(const uint8_t timer_num);

void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);
bool HAL_timer_interrupt_enabled(const uint8_t timer_num);

#define HAL_timer_isr_prologue(T) NOOP
#define HAL_timer_isr_epilogue(T) NOOP
