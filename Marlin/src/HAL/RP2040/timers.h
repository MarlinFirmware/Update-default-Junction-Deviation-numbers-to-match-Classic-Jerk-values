/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#include <stdint.h>

#include "../../core/macros.h"

#ifdef PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
  #undef PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
  #define PICO_TIME_DEFAULT_ALARM_POOL_DISABLED 0
#else
  #define PICO_TIME_DEFAULT_ALARM_POOL_DISABLED 0
#endif

// ------------------------
// Defines
// ------------------------

//#define _HAL_TIMER(T) _CAT(LPC_TIM, T)
//#define _HAL_TIMER_IRQ(T) TIMER##T##_IRQn
//#define __HAL_TIMER_ISR(T) extern "C" alarm_callback_t HAL_timer_alarm_pool_##T##_callback()
#define __HAL_TIMER_ISR(T) extern void HAL_timer_##T##_callback()
#define _HAL_TIMER_ISR(T)  __HAL_TIMER_ISR(T)

typedef uint64_t hal_timer_t;
#define HAL_TIMER_TYPE_MAX 0xFFFFFFFFFFFFFFFF

#define HAL_TIMER_RATE         (1000000ull)  // fixed value as we use a microsecond timesource
#ifndef MF_TIMER_STEP
  #define MF_TIMER_STEP        0  // Timer Index for Stepper
#endif
#ifndef MF_TIMER_PULSE
  #define MF_TIMER_PULSE       MF_TIMER_STEP
#endif
#ifndef MF_TIMER_TEMP
  #define MF_TIMER_TEMP        1  // Timer Index for Temperature
#endif
#ifndef MF_TIMER_PWM
  #define MF_TIMER_PWM         3  // Timer Index for PWM
#endif

#define TEMP_TIMER_RATE        HAL_TIMER_RATE
#define TEMP_TIMER_FREQUENCY   1000 // temperature interrupt frequency

#define STEPPER_TIMER_RATE     HAL_TIMER_RATE / 10 // 100khz roughly
#define STEPPER_TIMER_TICKS_PER_US (0.1) // fixed value as we use a microsecond timesource
#define STEPPER_TIMER_PRESCALE (10)

#define PULSE_TIMER_RATE       STEPPER_TIMER_RATE   // frequency of pulse timer
#define PULSE_TIMER_PRESCALE   STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US STEPPER_TIMER_TICKS_PER_US

#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt(MF_TIMER_STEP)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_disable_interrupt(MF_TIMER_STEP)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(MF_TIMER_STEP)

#define ENABLE_TEMPERATURE_INTERRUPT() HAL_timer_enable_interrupt(MF_TIMER_TEMP)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt(MF_TIMER_TEMP)

#ifndef HAL_STEP_TIMER_ISR
  #define HAL_STEP_TIMER_ISR() _HAL_TIMER_ISR(MF_TIMER_STEP)
#endif
#ifndef HAL_TEMP_TIMER_ISR
  #define HAL_TEMP_TIMER_ISR() _HAL_TIMER_ISR(MF_TIMER_TEMP)
#endif

// Timer references by index
//#define STEP_TIMER_PTR _HAL_TIMER(MF_TIMER_STEP)
//#define TEMP_TIMER_PTR _HAL_TIMER(MF_TIMER_TEMP)

extern alarm_pool_t* HAL_timer_pool_0;
extern alarm_pool_t* HAL_timer_pool_1;
extern alarm_pool_t* HAL_timer_pool_2;
extern alarm_pool_t* HAL_timer_pool_3;

extern struct repeating_timer HAL_timer_0;

void HAL_timer_0_callback();
void HAL_timer_1_callback();
void HAL_timer_2_callback();
void HAL_timer_3_callback();

int64_t HAL_timer_alarm_pool_0_callback(long int, void*);
int64_t HAL_timer_alarm_pool_1_callback(long int, void*);
int64_t HAL_timer_alarm_pool_2_callback(long int, void*);
int64_t HAL_timer_alarm_pool_3_callback(long int, void*);

bool HAL_timer_repeating_0_callback(repeating_timer* timer);
bool HAL_timer_repeating_1_callback(repeating_timer* timer);
bool HAL_timer_repeating_2_callback(repeating_timer* timer);
bool HAL_timer_repeating_3_callback(repeating_timer* timer);

extern volatile bool HAL_timer_irq_en[4];

// ------------------------
// Public functions
// ------------------------

void HAL_timer_init();
void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);
void HAL_timer_stop(const uint8_t timer_num);

FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timer_num, hal_timer_t compare) {

  if (timer_num == MF_TIMER_STEP){
    if (compare == HAL_TIMER_TYPE_MAX){
       HAL_timer_stop(timer_num);
       return;
    }
  }

 compare = compare *10; //Dirty fix, figure out a proper way

  switch (timer_num) {
    case 0:
      alarm_pool_add_alarm_in_us(HAL_timer_pool_0 ,compare , HAL_timer_alarm_pool_0_callback ,0 ,false );
      break;

    case 1:
      alarm_pool_add_alarm_in_us(HAL_timer_pool_1 ,compare , HAL_timer_alarm_pool_1_callback ,0 ,false );
      break;

    case 2:
      alarm_pool_add_alarm_in_us(HAL_timer_pool_2 ,compare , HAL_timer_alarm_pool_2_callback ,0 ,false );
      break;

    case 3:
      alarm_pool_add_alarm_in_us(HAL_timer_pool_3 ,compare , HAL_timer_alarm_pool_3_callback ,0 ,false );
      break;
  }
}

FORCE_INLINE static hal_timer_t HAL_timer_get_compare(const uint8_t timer_num) {
  return 0;
}

FORCE_INLINE static hal_timer_t HAL_timer_get_count(const uint8_t timer_num) {
  if (timer_num == MF_TIMER_STEP) return 0ull;
  return time_us_64();
}


FORCE_INLINE static void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  HAL_timer_irq_en[timer_num] = 1;
}

FORCE_INLINE static void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  HAL_timer_irq_en[timer_num] = 0;
}

FORCE_INLINE static bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  return HAL_timer_irq_en[timer_num]; //lucky coincidence that timer_num and rp2040 irq num matches
}

FORCE_INLINE static void HAL_timer_isr_prologue(const uint8_t timer_num) {
  return;
}

#define HAL_timer_isr_epilogue(T) NOOP
