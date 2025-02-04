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

/**
 * This module is stripped down version of the LPC1768_PWM.h file from
 * PR #7500. It is hardwired for the PRINTRBOARD_G2 Motor Current needs.
 */

#include "../../../inc/MarlinConfigPre.h"
#include "../../../module/stepper.h"
//C:\Users\bobku\Documents\GitHub\Marlin-Bob-2\Marlin\src\module\stepper.h
//C:\Users\bobku\Documents\GitHub\Marlin-Bob-2\Marlin\src\HAL\HAL_DUE\G2_PWM.h

#define PWM_PERIOD_US  100  // base repetition rate in micro seconds

typedef struct {       // holds the data needed by the ISR to control the Vref pin
  volatile uint32_t* set_register;
  volatile uint32_t* clr_register;
  uint32_t write_mask;
} PWM_map;

#define G2_VREF(I) (uint32_t)(I * 5 * 0.15)   // desired Vref * 1000 (scaled so don't loose accuracy in next step)

#define G2_VREF_COUNT(Q) (uint32_t)map(constrain(Q, 500, 3.3 * 1000), 0, 3.3 * 1000, 0, PWM_PERIOD_US)  // under 500  the results are very non-linear

extern volatile uint32_t *SODR_A, *SODR_B, *CODR_A, *CODR_B;

#define _PIN(IO) (DIO ## IO ## _PIN)

#define PWM_MAP_INIT_ROW(IO,ZZ) { ZZ == 'A' ? SODR_A : SODR_B,  ZZ == 'A' ? CODR_A : CODR_B, 1 << _PIN(IO) }

#define PWM_MAP_INIT {  PWM_MAP_INIT_ROW(MOTOR_CURRENT_PWM_X_PIN, 'B'), \
                        PWM_MAP_INIT_ROW(MOTOR_CURRENT_PWM_Y_PIN, 'B'), \
                        PWM_MAP_INIT_ROW(MOTOR_CURRENT_PWM_Z_PIN, 'B'), \
                        PWM_MAP_INIT_ROW(MOTOR_CURRENT_PWM_E_PIN, 'A'), \
                     };

#define NUM_PWMS 4

extern PWM_map ISR_table[NUM_PWMS];

extern uint32_t motor_current_setting[3];

#define IR_BIT(p) (WITHIN(p, 0, 3) ? (p) : (p) + 4)
#define COPY_ACTIVE_TABLE() do{ for (uint_fast8_t i = 0; i < 6; ++i) work_table[i] = active_table[i]; }while(0)

#define PWM_MR0 19999         // base repetition rate minus one count - 20mS
#define PWM_PR 24             // prescaler value - prescaler divide by 24 + 1  -  1 MHz output
#define PWM_PCLKSEL0 0x00     // select clock source for prescaler - defaults to 25MHz on power up
                              // 0: 25MHz, 1: 100MHz, 2: 50MHz, 3: 12.5MHZ to PWM1 prescaler
#define MR0_MARGIN 200        // if channel value too close to MR0 the system locks up

extern bool PWM_table_swap;   // flag to tell the ISR that the tables have been swapped

#define HAL_G2_PWM_ISR  void PWM_Handler()

extern volatile uint32_t PWM_ISR1_STATUS, PWM_ISR2_STATUS;
