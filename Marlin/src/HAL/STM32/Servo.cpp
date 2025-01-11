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
#include "../platforms.h"

#ifdef HAL_STM32

#include "../../inc/MarlinConfig.h"

#if HAS_SERVOS

#include "Servo.h"

#if ENABLED(CAN_HOST)
  #include "../shared/CAN_host.h"
#endif

static uint_fast8_t servoCount = 0;
static libServo *servos[NUM_SERVOS] = {0};
constexpr millis_t servoDelay[] = SERVO_DELAY;
static_assert(COUNT(servoDelay) == NUM_SERVOS, "SERVO_DELAY must be an array NUM_SERVOS long.");

// Initialize to the default timer priority. This will be overridden by a call from timers.cpp.
// This allows all timer interrupt priorities to be managed from a single location in the HAL.
static uint32_t servo_interrupt_priority = NVIC_EncodePriority(NVIC_GetPriorityGrouping(), TIM_IRQ_PRIO, TIM_IRQ_SUBPRIO);

// This must be called after the STM32 Servo class has initialized the timer.
// It may only be needed after the first call to attach(), but it is possible
// that is is necessary after every detach() call. To be safe this is currently
// called after every call to attach().
static void fixServoTimerInterruptPriority() {
  NVIC_SetPriority(getTimerUpIrq(TIMER_SERVO), servo_interrupt_priority);
}

libServo::libServo()
: delay(servoDelay[servoCount]),
  was_attached_before_pause(false),
  value_before_pause(0)
{
  servos[servoCount++] = this;
}

int8_t libServo::attach(const int pin) {
  if (servoCount >= MAX_SERVOS) return -1;
  if (pin > 0) servo_pin = pin;
  auto result = stm32_servo.attach(servo_pin);
  fixServoTimerInterruptPriority();
  return result;
}

int8_t libServo::attach(const int pin, const int min, const int max) {
  if (servoCount >= MAX_SERVOS) return -1;
  if (pin > 0) servo_pin = pin;
  auto result = stm32_servo.attach(servo_pin, min, max);
  fixServoTimerInterruptPriority();
  return result;
}

void libServo::move(const int value) {

  #if ENABLED(CAN_HOST) // Forward direct Servo command to head
    constexpr int angles[2] = Z_SERVO_ANGLES;
    // Translate M280 S10 to M401, M280 S90 to M402
    if (value == angles[0])
      CAN_host_send_gcode_2params('M', 401, 0, 0, 0, 0); // Deploy Angle: Send "M401" instead, enables interrupt etc.
    else if (value == angles[1])
      CAN_host_send_gcode_2params('M', 402, 0, 0, 0, 0); // Stow Angle: Send "M402" instead, enables interrupt etc.
    else
      CAN_host_send_gcode_2params('M', 280, 'S', value, 'P', 0); // M280 S[value] P0
  #endif

  if (attach(0) >= 0) {
    stm32_servo.write(value);
    safe_delay(delay);
    TERN_(DEACTIVATE_SERVOS_AFTER_MOVE, detach());
  }
}

void libServo::pause() {
  was_attached_before_pause = stm32_servo.attached();
  if (was_attached_before_pause) {
    value_before_pause = stm32_servo.read();
    stm32_servo.detach();
  }
}

void libServo::resume() {
  if (was_attached_before_pause) {
    attach();
    move(value_before_pause);
  }
}

void libServo::pause_all_servos() {
  for (auto& servo : servos)
    if (servo) servo->pause();
}

void libServo::resume_all_servos() {
  for (auto& servo : servos)
    if (servo) servo->resume();
}

void libServo::setInterruptPriority(uint32_t preemptPriority, uint32_t subPriority) {
  servo_interrupt_priority = NVIC_EncodePriority(NVIC_GetPriorityGrouping(), preemptPriority, subPriority);
}

#endif // HAS_SERVOS
#endif // HAL_STM32
