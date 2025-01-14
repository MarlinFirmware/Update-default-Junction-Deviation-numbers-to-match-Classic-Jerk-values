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
#pragma once

#include <Servo.h>

#if 1

#include "../../core/millis_t.h"

// Inherit and expand on the official library
class libServo {
  public:
    libServo();
    int8_t attach(const int pin = 0); // pin == 0 uses value from previous call
    int8_t attach(const int pin, const int min, const int max);
    void detach() { pico_servo.detach(); }
    int read() { return pico_servo.read(); }
    void move(const int value);

    void pause();
    void resume();

    static void pause_all_servos();
    static void resume_all_servos();
    static void setInterruptPriority(uint32_t preemptPriority, uint32_t subPriority);

  private:
    Servo pico_servo;

    int servo_pin = 0;
    millis_t delay = 0;

    bool was_attached_before_pause;
    int value_before_pause;
};

#else

class libServo: public Servo {
  public:
  void move(const int value) {
    constexpr uint16_t servo_delay[] = SERVO_DELAY;
    static_assert(COUNT(servo_delay) == NUM_SERVOS, "SERVO_DELAY must be an array NUM_SERVOS long.");

    if (attach(servo_info[servoIndex].Pin.nbr) >= 0) {    // try to reattach
      write(value);
      safe_delay(servo_delay[servoIndex]); // delay to allow servo to reach position
      TERN_(DEACTIVATE_SERVOS_AFTER_MOVE, detach());
    }

  }
};

class libServo;
typedef libServo hal_servo_t;

#endif
