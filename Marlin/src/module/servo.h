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
 * module/servo.h
 */

#include "../inc/MarlinConfig.h"
#include "../HAL/shared/servo.h"

#if HAS_SERVO_ANGLES

  #if ENABLED(SWITCHING_EXTRUDER)
    // Switching extruder can have 2 or 4 angles
    #if EXTRUDERS > 3
      #define REQ_ANGLES 4
    #else
      #define REQ_ANGLES 2
    #endif
    constexpr uint16_t sase[] = SWITCHING_EXTRUDER_SERVO_ANGLES;
    static_assert(COUNT(sase) == REQ_ANGLES, "SWITCHING_EXTRUDER_SERVO_ANGLES needs " STRINGIFY(REQ_ANGLES) " angles.");
  #else
    constexpr uint16_t sase[4] = { 0 };
  #endif

  #if ENABLED(SWITCHING_NOZZLE)
    #if SWITCHING_NOZZLE_TWO_SERVOS
      constexpr uint16_t sasn[][2] = SWITCHING_NOZZLE_SERVO_ANGLES;
      static_assert(COUNT(sasn) == 2, "SWITCHING_NOZZLE_SERVO_ANGLES (with SWITCHING_NOZZLE_E1_SERVO_NR) needs 2 sets of angles: { { lower, raise }, { lower, raise } }.");
    #else
      constexpr uint16_t sasn[] = SWITCHING_NOZZLE_SERVO_ANGLES;
      static_assert(COUNT(sasn) == 2, "SWITCHING_NOZZLE_SERVO_ANGLES needs two angles: { E0, E1 }.");
    #endif
  #else
    constexpr uint16_t sasn[2] = { 0 };
  #endif

  #if HAS_Z_SERVO_PROBE
    #if ENABLED(BLTOUCH)
      #include "../feature/bltouch.h"
      #undef Z_SERVO_ANGLES
      #define Z_SERVO_ANGLES { BLTOUCH_DEPLOY, BLTOUCH_STOW }
    #endif
    constexpr uint16_t sazp[] = Z_SERVO_ANGLES;
    static_assert(COUNT(sazp) == 2, "Z_SERVO_ANGLES needs 2 angles.");
  #else
    constexpr uint16_t sazp[2] = { 0 };
  #endif

  #ifndef SWITCHING_EXTRUDER_SERVO_NR
    #define SWITCHING_EXTRUDER_SERVO_NR -1
  #endif
  #ifndef SWITCHING_EXTRUDER_E23_SERVO_NR
    #define SWITCHING_EXTRUDER_E23_SERVO_NR -1
  #endif
  #ifndef SWITCHING_NOZZLE_SERVO_NR
    #define SWITCHING_NOZZLE_SERVO_NR -1
  #endif
  #ifndef SWITCHING_NOZZLE_E1_SERVO_NR
    #define SWITCHING_NOZZLE_E1_SERVO_NR -1
  #endif
  #ifndef Z_PROBE_SERVO_NR
    #define Z_PROBE_SERVO_NR -1
  #endif

  #define SASN(J,I) TERN(SWITCHING_NOZZLE_TWO_SERVOS, sasn[J][I], sasn[I])

  #define ASRC(N,I) ( \
      N == SWITCHING_EXTRUDER_SERVO_NR     ? sase[I]   \
    : N == SWITCHING_EXTRUDER_E23_SERVO_NR ? sase[I+2] \
    : N == SWITCHING_NOZZLE_SERVO_NR       ? SASN(0,I) \
    : N == SWITCHING_NOZZLE_E1_SERVO_NR    ? SASN(1,I) \
    : N == Z_PROBE_SERVO_NR                ? sazp[I]   \
    : 0                                                )

  #if ENABLED(EDITABLE_SERVO_ANGLES)
    extern uint16_t servo_angles[NUM_SERVOS][2];
    #define CONST_SERVO_ANGLES base_servo_angles
  #else
    #define CONST_SERVO_ANGLES servo_angles
  #endif

  #define _ASRC_PAIR(N) { ASRC(N,0), ASRC(N,1) },
  constexpr uint16_t CONST_SERVO_ANGLES [NUM_SERVOS][2] = { REPEAT(NUM_SERVOS, _ASRC_PAIR) };

  #if HAS_Z_SERVO_PROBE
    #define DEPLOY_Z_SERVO() servo[Z_PROBE_SERVO_NR].move(servo_angles[Z_PROBE_SERVO_NR][0])
    #define STOW_Z_SERVO() servo[Z_PROBE_SERVO_NR].move(servo_angles[Z_PROBE_SERVO_NR][1])
  #endif

#endif // HAS_SERVO_ANGLES

extern hal_servo_t servo[NUM_SERVOS];
void servo_init();
