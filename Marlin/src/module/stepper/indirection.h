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
 * stepper/indirection.h - Stepper Indirection Macros
 *
 * Each axis in a machine may have between 1 and 4 stepper motors.
 * Currently X and Y allow for 1 or 2 steppers. Z can have up to 4.
 * Extruders usually have one E stepper per nozzle.
 *
 * XYZ Special Cases
 *  - Delta:  3 steppers contribute to X, Y, and Z.
 *  - SCARA:  A and B steppers contribute to X and Y by angular transformation.
 *  - CoreXY: A and B steppers contribute to X and Y in combination.
 *  - CoreXZ: A and B steppers contribute to X and Z in combination.
 *  - CoreYZ: A and B steppers contribute to Y and Z in combination.
 *
 * E Special Cases
 *  - SINGLENOZZLE: All Extruders have a single nozzle so there is one heater and no XYZ offset.
 *  - Switching Extruder: One stepper is used for each pair of nozzles with a switching mechanism.
 *  - Duplication Mode: Two or more steppers move in sync when `extruder_duplication_enabled` is set.
 *                      With MULTI_NOZZLE_DUPLICATION a `duplication_e_mask` is also used.
 *  - Průša MMU1: One stepper is used with a switching mechanism. Odd numbered E indexes are reversed.
 *  - Průša MMU2: One stepper is used with a switching mechanism.
 *  - E_DUAL_STEPPER_DRIVERS: Two steppers always move in sync, possibly with opposite DIR states.
 *
 * Direct Stepper Control
 *  Where "Q" represents X Y Z I J K U V W / X2 Y2 Z2 Z3 Z4 / E0 E1 E2 E3 E4 E5 E6 E7
 *  Here each E index corresponds to a single E stepper driver.
 *
 *    Q_ENABLE_INIT()   Q_ENABLE_WRITE(S)   Q_ENABLE_READ()
 *    Q_DIR_INIT()      Q_DIR_WRITE(S)      Q_DIR_READ()
 *    Q_STEP_INIT()     Q_STEP_WRITE(S)     Q_STEP_READ()
 *
 *  Steppers may not have an enable state or may be enabled by other methods
 *  beyond a single pin (SOFTWARE_DRIVER_ENABLE) so these can be overriden:
 *    ENABLE_STEPPER_Q() DISABLE_STEPPER_Q()
 *
 * Axis Stepper Control (X Y Z I J K U V W)
 *  SOFTWARE_DRIVER_ENABLE gives all axes a status flag, so these macros will
 *  skip sending commands to steppers that are already in the desired state:
 *   ENABLE_AXIS_Q()  DISABLE_AXIS_Q()
 *
 * E-Axis Stepper Control (0..n)
 *  For these macros the E index indicates a logical extruder (e.g., active_extruder).
 *
 *    E_STEP_WRITE(E,V)  FWD_E_DIR(E)  REV_E_DIR(E)
 *
 */
#include "../../inc/MarlinConfig.h"

#if HAS_TRINAMIC_CONFIG
  #include "trinamic.h"
#endif

void restore_stepper_drivers();  // Called by powerManager.power_on()
void reset_stepper_drivers();    // Called by settings.load / settings.reset

#define INVERT_DIR(AXIS, D) (TERN_(INVERT_## AXIS ##_DIR, !)(D))

// X Stepper
#if HAS_X_AXIS
  #ifndef X_ENABLE_INIT
    #define X_ENABLE_INIT() SET_OUTPUT(X_ENABLE_PIN)
    #define X_ENABLE_WRITE(STATE) WRITE(X_ENABLE_PIN,STATE)
    #define X_ENABLE_READ() bool(READ(X_ENABLE_PIN))
  #endif
  #ifndef X_DIR_INIT
    #define X_DIR_INIT() SET_OUTPUT(X_DIR_PIN)
    #define X_DIR_WRITE(STATE) WRITE(X_DIR_PIN,INVERT_DIR(X, STATE))
    #define X_DIR_READ() INVERT_DIR(X, bool(READ(X_DIR_PIN)))
  #endif
  #define X_STEP_INIT() SET_OUTPUT(X_STEP_PIN)
  #ifndef X_STEP_WRITE
    #define X_STEP_WRITE(STATE) WRITE(X_STEP_PIN,STATE)
  #endif
  #define X_STEP_READ() bool(READ(X_STEP_PIN))
#endif

// Y Stepper
#if HAS_Y_AXIS
  #ifndef Y_ENABLE_INIT
    #define Y_ENABLE_INIT() SET_OUTPUT(Y_ENABLE_PIN)
    #define Y_ENABLE_WRITE(STATE) WRITE(Y_ENABLE_PIN,STATE)
    #define Y_ENABLE_READ() bool(READ(Y_ENABLE_PIN))
  #endif
  #ifndef Y_DIR_INIT
    #define Y_DIR_INIT() SET_OUTPUT(Y_DIR_PIN)
    #define Y_DIR_WRITE(STATE) WRITE(Y_DIR_PIN,INVERT_DIR(Y, STATE))
    #define Y_DIR_READ() INVERT_DIR(Y, bool(READ(Y_DIR_PIN)))
  #endif
  #define Y_STEP_INIT() SET_OUTPUT(Y_STEP_PIN)
  #ifndef Y_STEP_WRITE
    #define Y_STEP_WRITE(STATE) WRITE(Y_STEP_PIN,STATE)
  #endif
  #define Y_STEP_READ() bool(READ(Y_STEP_PIN))
#endif

// Z Stepper
#if HAS_Z_AXIS
  #ifndef Z_ENABLE_INIT
    #define Z_ENABLE_INIT() SET_OUTPUT(Z_ENABLE_PIN)
    #define Z_ENABLE_WRITE(STATE) WRITE(Z_ENABLE_PIN,STATE)
    #define Z_ENABLE_READ() bool(READ(Z_ENABLE_PIN))
  #endif
  #ifndef Z_DIR_INIT
    #define Z_DIR_INIT() SET_OUTPUT(Z_DIR_PIN)
    #define Z_DIR_WRITE(STATE) WRITE(Z_DIR_PIN,INVERT_DIR(Z, STATE))
    #define Z_DIR_READ() INVERT_DIR(Z, bool(READ(Z_DIR_PIN)))
  #endif
  #define Z_STEP_INIT() SET_OUTPUT(Z_STEP_PIN)
  #ifndef Z_STEP_WRITE
    #define Z_STEP_WRITE(STATE) WRITE(Z_STEP_PIN,STATE)
  #endif
  #define Z_STEP_READ() bool(READ(Z_STEP_PIN))
#endif

// X2 Stepper
#if HAS_X2_ENABLE
  #ifndef X2_ENABLE_INIT
    #define X2_ENABLE_INIT() SET_OUTPUT(X2_ENABLE_PIN)
    #define X2_ENABLE_WRITE(STATE) WRITE(X2_ENABLE_PIN,STATE)
    #define X2_ENABLE_READ() bool(READ(X2_ENABLE_PIN))
  #endif
  #ifndef X2_DIR_INIT
    #define X2_DIR_INIT() SET_OUTPUT(X2_DIR_PIN)
    #define X2_DIR_WRITE(STATE) WRITE(X2_DIR_PIN,INVERT_DIR(X2, STATE))
    #define X2_DIR_READ() INVERT_DIR(X2, bool(READ(X2_DIR_PIN)))
  #endif
  #define X2_STEP_INIT() SET_OUTPUT(X2_STEP_PIN)
  #ifndef X2_STEP_WRITE
    #define X2_STEP_WRITE(STATE) WRITE(X2_STEP_PIN,STATE)
  #endif
  #define X2_STEP_READ() bool(READ(X2_STEP_PIN))
#endif

// Y2 Stepper
#if HAS_Y2_ENABLE
  #ifndef Y2_ENABLE_INIT
    #define Y2_ENABLE_INIT() SET_OUTPUT(Y2_ENABLE_PIN)
    #define Y2_ENABLE_WRITE(STATE) WRITE(Y2_ENABLE_PIN,STATE)
    #define Y2_ENABLE_READ() bool(READ(Y2_ENABLE_PIN))
  #endif
  #ifndef Y2_DIR_INIT
    #define Y2_DIR_INIT() SET_OUTPUT(Y2_DIR_PIN)
    #define Y2_DIR_WRITE(STATE) WRITE(Y2_DIR_PIN,INVERT_DIR(Y2, STATE))
    #define Y2_DIR_READ() INVERT_DIR(Y2, bool(READ(Y2_DIR_PIN)))
  #endif
  #define Y2_STEP_INIT() SET_OUTPUT(Y2_STEP_PIN)
  #ifndef Y2_STEP_WRITE
    #define Y2_STEP_WRITE(STATE) WRITE(Y2_STEP_PIN,STATE)
  #endif
  #define Y2_STEP_READ() bool(READ(Y2_STEP_PIN))
#else
  #define Y2_DIR_WRITE(STATE) NOOP
#endif

// Z2 Stepper
#if HAS_Z2_ENABLE
  #ifndef Z2_ENABLE_INIT
    #define Z2_ENABLE_INIT() SET_OUTPUT(Z2_ENABLE_PIN)
    #define Z2_ENABLE_WRITE(STATE) WRITE(Z2_ENABLE_PIN,STATE)
    #define Z2_ENABLE_READ() bool(READ(Z2_ENABLE_PIN))
  #endif
  #ifndef Z2_DIR_INIT
    #define Z2_DIR_INIT() SET_OUTPUT(Z2_DIR_PIN)
    #define Z2_DIR_WRITE(STATE) WRITE(Z2_DIR_PIN,INVERT_DIR(Z2, STATE))
    #define Z2_DIR_READ() INVERT_DIR(Z2, bool(READ(Z2_DIR_PIN)))
  #endif
  #define Z2_STEP_INIT() SET_OUTPUT(Z2_STEP_PIN)
  #ifndef Z2_STEP_WRITE
    #define Z2_STEP_WRITE(STATE) WRITE(Z2_STEP_PIN,STATE)
  #endif
  #define Z2_STEP_READ() bool(READ(Z2_STEP_PIN))
#else
  #define Z2_DIR_WRITE(STATE) NOOP
#endif

// Z3 Stepper
#if HAS_Z3_ENABLE
  #ifndef Z3_ENABLE_INIT
    #define Z3_ENABLE_INIT() SET_OUTPUT(Z3_ENABLE_PIN)
    #define Z3_ENABLE_WRITE(STATE) WRITE(Z3_ENABLE_PIN,STATE)
    #define Z3_ENABLE_READ() bool(READ(Z3_ENABLE_PIN))
  #endif
  #ifndef Z3_DIR_INIT
    #define Z3_DIR_INIT() SET_OUTPUT(Z3_DIR_PIN)
    #define Z3_DIR_WRITE(STATE) WRITE(Z3_DIR_PIN,INVERT_DIR(Z3, STATE))
    #define Z3_DIR_READ() INVERT_DIR(Z3, bool(READ(Z3_DIR_PIN)))
  #endif
  #define Z3_STEP_INIT() SET_OUTPUT(Z3_STEP_PIN)
  #ifndef Z3_STEP_WRITE
    #define Z3_STEP_WRITE(STATE) WRITE(Z3_STEP_PIN,STATE)
  #endif
  #define Z3_STEP_READ() bool(READ(Z3_STEP_PIN))
#else
  #define Z3_DIR_WRITE(STATE) NOOP
#endif

// Z4 Stepper
#if HAS_Z4_ENABLE
  #ifndef Z4_ENABLE_INIT
    #define Z4_ENABLE_INIT() SET_OUTPUT(Z4_ENABLE_PIN)
    #define Z4_ENABLE_WRITE(STATE) WRITE(Z4_ENABLE_PIN,STATE)
    #define Z4_ENABLE_READ() bool(READ(Z4_ENABLE_PIN))
  #endif
  #ifndef Z4_DIR_INIT
    #define Z4_DIR_INIT() SET_OUTPUT(Z4_DIR_PIN)
    #define Z4_DIR_WRITE(STATE) WRITE(Z4_DIR_PIN,INVERT_DIR(Z4, STATE))
    #define Z4_DIR_READ() INVERT_DIR(Z4, bool(READ(Z4_DIR_PIN)))
  #endif
  #define Z4_STEP_INIT() SET_OUTPUT(Z4_STEP_PIN)
  #ifndef Z4_STEP_WRITE
    #define Z4_STEP_WRITE(STATE) WRITE(Z4_STEP_PIN,STATE)
  #endif
  #define Z4_STEP_READ() bool(READ(Z4_STEP_PIN))
#else
  #define Z4_DIR_WRITE(STATE) NOOP
#endif

// I Stepper
#if HAS_I_AXIS
  #ifndef I_ENABLE_INIT
    #define I_ENABLE_INIT() SET_OUTPUT(I_ENABLE_PIN)
    #define I_ENABLE_WRITE(STATE) WRITE(I_ENABLE_PIN,STATE)
    #define I_ENABLE_READ() bool(READ(I_ENABLE_PIN))
  #endif
  #ifndef I_DIR_INIT
    #define I_DIR_INIT() SET_OUTPUT(I_DIR_PIN)
    #define I_DIR_WRITE(STATE) WRITE(I_DIR_PIN,INVERT_DIR(I, STATE))
    #define I_DIR_READ() INVERT_DIR(I, bool(READ(I_DIR_PIN)))
  #endif
  #define I_STEP_INIT() SET_OUTPUT(I_STEP_PIN)
  #ifndef I_STEP_WRITE
    #define I_STEP_WRITE(STATE) WRITE(I_STEP_PIN,STATE)
  #endif
  #define I_STEP_READ() bool(READ(I_STEP_PIN))
#endif

// J Stepper
#if HAS_J_AXIS
  #ifndef J_ENABLE_INIT
    #define J_ENABLE_INIT() SET_OUTPUT(J_ENABLE_PIN)
    #define J_ENABLE_WRITE(STATE) WRITE(J_ENABLE_PIN,STATE)
    #define J_ENABLE_READ() bool(READ(J_ENABLE_PIN))
  #endif
  #ifndef J_DIR_INIT
    #define J_DIR_INIT() SET_OUTPUT(J_DIR_PIN)
    #define J_DIR_WRITE(STATE) WRITE(J_DIR_PIN,INVERT_DIR(J, STATE))
    #define J_DIR_READ() INVERT_DIR(J, bool(READ(J_DIR_PIN)))
  #endif
  #define J_STEP_INIT() SET_OUTPUT(J_STEP_PIN)
  #ifndef J_STEP_WRITE
    #define J_STEP_WRITE(STATE) WRITE(J_STEP_PIN,STATE)
  #endif
  #define J_STEP_READ() bool(READ(J_STEP_PIN))
#endif

// K Stepper
#if HAS_K_AXIS
  #ifndef K_ENABLE_INIT
    #define K_ENABLE_INIT() SET_OUTPUT(K_ENABLE_PIN)
    #define K_ENABLE_WRITE(STATE) WRITE(K_ENABLE_PIN,STATE)
    #define K_ENABLE_READ() bool(READ(K_ENABLE_PIN))
  #endif
  #ifndef K_DIR_INIT
    #define K_DIR_INIT() SET_OUTPUT(K_DIR_PIN)
    #define K_DIR_WRITE(STATE) WRITE(K_DIR_PIN,INVERT_DIR(K, STATE))
    #define K_DIR_READ() INVERT_DIR(K, bool(READ(K_DIR_PIN)))
  #endif
  #define K_STEP_INIT() SET_OUTPUT(K_STEP_PIN)
  #ifndef K_STEP_WRITE
    #define K_STEP_WRITE(STATE) WRITE(K_STEP_PIN,STATE)
  #endif
  #define K_STEP_READ() bool(READ(K_STEP_PIN))
#endif

// U Stepper
#if HAS_U_AXIS
  #ifndef U_ENABLE_INIT
    #define U_ENABLE_INIT() SET_OUTPUT(U_ENABLE_PIN)
    #define U_ENABLE_WRITE(STATE) WRITE(U_ENABLE_PIN,STATE)
    #define U_ENABLE_READ() bool(READ(U_ENABLE_PIN))
  #endif
  #ifndef U_DIR_INIT
    #define U_DIR_INIT() SET_OUTPUT(U_DIR_PIN)
    #define U_DIR_WRITE(STATE) WRITE(U_DIR_PIN,INVERT_DIR(U, STATE))
    #define U_DIR_READ() INVERT_DIR(U, bool(READ(U_DIR_PIN)))
  #endif
  #define U_STEP_INIT() SET_OUTPUT(U_STEP_PIN)
  #ifndef U_STEP_WRITE
    #define U_STEP_WRITE(STATE) WRITE(U_STEP_PIN,STATE)
  #endif
  #define U_STEP_READ() bool(READ(U_STEP_PIN))
#endif

// V Stepper
#if HAS_V_AXIS
  #ifndef V_ENABLE_INIT
    #define V_ENABLE_INIT() SET_OUTPUT(V_ENABLE_PIN)
    #define V_ENABLE_WRITE(STATE) WRITE(V_ENABLE_PIN,STATE)
    #define V_ENABLE_READ() bool(READ(V_ENABLE_PIN))
  #endif
  #ifndef V_DIR_INIT
    #define V_DIR_INIT() SET_OUTPUT(V_DIR_PIN)
    #define V_DIR_WRITE(STATE) WRITE(V_DIR_PIN,INVERT_DIR(V, STATE))
    #define V_DIR_READ() INVERT_DIR(V, bool(READ(V_DIR_PIN)))
  #endif
  #define V_STEP_INIT() SET_OUTPUT(V_STEP_PIN)
  #ifndef V_STEP_WRITE
    #define V_STEP_WRITE(STATE) WRITE(V_STEP_PIN,STATE)
  #endif
  #define V_STEP_READ() bool(READ(V_STEP_PIN))
#endif

// W Stepper
#if HAS_W_AXIS
  #ifndef W_ENABLE_INIT
    #define W_ENABLE_INIT() SET_OUTPUT(W_ENABLE_PIN)
    #define W_ENABLE_WRITE(STATE) WRITE(W_ENABLE_PIN,STATE)
    #define W_ENABLE_READ() bool(READ(W_ENABLE_PIN))
  #endif
  #ifndef W_DIR_INIT
    #define W_DIR_INIT() SET_OUTPUT(W_DIR_PIN)
    #define W_DIR_WRITE(STATE) WRITE(W_DIR_PIN,INVERT_DIR(W, STATE))
    #define W_DIR_READ() INVERT_DIR(W, bool(READ(W_DIR_PIN)))
  #endif
  #define W_STEP_INIT() SET_OUTPUT(W_STEP_PIN)
  #ifndef W_STEP_WRITE
    #define W_STEP_WRITE(STATE) WRITE(W_STEP_PIN,STATE)
  #endif
  #define W_STEP_READ() bool(READ(W_STEP_PIN))
#endif

// E0 Stepper
#ifndef E0_ENABLE_INIT
  #define E0_ENABLE_INIT() SET_OUTPUT(E0_ENABLE_PIN)
  #define E0_ENABLE_WRITE(STATE) WRITE(E0_ENABLE_PIN,STATE)
  #define E0_ENABLE_READ() bool(READ(E0_ENABLE_PIN))
#endif
#ifndef E0_DIR_INIT
  #define E0_DIR_INIT() SET_OUTPUT(E0_DIR_PIN)
  #define E0_DIR_WRITE(STATE) WRITE(E0_DIR_PIN,INVERT_DIR(E0, STATE))
  #define E0_DIR_READ() INVERT_DIR(E0, bool(READ(E0_DIR_PIN)))
#endif
#define E0_STEP_INIT() SET_OUTPUT(E0_STEP_PIN)
#ifndef E0_STEP_WRITE
  #define E0_STEP_WRITE(STATE) WRITE(E0_STEP_PIN,STATE)
#endif
#define E0_STEP_READ() bool(READ(E0_STEP_PIN))

// E1 Stepper
#ifndef E1_ENABLE_INIT
  #define E1_ENABLE_INIT() SET_OUTPUT(E1_ENABLE_PIN)
  #define E1_ENABLE_WRITE(STATE) WRITE(E1_ENABLE_PIN,STATE)
  #define E1_ENABLE_READ() bool(READ(E1_ENABLE_PIN))
#endif
#ifndef E1_DIR_INIT
  #define E1_DIR_INIT() SET_OUTPUT(E1_DIR_PIN)
  #define E1_DIR_WRITE(STATE) WRITE(E1_DIR_PIN,INVERT_DIR(E1, STATE))
  #define E1_DIR_READ() INVERT_DIR(E1, bool(READ(E1_DIR_PIN)))
#endif
#define E1_STEP_INIT() SET_OUTPUT(E1_STEP_PIN)
#ifndef E1_STEP_WRITE
  #define E1_STEP_WRITE(STATE) WRITE(E1_STEP_PIN,STATE)
#endif
#define E1_STEP_READ() bool(READ(E1_STEP_PIN))

// E2 Stepper
#ifndef E2_ENABLE_INIT
  #define E2_ENABLE_INIT() SET_OUTPUT(E2_ENABLE_PIN)
  #define E2_ENABLE_WRITE(STATE) WRITE(E2_ENABLE_PIN,STATE)
  #define E2_ENABLE_READ() bool(READ(E2_ENABLE_PIN))
#endif
#ifndef E2_DIR_INIT
  #define E2_DIR_INIT() SET_OUTPUT(E2_DIR_PIN)
  #define E2_DIR_WRITE(STATE) WRITE(E2_DIR_PIN,INVERT_DIR(E2, STATE))
  #define E2_DIR_READ() INVERT_DIR(E2, bool(READ(E2_DIR_PIN)))
#endif
#define E2_STEP_INIT() SET_OUTPUT(E2_STEP_PIN)
#ifndef E2_STEP_WRITE
  #define E2_STEP_WRITE(STATE) WRITE(E2_STEP_PIN,STATE)
#endif
#define E2_STEP_READ() bool(READ(E2_STEP_PIN))

// E3 Stepper
#ifndef E3_ENABLE_INIT
  #define E3_ENABLE_INIT() SET_OUTPUT(E3_ENABLE_PIN)
  #define E3_ENABLE_WRITE(STATE) WRITE(E3_ENABLE_PIN,STATE)
  #define E3_ENABLE_READ() bool(READ(E3_ENABLE_PIN))
#endif
#ifndef E3_DIR_INIT
  #define E3_DIR_INIT() SET_OUTPUT(E3_DIR_PIN)
  #define E3_DIR_WRITE(STATE) WRITE(E3_DIR_PIN,INVERT_DIR(E3, STATE))
  #define E3_DIR_READ() INVERT_DIR(E3, bool(READ(E3_DIR_PIN)))
#endif
#define E3_STEP_INIT() SET_OUTPUT(E3_STEP_PIN)
#ifndef E3_STEP_WRITE
  #define E3_STEP_WRITE(STATE) WRITE(E3_STEP_PIN,STATE)
#endif
#define E3_STEP_READ() bool(READ(E3_STEP_PIN))

// E4 Stepper
#ifndef E4_ENABLE_INIT
  #define E4_ENABLE_INIT() SET_OUTPUT(E4_ENABLE_PIN)
  #define E4_ENABLE_WRITE(STATE) WRITE(E4_ENABLE_PIN,STATE)
  #define E4_ENABLE_READ() bool(READ(E4_ENABLE_PIN))
#endif
#ifndef E4_DIR_INIT
  #define E4_DIR_INIT() SET_OUTPUT(E4_DIR_PIN)
  #define E4_DIR_WRITE(STATE) WRITE(E4_DIR_PIN,INVERT_DIR(E4, STATE))
  #define E4_DIR_READ() INVERT_DIR(E4, bool(READ(E4_DIR_PIN)))
#endif
#define E4_STEP_INIT() SET_OUTPUT(E4_STEP_PIN)
#ifndef E4_STEP_WRITE
  #define E4_STEP_WRITE(STATE) WRITE(E4_STEP_PIN,STATE)
#endif
#define E4_STEP_READ() bool(READ(E4_STEP_PIN))

// E5 Stepper
#ifndef E5_ENABLE_INIT
  #define E5_ENABLE_INIT() SET_OUTPUT(E5_ENABLE_PIN)
  #define E5_ENABLE_WRITE(STATE) WRITE(E5_ENABLE_PIN,STATE)
  #define E5_ENABLE_READ() bool(READ(E5_ENABLE_PIN))
#endif
#ifndef E5_DIR_INIT
  #define E5_DIR_INIT() SET_OUTPUT(E5_DIR_PIN)
  #define E5_DIR_WRITE(STATE) WRITE(E5_DIR_PIN,INVERT_DIR(E5, STATE))
  #define E5_DIR_READ() INVERT_DIR(E5, bool(READ(E5_DIR_PIN)))
#endif
#define E5_STEP_INIT() SET_OUTPUT(E5_STEP_PIN)
#ifndef E5_STEP_WRITE
  #define E5_STEP_WRITE(STATE) WRITE(E5_STEP_PIN,STATE)
#endif
#define E5_STEP_READ() bool(READ(E5_STEP_PIN))

// E6 Stepper
#ifndef E6_ENABLE_INIT
  #define E6_ENABLE_INIT() SET_OUTPUT(E6_ENABLE_PIN)
  #define E6_ENABLE_WRITE(STATE) WRITE(E6_ENABLE_PIN,STATE)
  #define E6_ENABLE_READ() bool(READ(E6_ENABLE_PIN))
#endif
#ifndef E6_DIR_INIT
  #define E6_DIR_INIT() SET_OUTPUT(E6_DIR_PIN)
  #define E6_DIR_WRITE(STATE) WRITE(E6_DIR_PIN,INVERT_DIR(E6, STATE))
  #define E6_DIR_READ() INVERT_DIR(E6, bool(READ(E6_DIR_PIN)))
#endif
#define E6_STEP_INIT() SET_OUTPUT(E6_STEP_PIN)
#ifndef E6_STEP_WRITE
  #define E6_STEP_WRITE(STATE) WRITE(E6_STEP_PIN,STATE)
#endif
#define E6_STEP_READ() bool(READ(E6_STEP_PIN))

// E7 Stepper
#ifndef E7_ENABLE_INIT
  #define E7_ENABLE_INIT() SET_OUTPUT(E7_ENABLE_PIN)
  #define E7_ENABLE_WRITE(STATE) WRITE(E7_ENABLE_PIN,STATE)
  #define E7_ENABLE_READ() bool(READ(E7_ENABLE_PIN))
#endif
#ifndef E7_DIR_INIT
  #define E7_DIR_INIT() SET_OUTPUT(E7_DIR_PIN)
  #define E7_DIR_WRITE(STATE) WRITE(E7_DIR_PIN,INVERT_DIR(E7, STATE))
  #define E7_DIR_READ() INVERT_DIR(E7, bool(READ(E7_DIR_PIN)))
#endif
#define E7_STEP_INIT() SET_OUTPUT(E7_STEP_PIN)
#ifndef E7_STEP_WRITE
  #define E7_STEP_WRITE(STATE) WRITE(E7_STEP_PIN,STATE)
#endif
#define E7_STEP_READ() bool(READ(E7_STEP_PIN))

/**
 * Extruder indirection for the single E axis
 */
#if HAS_SWITCHING_EXTRUDER // One stepper driver per two extruders, reversed on odd index
  #if EXTRUDERS > 7
    #define E_STEP_WRITE(E,V) do{ if (E < 2) { E0_STEP_WRITE(V); } else if (E < 4) { E1_STEP_WRITE(V); } else if (E < 6) { E2_STEP_WRITE(V); } else { E3_STEP_WRITE(V); } }while(0)
    #define    FWD_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(HIGH); break; case 1: E0_DIR_WRITE(LOW ); break; \
        case 2: E1_DIR_WRITE(HIGH); break; case 3: E1_DIR_WRITE(LOW ); break; \
        case 4: E2_DIR_WRITE(HIGH); break; case 5: E2_DIR_WRITE(LOW ); break; \
        case 6: E3_DIR_WRITE(HIGH); break; case 7: E3_DIR_WRITE(LOW ); break; \
      } }while(0)
    #define    REV_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(LOW ); break; case 1: E0_DIR_WRITE(HIGH); break; \
        case 2: E1_DIR_WRITE(LOW ); break; case 3: E1_DIR_WRITE(HIGH); break; \
        case 4: E2_DIR_WRITE(LOW ); break; case 5: E2_DIR_WRITE(HIGH); break; \
        case 6: E3_DIR_WRITE(LOW ); break; case 7: E3_DIR_WRITE(HIGH); break; \
      } }while(0)
  #elif EXTRUDERS > 6
    #define E_STEP_WRITE(E,V) do{ if (E < 2) { E0_STEP_WRITE(V); } else if (E < 4) { E1_STEP_WRITE(V); } else if (E < 6) { E2_STEP_WRITE(V); } else { E3_STEP_WRITE(V); } }while(0)
    #define    FWD_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(HIGH); break; case 1: E0_DIR_WRITE(LOW ); break; \
        case 2: E1_DIR_WRITE(HIGH); break; case 3: E1_DIR_WRITE(LOW ); break; \
        case 4: E2_DIR_WRITE(HIGH); break; case 5: E2_DIR_WRITE(LOW ); break; \
        case 6: E3_DIR_WRITE(HIGH); break; \
      } }while(0)
    #define    REV_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(LOW ); break; case 1: E0_DIR_WRITE(HIGH); break; \
        case 2: E1_DIR_WRITE(LOW ); break; case 3: E1_DIR_WRITE(HIGH); break; \
        case 4: E2_DIR_WRITE(LOW ); break; case 5: E2_DIR_WRITE(HIGH); break; \
        case 6: E3_DIR_WRITE(LOW ); } }while(0)
  #elif EXTRUDERS > 5
    #define E_STEP_WRITE(E,V) do{ if (E < 2) { E0_STEP_WRITE(V); } else if (E < 4) { E1_STEP_WRITE(V); } else { E2_STEP_WRITE(V); } }while(0)
    #define    FWD_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(HIGH); break; case 1: E0_DIR_WRITE(LOW ); break; \
        case 2: E1_DIR_WRITE(HIGH); break; case 3: E1_DIR_WRITE(LOW ); break; \
        case 4: E2_DIR_WRITE(HIGH); break; case 5: E2_DIR_WRITE(LOW ); break; \
      } }while(0)
    #define    REV_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(LOW ); break; case 1: E0_DIR_WRITE(HIGH); break; \
        case 2: E1_DIR_WRITE(LOW ); break; case 3: E1_DIR_WRITE(HIGH); break; \
        case 4: E2_DIR_WRITE(LOW ); break; case 5: E2_DIR_WRITE(HIGH); break; \
      } }while(0)
  #elif EXTRUDERS > 4
    #define E_STEP_WRITE(E,V) do{ if (E < 2) { E0_STEP_WRITE(V); } else if (E < 4) { E1_STEP_WRITE(V); } else { E2_STEP_WRITE(V); } }while(0)
    #define    FWD_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(HIGH); break; case 1: E0_DIR_WRITE(LOW ); break; \
        case 2: E1_DIR_WRITE(HIGH); break; case 3: E1_DIR_WRITE(LOW ); break; \
        case 4: E2_DIR_WRITE(HIGH); break; \
      } }while(0)
    #define    REV_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(LOW ); break; case 1: E0_DIR_WRITE(HIGH); break; \
        case 2: E1_DIR_WRITE(LOW ); break; case 3: E1_DIR_WRITE(HIGH); break; \
        case 4: E2_DIR_WRITE(LOW ); break; \
      } }while(0)
  #elif EXTRUDERS > 3
    #define E_STEP_WRITE(E,V) do{ if (E < 2) { E0_STEP_WRITE(V); } else { E1_STEP_WRITE(V); } }while(0)
    #define    FWD_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(HIGH); break; case 1: E0_DIR_WRITE(LOW ); break; \
        case 2: E1_DIR_WRITE(HIGH); break; case 3: E1_DIR_WRITE(LOW ); break; \
      } }while(0)
    #define    REV_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(LOW ); break; case 1: E0_DIR_WRITE(HIGH); break; \
        case 2: E1_DIR_WRITE(LOW ); break; case 3: E1_DIR_WRITE(HIGH); break; \
      } }while(0)
  #elif EXTRUDERS > 2
    #define E_STEP_WRITE(E,V) do{ if (E < 2) { E0_STEP_WRITE(V); } else { E1_STEP_WRITE(V); } }while(0)
    #define    FWD_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(HIGH); break; case 1: E0_DIR_WRITE(LOW ); break; \
        case 2: E1_DIR_WRITE(HIGH); break; \
      } }while(0)
    #define    REV_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(LOW ); break; case 1: E0_DIR_WRITE(HIGH); break; \
        case 2: E1_DIR_WRITE(LOW ); break; \
      } }while(0)
  #else
    #define E_STEP_WRITE(E,V) E0_STEP_WRITE(V)
    #define    FWD_E_DIR(E)   do{ E0_DIR_WRITE((E) ? LOW  : HIGH); }while(0)
    #define    REV_E_DIR(E)   do{ E0_DIR_WRITE((E) ? HIGH : LOW ); }while(0)
  #endif

  #define TOOL_ESTEPPER(T) ((T) >> 1)

#elif HAS_PRUSA_MMU2 || HAS_PRUSA_MMU3 // One multiplexed stepper driver

  #define E_STEP_WRITE(E,V) E0_STEP_WRITE(V)
  #define    FWD_E_DIR(E)   E0_DIR_WRITE(HIGH)
  #define    REV_E_DIR(E)   E0_DIR_WRITE(LOW )

#elif HAS_PRUSA_MMU1  // One multiplexed stepper driver, reversed on odd index

  #define E_STEP_WRITE(E,V) E0_STEP_WRITE(V)
  #define    FWD_E_DIR(E)   do{ E0_DIR_WRITE(TEST(E, 0) ? HIGH : LOW ); }while(0)
  #define    REV_E_DIR(E)   do{ E0_DIR_WRITE(TEST(E, 0) ? LOW  : HIGH); }while(0)

#elif E_STEPPERS > 1

  #if E_STEPPERS > 7

    #define _E_STEP_WRITE(E,V) do{ switch (E) { \
        case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); break; case 3: E3_STEP_WRITE(V); break; \
        case 4: E4_STEP_WRITE(V); break; case 5: E5_STEP_WRITE(V); break; case 6: E6_STEP_WRITE(V); break; case 7: E7_STEP_WRITE(V); break; \
      } }while(0)
    #define    _FWD_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(HIGH); break; case 1: E1_DIR_WRITE(HIGH); break; \
        case 2: E2_DIR_WRITE(HIGH); break; case 3: E3_DIR_WRITE(HIGH); break; \
        case 4: E4_DIR_WRITE(HIGH); break; case 5: E5_DIR_WRITE(HIGH); break; \
        case 6: E6_DIR_WRITE(HIGH); break; case 7: E7_DIR_WRITE(HIGH); break; \
      } }while(0)
    #define    _REV_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(LOW ); break; case 1: E1_DIR_WRITE(LOW ); break; \
        case 2: E2_DIR_WRITE(LOW ); break; case 3: E3_DIR_WRITE(LOW ); break; \
        case 4: E4_DIR_WRITE(LOW ); break; case 5: E5_DIR_WRITE(LOW ); break; \
        case 6: E6_DIR_WRITE(LOW ); break; case 7: E7_DIR_WRITE(LOW ); break; \
      } }while(0)

  #elif E_STEPPERS > 6

    #define _E_STEP_WRITE(E,V) do{ switch (E) { \
        case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); break; case 3: E3_STEP_WRITE(V); break; \
        case 4: E4_STEP_WRITE(V); break; case 5: E5_STEP_WRITE(V); break; case 6: E6_STEP_WRITE(V); break; \
      } }while(0)
    #define    _FWD_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(HIGH); break; case 1: E1_DIR_WRITE(HIGH); break; \
        case 2: E2_DIR_WRITE(HIGH); break; case 3: E3_DIR_WRITE(HIGH); break; \
        case 4: E4_DIR_WRITE(HIGH); break; case 5: E5_DIR_WRITE(HIGH); break; \
        case 6: E6_DIR_WRITE(HIGH); break; \
      } }while(0)
    #define    _REV_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(LOW ); break; case 1: E1_DIR_WRITE(LOW ); break; \
        case 2: E2_DIR_WRITE(LOW ); break; case 3: E3_DIR_WRITE(LOW ); break; \
        case 4: E4_DIR_WRITE(LOW ); break; case 5: E5_DIR_WRITE(LOW ); break; \
        case 6: E6_DIR_WRITE(LOW ); break; \
      } }while(0)

  #elif E_STEPPERS > 5

    #define _E_STEP_WRITE(E,V) do{ switch (E) { \
        case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); break; case 3: E3_STEP_WRITE(V); break; \
        case 4: E4_STEP_WRITE(V); break; case 5: E5_STEP_WRITE(V); break; \
      } }while(0)
    #define    _FWD_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(HIGH); break; case 1: E1_DIR_WRITE(HIGH); break; \
        case 2: E2_DIR_WRITE(HIGH); break; case 3: E3_DIR_WRITE(HIGH); break; \
        case 4: E4_DIR_WRITE(HIGH); break; case 5: E5_DIR_WRITE(HIGH); break; \
      } }while(0)
    #define    _REV_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(LOW ); break; case 1: E1_DIR_WRITE(LOW ); break; \
        case 2: E2_DIR_WRITE(LOW ); break; case 3: E3_DIR_WRITE(LOW ); break; \
        case 4: E4_DIR_WRITE(LOW ); break; case 5: E5_DIR_WRITE(LOW ); break; \
      } }while(0)

  #elif E_STEPPERS > 4

    #define _E_STEP_WRITE(E,V) do{ switch (E) { \
        case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); break; case 3: E3_STEP_WRITE(V); break; \
        case 4: E4_STEP_WRITE(V); break; \
      } }while(0)
    #define    _FWD_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(HIGH); break; case 1: E1_DIR_WRITE(HIGH); break; \
        case 2: E2_DIR_WRITE(HIGH); break; case 3: E3_DIR_WRITE(HIGH); break; \
        case 4: E4_DIR_WRITE(HIGH); break; \
      } }while(0)
    #define    _REV_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(LOW ); break; case 1: E1_DIR_WRITE(LOW ); break; \
        case 2: E2_DIR_WRITE(LOW ); break; case 3: E3_DIR_WRITE(LOW ); break; \
        case 4: E4_DIR_WRITE(LOW ); break; \
      } }while(0)

  #elif E_STEPPERS > 3

    #define _E_STEP_WRITE(E,V) do{ switch (E) { \
        case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); break; case 3: E3_STEP_WRITE(V); break; \
      } }while(0)
    #define    _FWD_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(HIGH); break; case 1: E1_DIR_WRITE(HIGH); break; \
        case 2: E2_DIR_WRITE(HIGH); break; case 3: E3_DIR_WRITE(HIGH); break; \
      } }while(0)
    #define    _REV_E_DIR(E)   do{ switch (E) { \
        case 0: E0_DIR_WRITE(LOW ); break; case 1: E1_DIR_WRITE(LOW ); break; \
        case 2: E2_DIR_WRITE(LOW ); break; case 3: E3_DIR_WRITE(LOW ); break; \
      } }while(0)

  #elif E_STEPPERS > 2

    #define _E_STEP_WRITE(E,V) do{ switch (E) { case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); } }while(0)
    #define    _FWD_E_DIR(E)   do{ switch (E) { case 0: E0_DIR_WRITE(HIGH); break; case 1: E1_DIR_WRITE(HIGH); break; case 2: E2_DIR_WRITE(HIGH); } }while(0)
    #define    _REV_E_DIR(E)   do{ switch (E) { case 0: E0_DIR_WRITE(LOW ); break; case 1: E1_DIR_WRITE(LOW ); break; case 2: E2_DIR_WRITE(LOW ); } }while(0)

  #else

    #define _E_STEP_WRITE(E,V) do{ if (E == 0) { E0_STEP_WRITE(V); } else { E1_STEP_WRITE(V); } }while(0)
    #define    _FWD_E_DIR(E)   do{ if (E == 0) { E0_DIR_WRITE(HIGH); } else { E1_DIR_WRITE(HIGH); } }while(0)
    #define    _REV_E_DIR(E)   do{ if (E == 0) { E0_DIR_WRITE(LOW ); } else { E1_DIR_WRITE(LOW ); } }while(0)
  #endif

  #if HAS_DUPLICATION_MODE

    #if ENABLED(MULTI_NOZZLE_DUPLICATION)
      #define DUPE(N,T,V) do{ if (TEST(duplication_e_mask, N)) E##N##_##T##_WRITE(V); }while(0);
    #else
      #define DUPE(N,T,V) E##N##_##T##_WRITE(V);
    #endif

    #define NDIR(N) DUPE(N,DIR,HIGH);
    #define RDIR(N) DUPE(N,DIR,LOW );

    #define E_STEP_WRITE(E,V) do{ if (extruder_duplication_enabled) { REPEAT2(E_STEPPERS, DUPE, STEP, V); } else _E_STEP_WRITE(E,V); }while(0)
    #define  FWD_E_DIR(E)     do{ if (extruder_duplication_enabled) { REPEAT(E_STEPPERS, NDIR); } else _FWD_E_DIR(E); }while(0)
    #define  REV_E_DIR(E)     do{ if (extruder_duplication_enabled) { REPEAT(E_STEPPERS, RDIR); } else _REV_E_DIR(E); }while(0)

  #else

    #define E_STEP_WRITE(E,V) _E_STEP_WRITE(E,V)
    #define    FWD_E_DIR(E)   _FWD_E_DIR(E)
    #define    REV_E_DIR(E)   _REV_E_DIR(E)

  #endif

#elif ENABLED(E_DUAL_STEPPER_DRIVERS)
  #define E_STEP_WRITE(E,V) do{ E0_STEP_WRITE(V); E1_STEP_WRITE(V); }while(0)
  #define    FWD_E_DIR(E)   do{ E0_DIR_WRITE(HIGH); E1_DIR_WRITE(INVERT_DIR(E1_VS_E0, HIGH)); }while(0)
  #define    REV_E_DIR(E)   do{ E0_DIR_WRITE(LOW ); E1_DIR_WRITE(INVERT_DIR(E1_VS_E0, LOW )); }while(0)

#elif E_STEPPERS == 1
  #define E_STEP_WRITE(E,V) E0_STEP_WRITE(V)
  #define    FWD_E_DIR(E)   E0_DIR_WRITE(HIGH)
  #define    REV_E_DIR(E)   E0_DIR_WRITE(LOW )

#else
  #define E_STEP_WRITE(E,V) NOOP
  #define    FWD_E_DIR(E)   NOOP
  #define    REV_E_DIR(E)   NOOP

#endif

#ifndef TOOL_ESTEPPER
  #define TOOL_ESTEPPER(T) (T)
#endif

//
// Individual stepper enable / disable macros
//

#ifndef ENABLE_STEPPER_X
  #define  ENABLE_STEPPER_X() TERN(HAS_X_ENABLE, X_ENABLE_WRITE( X_ENABLE_ON), NOOP)
#endif
#ifndef DISABLE_STEPPER_X
  #define DISABLE_STEPPER_X() TERN(HAS_X_ENABLE, X_ENABLE_WRITE(!X_ENABLE_ON), NOOP)
#endif

#ifndef ENABLE_STEPPER_X2
  #define  ENABLE_STEPPER_X2() TERN(HAS_X2_ENABLE, X2_ENABLE_WRITE( X_ENABLE_ON), NOOP)
#endif
#ifndef DISABLE_STEPPER_X2
  #define DISABLE_STEPPER_X2() TERN(HAS_X2_ENABLE, X2_ENABLE_WRITE(!X_ENABLE_ON), NOOP)
#endif

#ifndef ENABLE_STEPPER_Y
  #define  ENABLE_STEPPER_Y() TERN(HAS_Y_ENABLE, Y_ENABLE_WRITE( Y_ENABLE_ON), NOOP)
#endif
#ifndef DISABLE_STEPPER_Y
  #define DISABLE_STEPPER_Y() TERN(HAS_Y_ENABLE, Y_ENABLE_WRITE(!Y_ENABLE_ON), NOOP)
#endif

#ifndef ENABLE_STEPPER_Y2
  #define  ENABLE_STEPPER_Y2() TERN(HAS_Y2_ENABLE, Y2_ENABLE_WRITE( Y_ENABLE_ON), NOOP)
#endif
#ifndef DISABLE_STEPPER_Y2
  #define DISABLE_STEPPER_Y2() TERN(HAS_Y2_ENABLE, Y2_ENABLE_WRITE(!Y_ENABLE_ON), NOOP)
#endif

#ifndef ENABLE_STEPPER_Z
  #define  ENABLE_STEPPER_Z() TERN(HAS_Z_ENABLE, Z_ENABLE_WRITE( Z_ENABLE_ON), NOOP)
#endif
#ifndef DISABLE_STEPPER_Z
  #define DISABLE_STEPPER_Z() TERN(HAS_Z_ENABLE, Z_ENABLE_WRITE(!Z_ENABLE_ON), NOOP)
#endif

#ifndef ENABLE_STEPPER_Z2
  #define  ENABLE_STEPPER_Z2() TERN(HAS_Z2_ENABLE, Z2_ENABLE_WRITE( Z_ENABLE_ON), NOOP)
#endif
#ifndef DISABLE_STEPPER_Z2
  #define DISABLE_STEPPER_Z2() TERN(HAS_Z2_ENABLE, Z2_ENABLE_WRITE(!Z_ENABLE_ON), NOOP)
#endif

#ifndef ENABLE_STEPPER_Z3
  #define  ENABLE_STEPPER_Z3() TERN(HAS_Z3_ENABLE, Z3_ENABLE_WRITE( Z_ENABLE_ON), NOOP)
#endif
#ifndef DISABLE_STEPPER_Z3
  #define DISABLE_STEPPER_Z3() TERN(HAS_Z3_ENABLE, Z3_ENABLE_WRITE(!Z_ENABLE_ON), NOOP)
#endif

#ifndef ENABLE_STEPPER_Z4
  #define  ENABLE_STEPPER_Z4() TERN(HAS_Z4_ENABLE, Z4_ENABLE_WRITE( Z_ENABLE_ON), NOOP)
#endif
#ifndef DISABLE_STEPPER_Z4
  #define DISABLE_STEPPER_Z4() TERN(HAS_Z4_ENABLE, Z4_ENABLE_WRITE(!Z_ENABLE_ON), NOOP)
#endif

#ifndef ENABLE_STEPPER_I
  #define  ENABLE_STEPPER_I() TERN(HAS_I_ENABLE, I_ENABLE_WRITE( I_ENABLE_ON), NOOP)
#endif
#ifndef DISABLE_STEPPER_I
  #define DISABLE_STEPPER_I() TERN(HAS_I_ENABLE, I_ENABLE_WRITE(!I_ENABLE_ON), NOOP)
#endif

#ifndef ENABLE_STEPPER_J
  #define  ENABLE_STEPPER_J() TERN(HAS_J_ENABLE, J_ENABLE_WRITE( J_ENABLE_ON), NOOP)
#endif
#ifndef DISABLE_STEPPER_J
  #define DISABLE_STEPPER_J() TERN(HAS_J_ENABLE, J_ENABLE_WRITE(!J_ENABLE_ON), NOOP)
#endif

#ifndef ENABLE_STEPPER_K
  #define  ENABLE_STEPPER_K() TERN(HAS_K_ENABLE, K_ENABLE_WRITE( K_ENABLE_ON), NOOP)
#endif
#ifndef DISABLE_STEPPER_K
  #define DISABLE_STEPPER_K() TERN(HAS_K_ENABLE, K_ENABLE_WRITE(!K_ENABLE_ON), NOOP)
#endif

#ifndef ENABLE_STEPPER_U
  #if HAS_U_ENABLE
    #define  ENABLE_STEPPER_U() U_ENABLE_WRITE( U_ENABLE_ON)
  #else
    #define  ENABLE_STEPPER_U() NOOP
  #endif
#endif
#ifndef DISABLE_STEPPER_U
  #if HAS_U_ENABLE
    #define DISABLE_STEPPER_U() U_ENABLE_WRITE(!U_ENABLE_ON)
  #else
    #define DISABLE_STEPPER_U() NOOP
  #endif
#endif

#ifndef ENABLE_STEPPER_V
  #if HAS_V_ENABLE
    #define  ENABLE_STEPPER_V() V_ENABLE_WRITE( V_ENABLE_ON)
  #else
    #define  ENABLE_STEPPER_V() NOOP
  #endif
#endif
#ifndef DISABLE_STEPPER_V
  #if HAS_V_ENABLE
    #define DISABLE_STEPPER_V() V_ENABLE_WRITE(!V_ENABLE_ON)
  #else
    #define DISABLE_STEPPER_V() NOOP
  #endif
#endif

#ifndef ENABLE_STEPPER_W
  #if HAS_W_ENABLE
    #define  ENABLE_STEPPER_W() W_ENABLE_WRITE( W_ENABLE_ON)
  #else
    #define  ENABLE_STEPPER_W() NOOP
  #endif
#endif
#ifndef DISABLE_STEPPER_W
  #if HAS_W_ENABLE
    #define DISABLE_STEPPER_W() W_ENABLE_WRITE(!W_ENABLE_ON)
  #else
    #define DISABLE_STEPPER_W() NOOP
  #endif
#endif

#ifndef ENABLE_STEPPER_E0
  #define  ENABLE_STEPPER_E0() TERN(HAS_E0_ENABLE, E0_ENABLE_WRITE( E_ENABLE_ON), NOOP)
#endif
#ifndef DISABLE_STEPPER_E0
  #define DISABLE_STEPPER_E0() TERN(HAS_E0_ENABLE, E0_ENABLE_WRITE(!E_ENABLE_ON), NOOP)
#endif

#ifndef ENABLE_STEPPER_E1
  #if (E_STEPPERS > 1 || ENABLED(E_DUAL_STEPPER_DRIVERS)) && HAS_E1_ENABLE
    #define  ENABLE_STEPPER_E1() E1_ENABLE_WRITE( E_ENABLE_ON)
  #else
    #define  ENABLE_STEPPER_E1() NOOP
  #endif
#endif
#ifndef DISABLE_STEPPER_E1
  #if (E_STEPPERS > 1 || ENABLED(E_DUAL_STEPPER_DRIVERS)) && HAS_E1_ENABLE
    #define DISABLE_STEPPER_E1() E1_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define DISABLE_STEPPER_E1() NOOP
  #endif
#endif

#ifndef ENABLE_STEPPER_E2
  #if E_STEPPERS > 2 && HAS_E2_ENABLE
    #define  ENABLE_STEPPER_E2() E2_ENABLE_WRITE( E_ENABLE_ON)
  #else
    #define  ENABLE_STEPPER_E2() NOOP
  #endif
#endif
#ifndef DISABLE_STEPPER_E2
  #if E_STEPPERS > 2 && HAS_E2_ENABLE
    #define DISABLE_STEPPER_E2() E2_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define DISABLE_STEPPER_E2() NOOP
  #endif
#endif

#ifndef ENABLE_STEPPER_E3
  #if E_STEPPERS > 3 && HAS_E3_ENABLE
    #define  ENABLE_STEPPER_E3() E3_ENABLE_WRITE( E_ENABLE_ON)
  #else
    #define  ENABLE_STEPPER_E3() NOOP
  #endif
#endif
#ifndef DISABLE_STEPPER_E3
  #if E_STEPPERS > 3 && HAS_E3_ENABLE
    #define DISABLE_STEPPER_E3() E3_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define DISABLE_STEPPER_E3() NOOP
  #endif
#endif

#ifndef ENABLE_STEPPER_E4
  #if E_STEPPERS > 4 && HAS_E4_ENABLE
    #define  ENABLE_STEPPER_E4() E4_ENABLE_WRITE( E_ENABLE_ON)
  #else
    #define  ENABLE_STEPPER_E4() NOOP
  #endif
#endif
#ifndef DISABLE_STEPPER_E4
  #if E_STEPPERS > 4 && HAS_E4_ENABLE
    #define DISABLE_STEPPER_E4() E4_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define DISABLE_STEPPER_E4() NOOP
  #endif
#endif

#ifndef ENABLE_STEPPER_E5
  #if E_STEPPERS > 5 && HAS_E5_ENABLE
    #define  ENABLE_STEPPER_E5() E5_ENABLE_WRITE( E_ENABLE_ON)
  #else
    #define  ENABLE_STEPPER_E5() NOOP
  #endif
#endif
#ifndef DISABLE_STEPPER_E5
  #if E_STEPPERS > 5 && HAS_E5_ENABLE
    #define DISABLE_STEPPER_E5() E5_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define DISABLE_STEPPER_E5() NOOP
  #endif
#endif

#ifndef ENABLE_STEPPER_E6
  #if E_STEPPERS > 6 && HAS_E6_ENABLE
    #define  ENABLE_STEPPER_E6() E6_ENABLE_WRITE( E_ENABLE_ON)
  #else
    #define  ENABLE_STEPPER_E6() NOOP
  #endif
#endif
#ifndef DISABLE_STEPPER_E6
  #if E_STEPPERS > 6 && HAS_E6_ENABLE
    #define DISABLE_STEPPER_E6() E6_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define DISABLE_STEPPER_E6() NOOP
  #endif
#endif

#ifndef ENABLE_STEPPER_E7
  #if E_STEPPERS > 7 && HAS_E7_ENABLE
    #define  ENABLE_STEPPER_E7() E7_ENABLE_WRITE( E_ENABLE_ON)
  #else
    #define  ENABLE_STEPPER_E7() NOOP
  #endif
#endif
#ifndef DISABLE_STEPPER_E7
  #if E_STEPPERS > 7 && HAS_E7_ENABLE
    #define DISABLE_STEPPER_E7() E7_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define DISABLE_STEPPER_E7() NOOP
  #endif
#endif

//
// Axis steppers enable / disable macros
//
#if ENABLED(SOFTWARE_DRIVER_ENABLE)
  // Avoid expensive calls to enable / disable steppers
  extern xyz_bool_t axis_sw_enabled;
  #define SHOULD_ENABLE(N)  !axis_sw_enabled.N
  #define SHOULD_DISABLE(N)  axis_sw_enabled.N
  #define AFTER_CHANGE(N,TF) axis_sw_enabled.N = TF
#else
  #define SHOULD_ENABLE(N)   true
  #define SHOULD_DISABLE(N)  true
  #define AFTER_CHANGE(N,TF) NOOP
#endif

#if HAS_X_AXIS
  #define  ENABLE_AXIS_X() if (SHOULD_ENABLE(x))  {  ENABLE_STEPPER_X();  ENABLE_STEPPER_X2(); AFTER_CHANGE(x, true); }
  #define DISABLE_AXIS_X() if (SHOULD_DISABLE(x)) { DISABLE_STEPPER_X(); DISABLE_STEPPER_X2(); AFTER_CHANGE(x, false); set_axis_untrusted(X_AXIS); }
#else
  #define  ENABLE_AXIS_X() NOOP
  #define DISABLE_AXIS_X() NOOP
#endif

#if HAS_Y_AXIS
  #define  ENABLE_AXIS_Y() if (SHOULD_ENABLE(y))  {  ENABLE_STEPPER_Y();  ENABLE_STEPPER_Y2(); AFTER_CHANGE(y, true); }
  #define DISABLE_AXIS_Y() if (SHOULD_DISABLE(y)) { DISABLE_STEPPER_Y(); DISABLE_STEPPER_Y2(); AFTER_CHANGE(y, false); set_axis_untrusted(Y_AXIS); }
#else
  #define  ENABLE_AXIS_Y() NOOP
  #define DISABLE_AXIS_Y() NOOP
#endif

#if HAS_Z_AXIS
  #define  ENABLE_AXIS_Z() if (SHOULD_ENABLE(z))  {  ENABLE_STEPPER_Z();  ENABLE_STEPPER_Z2();  ENABLE_STEPPER_Z3();  ENABLE_STEPPER_Z4(); AFTER_CHANGE(z, true); }
  #define DISABLE_AXIS_Z() if (SHOULD_DISABLE(z)) { DISABLE_STEPPER_Z(); DISABLE_STEPPER_Z2(); DISABLE_STEPPER_Z3(); DISABLE_STEPPER_Z4(); AFTER_CHANGE(z, false); set_axis_untrusted(Z_AXIS); Z_RESET(); TERN_(BD_SENSOR, bdl.config_state = BDS_IDLE); }
#else
  #define  ENABLE_AXIS_Z() NOOP
  #define DISABLE_AXIS_Z() NOOP
#endif

#ifdef Z_IDLE_HEIGHT
  #define Z_RESET() do{ current_position.z = Z_IDLE_HEIGHT; sync_plan_position(); }while(0)
#else
  #define Z_RESET()
#endif

#if HAS_I_AXIS
  #define  ENABLE_AXIS_I() if (SHOULD_ENABLE(i))  {  ENABLE_STEPPER_I(); AFTER_CHANGE(i, true); }
  #define DISABLE_AXIS_I() if (SHOULD_DISABLE(i)) { DISABLE_STEPPER_I(); AFTER_CHANGE(i, false); set_axis_untrusted(I_AXIS); }
#else
  #define  ENABLE_AXIS_I() NOOP
  #define DISABLE_AXIS_I() NOOP
#endif
#if HAS_J_AXIS
  #define  ENABLE_AXIS_J() if (SHOULD_ENABLE(j))  {  ENABLE_STEPPER_J(); AFTER_CHANGE(j, true); }
  #define DISABLE_AXIS_J() if (SHOULD_DISABLE(j)) { DISABLE_STEPPER_J(); AFTER_CHANGE(j, false); set_axis_untrusted(J_AXIS); }
#else
  #define  ENABLE_AXIS_J() NOOP
  #define DISABLE_AXIS_J() NOOP
#endif
#if HAS_K_AXIS
  #define  ENABLE_AXIS_K() if (SHOULD_ENABLE(k))  {  ENABLE_STEPPER_K(); AFTER_CHANGE(k, true); }
  #define DISABLE_AXIS_K() if (SHOULD_DISABLE(k)) { DISABLE_STEPPER_K(); AFTER_CHANGE(k, false); set_axis_untrusted(K_AXIS); }
#else
  #define  ENABLE_AXIS_K() NOOP
  #define DISABLE_AXIS_K() NOOP
#endif

#if HAS_U_AXIS
  #define  ENABLE_AXIS_U() if (SHOULD_ENABLE(u))  {  ENABLE_STEPPER_U(); AFTER_CHANGE(u, true); }
  #define DISABLE_AXIS_U() if (SHOULD_DISABLE(u)) { DISABLE_STEPPER_U(); AFTER_CHANGE(u, false); set_axis_untrusted(U_AXIS); }
#else
  #define  ENABLE_AXIS_U() NOOP
  #define DISABLE_AXIS_U() NOOP
#endif
#if HAS_V_AXIS
  #define  ENABLE_AXIS_V() if (SHOULD_ENABLE(v))  {  ENABLE_STEPPER_V(); AFTER_CHANGE(v, true); }
  #define DISABLE_AXIS_V() if (SHOULD_DISABLE(v)) { DISABLE_STEPPER_V(); AFTER_CHANGE(v, false); set_axis_untrusted(V_AXIS); }
#else
  #define  ENABLE_AXIS_V() NOOP
  #define DISABLE_AXIS_V() NOOP
#endif
#if HAS_W_AXIS
  #define  ENABLE_AXIS_W() if (SHOULD_ENABLE(w))  {  ENABLE_STEPPER_W(); AFTER_CHANGE(w, true); }
  #define DISABLE_AXIS_W() if (SHOULD_DISABLE(w)) { DISABLE_STEPPER_W(); AFTER_CHANGE(w, false); set_axis_untrusted(W_AXIS); }
#else
  #define  ENABLE_AXIS_W() NOOP
  #define DISABLE_AXIS_W() NOOP
#endif

//
// Extruder steppers enable / disable macros
//

#if ENABLED(MIXING_EXTRUDER)

  /**
   * Mixing steppers keep all their enable (and direction) states synchronized
   */
  #define _CALL_ENA_E(N)  ENABLE_STEPPER_E##N () ;
  #define _CALL_DIS_E(N) DISABLE_STEPPER_E##N () ;
  #define  ENABLE_AXIS_E0() { RREPEAT(MIXING_STEPPERS, _CALL_ENA_E) }
  #define DISABLE_AXIS_E0() { RREPEAT(MIXING_STEPPERS, _CALL_DIS_E) }

#elif ENABLED(E_DUAL_STEPPER_DRIVERS)

  #define  ENABLE_AXIS_E0() do{  ENABLE_STEPPER_E0();  ENABLE_STEPPER_E1(); }while(0)
  #define DISABLE_AXIS_E0() do{ DISABLE_STEPPER_E0(); DISABLE_STEPPER_E1(); }while(0)

#endif

#ifndef ENABLE_AXIS_E0
  #if E_STEPPERS && HAS_E0_ENABLE
    #define  ENABLE_AXIS_E0() ENABLE_STEPPER_E0()
  #else
    #define  ENABLE_AXIS_E0() NOOP
  #endif
#endif
#ifndef DISABLE_AXIS_E0
  #if E_STEPPERS && HAS_E0_ENABLE
    #define DISABLE_AXIS_E0() DISABLE_STEPPER_E0()
  #else
    #define DISABLE_AXIS_E0() NOOP
  #endif
#endif

#ifndef ENABLE_AXIS_E1
  #if E_STEPPERS > 1 && HAS_E1_ENABLE
    #define  ENABLE_AXIS_E1() ENABLE_STEPPER_E1()
  #else
    #define  ENABLE_AXIS_E1() NOOP
  #endif
#endif
#ifndef DISABLE_AXIS_E1
  #if E_STEPPERS > 1 && HAS_E1_ENABLE
    #define DISABLE_AXIS_E1() DISABLE_STEPPER_E1()
  #else
    #define DISABLE_AXIS_E1() NOOP
  #endif
#endif

#ifndef ENABLE_AXIS_E2
  #if E_STEPPERS > 2 && HAS_E2_ENABLE
    #define  ENABLE_AXIS_E2() ENABLE_STEPPER_E2()
  #else
    #define  ENABLE_AXIS_E2() NOOP
  #endif
#endif
#ifndef DISABLE_AXIS_E2
  #if E_STEPPERS > 2 && HAS_E2_ENABLE
    #define DISABLE_AXIS_E2() DISABLE_STEPPER_E2()
  #else
    #define DISABLE_AXIS_E2() NOOP
  #endif
#endif

#ifndef ENABLE_AXIS_E3
  #if E_STEPPERS > 3 && HAS_E3_ENABLE
    #define  ENABLE_AXIS_E3() ENABLE_STEPPER_E3()
  #else
    #define  ENABLE_AXIS_E3() NOOP
  #endif
#endif
#ifndef DISABLE_AXIS_E3
  #if E_STEPPERS > 3 && HAS_E3_ENABLE
    #define DISABLE_AXIS_E3() DISABLE_STEPPER_E3()
  #else
    #define DISABLE_AXIS_E3() NOOP
  #endif
#endif

#ifndef ENABLE_AXIS_E4
  #if E_STEPPERS > 4 && HAS_E4_ENABLE
    #define  ENABLE_AXIS_E4() ENABLE_STEPPER_E4()
  #else
    #define  ENABLE_AXIS_E4() NOOP
  #endif
#endif
#ifndef DISABLE_AXIS_E4
  #if E_STEPPERS > 4 && HAS_E4_ENABLE
    #define DISABLE_AXIS_E4() DISABLE_STEPPER_E4()
  #else
    #define DISABLE_AXIS_E4() NOOP
  #endif
#endif

#ifndef ENABLE_AXIS_E5
  #if E_STEPPERS > 5 && HAS_E5_ENABLE
    #define  ENABLE_AXIS_E5() ENABLE_STEPPER_E5()
  #else
    #define  ENABLE_AXIS_E5() NOOP
  #endif
#endif
#ifndef DISABLE_AXIS_E5
  #if E_STEPPERS > 5 && HAS_E5_ENABLE
    #define DISABLE_AXIS_E5() DISABLE_STEPPER_E5()
  #else
    #define DISABLE_AXIS_E5() NOOP
  #endif
#endif

#ifndef ENABLE_AXIS_E6
  #if E_STEPPERS > 6 && HAS_E6_ENABLE
    #define  ENABLE_AXIS_E6() ENABLE_STEPPER_E6()
  #else
    #define  ENABLE_AXIS_E6() NOOP
  #endif
#endif
#ifndef DISABLE_AXIS_E6
  #if E_STEPPERS > 6 && HAS_E6_ENABLE
    #define DISABLE_AXIS_E6() DISABLE_STEPPER_E6()
  #else
    #define DISABLE_AXIS_E6() NOOP
  #endif
#endif

#ifndef ENABLE_AXIS_E7
  #if E_STEPPERS > 7 && HAS_E7_ENABLE
    #define  ENABLE_AXIS_E7() ENABLE_STEPPER_E7()
  #else
    #define  ENABLE_AXIS_E7() NOOP
  #endif
#endif
#ifndef DISABLE_AXIS_E7
  #if E_STEPPERS > 7 && HAS_E7_ENABLE
    #define DISABLE_AXIS_E7() DISABLE_STEPPER_E7()
  #else
    #define DISABLE_AXIS_E7() NOOP
  #endif
#endif
