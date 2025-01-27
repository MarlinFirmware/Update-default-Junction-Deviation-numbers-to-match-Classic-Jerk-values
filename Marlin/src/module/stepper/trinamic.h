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
 * stepper/trinamic.h
 * Stepper driver indirection for Trinamic
 */

#include <TMCStepper.h>
#if TMCSTEPPER_VERSION < 0x000500
  #error "Update TMCStepper library to 0.5.0 or newer."
#endif

#include "../../inc/MarlinConfig.h"
#include "../../feature/tmc_util.h"

#define CLASS_TMC2130 TMC2130Stepper
#define CLASS_TMC2160 TMC2160Stepper
#define CLASS_TMC2208 TMC2208Stepper
#define CLASS_TMC2209 TMC2209Stepper
#define CLASS_TMC2660 TMC2660Stepper
#define CLASS_TMC5130 TMC5130Stepper
#define CLASS_TMC5160 TMC5160Stepper

#define TMC_X_LABEL 'X', '0'
#define TMC_Y_LABEL 'Y', '0'
#define TMC_Z_LABEL 'Z', '0'

#define TMC_I_LABEL 'I', '0'
#define TMC_J_LABEL 'J', '0'
#define TMC_K_LABEL 'K', '0'
#define TMC_U_LABEL 'U', '0'
#define TMC_V_LABEL 'V', '0'
#define TMC_W_LABEL 'W', '0'

#define TMC_X2_LABEL 'X', '2'
#define TMC_Y2_LABEL 'Y', '2'
#define TMC_Z2_LABEL 'Z', '2'
#define TMC_Z3_LABEL 'Z', '3'
#define TMC_Z4_LABEL 'Z', '4'

#define TMC_E0_LABEL 'E', '0'
#define TMC_E1_LABEL 'E', '1'
#define TMC_E2_LABEL 'E', '2'
#define TMC_E3_LABEL 'E', '3'
#define TMC_E4_LABEL 'E', '4'
#define TMC_E5_LABEL 'E', '5'
#define TMC_E6_LABEL 'E', '6'
#define TMC_E7_LABEL 'E', '7'

#define __TMC_CLASS(TYPE, L, I, A) TMCMarlin<CLASS_##TYPE, L, I, A>
#define _TMC_CLASS(TYPE, LandI, A) __TMC_CLASS(TYPE, LandI, A)
#define TMC_CLASS(ST, A) _TMC_CLASS(ST##_DRIVER_TYPE, TMC_##ST##_LABEL, A##_AXIS)
#if ENABLED(DISTINCT_E_FACTORS)
  #define TMC_CLASS_E(N) TMC_CLASS(E##N, E##N)
#else
  #define TMC_CLASS_E(N) TMC_CLASS(E##N, E)
#endif

#if HAS_X_AXIS && !defined(CHOPPER_TIMING_X)
  #define CHOPPER_TIMING_X CHOPPER_TIMING
#endif
#if HAS_Y_AXIS && !defined(CHOPPER_TIMING_Y)
  #define CHOPPER_TIMING_Y CHOPPER_TIMING
#endif
#if HAS_Z_AXIS && !defined(CHOPPER_TIMING_Z)
  #define CHOPPER_TIMING_Z CHOPPER_TIMING
#endif
#if HAS_I_AXIS && !defined(CHOPPER_TIMING_I)
  #define CHOPPER_TIMING_I CHOPPER_TIMING
#endif
#if HAS_J_AXIS && !defined(CHOPPER_TIMING_J)
  #define CHOPPER_TIMING_J CHOPPER_TIMING
#endif
#if HAS_K_AXIS && !defined(CHOPPER_TIMING_K)
  #define CHOPPER_TIMING_K CHOPPER_TIMING
#endif
#if HAS_U_AXIS && !defined(CHOPPER_TIMING_U)
  #define CHOPPER_TIMING_U CHOPPER_TIMING
#endif
#if HAS_V_AXIS && !defined(CHOPPER_TIMING_V)
  #define CHOPPER_TIMING_V CHOPPER_TIMING
#endif
#if HAS_W_AXIS && !defined(CHOPPER_TIMING_W)
  #define CHOPPER_TIMING_W CHOPPER_TIMING
#endif
#if HAS_EXTRUDERS && !defined(CHOPPER_TIMING_E)
  #define CHOPPER_TIMING_E CHOPPER_TIMING
#endif

#if HAS_TMC_UART
  void tmc_serial_begin();
#endif

void restore_trinamic_drivers();
void reset_trinamic_drivers();

// X Stepper
#if AXIS_IS_TMC(X)
  extern TMC_CLASS(X, X) stepperX;
  static constexpr chopper_timing_t chopper_timing_X = CHOPPER_TIMING_X;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define X_ENABLE_INIT() NOOP
    #define X_ENABLE_WRITE(STATE) stepperX.toff((STATE)==X_ENABLE_ON ? chopper_timing_X.toff : 0)
    #define X_ENABLE_READ() stepperX.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(X)
    #define X_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(X_STEP_PIN); }while(0)
  #endif
#endif

// Y Stepper
#if AXIS_IS_TMC(Y)
  extern TMC_CLASS(Y, Y) stepperY;
  static constexpr chopper_timing_t chopper_timing_Y = CHOPPER_TIMING_Y;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define Y_ENABLE_INIT() NOOP
    #define Y_ENABLE_WRITE(STATE) stepperY.toff((STATE)==Y_ENABLE_ON ? chopper_timing_Y.toff : 0)
    #define Y_ENABLE_READ() stepperY.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(Y)
    #define Y_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(Y_STEP_PIN); }while(0)
  #endif
#endif

// Z Stepper
#if AXIS_IS_TMC(Z)
  extern TMC_CLASS(Z, Z) stepperZ;
  static constexpr chopper_timing_t chopper_timing_Z = CHOPPER_TIMING_Z;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define Z_ENABLE_INIT() NOOP
    #define Z_ENABLE_WRITE(STATE) stepperZ.toff((STATE)==Z_ENABLE_ON ? chopper_timing_Z.toff : 0)
    #define Z_ENABLE_READ() stepperZ.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(Z)
    #define Z_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(Z_STEP_PIN); }while(0)
  #endif
#endif

// X2 Stepper
#if HAS_X2_ENABLE && AXIS_IS_TMC(X2)
  extern TMC_CLASS(X2, X) stepperX2;
  #ifndef CHOPPER_TIMING_X2
    #define CHOPPER_TIMING_X2 CHOPPER_TIMING_X
  #endif
  static constexpr chopper_timing_t chopper_timing_X2 = CHOPPER_TIMING_X2;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define X2_ENABLE_INIT() NOOP
    #define X2_ENABLE_WRITE(STATE) stepperX2.toff((STATE)==X_ENABLE_ON ? chopper_timing_X2.toff : 0)
    #define X2_ENABLE_READ() stepperX2.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(X2)
    #define X2_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(X2_STEP_PIN); }while(0)
  #endif
#endif

// Y2 Stepper
#if HAS_Y2_ENABLE && AXIS_IS_TMC(Y2)
  extern TMC_CLASS(Y2, Y) stepperY2;
  #ifndef CHOPPER_TIMING_Y2
    #define CHOPPER_TIMING_Y2 CHOPPER_TIMING_Y
  #endif
  static constexpr chopper_timing_t chopper_timing_Y2 = CHOPPER_TIMING_Y2;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define Y2_ENABLE_INIT() NOOP
    #define Y2_ENABLE_WRITE(STATE) stepperY2.toff((STATE)==Y_ENABLE_ON ? chopper_timing_Y2.toff : 0)
    #define Y2_ENABLE_READ() stepperY2.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(Y2)
    #define Y2_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(Y2_STEP_PIN); }while(0)
  #endif
#endif

// Z2 Stepper
#if HAS_Z2_ENABLE && AXIS_IS_TMC(Z2)
  extern TMC_CLASS(Z2, Z) stepperZ2;
  #ifndef CHOPPER_TIMING_Z2
    #define CHOPPER_TIMING_Z2 CHOPPER_TIMING_Z
  #endif
  static constexpr chopper_timing_t chopper_timing_Z2 = CHOPPER_TIMING_Z2;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define Z2_ENABLE_INIT() NOOP
    #define Z2_ENABLE_WRITE(STATE) stepperZ2.toff((STATE)==Z_ENABLE_ON ? chopper_timing_Z2.toff : 0)
    #define Z2_ENABLE_READ() stepperZ2.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(Z2)
    #define Z2_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(Z2_STEP_PIN); }while(0)
  #endif
#endif

// Z3 Stepper
#if HAS_Z3_ENABLE && AXIS_IS_TMC(Z3)
  extern TMC_CLASS(Z3, Z) stepperZ3;
  #ifndef CHOPPER_TIMING_Z3
    #define CHOPPER_TIMING_Z3 CHOPPER_TIMING_Z
  #endif
  static constexpr chopper_timing_t chopper_timing_Z3 = CHOPPER_TIMING_Z3;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define Z3_ENABLE_INIT() NOOP
    #define Z3_ENABLE_WRITE(STATE) stepperZ3.toff((STATE)==Z_ENABLE_ON ? chopper_timing_Z3.toff : 0)
    #define Z3_ENABLE_READ() stepperZ3.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(Z3)
    #define Z3_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(Z3_STEP_PIN); }while(0)
  #endif
#endif

// Z4 Stepper
#if HAS_Z4_ENABLE && AXIS_IS_TMC(Z4)
  extern TMC_CLASS(Z4, Z) stepperZ4;
  #ifndef CHOPPER_TIMING_Z4
    #define CHOPPER_TIMING_Z4 CHOPPER_TIMING_Z
  #endif
  static constexpr chopper_timing_t chopper_timing_Z4 = CHOPPER_TIMING_Z4;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define Z4_ENABLE_INIT() NOOP
    #define Z4_ENABLE_WRITE(STATE) stepperZ4.toff((STATE)==Z_ENABLE_ON ? chopper_timing_Z4.toff : 0)
    #define Z4_ENABLE_READ() stepperZ4.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(Z4)
    #define Z4_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(Z4_STEP_PIN); }while(0)
  #endif
#endif

// I Stepper
#if AXIS_IS_TMC(I)
  extern TMC_CLASS(I, I) stepperI;
  static constexpr chopper_timing_t chopper_timing_I = CHOPPER_TIMING_I;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define I_ENABLE_INIT() NOOP
    #define I_ENABLE_WRITE(STATE) stepperI.toff((STATE)==I_ENABLE_ON ? chopper_timing_I.toff : 0)
    #define I_ENABLE_READ() stepperI.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(I)
    #define I_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(I_STEP_PIN); }while(0)
  #endif
#endif

// J Stepper
#if AXIS_IS_TMC(J)
  extern TMC_CLASS(J, J) stepperJ;
  static constexpr chopper_timing_t chopper_timing_J = CHOPPER_TIMING_J;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define J_ENABLE_INIT() NOOP
    #define J_ENABLE_WRITE(STATE) stepperJ.toff((STATE)==J_ENABLE_ON ? chopper_timing_J.toff : 0)
    #define J_ENABLE_READ() stepperJ.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(J)
    #define J_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(J_STEP_PIN); }while(0)
  #endif
#endif

// K Stepper
#if AXIS_IS_TMC(K)
  extern TMC_CLASS(K, K) stepperK;
  static constexpr chopper_timing_t chopper_timing_K = CHOPPER_TIMING_K;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define K_ENABLE_INIT() NOOP
    #define K_ENABLE_WRITE(STATE) stepperK.toff((STATE)==K_ENABLE_ON ? chopper_timing_K.toff : 0)
    #define K_ENABLE_READ() stepperK.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(K)
    #define K_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(K_STEP_PIN); }while(0)
  #endif
#endif

// U Stepper
#if AXIS_IS_TMC(U)
  extern TMC_CLASS(U, U) stepperU;
  static constexpr chopper_timing_t chopper_timing_U = CHOPPER_TIMING_U;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define U_ENABLE_INIT() NOOP
    #define U_ENABLE_WRITE(STATE) stepperU.toff((STATE)==U_ENABLE_ON ? chopper_timing_U.toff : 0)
    #define U_ENABLE_READ() stepperU.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(U)
    #define U_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(U_STEP_PIN); }while(0)
  #endif
#endif

// V Stepper
#if AXIS_IS_TMC(V)
  extern TMC_CLASS(V, V) stepperV;
  static constexpr chopper_timing_t chopper_timing_V = CHOPPER_TIMING_V;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define V_ENABLE_INIT() NOOP
    #define V_ENABLE_WRITE(STATE) stepperV.toff((STATE)==V_ENABLE_ON ? chopper_timing_V.toff : 0)
    #define V_ENABLE_READ() stepperV.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(V)
    #define V_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(V_STEP_PIN); }while(0)
  #endif
#endif

// W Stepper
#if AXIS_IS_TMC(W)
  extern TMC_CLASS(W, W) stepperW;
  static constexpr chopper_timing_t chopper_timing_W = CHOPPER_TIMING_W;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define W_ENABLE_INIT() NOOP
    #define W_ENABLE_WRITE(STATE) stepperW.toff((STATE)==W_ENABLE_ON ? chopper_timing_W.toff : 0)
    #define W_ENABLE_READ() stepperW.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(W)
    #define W_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(W_STEP_PIN); }while(0)
  #endif
#endif

// E0 Stepper
#if AXIS_IS_TMC(E0)
  extern TMC_CLASS_E(0) stepperE0;
  #ifndef CHOPPER_TIMING_E0
    #define CHOPPER_TIMING_E0 CHOPPER_TIMING_E
  #endif
  static constexpr chopper_timing_t chopper_timing_E0 = CHOPPER_TIMING_E0;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define E0_ENABLE_INIT() NOOP
    #define E0_ENABLE_WRITE(STATE) stepperE0.toff((STATE)==E_ENABLE_ON ? chopper_timing_E0.toff : 0)
    #define E0_ENABLE_READ() stepperE0.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(E0)
    #define E0_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(E0_STEP_PIN); }while(0)
  #endif
#endif

// E1 Stepper
#if AXIS_IS_TMC(E1)
  extern TMC_CLASS_E(1) stepperE1;
  #ifndef CHOPPER_TIMING_E1
    #define CHOPPER_TIMING_E1 CHOPPER_TIMING_E
  #endif
  static constexpr chopper_timing_t chopper_timing_E1 = CHOPPER_TIMING_E1;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define E1_ENABLE_INIT() NOOP
    #define E1_ENABLE_WRITE(STATE) stepperE1.toff((STATE)==E_ENABLE_ON ? chopper_timing_E1.toff : 0)
    #define E1_ENABLE_READ() stepperE1.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(E1)
    #define E1_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(E1_STEP_PIN); }while(0)
  #endif
#endif

// E2 Stepper
#if AXIS_IS_TMC(E2)
  extern TMC_CLASS_E(2) stepperE2;
  #ifndef CHOPPER_TIMING_E2
    #define CHOPPER_TIMING_E2 CHOPPER_TIMING_E
  #endif
  static constexpr chopper_timing_t chopper_timing_E2 = CHOPPER_TIMING_E2;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define E2_ENABLE_INIT() NOOP
    #define E2_ENABLE_WRITE(STATE) stepperE2.toff((STATE)==E_ENABLE_ON ? chopper_timing_E2.toff : 0)
    #define E2_ENABLE_READ() stepperE2.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(E2)
    #define E2_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(E2_STEP_PIN); }while(0)
  #endif
#endif

// E3 Stepper
#if AXIS_IS_TMC(E3)
  extern TMC_CLASS_E(3) stepperE3;
  #ifndef CHOPPER_TIMING_E3
    #define CHOPPER_TIMING_E3 CHOPPER_TIMING_E
  #endif
  static constexpr chopper_timing_t chopper_timing_E3 = CHOPPER_TIMING_E3;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define E3_ENABLE_INIT() NOOP
    #define E3_ENABLE_WRITE(STATE) stepperE3.toff((STATE)==E_ENABLE_ON ? chopper_timing_E3.toff : 0)
    #define E3_ENABLE_READ() stepperE3.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(E3)
    #define E3_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(E3_STEP_PIN); }while(0)
  #endif
#endif

// E4 Stepper
#if AXIS_IS_TMC(E4)
  extern TMC_CLASS_E(4) stepperE4;
  #ifndef CHOPPER_TIMING_E4
    #define CHOPPER_TIMING_E4 CHOPPER_TIMING_E
  #endif
  static constexpr chopper_timing_t chopper_timing_E4 = CHOPPER_TIMING_E4;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define E4_ENABLE_INIT() NOOP
    #define E4_ENABLE_WRITE(STATE) stepperE4.toff((STATE)==E_ENABLE_ON ? chopper_timing_E4.toff : 0)
    #define E4_ENABLE_READ() stepperE4.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(E4)
    #define E4_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(E4_STEP_PIN); }while(0)
  #endif
#endif

// E5 Stepper
#if AXIS_IS_TMC(E5)
  extern TMC_CLASS_E(5) stepperE5;
  #ifndef CHOPPER_TIMING_E5
    #define CHOPPER_TIMING_E5 CHOPPER_TIMING_E
  #endif
  static constexpr chopper_timing_t chopper_timing_E5 = CHOPPER_TIMING_E5;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define E5_ENABLE_INIT() NOOP
    #define E5_ENABLE_WRITE(STATE) stepperE5.toff((STATE)==E_ENABLE_ON ? chopper_timing_E5.toff : 0)
    #define E5_ENABLE_READ() stepperE5.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(E5)
    #define E5_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(E5_STEP_PIN); }while(0)
  #endif
#endif

// E6 Stepper
#if AXIS_IS_TMC(E6)
  extern TMC_CLASS_E(6) stepperE6;
  #ifndef CHOPPER_TIMING_E6
    #define CHOPPER_TIMING_E6 CHOPPER_TIMING_E
  #endif
  static constexpr chopper_timing_t chopper_timing_E6 = CHOPPER_TIMING_E6;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define E6_ENABLE_INIT() NOOP
    #define E6_ENABLE_WRITE(STATE) stepperE6.toff((STATE)==E_ENABLE_ON ? chopper_timing_E6.toff : 0)
    #define E6_ENABLE_READ() stepperE6.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(E6)
    #define E6_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(E6_STEP_PIN); }while(0)
  #endif
#endif

// E7 Stepper
#if AXIS_IS_TMC(E7)
  extern TMC_CLASS_E(7) stepperE7;
  #ifndef CHOPPER_TIMING_E7
    #define CHOPPER_TIMING_E7 CHOPPER_TIMING_E
  #endif
  static constexpr chopper_timing_t chopper_timing_E7 = CHOPPER_TIMING_E7;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define E7_ENABLE_INIT() NOOP
    #define E7_ENABLE_WRITE(STATE) stepperE7.toff((STATE)==E_ENABLE_ON ? chopper_timing_E7.toff : 0)
    #define E7_ENABLE_READ() stepperE7.isEnabled()
  #endif
  #if AXIS_HAS_DEDGE(E7)
    #define E7_STEP_WRITE(STATE) do{ if (STATE) TOGGLE(E7_STEP_PIN); }while(0)
  #endif
#endif
