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

#include "../../../inc/MarlinConfig.h"

#if HAS_TRINAMIC_CONFIG

#include "../../gcode.h"
#include "../../../feature/tmc_util.h"
#include "../../../module/stepper/indirection.h"
#include "../../../module/planner.h"
#include "../../queue.h"

#if ENABLED(MONITOR_DRIVER_STATUS)

  #define M91x_USE(ST) (AXIS_DRIVER_TYPE(ST, TMC2130) || AXIS_DRIVER_TYPE(ST, TMC2160) \
                     || AXIS_DRIVER_TYPE(ST, TMC2208) || AXIS_DRIVER_TYPE(ST, TMC2209) || AXIS_DRIVER_TYPE(ST, TMC2240) \
                     || AXIS_DRIVER_TYPE(ST, TMC2660) || AXIS_DRIVER_TYPE(ST, TMC5130) || AXIS_DRIVER_TYPE(ST, TMC5160))
  #define M91x_USE_E(N) (E_STEPPERS > N && M91x_USE(E##N))

  #if HAS_X_AXIS && (M91x_USE(X) || M91x_USE(X2))
    #define M91x_SOME_X 1
  #endif
  #if HAS_Y_AXIS && (M91x_USE(Y) || M91x_USE(Y2))
    #define M91x_SOME_Y 1
  #endif
  #if HAS_Z_AXIS && (M91x_USE(Z) || M91x_USE(Z2) || M91x_USE(Z3) || M91x_USE(Z4))
    #define M91x_SOME_Z 1
  #endif
  #if HAS_I_AXIS && M91x_USE(I)
    #define M91x_USE_I 1
  #endif
  #if HAS_J_AXIS && M91x_USE(J)
    #define M91x_USE_J 1
  #endif
  #if HAS_K_AXIS && M91x_USE(K)
    #define M91x_USE_K 1
  #endif
  #if HAS_U_AXIS && M91x_USE(U)
    #define M91x_USE_U 1
  #endif
  #if HAS_V_AXIS && M91x_USE(V)
    #define M91x_USE_V 1
  #endif
  #if HAS_W_AXIS && M91x_USE(W)
    #define M91x_USE_W 1
  #endif

  #if M91x_USE_E(0) || M91x_USE_E(1) || M91x_USE_E(2) || M91x_USE_E(3) || M91x_USE_E(4) || M91x_USE_E(5) || M91x_USE_E(6) || M91x_USE_E(7)
    #define M91x_SOME_E 1
  #endif

  #if !M91x_SOME_X && !M91x_SOME_Y && !M91x_SOME_Z && !M91x_USE_I && !M91x_USE_J && !M91x_USE_K && !M91x_USE_U && !M91x_USE_V && !M91x_USE_W && !M91x_SOME_E
    #error "MONITOR_DRIVER_STATUS requires at least one TMC2130, 2160, 2208, 2209, 2240, 2660, 5130, or 5160."
  #endif

  template<typename TMC>
  static void tmc_report_otpw(TMC &st) {
    st.printLabel();
    SERIAL_ECHOPGM(" temperature prewarn triggered: ");
    serialprint_truefalse(st.getOTPW());
    SERIAL_EOL();
  }

  template<typename TMC>
  static void tmc_clear_otpw(TMC &st) {
    st.clear_otpw();
    st.printLabel();
    SERIAL_ECHOLNPGM(" prewarn flag cleared");
  }

  /**
   * M911: Report TMC stepper driver overtemperature pre-warn flag
   *       This flag is held by the library, persisting until cleared by M912
   */
  void GcodeSuite::M911() {
    #if M91x_USE(X)
      tmc_report_otpw(stepperX);
    #endif
    #if M91x_USE(X2)
      tmc_report_otpw(stepperX2);
    #endif
    #if M91x_USE(Y)
      tmc_report_otpw(stepperY);
    #endif
    #if M91x_USE(Y2)
      tmc_report_otpw(stepperY2);
    #endif
    #if M91x_USE(Z)
      tmc_report_otpw(stepperZ);
    #endif
    #if M91x_USE(Z2)
      tmc_report_otpw(stepperZ2);
    #endif
    #if M91x_USE(Z3)
      tmc_report_otpw(stepperZ3);
    #endif
    #if M91x_USE(Z4)
      tmc_report_otpw(stepperZ4);
    #endif
    TERN_(M91x_USE_I, tmc_report_otpw(stepperI));
    TERN_(M91x_USE_J, tmc_report_otpw(stepperJ));
    TERN_(M91x_USE_K, tmc_report_otpw(stepperK));
    TERN_(M91x_USE_U, tmc_report_otpw(stepperU));
    TERN_(M91x_USE_V, tmc_report_otpw(stepperV));
    TERN_(M91x_USE_W, tmc_report_otpw(stepperW));
    #if M91x_USE_E(0)
      tmc_report_otpw(stepperE0);
    #endif
    #if M91x_USE_E(1)
      tmc_report_otpw(stepperE1);
    #endif
    #if M91x_USE_E(2)
      tmc_report_otpw(stepperE2);
    #endif
    #if M91x_USE_E(3)
      tmc_report_otpw(stepperE3);
    #endif
    #if M91x_USE_E(4)
      tmc_report_otpw(stepperE4);
    #endif
    #if M91x_USE_E(5)
      tmc_report_otpw(stepperE5);
    #endif
    #if M91x_USE_E(6)
      tmc_report_otpw(stepperE6);
    #endif
    #if M91x_USE_E(7)
      tmc_report_otpw(stepperE7);
    #endif
  }

  /**
   * M912: Clear TMC stepper driver overtemperature pre-warn flag held by the library
   *       Specify one or more axes with X, Y, Z, X1, Y1, Z1, X2, Y2, Z2, Z3, Z4, A, B, C, U, V, W, and E[index].
   *       If no axes are given, clear all.
   *
   * Examples:
   *       M912 X   ; clear X and X2
   *       M912 X1  ; clear X1 only
   *       M912 X2  ; clear X2 only
   *       M912 X E ; clear X, X2, and all E
   *       M912 E1  ; clear E1 only
   */
  void GcodeSuite::M912() {
    const bool hasX = TERN0(M91x_SOME_X, parser.seen(axis_codes.x)),
               hasY = TERN0(M91x_SOME_Y, parser.seen(axis_codes.y)),
               hasZ = TERN0(M91x_SOME_Z, parser.seen(axis_codes.z)),
               hasI = TERN0(M91x_USE_I,  parser.seen(axis_codes.i)),
               hasJ = TERN0(M91x_USE_J,  parser.seen(axis_codes.j)),
               hasK = TERN0(M91x_USE_K,  parser.seen(axis_codes.k)),
               hasU = TERN0(M91x_USE_U,  parser.seen(axis_codes.u)),
               hasV = TERN0(M91x_USE_V,  parser.seen(axis_codes.v)),
               hasW = TERN0(M91x_USE_W,  parser.seen(axis_codes.w)),
               hasE = TERN0(M91x_SOME_E, parser.seen(axis_codes.e));

    const bool hasNone = !hasE && !hasX && !hasY && !hasZ && !hasI && !hasJ && !hasK && !hasU && !hasV && !hasW;

    #if M91x_SOME_X
      const int8_t xval = int8_t(parser.byteval(axis_codes.x, 0xFF));
      #if M91x_USE(X)
        if (hasNone || xval == 1 || (hasX && xval < 0)) tmc_clear_otpw(stepperX);
      #endif
      #if M91x_USE(X2)
        if (hasNone || xval == 2 || (hasX && xval < 0)) tmc_clear_otpw(stepperX2);
      #endif
    #endif

    #if M91x_SOME_Y
      const int8_t yval = int8_t(parser.byteval(axis_codes.y, 0xFF));
      #if M91x_USE(Y)
        if (hasNone || yval == 1 || (hasY && yval < 0)) tmc_clear_otpw(stepperY);
      #endif
      #if M91x_USE(Y2)
        if (hasNone || yval == 2 || (hasY && yval < 0)) tmc_clear_otpw(stepperY2);
      #endif
    #endif

    #if M91x_SOME_Z
      const int8_t zval = int8_t(parser.byteval(axis_codes.z, 0xFF));
      #if M91x_USE(Z)
        if (hasNone || zval == 1 || (hasZ && zval < 0)) tmc_clear_otpw(stepperZ);
      #endif
      #if M91x_USE(Z2)
        if (hasNone || zval == 2 || (hasZ && zval < 0)) tmc_clear_otpw(stepperZ2);
      #endif
      #if M91x_USE(Z3)
        if (hasNone || zval == 3 || (hasZ && zval < 0)) tmc_clear_otpw(stepperZ3);
      #endif
      #if M91x_USE(Z4)
        if (hasNone || zval == 4 || (hasZ && zval < 0)) tmc_clear_otpw(stepperZ4);
      #endif
    #endif

    #if M91x_USE_I
      const int8_t ival = int8_t(parser.byteval(axis_codes.i, 0xFF));
      if (hasNone || ival == 1 || (hasI && ival < 0)) tmc_clear_otpw(stepperI);
    #endif
    #if M91x_USE_J
      const int8_t jval = int8_t(parser.byteval(axis_codes.j, 0xFF));
      if (hasNone || jval == 1 || (hasJ && jval < 0)) tmc_clear_otpw(stepperJ);
    #endif
    #if M91x_USE_K
      const int8_t kval = int8_t(parser.byteval(axis_codes.k, 0xFF));
      if (hasNone || kval == 1 || (hasK && kval < 0)) tmc_clear_otpw(stepperK);
    #endif
    #if M91x_USE_U
      const int8_t uval = int8_t(parser.byteval(axis_codes.u, 0xFF));
      if (hasNone || uval == 1 || (hasU && uval < 0)) tmc_clear_otpw(stepperU);
    #endif
    #if M91x_USE_V
      const int8_t vval = int8_t(parser.byteval(axis_codes.v, 0xFF));
      if (hasNone || vval == 1 || (hasV && vval < 0)) tmc_clear_otpw(stepperV);
    #endif
    #if M91x_USE_W
      const int8_t wval = int8_t(parser.byteval(axis_codes.w, 0xFF));
      if (hasNone || wval == 1 || (hasW && wval < 0)) tmc_clear_otpw(stepperW);
    #endif

    #if M91x_SOME_E
      const int8_t eval = int8_t(parser.byteval(axis_codes.e, 0xFF));
      #if M91x_USE_E(0)
        if (hasNone || eval == 0 || (hasE && eval < 0)) tmc_clear_otpw(stepperE0);
      #endif
      #if M91x_USE_E(1)
        if (hasNone || eval == 1 || (hasE && eval < 0)) tmc_clear_otpw(stepperE1);
      #endif
      #if M91x_USE_E(2)
        if (hasNone || eval == 2 || (hasE && eval < 0)) tmc_clear_otpw(stepperE2);
      #endif
      #if M91x_USE_E(3)
        if (hasNone || eval == 3 || (hasE && eval < 0)) tmc_clear_otpw(stepperE3);
      #endif
      #if M91x_USE_E(4)
        if (hasNone || eval == 4 || (hasE && eval < 0)) tmc_clear_otpw(stepperE4);
      #endif
      #if M91x_USE_E(5)
        if (hasNone || eval == 5 || (hasE && eval < 0)) tmc_clear_otpw(stepperE5);
      #endif
      #if M91x_USE_E(6)
        if (hasNone || eval == 6 || (hasE && eval < 0)) tmc_clear_otpw(stepperE6);
      #endif
      #if M91x_USE_E(7)
        if (hasNone || eval == 7 || (hasE && eval < 0)) tmc_clear_otpw(stepperE7);
      #endif
    #endif
  }

#endif // MONITOR_DRIVER_STATUS

#if ENABLED(HYBRID_THRESHOLD)

  template<typename TMC>
  static void tmc_print_pwmthrs(TMC &st) {
    st.printLabel();
    SERIAL_ECHOLNPGM(" stealthChop max speed: ", st.get_pwm_thrs());
  }

  /**
   * M913: Set HYBRID_THRESHOLD speed.
   */
  void GcodeSuite::M913() {
    #define TMC_SAY_PWMTHRS(A,Q) tmc_print_pwmthrs(stepper##Q)
    #define TMC_SET_PWMTHRS(A,Q) stepper##Q.set_pwm_thrs(value)
    #define TMC_SAY_PWMTHRS_E(E) tmc_print_pwmthrs(stepperE##E)
    #define TMC_SET_PWMTHRS_E(E) stepperE##E.set_pwm_thrs(value)

    bool report = true;
    #if AXIS_IS_TMC(X2) || AXIS_IS_TMC(Y2) || AXIS_IS_TMC(Z2) || AXIS_IS_TMC(Z3) || AXIS_IS_TMC(Z4)
      const uint8_t index = parser.byteval('I');
    #elif AXIS_IS_TMC(X) || AXIS_IS_TMC(Y) || AXIS_IS_TMC(Z)
      constexpr uint8_t index = 0;
    #endif
    LOOP_LOGICAL_AXES(i) if (int32_t value = parser.longval(AXIS_CHAR(i))) {
      report = false;
      switch (i) {
        #if X_HAS_STEALTHCHOP || X2_HAS_STEALTHCHOP
          case X_AXIS:
            TERN_(X_HAS_STEALTHCHOP,  if (index < 2) TMC_SET_PWMTHRS(X,X));
            TERN_(X2_HAS_STEALTHCHOP, if (!index || index == 2) TMC_SET_PWMTHRS(X,X2));
            break;
        #endif

        #if Y_HAS_STEALTHCHOP || Y2_HAS_STEALTHCHOP
          case Y_AXIS:
            TERN_(Y_HAS_STEALTHCHOP,  if (index < 2) TMC_SET_PWMTHRS(Y,Y));
            TERN_(Y2_HAS_STEALTHCHOP, if (!index || index == 2) TMC_SET_PWMTHRS(Y,Y2));
            break;
        #endif

        #if Z_HAS_STEALTHCHOP || Z2_HAS_STEALTHCHOP || Z3_HAS_STEALTHCHOP || Z4_HAS_STEALTHCHOP
          case Z_AXIS:
            TERN_(Z_HAS_STEALTHCHOP,  if (index < 2) TMC_SET_PWMTHRS(Z,Z));
            TERN_(Z2_HAS_STEALTHCHOP, if (!index || index == 2) TMC_SET_PWMTHRS(Z,Z2));
            TERN_(Z3_HAS_STEALTHCHOP, if (!index || index == 3) TMC_SET_PWMTHRS(Z,Z3));
            TERN_(Z4_HAS_STEALTHCHOP, if (!index || index == 4) TMC_SET_PWMTHRS(Z,Z4));
            break;
        #endif

        #if I_HAS_STEALTHCHOP
          case I_AXIS: TMC_SET_PWMTHRS(I,I); break;
        #endif
        #if J_HAS_STEALTHCHOP
          case J_AXIS: TMC_SET_PWMTHRS(J,J); break;
        #endif
        #if K_HAS_STEALTHCHOP
          case K_AXIS: TMC_SET_PWMTHRS(K,K); break;
        #endif
        #if U_HAS_STEALTHCHOP
          case U_AXIS: TMC_SET_PWMTHRS(U,U); break;
        #endif
        #if V_HAS_STEALTHCHOP
          case V_AXIS: TMC_SET_PWMTHRS(V,V); break;
        #endif
        #if W_HAS_STEALTHCHOP
          case W_AXIS: TMC_SET_PWMTHRS(W,W); break;
        #endif

        #if E0_HAS_STEALTHCHOP || E1_HAS_STEALTHCHOP || E2_HAS_STEALTHCHOP || E3_HAS_STEALTHCHOP || E4_HAS_STEALTHCHOP || E5_HAS_STEALTHCHOP || E6_HAS_STEALTHCHOP || E7_HAS_STEALTHCHOP
          case E_AXIS: {
            const int8_t eindex = get_target_e_stepper_from_command(-2);
            TERN_(E0_HAS_STEALTHCHOP, if (eindex < 0 || eindex == 0) TMC_SET_PWMTHRS_E(0));
            TERN_(E1_HAS_STEALTHCHOP, if (eindex < 0 || eindex == 1) TMC_SET_PWMTHRS_E(1));
            TERN_(E2_HAS_STEALTHCHOP, if (eindex < 0 || eindex == 2) TMC_SET_PWMTHRS_E(2));
            TERN_(E3_HAS_STEALTHCHOP, if (eindex < 0 || eindex == 3) TMC_SET_PWMTHRS_E(3));
            TERN_(E4_HAS_STEALTHCHOP, if (eindex < 0 || eindex == 4) TMC_SET_PWMTHRS_E(4));
            TERN_(E5_HAS_STEALTHCHOP, if (eindex < 0 || eindex == 5) TMC_SET_PWMTHRS_E(5));
            TERN_(E6_HAS_STEALTHCHOP, if (eindex < 0 || eindex == 6) TMC_SET_PWMTHRS_E(6));
            TERN_(E7_HAS_STEALTHCHOP, if (eindex < 0 || eindex == 7) TMC_SET_PWMTHRS_E(7));
          } break;
        #endif // E_STEPPERS
      }
    }

    if (report) {
      TERN_( X_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(X,X));
      TERN_(X2_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(X,X2));
      TERN_( Y_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(Y,Y));
      TERN_(Y2_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(Y,Y2));
      TERN_( Z_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(Z,Z));
      TERN_(Z2_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(Z,Z2));
      TERN_(Z3_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(Z,Z3));
      TERN_(Z4_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(Z,Z4));

      TERN_( I_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(I,I));
      TERN_( J_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(J,J));
      TERN_( K_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(K,K));
      TERN_( U_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(U,U));
      TERN_( V_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(V,V));
      TERN_( W_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS(W,W));

      TERN_(E0_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS_E(0));
      TERN_(E1_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS_E(1));
      TERN_(E2_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS_E(2));
      TERN_(E3_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS_E(3));
      TERN_(E4_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS_E(4));
      TERN_(E5_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS_E(5));
      TERN_(E6_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS_E(6));
      TERN_(E7_HAS_STEALTHCHOP, TMC_SAY_PWMTHRS_E(7));
    }
  }

  void GcodeSuite::M913_report(const bool forReplay/*=true*/) {
    TERN_(MARLIN_SMALL_BUILD, return);

    report_heading(forReplay, F(STR_HYBRID_THRESHOLD));

    auto say_M913 = [](const bool forReplay) {
      report_echo_start(forReplay);
      SERIAL_ECHOPGM("  M913");
    };

    #if X_HAS_STEALTHCHOP || Y_HAS_STEALTHCHOP || Z_HAS_STEALTHCHOP
      say_M913(forReplay);
      #if X_HAS_STEALTHCHOP
        SERIAL_ECHOPGM_P(SP_X_STR, stepperX.get_pwm_thrs());
      #endif
      #if Y_HAS_STEALTHCHOP
        SERIAL_ECHOPGM_P(SP_Y_STR, stepperY.get_pwm_thrs());
      #endif
      #if Z_HAS_STEALTHCHOP
        SERIAL_ECHOPGM_P(SP_Z_STR, stepperZ.get_pwm_thrs());
      #endif
      SERIAL_EOL();
    #endif

    #if X2_HAS_STEALTHCHOP || Y2_HAS_STEALTHCHOP || Z2_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOPGM(" I2");
      #if X2_HAS_STEALTHCHOP
        SERIAL_ECHOPGM_P(SP_X_STR, stepperX2.get_pwm_thrs());
      #endif
      #if Y2_HAS_STEALTHCHOP
        SERIAL_ECHOPGM_P(SP_Y_STR, stepperY2.get_pwm_thrs());
      #endif
      #if Z2_HAS_STEALTHCHOP
        SERIAL_ECHOPGM_P(SP_Z_STR, stepperZ2.get_pwm_thrs());
      #endif
      SERIAL_EOL();
    #endif

    #if Z3_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM(" I3 Z", stepperZ3.get_pwm_thrs());
    #endif

    #if Z4_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM(" I4 Z", stepperZ4.get_pwm_thrs());
    #endif

    #if I_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM_P(SP_I_STR, stepperI.get_pwm_thrs());
    #endif
    #if J_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM_P(SP_J_STR, stepperJ.get_pwm_thrs());
    #endif
    #if K_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM_P(SP_K_STR, stepperK.get_pwm_thrs());
    #endif
    #if U_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM_P(SP_U_STR, stepperU.get_pwm_thrs());
    #endif
    #if V_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM_P(SP_V_STR, stepperV.get_pwm_thrs());
    #endif
    #if W_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM_P(SP_W_STR, stepperW.get_pwm_thrs());
    #endif

    #if E0_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM(" T0 E", stepperE0.get_pwm_thrs());
    #endif
    #if E1_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM(" T1 E", stepperE1.get_pwm_thrs());
    #endif
    #if E2_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM(" T2 E", stepperE2.get_pwm_thrs());
    #endif
    #if E3_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM(" T3 E", stepperE3.get_pwm_thrs());
    #endif
    #if E4_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM(" T4 E", stepperE4.get_pwm_thrs());
    #endif
    #if E5_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM(" T5 E", stepperE5.get_pwm_thrs());
    #endif
    #if E6_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM(" T6 E", stepperE6.get_pwm_thrs());
    #endif
    #if E7_HAS_STEALTHCHOP
      say_M913(forReplay);
      SERIAL_ECHOLNPGM(" T7 E", stepperE7.get_pwm_thrs());
    #endif
    SERIAL_EOL();
  }

#endif // HYBRID_THRESHOLD

#if USE_SENSORLESS

  template<typename TMC>
  static void tmc_print_sgt(TMC &st) {
    st.printLabel();
    SERIAL_ECHOPGM(" homing sensitivity: ");
    SERIAL_PRINTLN(st.homing_threshold(), PrintBase::Dec);
  }

  /**
   * M914: Set StallGuard sensitivity.
   */
  void GcodeSuite::M914() {
    bool report = true;
    const uint8_t index = parser.byteval('I');
    LOOP_NUM_AXES(i) if (parser.seen(AXIS_CHAR(i))) {
      const int16_t value = parser.value_int();
      report = false;
      switch (i) {
        #if X_SENSORLESS
          case X_AXIS:
            if (index < 2) stepperX.homing_threshold(value);
            TERN_(X2_SENSORLESS, if (!index || index == 2) stepperX2.homing_threshold(value));
            break;
        #endif
        #if Y_SENSORLESS
          case Y_AXIS:
            if (index < 2) stepperY.homing_threshold(value);
            TERN_(Y2_SENSORLESS, if (!index || index == 2) stepperY2.homing_threshold(value));
            break;
        #endif
        #if Z_SENSORLESS
          case Z_AXIS:
            if (index < 2) stepperZ.homing_threshold(value);
            TERN_(Z2_SENSORLESS, if (!index || index == 2) stepperZ2.homing_threshold(value));
            TERN_(Z3_SENSORLESS, if (!index || index == 3) stepperZ3.homing_threshold(value));
            TERN_(Z4_SENSORLESS, if (!index || index == 4) stepperZ4.homing_threshold(value));
            break;
        #endif
        #if I_SENSORLESS
          case I_AXIS: stepperI.homing_threshold(value); break;
        #endif
        #if J_SENSORLESS
          case J_AXIS: stepperJ.homing_threshold(value); break;
        #endif
        #if K_SENSORLESS
          case K_AXIS: stepperK.homing_threshold(value); break;
        #endif
        #if U_SENSORLESS && AXIS_HAS_STALLGUARD(U)
          case U_AXIS: stepperU.homing_threshold(value); break;
        #endif
        #if V_SENSORLESS && AXIS_HAS_STALLGUARD(V)
          case V_AXIS: stepperV.homing_threshold(value); break;
        #endif
        #if W_SENSORLESS && AXIS_HAS_STALLGUARD(W)
          case W_AXIS: stepperW.homing_threshold(value); break;
        #endif
      }
    }

    if (report) {
      TERN_(X_SENSORLESS, tmc_print_sgt(stepperX));
      TERN_(X2_SENSORLESS, tmc_print_sgt(stepperX2));
      TERN_(Y_SENSORLESS, tmc_print_sgt(stepperY));
      TERN_(Y2_SENSORLESS, tmc_print_sgt(stepperY2));
      TERN_(Z_SENSORLESS, tmc_print_sgt(stepperZ));
      TERN_(Z2_SENSORLESS, tmc_print_sgt(stepperZ2));
      TERN_(Z3_SENSORLESS, tmc_print_sgt(stepperZ3));
      TERN_(Z4_SENSORLESS, tmc_print_sgt(stepperZ4));
      TERN_(I_SENSORLESS, tmc_print_sgt(stepperI));
      TERN_(J_SENSORLESS, tmc_print_sgt(stepperJ));
      TERN_(K_SENSORLESS, tmc_print_sgt(stepperK));
      TERN_(U_SENSORLESS, tmc_print_sgt(stepperU));
      TERN_(V_SENSORLESS, tmc_print_sgt(stepperV));
      TERN_(W_SENSORLESS, tmc_print_sgt(stepperW));
    }
  }

  void GcodeSuite::M914_report(const bool forReplay/*=true*/) {
    TERN_(MARLIN_SMALL_BUILD, return);

    report_heading(forReplay, F(STR_STALLGUARD_THRESHOLD));

    auto say_M914 = [](const bool forReplay) {
      report_echo_start(forReplay);
      SERIAL_ECHOPGM("  M914");
    };

    #if X_SENSORLESS || Y_SENSORLESS || Z_SENSORLESS
      say_M914(forReplay);
      #if X_SENSORLESS
        SERIAL_ECHOPGM_P(SP_X_STR, stepperX.homing_threshold());
      #endif
      #if Y_SENSORLESS
        SERIAL_ECHOPGM_P(SP_Y_STR, stepperY.homing_threshold());
      #endif
      #if Z_SENSORLESS
        SERIAL_ECHOPGM_P(SP_Z_STR, stepperZ.homing_threshold());
      #endif
      SERIAL_EOL();
    #endif

    #if X2_SENSORLESS || Y2_SENSORLESS || Z2_SENSORLESS
      say_M914(forReplay);
      SERIAL_ECHOPGM(" I2");
      #if X2_SENSORLESS
        SERIAL_ECHOPGM_P(SP_X_STR, stepperX2.homing_threshold());
      #endif
      #if Y2_SENSORLESS
        SERIAL_ECHOPGM_P(SP_Y_STR, stepperY2.homing_threshold());
      #endif
      #if Z2_SENSORLESS
        SERIAL_ECHOPGM_P(SP_Z_STR, stepperZ2.homing_threshold());
      #endif
      SERIAL_EOL();
    #endif

    #if Z3_SENSORLESS
      say_M914(forReplay);
      SERIAL_ECHOLNPGM(" I3 Z", stepperZ3.homing_threshold());
    #endif

    #if Z4_SENSORLESS
      say_M914(forReplay);
      SERIAL_ECHOLNPGM(" I4 Z", stepperZ4.homing_threshold());
    #endif

    #if I_SENSORLESS
      say_M914(forReplay);
      SERIAL_ECHOLNPGM_P(SP_I_STR, stepperI.homing_threshold());
    #endif
    #if J_SENSORLESS
      say_M914(forReplay);
      SERIAL_ECHOLNPGM_P(SP_J_STR, stepperJ.homing_threshold());
    #endif
    #if K_SENSORLESS
      say_M914(forReplay);
      SERIAL_ECHOLNPGM_P(SP_K_STR, stepperK.homing_threshold());
    #endif
    #if U_SENSORLESS
      say_M914(forReplay);
      SERIAL_ECHOLNPGM_P(SP_U_STR, stepperU.homing_threshold());
    #endif
    #if V_SENSORLESS
      say_M914(forReplay);
      SERIAL_ECHOLNPGM_P(SP_V_STR, stepperV.homing_threshold());
    #endif
    #if W_SENSORLESS
      say_M914(forReplay);
      SERIAL_ECHOLNPGM_P(SP_W_STR, stepperW.homing_threshold());
    #endif
  }

#endif // USE_SENSORLESS

#endif // HAS_TRINAMIC_CONFIG
