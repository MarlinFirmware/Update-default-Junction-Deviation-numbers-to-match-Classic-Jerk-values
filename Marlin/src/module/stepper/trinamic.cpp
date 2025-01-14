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

/**
 * stepper/trinamic.cpp
 * Stepper driver indirection for Trinamic
 */

#include "../../inc/MarlinConfig.h"

#if HAS_TRINAMIC_CONFIG

#include "trinamic.h"
#include "../stepper.h"

#include <HardwareSerial.h>
#include <SPI.h>

enum StealthIndex : uint8_t {
  LOGICAL_AXIS_LIST(STEALTH_AXIS_E, STEALTH_AXIS_X, STEALTH_AXIS_Y, STEALTH_AXIS_Z, STEALTH_AXIS_I, STEALTH_AXIS_J, STEALTH_AXIS_K, STEALTH_AXIS_U, STEALTH_AXIS_V, STEALTH_AXIS_W)
};
#define TMC_INIT(ST, STEALTH_INDEX) tmc_init(stepper##ST, ST##_CURRENT, ST##_MICROSTEPS, ST##_HYBRID_THRESHOLD, stealthchop_by_axis[STEALTH_INDEX], chopper_timing_##ST, ST##_INTERPOLATE, ST##_HOLD_MULTIPLIER)

//   IC = TMC model number
//   ST = Stepper object letter
//   L  = Label characters
//   AI = Axis Enum Index
// SWHW = SW/SH UART selection
#if ENABLED(TMC_USE_SW_SPI)
  #define __TMC_SPI_DEFINE(IC, ST, L, AI) TMCMarlin<IC##Stepper, L, AI> stepper##ST(ST##_CS_PIN, float(ST##_RSENSE), TMC_SPI_MOSI, TMC_SPI_MISO, TMC_SPI_SCK, ST##_CHAIN_POS)
#else
  #define __TMC_SPI_DEFINE(IC, ST, L, AI) TMCMarlin<IC##Stepper, L, AI> stepper##ST(ST##_CS_PIN, float(ST##_RSENSE), ST##_CHAIN_POS)
#endif

#if ENABLED(TMC_SERIAL_MULTIPLEXER)
  #define TMC_UART_HW_DEFINE(IC, ST, L, AI) TMCMarlin<IC##Stepper, L, AI> stepper##ST(&ST##_HARDWARE_SERIAL, float(ST##_RSENSE), ST##_SLAVE_ADDRESS, SERIAL_MUL_PIN1, SERIAL_MUL_PIN2)
#else
  #define TMC_UART_HW_DEFINE(IC, ST, L, AI) TMCMarlin<IC##Stepper, L, AI> stepper##ST(&ST##_HARDWARE_SERIAL, float(ST##_RSENSE), ST##_SLAVE_ADDRESS)
#endif

#define TMC_UART_SW_DEFINE(IC, ST, L, AI) TMCMarlin<IC##Stepper, L, AI> stepper##ST(ST##_SERIAL_RX_PIN, ST##_SERIAL_TX_PIN, float(ST##_RSENSE), ST##_SLAVE_ADDRESS)

#define _TMC_SPI_DEFINE(IC, ST, AI) __TMC_SPI_DEFINE(IC, ST, TMC_##ST##_LABEL, AI)
#define TMC_SPI_DEFINE(ST, AI) _TMC_SPI_DEFINE(ST##_DRIVER_TYPE, ST, AI##_AXIS)

#define _TMC_UART_DEFINE(SWHW, IC, ST, AI) TMC_UART_##SWHW##_DEFINE(IC, ST, TMC_##ST##_LABEL, AI)
#define TMC_UART_DEFINE(SWHW, ST, AI) _TMC_UART_DEFINE(SWHW, ST##_DRIVER_TYPE, ST, AI##_AXIS)

#if ENABLED(DISTINCT_E_FACTORS)
  #define TMC_SPI_DEFINE_E(AI) TMC_SPI_DEFINE(E##AI, E##AI)
  #define TMC_UART_DEFINE_E(SWHW, AI) TMC_UART_DEFINE(SWHW, E##AI, E##AI)
#else
  #define TMC_SPI_DEFINE_E(AI) TMC_SPI_DEFINE(E##AI, E)
  #define TMC_UART_DEFINE_E(SWHW, AI) TMC_UART_DEFINE(SWHW, E##AI, E)
#endif

// Stepper objects of TMC2130/TMC2160/TMC2660/TMC5130/TMC5160 steppers used
#if AXIS_HAS_SPI(X)
  TMC_SPI_DEFINE(X, X);
#endif
#if AXIS_HAS_SPI(X2)
  TMC_SPI_DEFINE(X2, X);
#endif
#if AXIS_HAS_SPI(Y)
  TMC_SPI_DEFINE(Y, Y);
#endif
#if AXIS_HAS_SPI(Y2)
  TMC_SPI_DEFINE(Y2, Y);
#endif
#if AXIS_HAS_SPI(Z)
  TMC_SPI_DEFINE(Z, Z);
#endif
#if AXIS_HAS_SPI(Z2)
  TMC_SPI_DEFINE(Z2, Z);
#endif
#if AXIS_HAS_SPI(Z3)
  TMC_SPI_DEFINE(Z3, Z);
#endif
#if AXIS_HAS_SPI(Z4)
  TMC_SPI_DEFINE(Z4, Z);
#endif
#if AXIS_HAS_SPI(I)
  TMC_SPI_DEFINE(I, I);
#endif
#if AXIS_HAS_SPI(J)
  TMC_SPI_DEFINE(J, J);
#endif
#if AXIS_HAS_SPI(K)
  TMC_SPI_DEFINE(K, K);
#endif
#if AXIS_HAS_SPI(U)
  TMC_SPI_DEFINE(U, U);
#endif
#if AXIS_HAS_SPI(V)
  TMC_SPI_DEFINE(V, V);
#endif
#if AXIS_HAS_SPI(W)
  TMC_SPI_DEFINE(W, W);
#endif
#if AXIS_HAS_SPI(E0)
  TMC_SPI_DEFINE_E(0);
#endif
#if AXIS_HAS_SPI(E1)
  TMC_SPI_DEFINE_E(1);
#endif
#if AXIS_HAS_SPI(E2)
  TMC_SPI_DEFINE_E(2);
#endif
#if AXIS_HAS_SPI(E3)
  TMC_SPI_DEFINE_E(3);
#endif
#if AXIS_HAS_SPI(E4)
  TMC_SPI_DEFINE_E(4);
#endif
#if AXIS_HAS_SPI(E5)
  TMC_SPI_DEFINE_E(5);
#endif
#if AXIS_HAS_SPI(E6)
  TMC_SPI_DEFINE_E(6);
#endif
#if AXIS_HAS_SPI(E7)
  TMC_SPI_DEFINE_E(7);
#endif

#ifndef TMC_BAUD_RATE
  // Reduce baud rate for boards not already overriding TMC_BAUD_RATE for software serial.
  // Testing has shown that 115200 is not 100% reliable on AVR platforms, occasionally
  // failing to read status properly. 32-bit platforms typically define an even lower
  // TMC_BAUD_RATE, due to differences in how SoftwareSerial libraries work on different
  // platforms.
  #define TMC_BAUD_RATE TERN(HAS_TMC_SW_SERIAL, 57600, 115200)
#endif

#ifndef TMC_X_BAUD_RATE
  #define TMC_X_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_X2_BAUD_RATE
  #define TMC_X2_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_Y_BAUD_RATE
  #define TMC_Y_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_Y2_BAUD_RATE
  #define TMC_Y2_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_Z_BAUD_RATE
  #define TMC_Z_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_Z2_BAUD_RATE
  #define TMC_Z2_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_Z3_BAUD_RATE
  #define TMC_Z3_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_Z4_BAUD_RATE
  #define TMC_Z4_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_I_BAUD_RATE
  #define TMC_I_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_J_BAUD_RATE
  #define TMC_J_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_K_BAUD_RATE
  #define TMC_K_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_U_BAUD_RATE
  #define TMC_U_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_V_BAUD_RATE
  #define TMC_V_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_W_BAUD_RATE
  #define TMC_W_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_E0_BAUD_RATE
  #define TMC_E0_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_E1_BAUD_RATE
  #define TMC_E1_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_E2_BAUD_RATE
  #define TMC_E2_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_E3_BAUD_RATE
  #define TMC_E3_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_E4_BAUD_RATE
  #define TMC_E4_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_E5_BAUD_RATE
  #define TMC_E5_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_E6_BAUD_RATE
  #define TMC_E6_BAUD_RATE TMC_BAUD_RATE
#endif
#ifndef TMC_E7_BAUD_RATE
  #define TMC_E7_BAUD_RATE TMC_BAUD_RATE
#endif

#if HAS_DRIVER(TMC2130)
  template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
  void tmc_init(TMCMarlin<TMC2130Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &st, const uint16_t mA, const uint16_t microsteps, const uint32_t hyb_thrs, const bool stealth, const chopper_timing_t &chop_init, const bool interpolate, float hold_multiplier) {
    st.begin();

    CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01;
    chopconf.toff = chop_init.toff;
    chopconf.intpol = interpolate;
    chopconf.hend = chop_init.hend + 3;
    chopconf.hstrt = chop_init.hstrt - 1;
    TERN_(EDGE_STEPPING, chopconf.dedge = true);
    st.CHOPCONF(chopconf.sr);

    st.rms_current(mA, hold_multiplier);
    st.microsteps(microsteps);
    st.iholddelay(10);
    st.TPOWERDOWN(128); // ~2s until driver lowers to hold current

    st.en_pwm_mode(stealth);
    st.stored.stealthChop_enabled = stealth;

    PWMCONF_t pwmconf{0};
    pwmconf.pwm_freq = 0b01; // f_pwm = 2/683 f_clk
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_grad = 5;
    pwmconf.pwm_ampl = 180;
    st.PWMCONF(pwmconf.sr);

    TERN(HYBRID_THRESHOLD, st.set_pwm_thrs(hyb_thrs), UNUSED(hyb_thrs));

    st.GSTAT(); // Clear GSTAT
  }
#endif // TMC2130

#if HAS_DRIVER(TMC2160)
  template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
  void tmc_init(TMCMarlin<TMC2160Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &st, const uint16_t mA, const uint16_t microsteps, const uint32_t hyb_thrs, const bool stealth, const chopper_timing_t &chop_init, const bool interpolate, float hold_multiplier) {
    st.begin();

    CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01;
    chopconf.toff = chop_init.toff;
    chopconf.intpol = interpolate;
    chopconf.hend = chop_init.hend + 3;
    chopconf.hstrt = chop_init.hstrt - 1;
    TERN_(EDGE_STEPPING, chopconf.dedge = true);
    st.CHOPCONF(chopconf.sr);

    st.rms_current(mA, hold_multiplier);
    st.microsteps(microsteps);
    st.iholddelay(10);
    st.TPOWERDOWN(128); // ~2s until driver lowers to hold current

    st.en_pwm_mode(stealth);
    st.stored.stealthChop_enabled = stealth;

    TMC2160_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    st.PWMCONF(pwmconf.sr);

    TERN(HYBRID_THRESHOLD, st.set_pwm_thrs(hyb_thrs), UNUSED(hyb_thrs));

    st.GSTAT(); // Clear GSTAT
  }
#endif // TMC2160

//
// TMC2208/2209 Driver objects and inits
//
#if HAS_TMC_UART
  #if AXIS_HAS_UART(X)
    #ifdef X_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, X, X);
      #define X_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE(SW, X, X);
      #define X_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(X2)
    #ifdef X2_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, X2, X);
      #define X2_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE(SW, X2, X);
      #define X2_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(Y)
    #ifdef Y_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, Y, Y);
      #define Y_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE(SW, Y, Y);
      #define Y_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(Y2)
    #ifdef Y2_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, Y2, Y);
      #define Y2_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE(SW, Y2, Y);
      #define Y2_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(Z)
    #ifdef Z_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, Z, Z);
      #define Z_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE(SW, Z, Z);
      #define Z_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(Z2)
    #ifdef Z2_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, Z2, Z);
      #define Z2_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE(SW, Z2, Z);
      #define Z2_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(Z3)
    #ifdef Z3_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, Z3, Z);
      #define Z3_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE(SW, Z3, Z);
      #define Z3_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(Z4)
    #ifdef Z4_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, Z4, Z);
      #define Z4_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE(SW, Z4, Z);
      #define Z4_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(I)
    #ifdef I_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, I, I);
      #define I_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE(SW, I, I);
      #define I_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(J)
    #ifdef J_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, J, J);
      #define J_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE(SW, J, J);
      #define J_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(K)
    #ifdef K_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, K, K);
      #define K_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE(SW, K, K);
      #define K_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(U)
    #ifdef U_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, U, U);
      #define U_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE(SW, U, U);
      #define U_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(V)
    #ifdef V_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, V, V);
    #else
      TMC_UART_DEFINE(SW, V, V);
      #define V_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(W)
    #ifdef W_HARDWARE_SERIAL
      TMC_UART_DEFINE(HW, W, W);
      #define W_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE(SW, W, W);
      #define W_HAS_SW_SERIAL 1
    #endif
  #endif

  #if AXIS_HAS_UART(E0)
    #ifdef E0_HARDWARE_SERIAL
      TMC_UART_DEFINE_E(HW, 0);
      #define E0_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE_E(SW, 0);
      #define E0_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(E1)
    #ifdef E1_HARDWARE_SERIAL
      TMC_UART_DEFINE_E(HW, 1);
      #define E1_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE_E(SW, 1);
      #define E1_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(E2)
    #ifdef E2_HARDWARE_SERIAL
      TMC_UART_DEFINE_E(HW, 2);
      #define E2_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE_E(SW, 2);
      #define E2_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(E3)
    #ifdef E3_HARDWARE_SERIAL
      TMC_UART_DEFINE_E(HW, 3);
      #define E3_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE_E(SW, 3);
      #define E3_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(E4)
    #ifdef E4_HARDWARE_SERIAL
      TMC_UART_DEFINE_E(HW, 4);
      #define E4_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE_E(SW, 4);
      #define E4_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(E5)
    #ifdef E5_HARDWARE_SERIAL
      TMC_UART_DEFINE_E(HW, 5);
      #define E5_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE_E(SW, 5);
      #define E5_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(E6)
    #ifdef E6_HARDWARE_SERIAL
      TMC_UART_DEFINE_E(HW, 6);
      #define E6_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE_E(SW, 6);
      #define E6_HAS_SW_SERIAL 1
    #endif
  #endif
  #if AXIS_HAS_UART(E7)
    #ifdef E7_HARDWARE_SERIAL
      TMC_UART_DEFINE_E(HW, 7);
      #define E7_HAS_HW_SERIAL 1
    #else
      TMC_UART_DEFINE_E(SW, 7);
      #define E7_HAS_SW_SERIAL 1
    #endif
  #endif

  #define _EN_ITEM(N) , E##N
  enum TMCAxis : uint8_t { MAIN_AXIS_NAMES_ X2, Y2, Z2, Z3, Z4 REPEAT(EXTRUDERS, _EN_ITEM), TOTAL };
  #undef _EN_ITEM

  void tmc_serial_begin() {
    #if HAS_TMC_HW_SERIAL
      struct {
        const void *ptr[TMCAxis::TOTAL];
        bool began(const TMCAxis a, const void * const p) {
          for (uint8_t i = 0; i < a; ++i) if (p == ptr[i]) return true;
          ptr[a] = p; return false;
        };
      } sp_helper;

      #define HW_SERIAL_BEGIN(A) do{ if (!sp_helper.began(TMCAxis::A, &A##_HARDWARE_SERIAL)) \
                                          A##_HARDWARE_SERIAL.begin(TMC_##A##_BAUD_RATE); }while(0)
    #endif

    #if AXIS_HAS_UART(X)
      #ifdef X_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(X);
      #else
        stepperX.beginSerial(TMC_X_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(X2)
      #ifdef X2_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(X2);
      #else
        stepperX2.beginSerial(TMC_X2_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(Y)
      #ifdef Y_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(Y);
      #else
        stepperY.beginSerial(TMC_Y_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(Y2)
      #ifdef Y2_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(Y2);
      #else
        stepperY2.beginSerial(TMC_Y2_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(Z)
      #ifdef Z_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(Z);
      #else
        stepperZ.beginSerial(TMC_Z_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(Z2)
      #ifdef Z2_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(Z2);
      #else
        stepperZ2.beginSerial(TMC_Z2_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(Z3)
      #ifdef Z3_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(Z3);
      #else
        stepperZ3.beginSerial(TMC_Z3_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(Z4)
      #ifdef Z4_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(Z4);
      #else
        stepperZ4.beginSerial(TMC_Z4_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(I)
      #ifdef I_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(I);
      #else
        stepperI.beginSerial(TMC_I_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(J)
      #ifdef J_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(J);
      #else
        stepperJ.beginSerial(TMC_J_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(K)
      #ifdef K_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(K);
      #else
        stepperK.beginSerial(TMC_K_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(U)
      #ifdef U_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(U);
      #else
        stepperU.beginSerial(TMC_U_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(V)
      #ifdef V_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(V);
      #else
        stepperV.beginSerial(TMC_V_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(W)
      #ifdef W_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(W);
      #else
        stepperW.beginSerial(TMC_W_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(E0)
      #ifdef E0_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E0);
      #else
        stepperE0.beginSerial(TMC_E0_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(E1)
      #ifdef E1_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E1);
      #else
        stepperE1.beginSerial(TMC_E1_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(E2)
      #ifdef E2_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E2);
      #else
        stepperE2.beginSerial(TMC_E2_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(E3)
      #ifdef E3_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E3);
      #else
        stepperE3.beginSerial(TMC_E3_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(E4)
      #ifdef E4_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E4);
      #else
        stepperE4.beginSerial(TMC_E4_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(E5)
      #ifdef E5_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E5);
      #else
        stepperE5.beginSerial(TMC_E5_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(E6)
      #ifdef E6_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E6);
      #else
        stepperE6.beginSerial(TMC_E6_BAUD_RATE);
      #endif
    #endif
    #if AXIS_HAS_UART(E7)
      #ifdef E7_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E7);
      #else
        stepperE7.beginSerial(TMC_E7_BAUD_RATE);
      #endif
    #endif
  }
#endif

#if HAS_DRIVER(TMC2208)
  template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
  void tmc_init(TMCMarlin<TMC2208Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &st, const uint16_t mA, const uint16_t microsteps, const uint32_t hyb_thrs, const bool stealth, const chopper_timing_t &chop_init, const bool interpolate, float hold_multiplier) {
    TMC2208_n::GCONF_t gconf{0};
    gconf.pdn_disable = true; // Use UART
    gconf.mstep_reg_select = true; // Select microsteps with UART
    gconf.i_scale_analog = false;
    gconf.en_spreadcycle = !stealth;
    st.GCONF(gconf.sr);
    st.stored.stealthChop_enabled = stealth;

    TMC2208_n::CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01; // blank_time = 24
    chopconf.toff = chop_init.toff;
    chopconf.intpol = interpolate;
    chopconf.hend = chop_init.hend + 3;
    chopconf.hstrt = chop_init.hstrt - 1;
    TERN_(EDGE_STEPPING, chopconf.dedge = true);
    st.CHOPCONF(chopconf.sr);

    st.rms_current(mA, hold_multiplier);
    st.microsteps(microsteps);
    st.iholddelay(10);
    st.TPOWERDOWN(128); // ~2s until driver lowers to hold current

    TMC2208_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    st.PWMCONF(pwmconf.sr);

    TERN(HYBRID_THRESHOLD, st.set_pwm_thrs(hyb_thrs), UNUSED(hyb_thrs));

    st.GSTAT(0b111); // Clear
    delay(200);
  }
#endif // TMC2208

#if HAS_DRIVER(TMC2209)
  template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
  void tmc_init(TMCMarlin<TMC2209Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &st, const uint16_t mA, const uint16_t microsteps, const uint32_t hyb_thrs, const bool stealth, const chopper_timing_t &chop_init, const bool interpolate, float hold_multiplier) {
    TMC2208_n::GCONF_t gconf{0};
    gconf.pdn_disable = true; // Use UART
    gconf.mstep_reg_select = true; // Select microsteps with UART
    gconf.i_scale_analog = false;
    gconf.en_spreadcycle = !stealth;
    st.GCONF(gconf.sr);
    st.stored.stealthChop_enabled = stealth;

    TMC2208_n::CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01; // blank_time = 24
    chopconf.toff = chop_init.toff;
    chopconf.intpol = interpolate;
    chopconf.hend = chop_init.hend + 3;
    chopconf.hstrt = chop_init.hstrt - 1;
    TERN_(EDGE_STEPPING, chopconf.dedge = true);
    st.CHOPCONF(chopconf.sr);

    st.rms_current(mA, hold_multiplier);
    st.microsteps(microsteps);
    st.iholddelay(10);
    st.TPOWERDOWN(128); // ~2s until driver lowers to hold current

    TMC2208_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    st.PWMCONF(pwmconf.sr);

    TERN(HYBRID_THRESHOLD, st.set_pwm_thrs(hyb_thrs), UNUSED(hyb_thrs));

    st.GSTAT(0b111); // Clear
    delay(200);
  }
#endif // TMC2209

#if HAS_DRIVER(TMC2660)
  template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
  void tmc_init(TMCMarlin<TMC2660Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &st, const uint16_t mA, const uint16_t microsteps, const uint32_t, const bool, const chopper_timing_t &chop_init, const bool interpolate, float hold_multiplier) {
    st.begin();

    TMC2660_n::CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01;
    chopconf.toff = chop_init.toff;
    chopconf.hend = chop_init.hend + 3;
    chopconf.hstrt = chop_init.hstrt - 1;
    st.CHOPCONF(chopconf.sr);

    st.sdoff(0);
    st.rms_current(mA);
    st.microsteps(microsteps);
    TERN_(EDGE_STEPPING, st.dedge(true));
    st.intpol(interpolate);
    st.diss2g(true); // Disable short to ground protection. Too many false readings?
    TERN_(TMC_DEBUG, st.rdsel(0b01));
  }
#endif // TMC2660

#if HAS_DRIVER(TMC5130)
  template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
  void tmc_init(TMCMarlin<TMC5130Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &st, const uint16_t mA, const uint16_t microsteps, const uint32_t hyb_thrs, const bool stealth, const chopper_timing_t &chop_init, const bool interpolate, float hold_multiplier) {
    st.begin();

    CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01;
    chopconf.toff = chop_init.toff;
    chopconf.intpol = interpolate;
    chopconf.hend = chop_init.hend + 3;
    chopconf.hstrt = chop_init.hstrt - 1;
    TERN_(EDGE_STEPPING, chopconf.dedge = true);
    st.CHOPCONF(chopconf.sr);

    st.rms_current(mA, hold_multiplier);
    st.microsteps(microsteps);
    st.iholddelay(10);
    st.TPOWERDOWN(128); // ~2s until driver lowers to hold current

    st.en_pwm_mode(stealth);
    st.stored.stealthChop_enabled = stealth;

    PWMCONF_t pwmconf{0};
    pwmconf.pwm_freq = 0b01; // f_pwm = 2/683 f_clk
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_grad = 5;
    pwmconf.pwm_ampl = 180;
    st.PWMCONF(pwmconf.sr);

    TERN(HYBRID_THRESHOLD, st.set_pwm_thrs(hyb_thrs), UNUSED(hyb_thrs));

    st.GSTAT(); // Clear GSTAT
  }
#endif // TMC5130

#if HAS_DRIVER(TMC5160)
  template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
  void tmc_init(TMCMarlin<TMC5160Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &st, const uint16_t mA, const uint16_t microsteps, const uint32_t hyb_thrs, const bool stealth, const chopper_timing_t &chop_init, const bool interpolate, float hold_multiplier) {
    st.begin();

    CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01;
    chopconf.toff = chop_init.toff;
    chopconf.intpol = interpolate;
    chopconf.hend = chop_init.hend + 3;
    chopconf.hstrt = chop_init.hstrt - 1;
    TERN_(EDGE_STEPPING, chopconf.dedge = true);
    st.CHOPCONF(chopconf.sr);

    st.rms_current(mA, hold_multiplier);
    st.microsteps(microsteps);
    st.iholddelay(10);
    st.TPOWERDOWN(128); // ~2s until driver lowers to hold current

    st.en_pwm_mode(stealth);
    st.stored.stealthChop_enabled = stealth;

    TMC2160_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    st.PWMCONF(pwmconf.sr);

    TERN(HYBRID_THRESHOLD, st.set_pwm_thrs(hyb_thrs), UNUSED(hyb_thrs));
    st.GSTAT(); // Clear GSTAT
  }
#endif // TMC5160

void restore_trinamic_drivers() {
  #if AXIS_IS_TMC(X)
    stepperX.push();
  #endif
  #if AXIS_IS_TMC(X2)
    stepperX2.push();
  #endif
  #if AXIS_IS_TMC(Y)
    stepperY.push();
  #endif
  #if AXIS_IS_TMC(Y2)
    stepperY2.push();
  #endif
  #if AXIS_IS_TMC(Z)
    stepperZ.push();
  #endif
  #if AXIS_IS_TMC(Z2)
    stepperZ2.push();
  #endif
  #if AXIS_IS_TMC(Z3)
    stepperZ3.push();
  #endif
  #if AXIS_IS_TMC(Z4)
    stepperZ4.push();
  #endif
  #if AXIS_IS_TMC(I)
    stepperI.push();
  #endif
  #if AXIS_IS_TMC(J)
    stepperJ.push();
  #endif
  #if AXIS_IS_TMC(K)
    stepperK.push();
  #endif
  #if AXIS_IS_TMC(U)
    stepperU.push();
  #endif
  #if AXIS_IS_TMC(V)
    stepperV.push();
  #endif
  #if AXIS_IS_TMC(W)
    stepperW.push();
  #endif
  #if AXIS_IS_TMC(E0)
    stepperE0.push();
  #endif
  #if AXIS_IS_TMC(E1)
    stepperE1.push();
  #endif
  #if AXIS_IS_TMC(E2)
    stepperE2.push();
  #endif
  #if AXIS_IS_TMC(E3)
    stepperE3.push();
  #endif
  #if AXIS_IS_TMC(E4)
    stepperE4.push();
  #endif
  #if AXIS_IS_TMC(E5)
    stepperE5.push();
  #endif
  #if AXIS_IS_TMC(E6)
    stepperE6.push();
  #endif
  #if AXIS_IS_TMC(E7)
    stepperE7.push();
  #endif
}

void reset_trinamic_drivers() {
  static constexpr bool stealthchop_by_axis[] = LOGICAL_AXIS_ARRAY(
    ENABLED(STEALTHCHOP_E),
    ENABLED(STEALTHCHOP_XY), ENABLED(STEALTHCHOP_XY), ENABLED(STEALTHCHOP_Z),
    ENABLED(STEALTHCHOP_I), ENABLED(STEALTHCHOP_J), ENABLED(STEALTHCHOP_K),
    ENABLED(STEALTHCHOP_U), ENABLED(STEALTHCHOP_V), ENABLED(STEALTHCHOP_W)
  );

  #if AXIS_IS_TMC(X)
    TMC_INIT(X, STEALTH_AXIS_X);
  #endif
  #if AXIS_IS_TMC(X2)
    TMC_INIT(X2, STEALTH_AXIS_X);
  #endif
  #if AXIS_IS_TMC(Y)
    TMC_INIT(Y, STEALTH_AXIS_Y);
  #endif
  #if AXIS_IS_TMC(Y2)
    TMC_INIT(Y2, STEALTH_AXIS_Y);
  #endif
  #if AXIS_IS_TMC(Z)
    TMC_INIT(Z, STEALTH_AXIS_Z);
  #endif
  #if AXIS_IS_TMC(Z2)
    TMC_INIT(Z2, STEALTH_AXIS_Z);
  #endif
  #if AXIS_IS_TMC(Z3)
    TMC_INIT(Z3, STEALTH_AXIS_Z);
  #endif
  #if AXIS_IS_TMC(Z4)
    TMC_INIT(Z4, STEALTH_AXIS_Z);
  #endif
  #if AXIS_IS_TMC(I)
    TMC_INIT(I, STEALTH_AXIS_I);
  #endif
  #if AXIS_IS_TMC(J)
    TMC_INIT(J, STEALTH_AXIS_J);
  #endif
  #if AXIS_IS_TMC(K)
    TMC_INIT(K, STEALTH_AXIS_K);
  #endif
  #if AXIS_IS_TMC(U)
    TMC_INIT(U, STEALTH_AXIS_U);
  #endif
  #if AXIS_IS_TMC(V)
    TMC_INIT(V, STEALTH_AXIS_V);
  #endif
  #if AXIS_IS_TMC(W)
    TMC_INIT(W, STEALTH_AXIS_W);
  #endif
  #if AXIS_IS_TMC(E0)
    TMC_INIT(E0, STEALTH_AXIS_E);
  #endif
  #if AXIS_IS_TMC(E1)
    TMC_INIT(E1, STEALTH_AXIS_E);
  #endif
  #if AXIS_IS_TMC(E2)
    TMC_INIT(E2, STEALTH_AXIS_E);
  #endif
  #if AXIS_IS_TMC(E3)
    TMC_INIT(E3, STEALTH_AXIS_E);
  #endif
  #if AXIS_IS_TMC(E4)
    TMC_INIT(E4, STEALTH_AXIS_E);
  #endif
  #if AXIS_IS_TMC(E5)
    TMC_INIT(E5, STEALTH_AXIS_E);
  #endif
  #if AXIS_IS_TMC(E6)
    TMC_INIT(E6, STEALTH_AXIS_E);
  #endif
  #if AXIS_IS_TMC(E7)
    TMC_INIT(E7, STEALTH_AXIS_E);
  #endif

  #if USE_SENSORLESS
    TERN_(X_SENSORLESS, stepperX.homing_threshold(X_STALL_SENSITIVITY));
    TERN_(X2_SENSORLESS, stepperX2.homing_threshold(X2_STALL_SENSITIVITY));
    TERN_(Y_SENSORLESS, stepperY.homing_threshold(Y_STALL_SENSITIVITY));
    TERN_(Y2_SENSORLESS, stepperY2.homing_threshold(Y2_STALL_SENSITIVITY));
    TERN_(Z_SENSORLESS, stepperZ.homing_threshold(Z_STALL_SENSITIVITY));
    TERN_(Z2_SENSORLESS, stepperZ2.homing_threshold(Z2_STALL_SENSITIVITY));
    TERN_(Z3_SENSORLESS, stepperZ3.homing_threshold(Z3_STALL_SENSITIVITY));
    TERN_(Z4_SENSORLESS, stepperZ4.homing_threshold(Z4_STALL_SENSITIVITY));
    TERN_(I_SENSORLESS, stepperI.homing_threshold(I_STALL_SENSITIVITY));
    TERN_(J_SENSORLESS, stepperJ.homing_threshold(J_STALL_SENSITIVITY));
    TERN_(K_SENSORLESS, stepperK.homing_threshold(K_STALL_SENSITIVITY));
    TERN_(U_SENSORLESS, stepperU.homing_threshold(U_STALL_SENSITIVITY));
    TERN_(V_SENSORLESS, stepperV.homing_threshold(V_STALL_SENSITIVITY));
    TERN_(W_SENSORLESS, stepperW.homing_threshold(W_STALL_SENSITIVITY));
  #endif

  #ifdef TMC_ADV
    TMC_ADV()
  #endif

  stepper.apply_directions();
}

// TMC Slave Address Conflict Detection
//
// Conflict detection is performed in the following way. Similar methods are used for
// hardware and software serial, but the implementations are independent.
//
// 1. Populate a data structure with UART parameters and addresses for all possible axis.
//      If an axis is not in use, populate it with recognizable placeholder data.
// 2. For each axis in use, static_assert using a constexpr function, which counts the
//      number of matching/conflicting axis. If the value is not exactly 1, fail.

#if ANY_AXIS_HAS(HW_SERIAL)
  // Hardware serial names are compared as strings, since actually resolving them cannot occur in a constexpr.
  // Using a fixed-length character array for the port name allows this to be constexpr compatible.
  struct SanityHwSerialDetails { const char port[20]; uint32_t address; };
  #define TMC_HW_DETAIL_ARGS(A) TERN(A##_HAS_HW_SERIAL, STRINGIFY(A##_HARDWARE_SERIAL), ""), TERN0(A##_HAS_HW_SERIAL, A##_SLAVE_ADDRESS)
  #define TMC_HW_DETAIL(A) { TMC_HW_DETAIL_ARGS(A) }
  constexpr SanityHwSerialDetails sanity_tmc_hw_details[] = { MAPLIST(TMC_HW_DETAIL, ALL_AXIS_NAMES) };

  // constexpr compatible string comparison
  constexpr bool str_eq_ce(const char * a, const char * b) {
    return *a == *b && (*a == '\0' || str_eq_ce(a+1,b+1));
  }

  constexpr bool sc_hw_done(size_t start, size_t end) { return start == end; }
  constexpr bool sc_hw_skip(const char *port_name) { return !(*port_name); }
  constexpr bool sc_hw_match(const char *port_name, uint32_t address, size_t start, size_t end) {
    return !sc_hw_done(start, end) && !sc_hw_skip(port_name) && (address == sanity_tmc_hw_details[start].address && str_eq_ce(port_name, sanity_tmc_hw_details[start].port));
  }
  constexpr int count_tmc_hw_serial_matches(const char *port_name, uint32_t address, size_t start, size_t end) {
    return sc_hw_done(start, end) ? 0 : ((sc_hw_skip(port_name) ? 0 : (sc_hw_match(port_name, address, start, end) ? 1 : 0)) + count_tmc_hw_serial_matches(port_name, address, start + 1, end));
  }

  #define TMC_HWSERIAL_CONFLICT_MSG(A) STRINGIFY(A) "_SLAVE_ADDRESS conflicts with another driver using the same " STRINGIFY(A) "_HARDWARE_SERIAL"
  #define SA_NO_TMC_HW_C(A) static_assert(1 >= count_tmc_hw_serial_matches(TMC_HW_DETAIL_ARGS(A), 0, COUNT(sanity_tmc_hw_details)), TMC_HWSERIAL_CONFLICT_MSG(A));
  MAP(SA_NO_TMC_HW_C, ALL_AXIS_NAMES)
#endif

#if ANY_AXIS_HAS(SW_SERIAL)
  struct SanitySwSerialDetails { int32_t txpin; int32_t rxpin; uint32_t address; };
  #define TMC_SW_DETAIL_ARGS(A) TERN(A##_HAS_SW_SERIAL, A##_SERIAL_TX_PIN, -1), TERN(A##_HAS_SW_SERIAL, A##_SERIAL_RX_PIN, -1), TERN0(A##_HAS_SW_SERIAL, A##_SLAVE_ADDRESS)
  #define TMC_SW_DETAIL(A) { TMC_SW_DETAIL_ARGS(A) }
  constexpr SanitySwSerialDetails sanity_tmc_sw_details[] = { MAPLIST(TMC_SW_DETAIL, ALL_AXIS_NAMES) };

  constexpr bool sc_sw_done(size_t start, size_t end) { return start == end; }
  constexpr bool sc_sw_skip(int32_t txpin) { return txpin < 0; }
  constexpr bool sc_sw_match(int32_t txpin, int32_t rxpin, uint32_t address, size_t start, size_t end) {
    return !sc_sw_done(start, end) && !sc_sw_skip(txpin) && (txpin == sanity_tmc_sw_details[start].txpin || rxpin == sanity_tmc_sw_details[start].rxpin) && (address == sanity_tmc_sw_details[start].address);
  }
  constexpr int count_tmc_sw_serial_matches(int32_t txpin, int32_t rxpin, uint32_t address, size_t start, size_t end) {
    return sc_sw_done(start, end) ? 0 : ((sc_sw_skip(txpin) ? 0 : (sc_sw_match(txpin, rxpin, address, start, end) ? 1 : 0)) + count_tmc_sw_serial_matches(txpin, rxpin, address, start + 1, end));
  }

  #define TMC_SWSERIAL_CONFLICT_MSG(A) STRINGIFY(A) "_SLAVE_ADDRESS conflicts with another driver using the same " STRINGIFY(A) "_SERIAL_RX_PIN or " STRINGIFY(A) "_SERIAL_TX_PIN"
  #define SA_NO_TMC_SW_C(A) static_assert(1 >= count_tmc_sw_serial_matches(TMC_SW_DETAIL_ARGS(A), 0, COUNT(sanity_tmc_sw_details)), TMC_SWSERIAL_CONFLICT_MSG(A));
  MAP(SA_NO_TMC_SW_C, ALL_AXIS_NAMES)
#endif

#endif // HAS_TRINAMIC_CONFIG
