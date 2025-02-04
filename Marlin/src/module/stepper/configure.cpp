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

/**
 * stepper/configure.cpp
 *
 * Methods to configure stepper drivers
 * - Motor Current (SPI)
 * - Motor Current (PWM)
 * - Micro-stepping
 */

#include "../../inc/MarlinConfig.h"
#include "../stepper.h"

/**
 * Software-controlled Stepper Motor Current
 */

#if HAS_MOTOR_CURRENT_SPI

  // From Arduino DigitalPotControl example
  void Stepper::set_digipot_value_spi(const int16_t address, const int16_t value) {
    WRITE(DIGIPOTSS_PIN, LOW);  // Take the SS pin low to select the chip
    SPI.transfer(address);      // Send the address and value via SPI
    SPI.transfer(value);
    WRITE(DIGIPOTSS_PIN, HIGH); // Take the SS pin high to de-select the chip
    //delay(10);
  }

#endif // HAS_MOTOR_CURRENT_SPI

#if HAS_MOTOR_CURRENT_PWM

  void Stepper::refresh_motor_power() {
    if (!initialized) return;
    for (uint8_t i = 0; i < COUNT(motor_current_setting); ++i) {
      switch (i) {
        #if ANY_PIN(MOTOR_CURRENT_PWM_XY, MOTOR_CURRENT_PWM_X, MOTOR_CURRENT_PWM_Y, MOTOR_CURRENT_PWM_I, MOTOR_CURRENT_PWM_J, MOTOR_CURRENT_PWM_K, MOTOR_CURRENT_PWM_U, MOTOR_CURRENT_PWM_V, MOTOR_CURRENT_PWM_W)
          case 0:
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
          case 1:
        #endif
        #if HAS_MOTOR_CURRENT_PWM_E
          case 2:
        #endif
            set_digipot_current(i, motor_current_setting[i]);
        default: break;
      }
    }
  }

#endif // HAS_MOTOR_CURRENT_PWM

#if !MB(PRINTRBOARD_G2)

  #if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM

    void Stepper::set_digipot_current(const uint8_t driver, const int16_t current) {
      if (WITHIN(driver, 0, MOTOR_CURRENT_COUNT - 1))
        motor_current_setting[driver] = current; // update motor_current_setting

      if (!initialized) return;

      #if HAS_MOTOR_CURRENT_SPI

        //SERIAL_ECHOLNPGM("Digipotss current ", current);

        const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
        set_digipot_value_spi(digipot_ch[driver], current);

      #elif HAS_MOTOR_CURRENT_PWM

        #define _WRITE_CURRENT_PWM(P) hal.set_pwm_duty(pin_t(MOTOR_CURRENT_PWM_## P ##_PIN), 255L * current / (MOTOR_CURRENT_PWM_RANGE))
        switch (driver) {
          case 0:
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_X)
              _WRITE_CURRENT_PWM(X);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_Y)
              _WRITE_CURRENT_PWM(Y);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
              _WRITE_CURRENT_PWM(XY);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_I)
              _WRITE_CURRENT_PWM(I);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_J)
              _WRITE_CURRENT_PWM(J);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_K)
              _WRITE_CURRENT_PWM(K);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_U)
              _WRITE_CURRENT_PWM(U);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_V)
              _WRITE_CURRENT_PWM(V);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_W)
              _WRITE_CURRENT_PWM(W);
            #endif
            break;
          case 1:
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
              _WRITE_CURRENT_PWM(Z);
            #endif
            break;
          case 2:
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
              _WRITE_CURRENT_PWM(E);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_E0)
              _WRITE_CURRENT_PWM(E0);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_E1)
              _WRITE_CURRENT_PWM(E1);
            #endif
            break;
        }
      #endif
    }

    void Stepper::digipot_init() {

      #if HAS_MOTOR_CURRENT_SPI

        SPI.begin();
        SET_OUTPUT(DIGIPOTSS_PIN);

        for (uint8_t i = 0; i < COUNT(motor_current_setting); ++i)
          set_digipot_current(i, motor_current_setting[i]);

      #elif HAS_MOTOR_CURRENT_PWM

        #ifdef __SAM3X8E__
          #define _RESET_CURRENT_PWM_FREQ(P) NOOP
        #else
          #define _RESET_CURRENT_PWM_FREQ(P) hal.set_pwm_frequency(pin_t(P), MOTOR_CURRENT_PWM_FREQUENCY)
        #endif
        #define INIT_CURRENT_PWM(P) do{ SET_PWM(MOTOR_CURRENT_PWM_## P ##_PIN); _RESET_CURRENT_PWM_FREQ(MOTOR_CURRENT_PWM_## P ##_PIN); }while(0)

        #if PIN_EXISTS(MOTOR_CURRENT_PWM_X)
          INIT_CURRENT_PWM(X);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_Y)
          INIT_CURRENT_PWM(Y);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
          INIT_CURRENT_PWM(XY);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_I)
          INIT_CURRENT_PWM(I);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_J)
          INIT_CURRENT_PWM(J);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_K)
          INIT_CURRENT_PWM(K);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_U)
          INIT_CURRENT_PWM(U);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_V)
          INIT_CURRENT_PWM(V);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_W)
          INIT_CURRENT_PWM(W);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
          INIT_CURRENT_PWM(Z);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
          INIT_CURRENT_PWM(E);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_E0)
          INIT_CURRENT_PWM(E0);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_E1)
          INIT_CURRENT_PWM(E1);
        #endif

        refresh_motor_power();

      #endif
    }

  #endif

#else // PRINTRBOARD_G2

  #include HAL_PATH(.., fastio/G2_PWM.h)

#endif

/**
 * Software-controlled Microstepping
 */

#if HAS_MICROSTEPS

  void Stepper::microstep_init() {
    #if HAS_X_MS_PINS
      SET_OUTPUT(X_MS1_PIN); SET_OUTPUT(X_MS2_PIN);
      #if PIN_EXISTS(X_MS3)
        SET_OUTPUT(X_MS3_PIN);
      #endif
    #endif
    #if HAS_X2_MS_PINS
      SET_OUTPUT(X2_MS1_PIN); SET_OUTPUT(X2_MS2_PIN);
      #if PIN_EXISTS(X2_MS3)
        SET_OUTPUT(X2_MS3_PIN);
      #endif
    #endif
    #if HAS_Y_MS_PINS
      SET_OUTPUT(Y_MS1_PIN); SET_OUTPUT(Y_MS2_PIN);
      #if PIN_EXISTS(Y_MS3)
        SET_OUTPUT(Y_MS3_PIN);
      #endif
    #endif
    #if HAS_Y2_MS_PINS
      SET_OUTPUT(Y2_MS1_PIN); SET_OUTPUT(Y2_MS2_PIN);
      #if PIN_EXISTS(Y2_MS3)
        SET_OUTPUT(Y2_MS3_PIN);
      #endif
    #endif
    #if HAS_Z_MS_PINS
      SET_OUTPUT(Z_MS1_PIN); SET_OUTPUT(Z_MS2_PIN);
      #if PIN_EXISTS(Z_MS3)
        SET_OUTPUT(Z_MS3_PIN);
      #endif
    #endif
    #if HAS_Z2_MS_PINS
      SET_OUTPUT(Z2_MS1_PIN); SET_OUTPUT(Z2_MS2_PIN);
      #if PIN_EXISTS(Z2_MS3)
        SET_OUTPUT(Z2_MS3_PIN);
      #endif
    #endif
    #if HAS_Z3_MS_PINS
      SET_OUTPUT(Z3_MS1_PIN); SET_OUTPUT(Z3_MS2_PIN);
      #if PIN_EXISTS(Z3_MS3)
        SET_OUTPUT(Z3_MS3_PIN);
      #endif
    #endif
    #if HAS_Z4_MS_PINS
      SET_OUTPUT(Z4_MS1_PIN); SET_OUTPUT(Z4_MS2_PIN);
      #if PIN_EXISTS(Z4_MS3)
        SET_OUTPUT(Z4_MS3_PIN);
      #endif
    #endif
    #if HAS_I_MS_PINS
      SET_OUTPUT(I_MS1_PIN); SET_OUTPUT(I_MS2_PIN);
      #if PIN_EXISTS(I_MS3)
        SET_OUTPUT(I_MS3_PIN);
      #endif
    #endif
    #if HAS_J_MS_PINS
      SET_OUTPUT(J_MS1_PIN); SET_OUTPUT(J_MS2_PIN);
      #if PIN_EXISTS(J_MS3)
        SET_OUTPUT(J_MS3_PIN);
      #endif
    #endif
    #if HAS_K_MS_PINS
      SET_OUTPUT(K_MS1_PIN); SET_OUTPUT(K_MS2_PIN);
      #if PIN_EXISTS(K_MS3)
        SET_OUTPUT(K_MS3_PIN);
      #endif
    #endif
    #if HAS_U_MS_PINS
      SET_OUTPUT(U_MS1_PIN); SET_OUTPUT(U_MS2_PIN);
      #if PIN_EXISTS(U_MS3)
        SET_OUTPUT(U_MS3_PIN);
      #endif
    #endif
    #if HAS_V_MS_PINS
      SET_OUTPUT(V_MS1_PIN); SET_OUTPUT(V_MS2_PIN);
      #if PIN_EXISTS(V_MS3)
        SET_OUTPUT(V_MS3_PIN);
      #endif
    #endif
    #if HAS_W_MS_PINS
      SET_OUTPUT(W_MS1_PIN); SET_OUTPUT(W_MS2_PIN);
      #if PIN_EXISTS(W_MS3)
        SET_OUTPUT(W_MS3_PIN);
      #endif
    #endif
    #if HAS_E0_MS_PINS
      SET_OUTPUT(E0_MS1_PIN); SET_OUTPUT(E0_MS2_PIN);
      #if PIN_EXISTS(E0_MS3)
        SET_OUTPUT(E0_MS3_PIN);
      #endif
    #endif
    #if HAS_E1_MS_PINS
      SET_OUTPUT(E1_MS1_PIN); SET_OUTPUT(E1_MS2_PIN);
      #if PIN_EXISTS(E1_MS3)
        SET_OUTPUT(E1_MS3_PIN);
      #endif
    #endif
    #if HAS_E2_MS_PINS
      SET_OUTPUT(E2_MS1_PIN); SET_OUTPUT(E2_MS2_PIN);
      #if PIN_EXISTS(E2_MS3)
        SET_OUTPUT(E2_MS3_PIN);
      #endif
    #endif
    #if HAS_E3_MS_PINS
      SET_OUTPUT(E3_MS1_PIN); SET_OUTPUT(E3_MS2_PIN);
      #if PIN_EXISTS(E3_MS3)
        SET_OUTPUT(E3_MS3_PIN);
      #endif
    #endif
    #if HAS_E4_MS_PINS
      SET_OUTPUT(E4_MS1_PIN); SET_OUTPUT(E4_MS2_PIN);
      #if PIN_EXISTS(E4_MS3)
        SET_OUTPUT(E4_MS3_PIN);
      #endif
    #endif
    #if HAS_E5_MS_PINS
      SET_OUTPUT(E5_MS1_PIN); SET_OUTPUT(E5_MS2_PIN);
      #if PIN_EXISTS(E5_MS3)
        SET_OUTPUT(E5_MS3_PIN);
      #endif
    #endif
    #if HAS_E6_MS_PINS
      SET_OUTPUT(E6_MS1_PIN); SET_OUTPUT(E6_MS2_PIN);
      #if PIN_EXISTS(E6_MS3)
        SET_OUTPUT(E6_MS3_PIN);
      #endif
    #endif
    #if HAS_E7_MS_PINS
      SET_OUTPUT(E7_MS1_PIN); SET_OUTPUT(E7_MS2_PIN);
      #if PIN_EXISTS(E7_MS3)
        SET_OUTPUT(E7_MS3_PIN);
      #endif
    #endif

    static const uint8_t microstep_modes[] = MICROSTEP_MODES;
    for (uint16_t i = 0; i < COUNT(microstep_modes); i++)
      microstep_mode(i, microstep_modes[i]);
  }

  void Stepper::microstep_ms(const uint8_t driver, const int8_t ms1, const int8_t ms2, const int8_t ms3) {
    if (ms1 >= 0) switch (driver) {
      #if HAS_X_MS_PINS || HAS_X2_MS_PINS
        case X_AXIS:
          #if HAS_X_MS_PINS
            WRITE(X_MS1_PIN, ms1);
          #endif
          #if HAS_X2_MS_PINS
            WRITE(X2_MS1_PIN, ms1);
          #endif
          break;
      #endif
      #if HAS_Y_MS_PINS || HAS_Y2_MS_PINS
        case Y_AXIS:
          #if HAS_Y_MS_PINS
            WRITE(Y_MS1_PIN, ms1);
          #endif
          #if HAS_Y2_MS_PINS
            WRITE(Y2_MS1_PIN, ms1);
          #endif
          break;
      #endif
      #if HAS_SOME_Z_MS_PINS
        case Z_AXIS:
          #if HAS_Z_MS_PINS
            WRITE(Z_MS1_PIN, ms1);
          #endif
          #if HAS_Z2_MS_PINS
            WRITE(Z2_MS1_PIN, ms1);
          #endif
          #if HAS_Z3_MS_PINS
            WRITE(Z3_MS1_PIN, ms1);
          #endif
          #if HAS_Z4_MS_PINS
            WRITE(Z4_MS1_PIN, ms1);
          #endif
          break;
      #endif
      #if HAS_I_MS_PINS
        case  I_AXIS: WRITE(I_MS1_PIN, ms1); break;
      #endif
      #if HAS_J_MS_PINS
        case  J_AXIS: WRITE(J_MS1_PIN, ms1); break;
      #endif
      #if HAS_K_MS_PINS
        case  K_AXIS: WRITE(K_MS1_PIN, ms1); break;
      #endif
      #if HAS_U_MS_PINS
        case  U_AXIS: WRITE(U_MS1_PIN, ms1); break;
      #endif
      #if HAS_V_MS_PINS
        case  V_AXIS: WRITE(V_MS1_PIN, ms1); break;
      #endif
      #if HAS_W_MS_PINS
        case  W_AXIS: WRITE(W_MS1_PIN, ms1); break;
      #endif
      #if HAS_E0_MS_PINS
        case  E_AXIS: WRITE(E0_MS1_PIN, ms1); break;
      #endif
      #if HAS_E1_MS_PINS
        case (E_AXIS + 1): WRITE(E1_MS1_PIN, ms1); break;
      #endif
      #if HAS_E2_MS_PINS
        case (E_AXIS + 2): WRITE(E2_MS1_PIN, ms1); break;
      #endif
      #if HAS_E3_MS_PINS
        case (E_AXIS + 3): WRITE(E3_MS1_PIN, ms1); break;
      #endif
      #if HAS_E4_MS_PINS
        case (E_AXIS + 4): WRITE(E4_MS1_PIN, ms1); break;
      #endif
      #if HAS_E5_MS_PINS
        case (E_AXIS + 5): WRITE(E5_MS1_PIN, ms1); break;
      #endif
      #if HAS_E6_MS_PINS
        case (E_AXIS + 6): WRITE(E6_MS1_PIN, ms1); break;
      #endif
      #if HAS_E7_MS_PINS
        case (E_AXIS + 7): WRITE(E7_MS1_PIN, ms1); break;
      #endif
    }
    if (ms2 >= 0) switch (driver) {
      #if HAS_X_MS_PINS || HAS_X2_MS_PINS
        case X_AXIS:
          #if HAS_X_MS_PINS
            WRITE(X_MS2_PIN, ms2);
          #endif
          #if HAS_X2_MS_PINS
            WRITE(X2_MS2_PIN, ms2);
          #endif
          break;
      #endif
      #if HAS_Y_MS_PINS || HAS_Y2_MS_PINS
        case Y_AXIS:
          #if HAS_Y_MS_PINS
            WRITE(Y_MS2_PIN, ms2);
          #endif
          #if HAS_Y2_MS_PINS
            WRITE(Y2_MS2_PIN, ms2);
          #endif
          break;
      #endif
      #if HAS_SOME_Z_MS_PINS
        case Z_AXIS:
          #if HAS_Z_MS_PINS
            WRITE(Z_MS2_PIN, ms2);
          #endif
          #if HAS_Z2_MS_PINS
            WRITE(Z2_MS2_PIN, ms2);
          #endif
          #if HAS_Z3_MS_PINS
            WRITE(Z3_MS2_PIN, ms2);
          #endif
          #if HAS_Z4_MS_PINS
            WRITE(Z4_MS2_PIN, ms2);
          #endif
          break;
      #endif
      #if HAS_I_MS_PINS
        case  I_AXIS: WRITE(I_MS2_PIN, ms2); break;
      #endif
      #if HAS_J_MS_PINS
        case  J_AXIS: WRITE(J_MS2_PIN, ms2); break;
      #endif
      #if HAS_K_MS_PINS
        case  K_AXIS: WRITE(K_MS2_PIN, ms2); break;
      #endif
      #if HAS_U_MS_PINS
        case  U_AXIS: WRITE(U_MS2_PIN, ms2); break;
      #endif
      #if HAS_V_MS_PINS
        case  V_AXIS: WRITE(V_MS2_PIN, ms2); break;
      #endif
      #if HAS_W_MS_PINS
        case  W_AXIS: WRITE(W_MS2_PIN, ms2); break;
      #endif
      #if HAS_E0_MS_PINS
        case  E_AXIS: WRITE(E0_MS2_PIN, ms2); break;
      #endif
      #if HAS_E1_MS_PINS
        case (E_AXIS + 1): WRITE(E1_MS2_PIN, ms2); break;
      #endif
      #if HAS_E2_MS_PINS
        case (E_AXIS + 2): WRITE(E2_MS2_PIN, ms2); break;
      #endif
      #if HAS_E3_MS_PINS
        case (E_AXIS + 3): WRITE(E3_MS2_PIN, ms2); break;
      #endif
      #if HAS_E4_MS_PINS
        case (E_AXIS + 4): WRITE(E4_MS2_PIN, ms2); break;
      #endif
      #if HAS_E5_MS_PINS
        case (E_AXIS + 5): WRITE(E5_MS2_PIN, ms2); break;
      #endif
      #if HAS_E6_MS_PINS
        case (E_AXIS + 6): WRITE(E6_MS2_PIN, ms2); break;
      #endif
      #if HAS_E7_MS_PINS
        case (E_AXIS + 7): WRITE(E7_MS2_PIN, ms2); break;
      #endif
    }
    if (ms3 >= 0) switch (driver) {
      #if HAS_X_MS_PINS || HAS_X2_MS_PINS
        case X_AXIS:
          #if HAS_X_MS_PINS && PIN_EXISTS(X_MS3)
            WRITE(X_MS3_PIN, ms3);
          #endif
          #if HAS_X2_MS_PINS && PIN_EXISTS(X2_MS3)
            WRITE(X2_MS3_PIN, ms3);
          #endif
          break;
      #endif
      #if HAS_Y_MS_PINS || HAS_Y2_MS_PINS
        case Y_AXIS:
          #if HAS_Y_MS_PINS && PIN_EXISTS(Y_MS3)
            WRITE(Y_MS3_PIN, ms3);
          #endif
          #if HAS_Y2_MS_PINS && PIN_EXISTS(Y2_MS3)
            WRITE(Y2_MS3_PIN, ms3);
          #endif
          break;
      #endif
      #if HAS_SOME_Z_MS_PINS
        case Z_AXIS:
          #if HAS_Z_MS_PINS && PIN_EXISTS(Z_MS3)
            WRITE(Z_MS3_PIN, ms3);
          #endif
          #if HAS_Z2_MS_PINS && PIN_EXISTS(Z2_MS3)
            WRITE(Z2_MS3_PIN, ms3);
          #endif
          #if HAS_Z3_MS_PINS && PIN_EXISTS(Z3_MS3)
            WRITE(Z3_MS3_PIN, ms3);
          #endif
          #if HAS_Z4_MS_PINS && PIN_EXISTS(Z4_MS3)
            WRITE(Z4_MS3_PIN, ms3);
          #endif
          break;
      #endif
      #if HAS_I_MS_PINS && PIN_EXISTS(I_MS3)
        case  I_AXIS: WRITE(I_MS3_PIN, ms3); break;
      #endif
      #if HAS_J_MS_PINS && PIN_EXISTS(J_MS3)
        case  J_AXIS: WRITE(J_MS3_PIN, ms3); break;
      #endif
      #if HAS_K_MS_PINS && PIN_EXISTS(K_MS3)
        case  K_AXIS: WRITE(K_MS3_PIN, ms3); break;
      #endif
      #if HAS_U_MS_PINS && PIN_EXISTS(U_MS3)
        case  U_AXIS: WRITE(U_MS3_PIN, ms3); break;
      #endif
      #if HAS_V_MS_PINS && PIN_EXISTS(V_MS3)
        case  V_AXIS: WRITE(V_MS3_PIN, ms3); break;
      #endif
      #if HAS_W_MS_PINS && PIN_EXISTS(W_MS3)
        case  W_AXIS: WRITE(W_MS3_PIN, ms3); break;
      #endif
      #if HAS_E0_MS_PINS && PIN_EXISTS(E0_MS3)
        case  E_AXIS: WRITE(E0_MS3_PIN, ms3); break;
      #endif
      #if HAS_E1_MS_PINS && PIN_EXISTS(E1_MS3)
        case (E_AXIS + 1): WRITE(E1_MS3_PIN, ms3); break;
      #endif
      #if HAS_E2_MS_PINS && PIN_EXISTS(E2_MS3)
        case (E_AXIS + 2): WRITE(E2_MS3_PIN, ms3); break;
      #endif
      #if HAS_E3_MS_PINS && PIN_EXISTS(E3_MS3)
        case (E_AXIS + 3): WRITE(E3_MS3_PIN, ms3); break;
      #endif
      #if HAS_E4_MS_PINS && PIN_EXISTS(E4_MS3)
        case (E_AXIS + 4): WRITE(E4_MS3_PIN, ms3); break;
      #endif
      #if HAS_E5_MS_PINS && PIN_EXISTS(E5_MS3)
        case (E_AXIS + 5): WRITE(E5_MS3_PIN, ms3); break;
      #endif
      #if HAS_E6_MS_PINS && PIN_EXISTS(E6_MS3)
        case (E_AXIS + 6): WRITE(E6_MS3_PIN, ms3); break;
      #endif
      #if HAS_E7_MS_PINS && PIN_EXISTS(E7_MS3)
        case (E_AXIS + 7): WRITE(E7_MS3_PIN, ms3); break;
      #endif
    }
  }

  // MS1 MS2 MS3 Stepper Driver Microstepping mode table
  #ifndef MICROSTEP1
    #define MICROSTEP1 LOW,LOW,LOW
  #endif
  #if ENABLED(HEROIC_STEPPER_DRIVERS)
    #ifndef MICROSTEP128
      #define MICROSTEP128 LOW,HIGH,LOW
    #endif
  #else
    #ifndef MICROSTEP2
      #define MICROSTEP2 HIGH,LOW,LOW
    #endif
    #ifndef MICROSTEP4
      #define MICROSTEP4 LOW,HIGH,LOW
    #endif
  #endif
  #ifndef MICROSTEP8
    #define MICROSTEP8 HIGH,HIGH,LOW
  #endif
  #ifndef MICROSTEP16
    #define MICROSTEP16 HIGH,HIGH,LOW
  #endif

  void Stepper::microstep_mode(const uint8_t driver, const uint8_t stepping_mode) {
    switch (stepping_mode) {
      #ifdef MICROSTEP1
        case 1: microstep_ms(driver, MICROSTEP1); break;
      #endif
      #ifdef MICROSTEP2
        case 2: microstep_ms(driver, MICROSTEP2); break;
      #endif
      #ifdef MICROSTEP4
        case 4: microstep_ms(driver, MICROSTEP4); break;
      #endif
      #ifdef MICROSTEP8
        case 8: microstep_ms(driver, MICROSTEP8); break;
      #endif
      #ifdef MICROSTEP16
        case 16: microstep_ms(driver, MICROSTEP16); break;
      #endif
      #ifdef MICROSTEP32
        case 32: microstep_ms(driver, MICROSTEP32); break;
      #endif
      #ifdef MICROSTEP64
        case 64: microstep_ms(driver, MICROSTEP64); break;
      #endif
      #ifdef MICROSTEP128
        case 128: microstep_ms(driver, MICROSTEP128); break;
      #endif

      default: SERIAL_ERROR_MSG("Microsteps unavailable"); break;
    }
  }

  void Stepper::microstep_readings() {
    #define PIN_CHAR(P) SERIAL_CHAR('0' + READ(P##_PIN))
    #define MS_LINE(A)  do{ SERIAL_ECHOPGM(" " STRINGIFY(A) ":"); PIN_CHAR(A##_MS1); PIN_CHAR(A##_MS2); }while(0)
    SERIAL_ECHOPGM("MS1|2|3 Pins");
    #if HAS_X_MS_PINS
      MS_LINE(X);
      #if PIN_EXISTS(X_MS3)
        PIN_CHAR(X_MS3);
      #endif
    #endif
    #if HAS_Y_MS_PINS
      MS_LINE(Y);
      #if PIN_EXISTS(Y_MS3)
        PIN_CHAR(Y_MS3);
      #endif
    #endif
    #if HAS_Z_MS_PINS
      MS_LINE(Z);
      #if PIN_EXISTS(Z_MS3)
        PIN_CHAR(Z_MS3);
      #endif
    #endif
    #if HAS_I_MS_PINS
      MS_LINE(I);
      #if PIN_EXISTS(I_MS3)
        PIN_CHAR(I_MS3);
      #endif
    #endif
    #if HAS_J_MS_PINS
      MS_LINE(J);
      #if PIN_EXISTS(J_MS3)
        PIN_CHAR(J_MS3);
      #endif
    #endif
    #if HAS_K_MS_PINS
      MS_LINE(K);
      #if PIN_EXISTS(K_MS3)
        PIN_CHAR(K_MS3);
      #endif
    #endif
    #if HAS_U_MS_PINS
      MS_LINE(U);
      #if PIN_EXISTS(U_MS3)
        PIN_CHAR(U_MS3);
      #endif
    #endif
    #if HAS_V_MS_PINS
      MS_LINE(V);
      #if PIN_EXISTS(V_MS3)
        PIN_CHAR(V_MS3);
      #endif
    #endif
    #if HAS_W_MS_PINS
      MS_LINE(W);
      #if PIN_EXISTS(W_MS3)
        PIN_CHAR(W_MS3);
      #endif
    #endif
    #if HAS_E0_MS_PINS
      MS_LINE(E0);
      #if PIN_EXISTS(E0_MS3)
        PIN_CHAR(E0_MS3);
      #endif
    #endif
    #if HAS_E1_MS_PINS
      MS_LINE(E1);
      #if PIN_EXISTS(E1_MS3)
        PIN_CHAR(E1_MS3);
      #endif
    #endif
    #if HAS_E2_MS_PINS
      MS_LINE(E2);
      #if PIN_EXISTS(E2_MS3)
        PIN_CHAR(E2_MS3);
      #endif
    #endif
    #if HAS_E3_MS_PINS
      MS_LINE(E3);
      #if PIN_EXISTS(E3_MS3)
        PIN_CHAR(E3_MS3);
      #endif
    #endif
    #if HAS_E4_MS_PINS
      MS_LINE(E4);
      #if PIN_EXISTS(E4_MS3)
        PIN_CHAR(E4_MS3);
      #endif
    #endif
    #if HAS_E5_MS_PINS
      MS_LINE(E5);
      #if PIN_EXISTS(E5_MS3)
        PIN_CHAR(E5_MS3);
      #endif
    #endif
    #if HAS_E6_MS_PINS
      MS_LINE(E6);
      #if PIN_EXISTS(E6_MS3)
        PIN_CHAR(E6_MS3);
      #endif
    #endif
    #if HAS_E7_MS_PINS
      MS_LINE(E7);
      #if PIN_EXISTS(E7_MS3)
        PIN_CHAR(E7_MS3);
      #endif
    #endif
    SERIAL_EOL();
  }

#endif // HAS_MICROSTEPS
