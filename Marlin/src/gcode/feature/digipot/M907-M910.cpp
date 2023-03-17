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

#if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM || HAS_MOTOR_CURRENT_I2C || HAS_MOTOR_CURRENT_DAC

#include "../../gcode.h"

#if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM
  #include "../../../module/stepper.h"
#endif

#if HAS_MOTOR_CURRENT_I2C
  #include "../../../feature/digipot/digipot.h"
#endif

#if HAS_MOTOR_CURRENT_DAC
  #include "../../../feature/dac/stepper_dac.h"
#endif

/**
 * M907: Set digital trimpot motor current using axis codes X [Y] [Z] [I] [J] [K] [U] [V] [W] [E]
 *   B<current> - Special case for E1 (Requires DIGIPOTSS_PIN or DIGIPOT_MCP4018 or DIGIPOT_MCP4451)
 *   C<current> - Special case for E2 (Requires DIGIPOTSS_PIN or DIGIPOT_MCP4018 or DIGIPOT_MCP4451)
 *   S<current> - Set current in mA for all axes (Requires DIGIPOTSS_PIN or DIGIPOT_MCP4018 or DIGIPOT_MCP4451), or
 *                Set percentage of max current for all axes (Requires HAS_DIGIPOT_DAC)
 */
void GcodeSuite::M907() {
  #if HAS_MOTOR_CURRENT_SPI

    if (!parser.seen("BS" STR_AXES_LOGICAL))
      return M907_report();

    if (parser.seenval('S')) for (uint_fast8_t i = 0; i < MOTOR_CURRENT_COUNT; ++i) stepper.set_digipot_current(i, parser.value_int());
    LOOP_LOGICAL_AXES(i) if (parser.seenval(IAXIS_CHAR(i))) stepper.set_digipot_current(i, parser.value_int());    // X Y Z (I J K U V W) E (map to drivers according to DIGIPOT_CHANNELS. Default with NUM_AXES 3: map X Y Z E to X Y Z E0)
    // Additional extruders use B,C.
    // TODO: Change these parameters because 'E' is used and D should be reserved for debugging. B<index>?
    #if E_STEPPERS >= 2
      if (parser.seenval('B')) stepper.set_digipot_current(E_AXIS + 1, parser.value_int());
      #if E_STEPPERS >= 3
        if (parser.seenval('C')) stepper.set_digipot_current(E_AXIS + 2, parser.value_int());
      #endif
    #endif

  #elif HAS_MOTOR_CURRENT_PWM

    #if ANY_PIN(MOTOR_CURRENT_PWM_X, MOTOR_CURRENT_PWM_Y, MOTOR_CURRENT_PWM_XY, MOTOR_CURRENT_PWM_I, MOTOR_CURRENT_PWM_J, MOTOR_CURRENT_PWM_K, MOTOR_CURRENT_PWM_U, MOTOR_CURRENT_PWM_V, MOTOR_CURRENT_PWM_W)
      #define HAS_X_Y_XY_I_J_K_U_V_W 1
    #endif

    #if HAS_X_Y_XY_I_J_K_U_V_W || HAS_MOTOR_CURRENT_PWM_E || PIN_EXISTS(MOTOR_CURRENT_PWM_Z)

      if (!parser.seen("S"
        #if HAS_X_Y_XY_I_J_K_U_V_W
          "XY" SECONDARY_AXIS_GANG("I", "J", "K", "U", "V", "W")
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
          "Z"
        #endif
        #if HAS_MOTOR_CURRENT_PWM_E
          "E"
        #endif
      )) return M907_report();

      if (parser.seenval('S')) for (uint_fast8_t a = 0; a < MOTOR_CURRENT_COUNT; ++a) stepper.set_digipot_current(a, parser.value_int());

      #if HAS_X_Y_XY_I_J_K_U_V_W
        if (NUM_AXIS_GANG(
               parser.seenval('X'), || parser.seenval('Y'), || false,
            || parser.seenval('I'), || parser.seenval('J'), || parser.seenval('K'),
            || parser.seenval('U'), || parser.seenval('V'), || parser.seenval('W')
        )) stepper.set_digipot_current(0, parser.value_int());
      #endif
      #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
        if (parser.seenval('Z')) stepper.set_digipot_current(1, parser.value_int());
      #endif
      #if HAS_MOTOR_CURRENT_PWM_E
        if (parser.seenval('E')) stepper.set_digipot_current(2, parser.value_int());
      #endif

    #endif

  #endif // HAS_MOTOR_CURRENT_PWM

  #if HAS_MOTOR_CURRENT_I2C
    // this one uses actual amps in floating point
    if (parser.seenval('S')) for (uint_fast8_t q = 0; q < DIGIPOT_I2C_NUM_CHANNELS; ++q) digipot_i2c.set_current(q, parser.value_float());
      LOOP_LOGICAL_AXES(i) if (parser.seenval(IAXIS_CHAR(i))) digipot_i2c.set_current(i, parser.value_float());      // X Y Z (I J K U V W) E (map to drivers according to pots adresses. Default with NUM_AXES 3 X Y Z E: map to X Y Z E0)
    // Additional extruders use B,C,D.
    // TODO: Change these parameters because 'E' is used and because 'D' should be reserved for debugging. B<index>?
    #if E_STEPPERS >= 2
      for (uint_fast8_t i = E_AXIS + 1; i < _MAX(DIGIPOT_I2C_NUM_CHANNELS, (NUM_AXES + 3)); i++)
        if (parser.seenval('B' + i - (E_AXIS + 1))) digipot_i2c.set_current(i, parser.value_float());
    #endif
  #endif

  #if HAS_MOTOR_CURRENT_DAC
    if (parser.seenval('S')) {
      const float dac_percent = parser.value_float();
      LOOP_LOGICAL_AXES(i) stepper_dac.set_current_percent(i, dac_percent);
    }
    LOOP_LOGICAL_AXES(i) if (parser.seenval(IAXIS_CHAR(i))) stepper_dac.set_current_percent(i, parser.value_float());   // X Y Z (I J K U V W) E (map to drivers according to DAC_STEPPER_ORDER. Default with NUM_AXES 3: X Y Z E map to X Y Z E0)
  #endif
}

#if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM

  void GcodeSuite::M907_report(const bool forReplay/*=true*/) {
    TERN_(MARLIN_SMALL_BUILD, return);

    report_heading_etc(forReplay, F(STR_STEPPER_MOTOR_CURRENTS));
    #if HAS_MOTOR_CURRENT_PWM
      SERIAL_ECHOLNPGM_P(                                     // PWM-based has 3 values:
          PSTR("  M907 X"), stepper.motor_current_setting[0]  // X, Y, (I, J, K, U, V, W)
        , SP_Z_STR,         stepper.motor_current_setting[1]  // Z
        #if HAS_MOTOR_CURRENT_PWM_E
          , SP_E_STR,       stepper.motor_current_setting[2]  // E
        #endif
      );
    #elif HAS_MOTOR_CURRENT_SPI
      SERIAL_ECHOPGM("  M907");                               // SPI-based has 5 values:
      LOOP_LOGICAL_AXES(q) {                                  // X Y Z (I J K U V W) E (map to X Y Z (I J K U V W) E0 by default)
        SERIAL_CHAR(' ', IAXIS_CHAR(q));
        SERIAL_ECHO(stepper.motor_current_setting[q]);
      }
      #if E_STEPPERS >= 2
        SERIAL_ECHOPGM_P(PSTR(" B"), stepper.motor_current_setting[E_AXIS + 1]  // B (maps to E1 with NUM_AXES 3 according to DIGIPOT_CHANNELS)
          #if E_STEPPERS >= 3
            , PSTR(" C"), stepper.motor_current_setting[E_AXIS + 2]             // C (mapping to E2 must be defined by DIGIPOT_CHANNELS)
          #endif
        );
      #endif
      SERIAL_EOL();
    #endif
  }

#endif // HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM

#if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_DAC

  /**
   * M908: Control digital trimpot directly (M908 P<pin> S<current>)
   */
  void GcodeSuite::M908() {
    TERN_(HAS_MOTOR_CURRENT_SPI, stepper.set_digipot_value_spi(parser.intval('P'), parser.intval('S')));
    TERN_(HAS_MOTOR_CURRENT_DAC, stepper_dac.set_current_value(parser.byteval('P', -1), parser.ushortval('S', 0)));
  }

  #if HAS_MOTOR_CURRENT_DAC

    void GcodeSuite::M909() { stepper_dac.print_values(); }
    void GcodeSuite::M910() { stepper_dac.commit_eeprom(); }

  #endif // HAS_MOTOR_CURRENT_DAC

#endif // HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_DAC

#endif // HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM || HAS_MOTOR_CURRENT_I2C || HAS_MOTOR_CURRENT_DAC
