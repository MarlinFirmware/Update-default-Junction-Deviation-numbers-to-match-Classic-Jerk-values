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

//
// Advanced Settings Menus
//

#include "../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_MENU

#include "menu_item.h"
#include "../../MarlinCore.h"
#include "../../module/planner.h"
#include "../../module/stepper.h"

#if DISABLED(NO_VOLUMETRICS)
  #include "../../gcode/parser.h"
#endif

#if HAS_BED_PROBE
  #include "../../module/probe.h"
#endif

#if HAS_PID_HEATING
  #include "../../module/temperature.h"
#endif

#if HAS_FILAMENT_RUNOUT_DISTANCE
  #include "../../feature/runout.h"
#endif

#if ENABLED(SD_FIRMWARE_UPDATE)
  #include "../../module/settings.h"
#endif

#if ENABLED(PASSWORD_FEATURE)
  #include "../../feature/password/password.h"
#endif

void menu_tmc();
void menu_backlash();

#if HAS_MOTOR_CURRENT_DAC

  #include "../../feature/dac/stepper_dac.h"

  void menu_dac() {
    static xyze_uint8_t driverPercent;
    LOOP_LOGICAL_AXES(i) driverPercent[i] = stepper_dac.get_current_percent((AxisEnum)i);
    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    LOOP_LOGICAL_AXES(a)
      EDIT_ITEM_N(uint8, a, MSG_DAC_PERCENT_N, &driverPercent[a], 0, 100, []{ stepper_dac.set_current_percents(driverPercent); });

    ACTION_ITEM(MSG_DAC_EEPROM_WRITE, stepper_dac.commit_eeprom);
    END_MENU();
  }

#endif

#if HAS_MOTOR_CURRENT_PWM

  void menu_pwm() {
    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);
    #define EDIT_CURRENT_PWM(LABEL,I) EDIT_ITEM_F(long5, F(LABEL), &stepper.motor_current_setting[I], 100, 2000, stepper.refresh_motor_power)
    #if ANY_PIN(MOTOR_CURRENT_PWM_XY, MOTOR_CURRENT_PWM_X, MOTOR_CURRENT_PWM_Y)
      EDIT_CURRENT_PWM(STR_A STR_B, 0);
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
      EDIT_CURRENT_PWM(STR_C, 1);
    #endif
    #if HAS_MOTOR_CURRENT_PWM_E
      EDIT_CURRENT_PWM(STR_E, 2);
    #endif
    END_MENU();
  }

#endif

#if DISABLED(NO_VOLUMETRICS) || ENABLED(ADVANCED_PAUSE_FEATURE)
  //
  // Advanced Settings > Filament
  //
  void menu_advanced_filament() {
    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    #if ENABLED(LIN_ADVANCE)
      #if DISTINCT_E < 2
        EDIT_ITEM(float42_52, MSG_ADVANCE_K, &planner.extruder_advance_K[0], 0, 10);
      #else
        EXTRUDER_LOOP()
          EDIT_ITEM_N(float42_52, e, MSG_ADVANCE_K_E, &planner.extruder_advance_K[e], 0, 10);
      #endif
    #endif

    #if DISABLED(NO_VOLUMETRICS)
      EDIT_ITEM(bool, MSG_VOLUMETRIC_ENABLED, &parser.volumetric_enabled, planner.calculate_volumetric_multipliers);

      #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
        EDIT_ITEM_FAST(float42_52, MSG_VOLUMETRIC_LIMIT, &planner.volumetric_extruder_limit[active_extruder], 0.0f, float(VOLUMETRIC_EXTRUDER_LIMIT_MAX), planner.calculate_volumetric_extruder_limits);
        #if HAS_MULTI_EXTRUDER
          EXTRUDER_LOOP()
            EDIT_ITEM_FAST_N(float42_52, e, MSG_VOLUMETRIC_LIMIT_E, &planner.volumetric_extruder_limit[e], 0.0f, float(VOLUMETRIC_EXTRUDER_LIMIT_MAX), planner.calculate_volumetric_extruder_limits);
        #endif
      #endif

      if (parser.volumetric_enabled) {
        EDIT_ITEM_FAST(float43, MSG_FILAMENT_DIAM, &planner.filament_size[active_extruder], 1.5f, 3.25f, planner.calculate_volumetric_multipliers);
        #if HAS_MULTI_EXTRUDER
          EXTRUDER_LOOP()
            EDIT_ITEM_FAST_N(float43, e, MSG_FILAMENT_DIAM_E, &planner.filament_size[e], 1.5f, 3.25f, planner.calculate_volumetric_multipliers);
        #endif
      }
    #endif

    #if ENABLED(CONFIGURE_FILAMENT_CHANGE)
      constexpr float extrude_maxlength = TERN(PREVENT_LENGTHY_EXTRUDE, EXTRUDE_MAXLENGTH, 999);

      EDIT_ITEM_FAST(float4, MSG_FILAMENT_UNLOAD, &fc_settings[active_extruder].unload_length, 0, extrude_maxlength);
      #if HAS_MULTI_EXTRUDER
        EXTRUDER_LOOP()
          EDIT_ITEM_FAST_N(float4, e, MSG_FILAMENTUNLOAD_E, &fc_settings[e].unload_length, 0, extrude_maxlength);
      #endif

      EDIT_ITEM_FAST(float4, MSG_FILAMENT_LOAD, &fc_settings[active_extruder].load_length, 0, extrude_maxlength);
      #if HAS_MULTI_EXTRUDER
        EXTRUDER_LOOP()
          EDIT_ITEM_FAST_N(float4, e, MSG_FILAMENTLOAD_E, &fc_settings[e].load_length, 0, extrude_maxlength);
      #endif
    #endif

    #if HAS_FILAMENT_RUNOUT_DISTANCE
      editable.decimal = runout.runout_distance();
      auto set_runout_distance = []{ runout.set_runout_distance(editable.decimal); };
      #if ENABLED(FILAMENT_MOTION_SENSOR)
        EDIT_ITEM_FAST(float31, MSG_RUNOUT_DISTANCE_MM, &editable.decimal, 0.1, 10, set_runout_distance, true);
      #else
        EDIT_ITEM_FAST(float3, MSG_RUNOUT_DISTANCE_MM, &editable.decimal, 1, 999, set_runout_distance, true);
      #endif
    #endif

    END_MENU();
  }

#endif // !NO_VOLUMETRICS || ADVANCED_PAUSE_FEATURE

//
// Advanced Settings > Temperature helpers
//

#if ENABLED(PID_AUTOTUNE_MENU)

  #if ENABLED(PIDTEMP)
    int16_t autotune_temp[HOTENDS] = ARRAY_BY_HOTENDS1(PREHEAT_1_TEMP_HOTEND);
  #endif
  #if ENABLED(PIDTEMPBED)
    int16_t autotune_temp_bed = PREHEAT_1_TEMP_BED;
  #endif
  #if ENABLED(PIDTEMPCHAMBER)
    int16_t autotune_temp_chamber = PREHEAT_1_TEMP_CHAMBER;
  #endif

  #include "../../gcode/queue.h"

  void _lcd_autotune(const heater_id_t hid) {
    char cmd[30];
    int16_t tune_temp;
    switch (hid) {
      #if ENABLED(PIDTEMPBED)
        case H_BED: tune_temp = autotune_temp_bed; break;
      #endif
      #if ENABLED(PIDTEMPCHAMBER)
        case H_CHAMBER: tune_temp = autotune_temp_chamber; break;
      #endif
      default:
        #if ENABLED(PIDTEMP)
          tune_temp = autotune_temp[hid]; break;
        #else
          return;
        #endif
    }
    sprintf_P(cmd, PSTR("M303 U1 E%i S%i"), hid, tune_temp);
    queue.inject(cmd);
    ui.return_to_status();
  }

#endif // PID_AUTOTUNE_MENU

#if ENABLED(PID_EDIT_MENU)

  // Placeholders for PID editing
  float raw_Kp, raw_Ki, raw_Kd;
  #if ENABLED(PID_EXTRUSION_SCALING)
    float raw_Kc;
  #endif
  #if ENABLED(PID_FAN_SCALING)
    float raw_Kf;
  #endif

  // Helpers for editing PID Kp, Ki and Kd values
  void apply_PID_p(const int8_t e) {
    switch (e) {
      #if ENABLED(PIDTEMPBED)
        case H_BED: thermalManager.temp_bed.pid.set_Kp(raw_Kp); break;
      #endif
      #if ENABLED(PIDTEMPCHAMBER)
        case H_CHAMBER: thermalManager.temp_chamber.pid.set_Kp(raw_Kp); break;
      #endif
      default:
        #if ENABLED(PIDTEMP)
          SET_HOTEND_PID(Kp, e, raw_Kp);
          thermalManager.updatePID();
        #endif
        break;
    }
  }
  void apply_PID_i(const int8_t e) {
    switch (e) {
      #if ENABLED(PIDTEMPBED)
        case H_BED: thermalManager.temp_bed.pid.set_Ki(raw_Ki); break;
      #endif
      #if ENABLED(PIDTEMPCHAMBER)
        case H_CHAMBER: thermalManager.temp_chamber.pid.set_Ki(raw_Ki); break;
      #endif
      default:
        #if ENABLED(PIDTEMP)
          SET_HOTEND_PID(Ki, e, raw_Ki);
          thermalManager.updatePID();
        #endif
        break;
    }
  }
  void apply_PID_d(const int8_t e) {
    switch (e) {
      #if ENABLED(PIDTEMPBED)
        case H_BED: thermalManager.temp_bed.pid.set_Kd(raw_Kd); break;
      #endif
      #if ENABLED(PIDTEMPCHAMBER)
        case H_CHAMBER: thermalManager.temp_chamber.pid.set_Kd(raw_Kd); break;
      #endif
      default:
        #if ENABLED(PIDTEMP)
          SET_HOTEND_PID(Kd, e, raw_Kd);
          thermalManager.updatePID();
        #endif
        break;
    }
  }
#endif

#if ALL(AUTOTEMP, HAS_TEMP_HOTEND) || ANY(PID_AUTOTUNE_MENU, PID_EDIT_MENU, MPC_AUTOTUNE_MENU, MPC_EDIT_MENU)
  #define SHOW_MENU_ADVANCED_TEMPERATURE 1
#endif

//
// Advanced Settings > Temperature
//
#if SHOW_MENU_ADVANCED_TEMPERATURE

  #if ALL(MPC_EDIT_MENU, MPC_INCLUDE_FAN)
    #define MPC_EDIT_DEFS(N) editable.decimal = thermalManager.temp_hotend[N].fanCoefficient()
  #else
    #define MPC_EDIT_DEFS(...)
  #endif

  void menu_advanced_temperature() {
    #if ENABLED(MPC_EDIT_MENU) && !HAS_MULTI_HOTEND
      MPC_EDIT_DEFS(0);
    #endif

    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    //
    // Autotemp, Min, Max, Fact
    //
    #if ALL(AUTOTEMP, HAS_TEMP_HOTEND)
      EDIT_ITEM(int3, MSG_MIN, &planner.autotemp.min, 0, thermalManager.hotend_max_target(0));
      EDIT_ITEM(int3, MSG_MAX, &planner.autotemp.max, 0, thermalManager.hotend_max_target(0));
      EDIT_ITEM(float42_52, MSG_FACTOR, &planner.autotemp.factor, 0, 10);
    #endif

    //
    // PID-P, PID-I, PID-D, PID-C, PID Autotune
    // PID-P E1, PID-I E1, PID-D E1, PID-C E1, PID Autotune E1
    // PID-P E2, PID-I E2, PID-D E2, PID-C E2, PID Autotune E2
    // PID-P E3, PID-I E3, PID-D E3, PID-C E3, PID Autotune E3
    // PID-P E4, PID-I E4, PID-D E4, PID-C E4, PID Autotune E4
    // PID-P E5, PID-I E5, PID-D E5, PID-C E5, PID Autotune E5
    //

    #if ALL(PIDTEMP, PID_EDIT_MENU)
      #define __PID_HOTEND_MENU_ITEMS(N) \
        raw_Kp = thermalManager.temp_hotend[N].pid.p(); \
        raw_Ki = thermalManager.temp_hotend[N].pid.i(); \
        raw_Kd = thermalManager.temp_hotend[N].pid.d(); \
        EDIT_ITEM_FAST_N(float41sign, N, MSG_PID_P_E, &raw_Kp, 1, 9990, []{ apply_PID_p(N); }); \
        EDIT_ITEM_FAST_N(float52sign, N, MSG_PID_I_E, &raw_Ki, 0.01f, 9990, []{ apply_PID_i(N); }); \
        EDIT_ITEM_FAST_N(float41sign, N, MSG_PID_D_E, &raw_Kd, 1, 9990, []{ apply_PID_d(N); })

      #if ENABLED(PID_EXTRUSION_SCALING)
        #define _PID_HOTEND_MENU_ITEMS(N) \
          __PID_HOTEND_MENU_ITEMS(N); \
          raw_Kc = thermalManager.temp_hotend[N].pid.c(); \
          EDIT_ITEM_N(float4, N, MSG_PID_C_E, &raw_Kc, 1, 9990, []{ SET_HOTEND_PID(Kc, N, raw_Kc); thermalManager.updatePID(); });
      #else
        #define _PID_HOTEND_MENU_ITEMS(N) __PID_HOTEND_MENU_ITEMS(N)
      #endif

      #if ENABLED(PID_FAN_SCALING)
        #define _HOTEND_PID_EDIT_MENU_ITEMS(N) \
          _PID_HOTEND_MENU_ITEMS(N); \
          raw_Kf = thermalManager.temp_hotend[N].pid.f(); \
          EDIT_ITEM_N(float4, N, MSG_PID_F_E, &raw_Kf, 1, 9990, []{ SET_HOTEND_PID(Kf, N, raw_Kf); thermalManager.updatePID(); });
      #else
        #define _HOTEND_PID_EDIT_MENU_ITEMS(N) _PID_HOTEND_MENU_ITEMS(N)
      #endif

    #else

      #define _HOTEND_PID_EDIT_MENU_ITEMS(N) NOOP

    #endif

    #if ENABLED(PID_EDIT_MENU) && ANY(PIDTEMPBED, PIDTEMPCHAMBER)
      #define _PID_EDIT_ITEMS_TMPL(N,T) \
        raw_Kp = T.pid.p(); \
        raw_Ki = T.pid.i(); \
        raw_Kd = T.pid.d(); \
        EDIT_ITEM_FAST_N(float41sign, N, MSG_PID_P_E, &raw_Kp, 1, 9990, []{ apply_PID_p(N); }); \
        EDIT_ITEM_FAST_N(float52sign, N, MSG_PID_I_E, &raw_Ki, 0.01f, 9990, []{ apply_PID_i(N); }); \
        EDIT_ITEM_FAST_N(float41sign, N, MSG_PID_D_E, &raw_Kd, 1, 9990, []{ apply_PID_d(N); })
    #endif

    #if ENABLED(PIDTEMP)
      #if ENABLED(PID_AUTOTUNE_MENU)
        #define HOTEND_PID_EDIT_MENU_ITEMS(N) \
          _HOTEND_PID_EDIT_MENU_ITEMS(N); \
          EDIT_ITEM_FAST_N(int3, N, MSG_PID_AUTOTUNE_E, &autotune_temp[N], 150, thermalManager.hotend_max_target(N), []{ _lcd_autotune(heater_id_t(MenuItemBase::itemIndex)); });
      #else
        #define HOTEND_PID_EDIT_MENU_ITEMS(N) _HOTEND_PID_EDIT_MENU_ITEMS(N);
      #endif

      HOTEND_PID_EDIT_MENU_ITEMS(0);
      #if ENABLED(PID_PARAMS_PER_HOTEND)
        REPEAT_S(1, HOTENDS, HOTEND_PID_EDIT_MENU_ITEMS);
      #endif
    #endif

    #if ENABLED(MPC_EDIT_MENU)

      #define _MPC_EDIT_ITEMS(N) \
        MPC_t &mpc = thermalManager.temp_hotend[MenuItemBase::itemIndex].mpc; \
        EDIT_ITEM_FAST_N(float41, N, MSG_MPC_POWER_E, &mpc.heater_power, 1, 200); \
        EDIT_ITEM_FAST_N(float31, N, MSG_MPC_BLOCK_HEAT_CAPACITY_E, &mpc.block_heat_capacity, 0, 40); \
        EDIT_ITEM_FAST_N(float43, N, MSG_SENSOR_RESPONSIVENESS_E, &mpc.sensor_responsiveness, 0, 1); \
        EDIT_ITEM_FAST_N(float43, N, MSG_MPC_AMBIENT_XFER_COEFF_E, &mpc.ambient_xfer_coeff_fan0, 0, 1)

      #if ENABLED(MPC_INCLUDE_FAN)
        #define MPC_EDIT_ITEMS(N) \
          _MPC_EDIT_ITEMS(N); \
          EDIT_ITEM_FAST_N(float43, N, MSG_MPC_AMBIENT_XFER_COEFF_FAN_E, &editable.decimal, 0, 1, []{ \
            thermalManager.temp_hotend[MenuItemBase::itemIndex].applyFanAdjustment(editable.decimal); \
          })
      #else
        #define MPC_EDIT_ITEMS _MPC_EDIT_ITEMS
      #endif

      #if HAS_MULTI_HOTEND
        static auto mpc_edit_hotend = [](const uint8_t e) {
          MPC_EDIT_DEFS(e);
          START_MENU();
          BACK_ITEM(MSG_TEMPERATURE);
          MPC_EDIT_ITEMS(e);
          END_MENU();
        };
        #define MPC_ENTRY(N) SUBMENU_N(N, MSG_MPC_EDIT, []{ mpc_edit_hotend(MenuItemBase::itemIndex); });
      #else
        #define MPC_ENTRY MPC_EDIT_ITEMS
      #endif

      REPEAT(HOTENDS, MPC_ENTRY);

    #endif // MPC_EDIT_MENU

    #if ENABLED(MPC_AUTOTUNE_MENU)
      ACTION_ITEM(MSG_MPC_AUTOTUNE, []{ queue.inject(F("M306 T")); ui.return_to_status(); });
    #endif

    #if ENABLED(PIDTEMPBED)
      #if ENABLED(PID_EDIT_MENU)
        _PID_EDIT_ITEMS_TMPL(H_BED, thermalManager.temp_bed);
      #endif
      #if ENABLED(PID_AUTOTUNE_MENU)
        EDIT_ITEM_FAST_N(int3, H_BED, MSG_PID_AUTOTUNE_E, &autotune_temp_bed, PREHEAT_1_TEMP_BED, BED_MAX_TARGET, []{ _lcd_autotune(H_BED); });
      #endif
    #endif

    #if ENABLED(PIDTEMPCHAMBER)
      #if ENABLED(PID_EDIT_MENU)
        _PID_EDIT_ITEMS_TMPL(H_CHAMBER, thermalManager.temp_chamber);
      #endif
      #if ENABLED(PID_AUTOTUNE_MENU)
        EDIT_ITEM_FAST_N(int3, H_CHAMBER, MSG_PID_AUTOTUNE_E, &autotune_temp_chamber, PREHEAT_1_TEMP_CHAMBER, CHAMBER_MAX_TARGET, []{ _lcd_autotune(H_CHAMBER); });
      #endif
    #endif

    END_MENU();
  }

#endif // SHOW_MENU_ADVANCED_TEMPERATURE

#if DISABLED(SLIM_LCD_MENUS)

  // M203 / M205 Velocity options
  void menu_advanced_velocity() {
    // M203 Max Feedrate
    constexpr xyze_feedrate_t max_fr_edit =
      #ifdef MAX_FEEDRATE_EDIT_VALUES
        MAX_FEEDRATE_EDIT_VALUES
      #elif ENABLED(LIMITED_MAX_FR_EDITING)
        DEFAULT_MAX_FEEDRATE
      #else
        LOGICAL_AXIS_ARRAY_1(9999)
      #endif
    ;
    #if ENABLED(LIMITED_MAX_FR_EDITING) && !defined(MAX_FEEDRATE_EDIT_VALUES)
      const xyze_feedrate_t max_fr_edit_scaled = max_fr_edit * 2;
    #else
      const xyze_feedrate_t &max_fr_edit_scaled = max_fr_edit;
    #endif

    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    LOOP_NUM_AXES(a)
      EDIT_ITEM_FAST_N(float5, a, MSG_VMAX_N, &planner.settings.max_feedrate_mm_s[a], 1, max_fr_edit_scaled[a]);

    #if E_STEPPERS
      EDIT_ITEM_FAST_N(float5, E_AXIS, MSG_VMAX_N, &planner.settings.max_feedrate_mm_s[E_AXIS_N(active_extruder)], 1, max_fr_edit_scaled.e);
    #endif
    #if ENABLED(DISTINCT_E_FACTORS)
      for (uint8_t n = 0; n < E_STEPPERS; ++n)
        EDIT_ITEM_FAST_N(float5, n, MSG_VMAX_EN, &planner.settings.max_feedrate_mm_s[E_AXIS_N(n)], 1, max_fr_edit_scaled.e);
    #endif

    // M205 S Min Feedrate
    EDIT_ITEM_FAST(float5, MSG_VMIN, &planner.settings.min_feedrate_mm_s, 0, 9999);

    // M205 T Min Travel Feedrate
    EDIT_ITEM_FAST(float5, MSG_VTRAV_MIN, &planner.settings.min_travel_feedrate_mm_s, 0, 9999);

    END_MENU();
  }

  // M201 / M204 Accelerations
  void menu_advanced_acceleration() {
    float max_accel = (
      #if NUM_AXES
        _MAX(NUM_AXIS_LIST(
          planner.settings.max_acceleration_mm_per_s2[A_AXIS],
          planner.settings.max_acceleration_mm_per_s2[B_AXIS],
          planner.settings.max_acceleration_mm_per_s2[C_AXIS],
          planner.settings.max_acceleration_mm_per_s2[I_AXIS],
          planner.settings.max_acceleration_mm_per_s2[J_AXIS],
          planner.settings.max_acceleration_mm_per_s2[K_AXIS],
          planner.settings.max_acceleration_mm_per_s2[U_AXIS],
          planner.settings.max_acceleration_mm_per_s2[V_AXIS],
          planner.settings.max_acceleration_mm_per_s2[W_AXIS]
        ))
      #else
        0
      #endif
    );

    // M201 settings
    constexpr xyze_ulong_t max_accel_edit =
      #ifdef MAX_ACCEL_EDIT_VALUES
        MAX_ACCEL_EDIT_VALUES
      #elif ENABLED(LIMITED_MAX_ACCEL_EDITING)
        DEFAULT_MAX_ACCELERATION
      #else
        LOGICAL_AXIS_ARRAY_1(99000)
      #endif
    ;
    #if ENABLED(LIMITED_MAX_ACCEL_EDITING) && !defined(MAX_ACCEL_EDIT_VALUES)
      const xyze_ulong_t max_accel_edit_scaled = max_accel_edit * 2;
    #else
      const xyze_ulong_t &max_accel_edit_scaled = max_accel_edit;
    #endif

    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    // M204 P Acceleration
    EDIT_ITEM_FAST(float5_25, MSG_ACC, &planner.settings.acceleration, 25, max_accel);

    #if HAS_EXTRUDERS
      // M204 R Retract Acceleration
      EDIT_ITEM_FAST(float5, MSG_A_RETRACT, &planner.settings.retract_acceleration, 100, planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(active_extruder)]);
    #endif

    // M204 T Travel Acceleration
    EDIT_ITEM_FAST(float5_25, MSG_A_TRAVEL, &planner.settings.travel_acceleration, 25, max_accel);

    #define EDIT_AMAX(Q,L) EDIT_ITEM_FAST_N(long5_25, _AXIS(Q), MSG_AMAX_N, &planner.settings.max_acceleration_mm_per_s2[_AXIS(Q)], L, max_accel_edit_scaled[_AXIS(Q)], []{ planner.refresh_acceleration_rates(); })
    NUM_AXIS_CODE(
      EDIT_AMAX(A, 100), EDIT_AMAX(B, 100), EDIT_AMAX(C, 10),
      EDIT_AMAX(I,  10), EDIT_AMAX(J,  10), EDIT_AMAX(K, 10),
      EDIT_AMAX(U,  10), EDIT_AMAX(V,  10), EDIT_AMAX(W, 10)
    );

    #if ENABLED(DISTINCT_E_FACTORS)
      EDIT_ITEM_FAST(long5_25, MSG_AMAX_E, &planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(active_extruder)], 100, max_accel_edit_scaled.e, []{ planner.refresh_acceleration_rates(); });
      for (uint8_t n = 0; n < E_STEPPERS; ++n)
        EDIT_ITEM_FAST_N(long5_25, n, MSG_AMAX_EN, &planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(n)], 100, max_accel_edit_scaled.e, []{
          if (MenuItemBase::itemIndex == active_extruder)
            planner.refresh_acceleration_rates();
       });
    #elif E_STEPPERS
      EDIT_ITEM_FAST(long5_25, MSG_AMAX_E, &planner.settings.max_acceleration_mm_per_s2[E_AXIS], 100, max_accel_edit_scaled.e, []{ planner.refresh_acceleration_rates(); });
    #endif

    #ifdef XY_FREQUENCY_LIMIT
      EDIT_ITEM(int8, MSG_XY_FREQUENCY_LIMIT, &planner.xy_freq_limit_hz, 0, 100, planner.refresh_frequency_limit, true);
      editable.uint8 = uint8_t(LROUND(planner.xy_freq_min_speed_factor * 255)); // percent to u8
      EDIT_ITEM(percent, MSG_XY_FREQUENCY_FEEDRATE, &editable.uint8, 3, 255, []{ planner.set_min_speed_factor_u8(editable.uint8); }, true);
    #endif

    END_MENU();
  }

  #if ENABLED(SHAPING_MENU)

    void menu_advanced_input_shaping() {
      constexpr float min_frequency = TERN(__AVR__, float(STEPPER_TIMER_RATE) / 2 / 0x10000, 1.0f);

      START_MENU();
      BACK_ITEM(MSG_ADVANCED_SETTINGS);

      // M593 F Frequency and D Damping ratio
      #define SHAPING_MENU_FOR_AXIS(A)                                                                                                                                        \
        editable.decimal = stepper.get_shaping_frequency(_AXIS(A));                                                                                                           \
        if (editable.decimal) {                                                                                                                                               \
          ACTION_ITEM_N(_AXIS(A), MSG_SHAPING_DISABLE_N, []{ stepper.set_shaping_frequency(_AXIS(A), 0.0f); ui.refresh(); });                                                   \
          EDIT_ITEM_FAST_N(float41, _AXIS(A), MSG_SHAPING_FREQ_N, &editable.decimal, min_frequency, 200.0f, []{ stepper.set_shaping_frequency(_AXIS(A), editable.decimal); });  \
          editable.decimal = stepper.get_shaping_damping_ratio(_AXIS(A));                                                                                                     \
          EDIT_ITEM_FAST_N(float42_52, _AXIS(A), MSG_SHAPING_ZETA_N, &editable.decimal, 0.0f, 1.0f, []{ stepper.set_shaping_damping_ratio(_AXIS(A), editable.decimal); });      \
        }                                                                                                                                                                     \
        else                                                                                                                                                                  \
          ACTION_ITEM_N(_AXIS(A), MSG_SHAPING_ENABLE_N, []{ stepper.set_shaping_frequency(_AXIS(A), (SHAPING_FREQ_##A) ?: (SHAPING_MIN_FREQ)); ui.refresh(); });

      TERN_(INPUT_SHAPING_X, SHAPING_MENU_FOR_AXIS(X))
      TERN_(INPUT_SHAPING_Y, SHAPING_MENU_FOR_AXIS(Y))
      TERN_(INPUT_SHAPING_Z, SHAPING_MENU_FOR_AXIS(Z))

      END_MENU();
    }

  #endif

  #if ENABLED(CLASSIC_JERK)

    void menu_advanced_jerk() {
      START_MENU();
      BACK_ITEM(MSG_ADVANCED_SETTINGS);

      constexpr xyze_float_t max_jerk_edit =
        #ifdef MAX_JERK_EDIT_VALUES
          MAX_JERK_EDIT_VALUES
        #elif ENABLED(LIMITED_JERK_EDITING)
          #define _JERK2(N) DEFAULT_##N##JERK * 2
          { MAPLIST(_JERK2, LOGICAL_AXIS_NAMES) }
        #else
          LOGICAL_AXIS_ARRAY_1(990)
        #endif
      ;

      LOOP_LOGICAL_AXES(a) {
        if (TERN0(HAS_C_AXIS, a == C_AXIS) || TERN0(HAS_EXTRUDERS, a == E_AXIS))
          EDIT_ITEM_FAST_N(float52sign, a, MSG_VN_JERK, &planner.max_jerk[a], 0.1f, max_jerk_edit[a]);
        else
          EDIT_ITEM_FAST_N(float3, a, MSG_VN_JERK, &planner.max_jerk[a], 1.0f, max_jerk_edit[a]);
      }

      END_MENU();
    }

  #endif

#endif // !SLIM_LCD_MENUS

#if ENABLED(EDITABLE_STEPS_PER_UNIT)

  // M92 Steps-per-mm
  void menu_advanced_steps_per_mm() {
    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    LOOP_NUM_AXES(a)
      EDIT_ITEM_FAST_N(float72, a, MSG_N_STEPS, &planner.settings.axis_steps_per_mm[a], 5, 9999, []{ planner.refresh_positioning(); });

    #if ENABLED(DISTINCT_E_FACTORS)
      for (uint8_t n = 0; n < E_STEPPERS; ++n)
        EDIT_ITEM_FAST_N(float72, n, MSG_EN_STEPS, &planner.settings.axis_steps_per_mm[E_AXIS_N(n)], 5, 9999, []{
          const uint8_t e = MenuItemBase::itemIndex;
          if (e == active_extruder)
            planner.refresh_positioning();
          else
            planner.mm_per_step[E_AXIS_N(e)] = 1.0f / planner.settings.axis_steps_per_mm[E_AXIS_N(e)];
        });
    #elif E_STEPPERS
      EDIT_ITEM_FAST_N(float72, E_AXIS, MSG_N_STEPS, &planner.settings.axis_steps_per_mm[E_AXIS], 5, 9999, []{ planner.refresh_positioning(); });
    #endif

    END_MENU();
  }

#endif // EDITABLE_STEPS_PER_UNIT

void menu_advanced_settings() {
  #if ANY(POLARGRAPH, SHAPING_MENU, HAS_BED_PROBE, EDITABLE_STEPS_PER_UNIT)
    const bool is_busy = printer_busy();
  #endif

  #if ENABLED(SD_FIRMWARE_UPDATE)
    bool sd_update_state = settings.sd_update_status();
  #endif

  START_MENU();
  BACK_ITEM(MSG_CONFIGURATION);

  #if DISABLED(SLIM_LCD_MENUS)

    #if ENABLED(POLARGRAPH)
      // M665 - Polargraph Settings
      if (!is_busy) {
        EDIT_ITEM_FAST(float4, MSG_SEGMENTS_PER_SECOND, &segments_per_second, 100, 9999);               // M665 S
        EDIT_ITEM_FAST(float51sign, MSG_DRAW_MIN_X, &draw_area_min.x, X_MIN_POS, draw_area_max.x - 10); // M665 L
        EDIT_ITEM_FAST(float51sign, MSG_DRAW_MAX_X, &draw_area_max.x, draw_area_min.x + 10, X_MAX_POS); // M665 R
        EDIT_ITEM_FAST(float51sign, MSG_DRAW_MIN_Y, &draw_area_min.y, Y_MIN_POS, draw_area_max.y - 10); // M665 T
        EDIT_ITEM_FAST(float51sign, MSG_DRAW_MAX_Y, &draw_area_max.y, draw_area_min.y + 10, Y_MAX_POS); // M665 B
        EDIT_ITEM_FAST(float51sign, MSG_MAX_BELT_LEN, &polargraph_max_belt_len, 500, 2000);             // M665 H
      }
    #endif

    #if HAS_HOME_OFFSET
      // M428 - Set Home Offsets
      ACTION_ITEM(MSG_SET_HOME_OFFSETS, []{ queue.inject(F("M428")); ui.return_to_status(); });
    #endif

    // M203 / M205 - Feedrate items
    SUBMENU(MSG_MAX_SPEED, menu_advanced_velocity);

    // M201 - Acceleration items
    SUBMENU(MSG_ACCELERATION, menu_advanced_acceleration);

    // M593 - Acceleration items
    #if ENABLED(SHAPING_MENU)
      SUBMENU(MSG_INPUT_SHAPING, menu_advanced_input_shaping);
    #endif

    #if ENABLED(CLASSIC_JERK)
      // M205 - Max Jerk
      SUBMENU(MSG_JERK, menu_advanced_jerk);
    #elif HAS_JUNCTION_DEVIATION
      EDIT_ITEM(float43, MSG_JUNCTION_DEVIATION, &planner.junction_deviation_mm, 0.001f, 0.3f
        OPTARG(HAS_LINEAR_E_JERK, planner.recalculate_max_e_jerk)
      );
    #endif

  #endif // !SLIM_LCD_MENUS

  // M92 - Steps Per mm
  #if ENABLED(EDITABLE_STEPS_PER_UNIT)
    if (!is_busy) SUBMENU(MSG_STEPS_PER_MM, menu_advanced_steps_per_mm);
  #endif

  #if ENABLED(BACKLASH_GCODE)
    SUBMENU(MSG_BACKLASH, menu_backlash);
  #endif

  #if HAS_MOTOR_CURRENT_DAC
    SUBMENU(MSG_DRIVE_STRENGTH, menu_dac);
  #endif
  #if HAS_MOTOR_CURRENT_PWM
    SUBMENU(MSG_DRIVE_STRENGTH, menu_pwm);
  #endif

  #if HAS_TRINAMIC_CONFIG
    SUBMENU(MSG_TMC_DRIVERS, menu_tmc);
  #endif

  #if SHOW_MENU_ADVANCED_TEMPERATURE
    SUBMENU(MSG_TEMPERATURE, menu_advanced_temperature);
  #endif

  #if DISABLED(NO_VOLUMETRICS) || ENABLED(ADVANCED_PAUSE_FEATURE)
    SUBMENU(MSG_FILAMENT, menu_advanced_filament);
  #elif ENABLED(LIN_ADVANCE)
    #if DISTINCT_E < 2
      EDIT_ITEM(float42_52, MSG_ADVANCE_K, &planner.extruder_advance_K[0], 0, 10);
    #else
      EXTRUDER_LOOP()
        EDIT_ITEM_N(float42_52, n, MSG_ADVANCE_K_E, &planner.extruder_advance_K[e], 0, 10);
    #endif
  #endif

  // M540 S - Abort on endstop hit when SD printing
  #if ENABLED(SD_ABORT_ON_ENDSTOP_HIT)
    EDIT_ITEM(bool, MSG_ENDSTOP_ABORT, &planner.abort_on_endstop_hit);
  #endif

  #if ENABLED(SD_FIRMWARE_UPDATE)
    EDIT_ITEM(bool, MSG_MEDIA_UPDATE, &sd_update_state, []{
      //
      // Toggle the SD Firmware Update state in EEPROM
      //
      const bool new_state = !settings.sd_update_status(),
                 didset = settings.set_sd_update_status(new_state);
      ui.completion_feedback(didset);
      ui.return_to_status();
      if (new_state) LCD_MESSAGE(MSG_RESET_PRINTER); else ui.reset_status();
    });
  #endif

  #if ENABLED(PASSWORD_FEATURE)
    SUBMENU(MSG_PASSWORD_SETTINGS, password.access_menu_password);
  #endif

  #if ENABLED(EEPROM_SETTINGS) && DISABLED(SLIM_LCD_MENUS)
    CONFIRM_ITEM(MSG_INIT_EEPROM,
      MSG_BUTTON_INIT, MSG_BUTTON_CANCEL,
      ui.init_eeprom, nullptr,
      GET_TEXT_F(MSG_INIT_EEPROM), (const char *)nullptr, F("?")
    );
  #endif

  END_MENU();
}

#endif // HAS_MARLINUI_MENU
