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

#include "../../inc/MarlinConfig.h"

#if ENABLED(REPETIER_GCODE_M360)

#include "../gcode.h"

#include "../../module/motion.h"
#include "../../module/planner.h"

#if HAS_EXTRUDERS
  #include "../../module/temperature.h"
#endif

static void config_prefix(PGM_P const name, PGM_P const pref=nullptr, const int8_t ind=-1) {
  SERIAL_ECHOPGM("Config:");
  if (pref) SERIAL_ECHOPGM_P(pref);
  if (ind >= 0) { SERIAL_ECHO(ind); SERIAL_CHAR(':'); }
  SERIAL_ECHOPGM_P(name, C(':'));
}
static void config_line(PGM_P const name, const float val, PGM_P const pref=nullptr, const int8_t ind=-1) {
  config_prefix(name, pref, ind);
  SERIAL_ECHOLN(val);
}
static void config_line(FSTR_P const name, const float val, FSTR_P const pref=nullptr, const int8_t ind=-1) {
  config_line(FTOP(name), val, FTOP(pref) , ind);
}
static void config_line_e(const int8_t e, PGM_P const name, const float val) {
  config_line(name, val, PSTR("Extr."), e + 1);
}
static void config_line_e(const int8_t e, FSTR_P const name, const float val) {
  config_line_e(e, FTOP(name), val);
}

/**
 * M360: Report Firmware configuration
 *       in RepRapFirmware-compatible format
 */
void GcodeSuite::M360() {
  PGMSTR(X_STR,    "X");
  PGMSTR(Y_STR,    "Y");
  PGMSTR(Z_STR,    "Z");
  #if ANY(CLASSIC_JERK, HAS_LINEAR_E_JERK)
    PGMSTR(JERK_STR, "Jerk");
  #endif

  //
  // Basics and Enabled items
  //
  config_line(F("Baudrate"),                    BAUDRATE);
  config_line(F("InputBuffer"),                 MAX_CMD_SIZE);
  config_line(F("PrintlineCache"),              BUFSIZE);
  config_line(F("MixingExtruder"),              ENABLED(MIXING_EXTRUDER));
  config_line(F("SDCard"),                      ENABLED(HAS_MEDIA));
  config_line(F("Fan"),                         ENABLED(HAS_FAN));
  config_line(F("LCD"),                         ENABLED(HAS_DISPLAY));
  config_line(F("SoftwarePowerSwitch"),         1);
  config_line(F("SupportLocalFilamentchange"),  ENABLED(ADVANCED_PAUSE_FEATURE));
  config_line(F("CaseLights"),                  ENABLED(CASE_LIGHT_ENABLE));
  config_line(F("ZProbe"),                      ENABLED(HAS_BED_PROBE));
  config_line(F("Autolevel"),                   ENABLED(HAS_LEVELING));
  config_line(F("EEPROM"),                      ENABLED(EEPROM_SETTINGS));

  //
  // Homing Directions
  //
  PGMSTR(H_DIR_STR, "HomeDir");
  config_line(H_DIR_STR, X_HOME_DIR, X_STR);
  config_line(H_DIR_STR, Y_HOME_DIR, Y_STR);
  config_line(H_DIR_STR, Z_HOME_DIR, Z_STR);

  //
  // XYZ Axis Jerk
  //
  #if ENABLED(CLASSIC_JERK)
    if (planner.max_jerk.x == planner.max_jerk.y)
      config_line(F("XY"), planner.max_jerk.x, FPSTR(JERK_STR));
    else {
      config_line(X_STR, planner.max_jerk.x, JERK_STR);
      config_line(Y_STR, planner.max_jerk.y, JERK_STR);
    }
    config_line(Z_STR, planner.max_jerk.z, JERK_STR);
  #endif

  //
  // Firmware Retraction
  //
  config_line(F("SupportG10G11"), ENABLED(FWRETRACT));
  #if ENABLED(FWRETRACT)
    PGMSTR(RET_STR, "Retraction");
    PGMSTR(UNRET_STR, "RetractionUndo");
    PGMSTR(SPEED_STR, "Speed");
    // M10 Retract with swap (long) moves
    config_line(F("Length"),     fwretract.settings.retract_length, FPSTR(RET_STR));
    config_line(SPEED_STR,       fwretract.settings.retract_feedrate_mm_s, RET_STR);
    config_line(F("ZLift"),      fwretract.settings.retract_zraise, FPSTR(RET_STR));
    config_line(F("LongLength"), fwretract.settings.swap_retract_length, FPSTR(RET_STR));
    // M11 Recover (undo) with swap (long) moves
    config_line(SPEED_STR,            fwretract.settings.retract_recover_feedrate_mm_s, UNRET_STR);
    config_line(F("ExtraLength"),     fwretract.settings.retract_recover_extra, FPSTR(UNRET_STR));
    config_line(F("ExtraLongLength"), fwretract.settings.swap_retract_recover_extra, FPSTR(UNRET_STR));
    config_line(F("LongSpeed"),       fwretract.settings.swap_retract_recover_feedrate_mm_s, FPSTR(UNRET_STR));
  #endif

  //
  // Workspace boundaries
  //
  const xyz_pos_t dmin = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
                  dmax = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
  xyz_pos_t cmin = dmin, cmax = dmax;
  apply_motion_limits(cmin);
  apply_motion_limits(cmax);
  const xyz_pos_t wmin = cmin.asLogical(), wmax = cmax.asLogical();

  PGMSTR(MIN_STR, "Min");
  PGMSTR(MAX_STR, "Max");
  PGMSTR(SIZE_STR, "Size");
  config_line(MIN_STR, wmin.x, X_STR);
  config_line(MIN_STR, wmin.y, Y_STR);
  config_line(MIN_STR, wmin.z, Z_STR);
  config_line(MAX_STR, wmax.x, X_STR);
  config_line(MAX_STR, wmax.y, Y_STR);
  config_line(MAX_STR, wmax.z, Z_STR);
  config_line(SIZE_STR, wmax.x - wmin.x, X_STR);
  config_line(SIZE_STR, wmax.y - wmin.y, Y_STR);
  config_line(SIZE_STR, wmax.z - wmin.z, Z_STR);

  //
  // Print and Travel Acceleration
  //
  #define _ACCEL(A,B) _MIN(planner.settings.max_acceleration_mm_per_s2[A##_AXIS], planner.settings.B)
  PGMSTR(P_ACC_STR, "PrintAccel");
  PGMSTR(T_ACC_STR, "TravelAccel");
  config_line(P_ACC_STR, _ACCEL(X, acceleration), X_STR);
  config_line(P_ACC_STR, _ACCEL(Y, acceleration), Y_STR);
  config_line(P_ACC_STR, _ACCEL(Z, acceleration), Z_STR);
  config_line(T_ACC_STR, _ACCEL(X, travel_acceleration), X_STR);
  config_line(T_ACC_STR, _ACCEL(Y, travel_acceleration), Y_STR);
  config_line(T_ACC_STR, _ACCEL(Z, travel_acceleration), Z_STR);

  config_prefix(PSTR("PrinterType"));
  SERIAL_ECHOLNPGM(
    TERN_(DELTA,         "Delta")
    TERN_(IS_SCARA,      "SCARA")
    TERN_(POLAR,         "Polar")
    TERN_(IS_CORE,       "Core")
    TERN_(MARKFORGED_XY, "MarkForgedXY")
    TERN_(MARKFORGED_YX, "MarkForgedYX")
    TERN_(IS_CARTESIAN,  "Cartesian")
  );

  //
  // Heated Bed
  //
  config_line(F("HeatedBed"), ENABLED(HAS_HEATED_BED));
  #if HAS_HEATED_BED
    config_line(F("MaxBedTemp"), BED_MAX_TARGET);
  #endif

  //
  // Per-Extruder settings
  //
  config_line(F("NumExtruder"), EXTRUDERS);
  #if HAS_EXTRUDERS
    EXTRUDER_LOOP() {
      #if HAS_LINEAR_E_JERK
        config_line_e(e, JERK_STR, planner.max_e_jerk[E_INDEX_N(e)]);
      #elif ENABLED(CLASSIC_JERK)
        config_line_e(e, JERK_STR, planner.max_jerk.e);
      #endif
      config_line_e(e, F("MaxSpeed"), planner.settings.max_feedrate_mm_s[E_AXIS_N(e)]);
      config_line_e(e, F("Acceleration"), planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(e)]);
      config_line_e(e, F("Diameter"), TERN(NO_VOLUMETRICS, DEFAULT_NOMINAL_FILAMENT_DIA, planner.filament_size[e]));
      config_line_e(e, F("MaxTemp"), thermalManager.hotend_maxtemp[e]);
    }
  #endif
}

#endif
