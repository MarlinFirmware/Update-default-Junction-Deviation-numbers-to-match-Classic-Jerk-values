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

#include "utility.h"

#include "../MarlinCore.h"
#include "../module/temperature.h"

#if ENABLED(MARLIN_DEV_MODE)
  MarlinError marlin_error_number;    // Error Number - Marlin can beep X times periodically, display, and emit...
#endif

void safe_delay(millis_t ms) {
  while (ms > 50) {
    ms -= 50;
    delay(50);
    thermalManager.task();
  }
  delay(ms);
  thermalManager.task(); // This keeps us safe if too many small safe_delay() calls are made
}

// A delay to provide brittle hosts time to receive bytes
#if ENABLED(SERIAL_OVERRUN_PROTECTION)

  #include "../gcode/gcode.h" // for set_autoreport_paused

  void serial_delay(const millis_t ms) {
    const bool was = gcode.set_autoreport_paused(true);
    safe_delay(ms);
    gcode.set_autoreport_paused(was);
  }
#endif

#if ENABLED(DEBUG_LEVELING_FEATURE)

  #include "../module/probe.h"
  #include "../module/motion.h"
  #include "../module/planner.h"
  #include "../libs/numtostr.h"
  #include "../feature/bedlevel/bedlevel.h"

  void log_machine_info() {
    SERIAL_ECHOLNPGM("Machine Type:"
      TERN_(DELTA,                 " Delta")
      TERN_(IS_SCARA,              " SCARA")
      TERN_(AXEL_TPARA,            " TPARA")
      TERN_(IS_CORE,               " Core")
      TERN_(BELTPRINTER,           " Belt Printer")
      TERN_(MARKFORGED_XY,         " MarkForgedXY")
      TERN_(MARKFORGED_YX,         " MarkForgedYX")
      TERN_(POLAR,                 " Polar")
      TERN_(POLARGRAPH,            " Polargraph")
      TERN_(ARTICULATED_ROBOT_ARM, " Robot Arm")
      TERN_(FOAMCUTTER_XYUV,       " Foam Cutter")
      TERN_(IS_CARTESIAN,          " Cartesian")
    );

    SERIAL_ECHOLNPGM("Probe: "
      TERN_(PROBE_MANUALLY,        "PROBE_MANUALLY")
      TERN_(NOZZLE_AS_PROBE,       "NOZZLE_AS_PROBE")
      TERN_(FIX_MOUNTED_PROBE,     "FIX_MOUNTED_PROBE")
      TERN_(HAS_Z_SERVO_PROBE,     TERN(BLTOUCH, "BLTOUCH", "SERVO PROBE"))
      TERN_(BD_SENSOR,             "BD_SENSOR")
      TERN_(TOUCH_MI_PROBE,        "TOUCH_MI_PROBE")
      TERN_(Z_PROBE_ALLEN_KEY,     "Z_PROBE_ALLEN_KEY")
      TERN_(Z_PROBE_SLED,          "Z_PROBE_SLED")
      TERN_(RACK_AND_PINION_PROBE, "RACK_AND_PINION_PROBE")
      TERN_(SOLENOID_PROBE,        "SOLENOID_PROBE")
      TERN_(SENSORLESS_PROBING,    "SENSORLESS_PROBING")
      TERN_(MAGLEV4,               "MAGLEV4")
      TERN_(MAG_MOUNTED_PROBE,     "MAG_MOUNTED_PROBE")
      TERN_(BIQU_MICROPROBE_V1,    "BIQU_MICROPROBE_V1")
      TERN_(BIQU_MICROPROBE_V2,    "BIQU_MICROPROBE_V2")
      IF_DISABLED(PROBE_SELECTED,  "NONE")
    );

    #if HAS_BED_PROBE

      #if !HAS_PROBE_XY_OFFSET
        SERIAL_ECHOPGM("Probe Offset X0 Y0 Z", probe.offset.z, " (");
      #else
        SERIAL_ECHOPGM_P(PSTR("Probe Offset X"), probe.offset_xy.x, SP_Y_STR, probe.offset_xy.y, SP_Z_STR, probe.offset.z);
        if (probe.offset_xy.x > 0)
          SERIAL_ECHOPGM(" (Right");
        else if (probe.offset_xy.x < 0)
          SERIAL_ECHOPGM(" (Left");
        else if (probe.offset_xy.y != 0)
          SERIAL_ECHOPGM(" (Middle");
        else
          SERIAL_ECHOPGM(" (Aligned With");

        if (probe.offset_xy.y > 0)
          SERIAL_ECHO(F(TERN(IS_SCARA, "-Distal", "-Back")));
        else if (probe.offset_xy.y < 0)
          SERIAL_ECHO(F(TERN(IS_SCARA, "-Proximal", "-Front")));
        else if (probe.offset_xy.x != 0)
          SERIAL_ECHOPGM("-Center");

        SERIAL_ECHOPGM(" & ");

      #endif

      SERIAL_ECHO(probe.offset.z < 0 ? F("Below") : probe.offset.z > 0 ? F("Above") : F("Same Z as"));
      SERIAL_ECHOLNPGM(" Nozzle)");

    #endif

    #if HAS_ABL_OR_UBL
      SERIAL_ECHOPGM("Auto Bed Leveling: "
        TERN_(AUTO_BED_LEVELING_LINEAR, "LINEAR")
        TERN_(AUTO_BED_LEVELING_BILINEAR, "BILINEAR")
        TERN_(AUTO_BED_LEVELING_3POINT, "3POINT")
        TERN_(AUTO_BED_LEVELING_UBL, "UBL")
      );

      if (planner.leveling_active) {
        SERIAL_ECHOLNPGM(" (enabled)");
        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          if (planner.z_fade_height)
            SERIAL_ECHOLNPGM("Z Fade: ", planner.z_fade_height);
        #endif
        #if ABL_PLANAR
          SERIAL_ECHOPGM("ABL Adjustment");
          LOOP_NUM_AXES(a) {
            SERIAL_ECHOPGM_P((PGM_P)pgm_read_ptr(&SP_AXIS_STR[a]));
            serial_offset(planner.get_axis_position_mm(AxisEnum(a)) - current_position[a]);
          }
        #else
          #if ENABLED(AUTO_BED_LEVELING_UBL)
            SERIAL_ECHOPGM("UBL Adjustment Z");
          #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
            SERIAL_ECHOPGM("ABL Adjustment Z");
          #endif
          const float rz = bedlevel.get_z_correction(current_position);
          SERIAL_ECHO(ftostr43sign(rz, '+'));
          #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
            if (planner.z_fade_height) {
              SERIAL_ECHOPGM(" (", ftostr43sign(rz * planner.fade_scaling_factor_for_z(current_position.z), '+'));
              SERIAL_CHAR(')');
            }
          #endif
        #endif
      }
      else
        SERIAL_ECHOLNPGM(" (disabled)");

      SERIAL_EOL();

    #elif ENABLED(MESH_BED_LEVELING)

      SERIAL_ECHOPGM("Mesh Bed Leveling");
      if (planner.leveling_active) {
        SERIAL_ECHOLNPGM(" (enabled)");
        const float z_offset = bedlevel.get_z_offset(),
                    z_correction = bedlevel.get_z_correction(current_position);
        SERIAL_ECHOPGM("MBL Adjustment Z", ftostr43sign(z_offset + z_correction, '+'));
        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          if (planner.z_fade_height) {
            SERIAL_ECHOPGM(" (", ftostr43sign(
              z_offset + z_correction * planner.fade_scaling_factor_for_z(current_position.z), '+'
            ));
            SERIAL_CHAR(')');
          }
        #endif
      }
      else
        SERIAL_ECHOPGM(" (disabled)");

      SERIAL_EOL();

    #endif // MESH_BED_LEVELING
  }

#endif // DEBUG_LEVELING_FEATURE
