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
 * feature/powerloss.h - Resume an SD print after power-loss
 */

#include "../sd/cardreader.h"
#include "../gcode/gcode.h"

#include "../inc/MarlinConfig.h"

#if ENABLED(CANCEL_OBJECTS)
  #include "cancel_object.h"
#endif

#if ENABLED(GCODE_REPEAT_MARKERS)
  #include "repeat.h"
#endif

#if ENABLED(MIXING_EXTRUDER)
  #include "mixing.h"
#endif

#if !defined(POWER_LOSS_STATE) && PIN_EXISTS(POWER_LOSS)
  #define POWER_LOSS_STATE HIGH
#endif

#if DISABLED(BACKUP_POWER_SUPPLY)
  #undef POWER_LOSS_ZRAISE    // No Z raise at outage without backup power
#endif
#ifndef POWER_LOSS_ZRAISE
  #define POWER_LOSS_ZRAISE 2 // Default Z-raise on outage or resume
#endif

//#define DEBUG_POWER_LOSS_RECOVERY
//#define SAVE_EACH_CMD_MODE
//#define SAVE_INFO_INTERVAL_MS 0

typedef struct {
  uint8_t valid_head;

  // Machine state
  xyze_pos_t current_position;
  uint16_t feedrate;
  int16_t feedrate_percentage;
  uint16_t flow_percentage[EXTRUDERS];

  float zraise;

  // Canceled objects
  #if ENABLED(CANCEL_OBJECTS)
    cancel_state_t cancel_state;
  #endif

  // Repeat information
  #if ENABLED(GCODE_REPEAT_MARKERS)
    Repeat stored_repeat;
  #endif

  #if HAS_HOME_OFFSET
    xyz_pos_t home_offset;
  #endif
  #if HAS_WORKSPACE_OFFSET
    xyz_pos_t workspace_offset;
  #endif
  #if HAS_MULTI_EXTRUDER
    uint8_t active_extruder;
  #endif

  #if DISABLED(NO_VOLUMETRICS)
    float filament_size[EXTRUDERS];
  #endif

  #if HAS_HOTEND
    celsius_t target_temperature[HOTENDS];
  #endif
  #if HAS_HEATED_BED
    celsius_t target_temperature_bed;
  #endif
  #if HAS_HEATED_CHAMBER
    celsius_t target_temperature_chamber;
  #endif
  #if HAS_FAN
    uint8_t fan_speed[FAN_COUNT];
  #endif

  #if HAS_LEVELING
    float fade;
  #endif

  #if ENABLED(FWRETRACT)
    float retract[EXTRUDERS], retract_hop;
  #endif

  // Mixing extruder and gradient
  #if ENABLED(MIXING_EXTRUDER)
    //uint_fast8_t selected_vtool;
    //mixer_comp_t color[NR_MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];
    #if ENABLED(GRADIENT_MIX)
      gradient_t gradient;
    #endif
  #endif

  // SD Filename and position
  char sd_filename[MAXPATHNAMELENGTH];
  volatile uint32_t sdpos;

  // Job elapsed time
  millis_t print_job_elapsed;

  // Relative axis modes
  relative_t axis_relative;

  // Misc. Marlin flags
  struct {
    bool raised:1;                // Raised before saved
    bool dryrun:1;                // M111 S8
    bool allow_cold_extrusion:1;  // M302 P1
    #if HAS_LEVELING
      bool leveling:1;            // M420 S
    #endif
    #if DISABLED(NO_VOLUMETRICS)
      bool volumetric_enabled:1;  // M200 S D
    #endif
  } flag;

  uint8_t valid_foot;

  bool valid() { return valid_head && valid_head == valid_foot; }

} job_recovery_info_t;

class PrintJobRecovery {
  public:
    static const char filename[5];

    static MediaFile file;
    static job_recovery_info_t info;

    static uint8_t queue_index_r;     //!< Queue index of the active command
    static uint32_t cmd_sdpos,        //!< SD position of the next command
                    sdpos[BUFSIZE];   //!< SD positions of queued commands

    #if HAS_PLR_UI_FLAG
      static bool ui_flag_resume;     //!< Flag the UI to show a dialog to Resume (M1000) or Cancel (M1000C)
    #endif

    static void init();
    static void prepare();

    static void setup() {
      #if PIN_EXISTS(OUTAGECON)
        OUT_WRITE(OUTAGECON_PIN, HIGH);
      #endif
      #if PIN_EXISTS(POWER_LOSS)
        #if ENABLED(POWER_LOSS_PULLUP)
          SET_INPUT_PULLUP(POWER_LOSS_PIN);
        #elif ENABLED(POWER_LOSS_PULLDOWN)
          SET_INPUT_PULLDOWN(POWER_LOSS_PIN);
        #else
          SET_INPUT(POWER_LOSS_PIN);
        #endif
      #endif
    }

    // Track each command's file offsets
    static uint32_t command_sdpos() { return sdpos[queue_index_r]; }
    static void commit_sdpos(const uint8_t index_w) { sdpos[index_w] = cmd_sdpos; }

    static bool enabled;
    static void enable(const bool onoff);
    static void changed();

    #if HAS_PLR_BED_THRESHOLD
      static celsius_t bed_temp_threshold;
    #endif

    static bool exists() { return card.jobRecoverFileExists(); }
    static void open(const bool read) { card.openJobRecoveryFile(read); }
    static void close() { file.close(); }

    static bool check();
    static void resume();
    static void purge();

    static void cancel() { purge(); }

    static void load();
    static void save(const bool force=ENABLED(SAVE_EACH_CMD_MODE), const float zraise=POWER_LOSS_ZRAISE, const bool raised=false);

    #if PIN_EXISTS(POWER_LOSS)
      static void outage() {
        static constexpr uint8_t OUTAGE_THRESHOLD = 3;
        static uint8_t outage_counter = 0;
        if (enabled && READ(POWER_LOSS_PIN) == POWER_LOSS_STATE) {
          outage_counter++;
          if (outage_counter >= OUTAGE_THRESHOLD) _outage();
        }
        else
          outage_counter = 0;
      }
    #endif

    // The referenced file exists
    static bool interrupted_file_exists() { return card.fileExists(info.sd_filename); }

    static bool valid() { return info.valid() && interrupted_file_exists(); }

    #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
      static void debug(FSTR_P const prefix);
    #else
      static void debug(FSTR_P const) {}
    #endif

  private:
    static void write();

    #if ENABLED(BACKUP_POWER_SUPPLY)
      static void retract_and_lift(const_float_t zraise);
    #endif

    #if PIN_EXISTS(POWER_LOSS) || ENABLED(DEBUG_POWER_LOSS_RECOVERY)
      friend class GcodeSuite;
      static void _outage(TERN_(DEBUG_POWER_LOSS_RECOVERY, const bool simulated=false));
    #endif
};

extern PrintJobRecovery recovery;
