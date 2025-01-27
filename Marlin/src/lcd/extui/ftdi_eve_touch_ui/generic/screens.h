/*************
 * screens.h *
 *************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

#pragma once

/********************************* DL CACHE SLOTS ******************************/

// In order to reduce SPI traffic, we cache display lists (DL) in RAMG. This
// is done using the CLCD::DLCache class, which takes a unique ID for each
// cache location. These IDs are defined here:

enum {
  STATUS_SCREEN_CACHE,
  MENU_SCREEN_CACHE,
  TUNE_SCREEN_CACHE,
  ALERT_BOX_CACHE,
  FILAMENT_PROMPT_BOX_CACHE,
  SPINNER_CACHE,
  ADVANCED_SETTINGS_SCREEN_CACHE,
  MOVE_AXIS_SCREEN_CACHE,
  TEMPERATURE_SCREEN_CACHE,
  STEPS_SCREEN_CACHE,
  MAX_FEEDRATE_SCREEN_CACHE,
  MAX_VELOCITY_SCREEN_CACHE,
  MAX_ACCELERATION_SCREEN_CACHE,
  DEFAULT_ACCELERATION_SCREEN_CACHE,
  FLOW_PERCENT_SCREEN_CACHE,
  #if HAS_LEVELING
    LEVELING_SCREEN_CACHE,
    #if HAS_BED_PROBE
      ZOFFSET_SCREEN_CACHE,
    #endif
    #if HAS_MESH
      BED_MESH_VIEW_SCREEN_CACHE,
      BED_MESH_EDIT_SCREEN_CACHE,
    #endif
  #endif
  #if ENABLED(BABYSTEPPING)
    ADJUST_OFFSETS_SCREEN_CACHE,
  #endif
  #if HAS_TRINAMIC_CONFIG
    STEPPER_CURRENT_SCREEN_CACHE,
    STEPPER_BUMP_SENSITIVITY_SCREEN_CACHE,
  #endif
  #if HAS_MULTI_HOTEND
    NOZZLE_OFFSET_SCREEN_CACHE,
  #endif
  #if ENABLED(BACKLASH_GCODE)
    BACKLASH_COMPENSATION_SCREEN_CACHE,
  #endif
  #if HAS_JUNCTION_DEVIATION
    JUNC_DEV_SCREEN_CACHE,
  #else
    JERK_SCREEN_CACHE,
  #endif
  #if ENABLED(CASE_LIGHT_ENABLE)
    CASE_LIGHT_SCREEN_CACHE,
  #endif
  #if ANY(LIN_ADVANCE, HAS_FILAMENT_SENSOR)
    FILAMENT_MENU_CACHE,
  #endif
  #if ENABLED(LIN_ADVANCE)
    LINEAR_ADVANCE_SCREEN_CACHE,
  #endif
  #if HAS_FILAMENT_SENSOR
    FILAMENT_RUNOUT_SCREEN_CACHE,
  #endif
  #if HAS_MEDIA
    FILES_SCREEN_CACHE,
  #endif
  #if ENABLED(CUSTOM_MENU_MAIN)
    CUSTOM_USER_MENUS_SCREEN_CACHE,
  #endif
  CHANGE_FILAMENT_SCREEN_CACHE,
  INTERFACE_SETTINGS_SCREEN_CACHE,
  INTERFACE_SOUNDS_SCREEN_CACHE,
  LOCK_SCREEN_CACHE,
  DISPLAY_TIMINGS_SCREEN_CACHE
};

// To save MCU RAM, the status message is "baked" in to the status screen
// cache, so we reserve a large chunk of memory for the DL cache

#define STATUS_SCREEN_DL_SIZE        4096
#define ALERT_BOX_DL_SIZE            3072
#define FILAMENT_PROMPT_BOX_DL_SIZE  3072
#define SPINNER_DL_SIZE              3072
#define FILE_SCREEN_DL_SIZE          4160
#define PRINTING_SCREEN_DL_SIZE      2048

/************************* MENU SCREEN DECLARATIONS *************************/

#include "base_screen.h"
#include "base_numeric_adjustment_screen.h"
#include "dialog_box_base_class.h"
#include "status_screen.h"
#include "main_menu.h"
#include "advanced_settings_menu.h"
#include "tune_menu.h"
#include "boot_screen.h"
#include "about_screen.h"
#include "kill_screen.h"
#include "alert_dialog_box.h"
#include "filament_prompt_dialog_box.h"
#include "spinner_dialog_box.h"
#include "restore_failsafe_dialog_box.h"
#include "save_settings_dialog_box.h"
#include "confirm_start_print_dialog_box.h"
#include "confirm_abort_print_dialog_box.h"
#include "confirm_user_request_alert_box.h"
#include "filament_prompt_box.h"
#include "touch_calibration_screen.h"
#include "touch_registers_screen.h"
#include "change_filament_screen.h"
#include "move_axis_screen.h"
#include "steps_screen.h"
#include "feedrate_percent_screen.h"
#include "max_velocity_screen.h"
#include "max_acceleration_screen.h"
#include "default_acceleration_screen.h"
#include "temperature_screen.h"
#include "interface_sounds_screen.h"
#include "interface_settings_screen.h"
#include "lock_screen.h"
#include "endstop_state_screen.h"
#include "display_tuning_screen.h"
#include "media_player_screen.h"
#include "flow_percent_screen.h"

#if ENABLED(PRINTCOUNTER)
  #include "statistics_screen.h"
#endif

#if HAS_TRINAMIC_CONFIG
  #include "stepper_current_screen.h"
  #include "stepper_bump_sensitivity_screen.h"
#endif

#if HAS_MULTI_HOTEND
  #include "nozzle_offsets_screen.h"
#endif

#if HAS_LEVELING
  #if ENABLED(TOUCH_UI_SYNDAVER_LEVEL)
    #include "syndaver_level/leveling_menu.h"
  #else
    #include "leveling_menu.h"
  #endif
  #if HAS_BED_PROBE
    #include "z_offset_screen.h"
  #endif
  #if HAS_MESH
    #include "bed_mesh_base.h"
    #include "bed_mesh_view_screen.h"
    #include "bed_mesh_edit_screen.h"
  #endif
#endif

#if ENABLED(CALIBRATION_GCODE)
  #include "confirm_auto_calibration_dialog_box.h"
#endif

#if ENABLED(BABYSTEPPING)
  #include "nudge_nozzle_screen.h"
#endif

#if ENABLED(BACKLASH_GCODE)
  #include "backlash_compensation_screen.h"
#endif

#if HAS_JUNCTION_DEVIATION
  #include "junction_deviation_screen.h"
#else
  #include "jerk_screen.h"
#endif

#if ENABLED(CASE_LIGHT_ENABLE)
  #include "case_light_screen.h"
#endif

#if ANY(LIN_ADVANCE, HAS_FILAMENT_SENSOR)
  #include "filament_menu.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "filament_runout_screen.h"
#endif

#if ENABLED(LIN_ADVANCE)
  #include "linear_advance_screen.h"
#endif

#if HAS_MEDIA
  #include "files_screen.h"
#endif

#if ENABLED(CUSTOM_MENU_MAIN)
  #include "custom_user_menus.h"
#endif

#if ENABLED(TOUCH_UI_DEVELOPER_MENU)
  #include "developer_menu.h"
  #include "confirm_erase_flash_dialog_box.h"
  #include "widget_demo_screen.h"
  #include "stress_test_screen.h"
#endif

#if NUM_LANGUAGES > 1
  #include "language_menu.h"
#endif
