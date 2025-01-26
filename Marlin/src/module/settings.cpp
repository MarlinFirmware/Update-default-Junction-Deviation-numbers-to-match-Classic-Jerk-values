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
 * settings.cpp
 *
 * Settings and EEPROM storage
 *
 * IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
 * in the functions below, also increment the version number. This makes sure that
 * the default values are used whenever there is a change to the data, to prevent
 * wrong data being written to the variables.
 *
 * ALSO: Variables in the Store and Retrieve sections must be in the same order.
 *       If a feature is disabled, some data must still be written that, when read,
 *       either sets a Sane Default, or results in No Change to the existing value.
 */

// Change EEPROM version if the structure changes
#define EEPROM_VERSION "V90"
#define EEPROM_OFFSET 100

// Check the integrity of data offsets.
// Can be disabled for production build.
//#define DEBUG_EEPROM_READWRITE
//#define DEBUG_EEPROM_OBSERVE

#include "settings.h"

#include "endstops.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"

#include "../lcd/marlinui.h"
#include "../libs/vector_3.h"   // for matrix_3x3
#include "../gcode/gcode.h"
#include "../MarlinCore.h"

#if ANY(EEPROM_SETTINGS, SD_FIRMWARE_UPDATE)
  #include "../HAL/shared/eeprom_api.h"
#endif

#if HAS_BED_PROBE
  #include "probe.h"
#endif

#if HAS_LEVELING
  #include "../feature/bedlevel/bedlevel.h"
  #if ENABLED(X_AXIS_TWIST_COMPENSATION)
    #include "../feature/x_twist.h"
  #endif
#endif

#if ENABLED(Z_STEPPER_AUTO_ALIGN)
  #include "../feature/z_stepper_align.h"
#endif

#if ENABLED(DWIN_LCD_PROUI)
  #include "../lcd/e3v2/proui/dwin.h"
  #include "../lcd/e3v2/proui/bedlevel_tools.h"
#endif

#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extui/ui_api.h"
#elif ENABLED(DWIN_CREALITY_LCD_JYERSUI)
  #include "../lcd/e3v2/jyersui/dwin.h"
#endif

#if ENABLED(HOST_PROMPT_SUPPORT)
  #include "../feature/host_actions.h"
#endif

#if HAS_SERVOS
  #include "servo.h"
#endif

#include "../feature/fwretract.h"

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../feature/powerloss.h"
#endif

#if HAS_POWER_MONITOR
  #include "../feature/power_monitor.h"
#endif

#include "../feature/pause.h"

#if ENABLED(BACKLASH_COMPENSATION)
  #include "../feature/backlash.h"
#endif

#if ENABLED(FT_MOTION)
  #include "../module/ft_motion.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "../feature/runout.h"
  #ifndef FIL_RUNOUT_ENABLED_DEFAULT
    #define FIL_RUNOUT_ENABLED_DEFAULT true
  #endif
#endif

#if ENABLED(ADVANCE_K_EXTRA)
  extern float other_extruder_advance_K[DISTINCT_E];
#endif

#if HAS_MULTI_EXTRUDER
  #include "tool_change.h"
  void M217_report(const bool eeprom);
#endif

#if ENABLED(BLTOUCH)
  #include "../feature/bltouch.h"
#endif

#if HAS_TRINAMIC_CONFIG
  #include "stepper/indirection.h"
  #include "../feature/tmc_util.h"
#endif

#if HAS_PTC
  #include "../feature/probe_temp_comp.h"
#endif

#include "../feature/controllerfan.h"

#if ENABLED(CASE_LIGHT_ENABLE)
  #include "../feature/caselight.h"
#endif

#if ENABLED(PASSWORD_FEATURE)
  #include "../feature/password/password.h"
#endif

#if ENABLED(TOUCH_SCREEN_CALIBRATION)
  #include "../lcd/tft_io/touch_calibration.h"
#endif

#if HAS_ETHERNET
  #include "../feature/ethernet.h"
#endif

#if ENABLED(SOUND_MENU_ITEM)
  #include "../libs/buzzer.h"
#endif

#if HAS_FANCHECK
  #include "../feature/fancheck.h"
#endif

#if DGUS_LCD_UI_MKS
  #include "../lcd/extui/dgus/DGUSScreenHandler.h"
  #include "../lcd/extui/dgus/DGUSDisplayDef.h"
#endif

#if ENABLED(HOTEND_IDLE_TIMEOUT)
  #include "../feature/hotend_idle.h"
#endif

#if HAS_PRUSA_MMU3
  #include "../feature/mmu3/mmu3.h"
  #include "../feature/mmu3/SpoolJoin.h"
  #include "../feature/mmu3/mmu3_reporting.h"
#endif

#pragma pack(push, 1) // No padding between variables

#if HAS_ETHERNET
  void ETH0_report();
  void MAC_report();
#endif

#define _EN_ITEM(N) , E##N
#define _EN1_ITEM(N) , E##N:1

typedef struct { uint16_t MAIN_AXIS_NAMES_ X2, Y2, Z2, Z3, Z4 REPEAT(E_STEPPERS, _EN_ITEM); } per_stepper_uint16_t;
typedef struct { uint32_t MAIN_AXIS_NAMES_ X2, Y2, Z2, Z3, Z4 REPEAT(E_STEPPERS, _EN_ITEM); } per_stepper_uint32_t;
typedef struct {  int16_t MAIN_AXIS_NAMES_ X2, Y2, Z2, Z3, Z4;                              } mot_stepper_int16_t;
typedef struct {     bool NUM_AXIS_LIST_(X:1, Y:1, Z:1, I:1, J:1, K:1, U:1, V:1, W:1) X2:1, Y2:1, Z2:1, Z3:1, Z4:1 REPEAT(E_STEPPERS, _EN1_ITEM); } per_stepper_bool_t;

#undef _EN_ITEM

// Limit an index to an array size
#define ALIM(I,ARR) _MIN(I, (signed)COUNT(ARR) - 1)

// Defaults for reset / fill in on load
static const uint32_t   _DMA[] PROGMEM = DEFAULT_MAX_ACCELERATION;
static const feedRate_t _DMF[] PROGMEM = DEFAULT_MAX_FEEDRATE;
#if ENABLED(EDITABLE_STEPS_PER_UNIT)
  static const float   _DASU[] PROGMEM = DEFAULT_AXIS_STEPS_PER_UNIT;
#endif

/**
 * Current EEPROM Layout
 *
 * Keep this data structure up to date so
 * EEPROM size is known at compile time!
 */
typedef struct SettingsDataStruct {
  char      version[4];                                 // Vnn\0
  #if ENABLED(EEPROM_INIT_NOW)
    uint32_t build_hash;                                // Unique build hash
  #endif
  uint16_t  crc;                                        // Data Checksum for validation
  uint16_t  data_size;                                  // Data Size for validation

  //
  // DISTINCT_E_FACTORS
  //
  uint8_t e_factors;                                    // DISTINCT_AXES - NUM_AXES

  //
  // Planner settings
  //
  planner_settings_t planner_settings;

  xyze_float_t planner_max_jerk;                        // M205 XYZE  planner.max_jerk
  float planner_junction_deviation_mm;                  // M205 J     planner.junction_deviation_mm

  //
  // Home Offset
  //
  #if NUM_AXES
    xyz_pos_t home_offset;                              // M206 XYZ / M665 TPZ
  #endif

  //
  // Hotend Offset
  //
  #if HAS_HOTEND_OFFSET
    xyz_pos_t hotend_offset[HOTENDS - 1];               // M218 XYZ
  #endif

  //
  // FILAMENT_RUNOUT_SENSOR
  //
  bool runout_sensor_enabled;                           // M412 S
  float runout_distance_mm;                             // M412 D

  //
  // ENABLE_LEVELING_FADE_HEIGHT
  //
  float planner_z_fade_height;                          // M420 Zn  planner.z_fade_height

  //
  // AUTOTEMP
  //
  #if ENABLED(AUTOTEMP)
    celsius_t planner_autotemp_max, planner_autotemp_min;
    float planner_autotemp_factor;
  #endif

  //
  // MESH_BED_LEVELING
  //
  float mbl_z_offset;                                   // bedlevel.z_offset
  uint8_t mesh_num_x, mesh_num_y;                       // GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y
  uint16_t mesh_check;                                  // Hash to check against X/Y
  float mbl_z_values[TERN(MESH_BED_LEVELING, GRID_MAX_POINTS_X, 3)]   // bedlevel.z_values
                    [TERN(MESH_BED_LEVELING, GRID_MAX_POINTS_Y, 3)];

  //
  // HAS_BED_PROBE
  //
  #if NUM_AXES
    xyz_pos_t probe_offset;                             // M851 X Y Z
  #endif

  //
  // ABL_PLANAR
  //
  matrix_3x3 planner_bed_level_matrix;                  // planner.bed_level_matrix

  //
  // AUTO_BED_LEVELING_BILINEAR
  //
  uint8_t grid_max_x, grid_max_y;                       // GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y
  uint16_t grid_check;                                  // Hash to check against X/Y
  xy_pos_t bilinear_grid_spacing, bilinear_start;       // G29 L F
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    bed_mesh_t z_values;                                // G29
  #else
    float z_values[3][3];
  #endif

  //
  // X_AXIS_TWIST_COMPENSATION
  //
  #if ENABLED(X_AXIS_TWIST_COMPENSATION)
    float xatc_spacing;                                 // M423 X Z
    float xatc_start;
    xatc_array_t xatc_z_offset;
  #endif

  //
  // AUTO_BED_LEVELING_UBL
  //
  bool planner_leveling_active;                         // M420 S  planner.leveling_active
  int8_t ubl_storage_slot;                              // bedlevel.storage_slot

  //
  // SERVO_ANGLES
  //
  #if HAS_SERVO_ANGLES
    uint16_t servo_angles[NUM_SERVOS][2];               // M281 P L U
  #endif

  //
  // Temperature first layer compensation values
  //
  #if HAS_PTC
    #if ENABLED(PTC_PROBE)
      int16_t z_offsets_probe[COUNT(ptc.z_offsets_probe)]; // M871 P I V
    #endif
    #if ENABLED(PTC_BED)
      int16_t z_offsets_bed[COUNT(ptc.z_offsets_bed)];     // M871 B I V
    #endif
    #if ENABLED(PTC_HOTEND)
      int16_t z_offsets_hotend[COUNT(ptc.z_offsets_hotend)]; // M871 E I V
    #endif
  #endif

  //
  // BLTOUCH
  //
  bool bltouch_od_5v_mode;
  #if HAS_BLTOUCH_HS_MODE
    bool bltouch_high_speed_mode;                       // M401 S
  #endif

  //
  // Kinematic Settings (Delta, SCARA, TPARA, Polargraph...)
  //
  #if IS_KINEMATIC
    float segments_per_second;                          // M665 S
    #if ENABLED(DELTA)
      float delta_height;                               // M666 H
      abc_float_t delta_endstop_adj;                    // M666 X Y Z
      float delta_radius,                               // M665 R
            delta_diagonal_rod;                         // M665 L
      abc_float_t delta_tower_angle_trim,               // M665 X Y Z
                  delta_diagonal_rod_trim;              // M665 A B C
    #elif ENABLED(POLARGRAPH)
      xy_pos_t draw_area_min, draw_area_max;            // M665 L R T B
      float polargraph_max_belt_len;                    // M665 H
    #endif

  #endif

  //
  // Extra Endstops offsets
  //
  #if HAS_EXTRA_ENDSTOPS
    float x2_endstop_adj,                               // M666 X
          y2_endstop_adj,                               // M666 Y
          z2_endstop_adj,                               // M666 (S2) Z
          z3_endstop_adj,                               // M666 (S3) Z
          z4_endstop_adj;                               // M666 (S4) Z
  #endif

  //
  // Z_STEPPER_AUTO_ALIGN, HAS_Z_STEPPER_ALIGN_STEPPER_XY
  //
  #if ENABLED(Z_STEPPER_AUTO_ALIGN)
    xy_pos_t z_stepper_align_xy[NUM_Z_STEPPERS];             // M422 S X Y
    #if HAS_Z_STEPPER_ALIGN_STEPPER_XY
      xy_pos_t z_stepper_align_stepper_xy[NUM_Z_STEPPERS];   // M422 W X Y
    #endif
  #endif

  //
  // Material Presets
  //
  #if HAS_PREHEAT
    preheat_t ui_material_preset[PREHEAT_COUNT];        // M145 S0 H B F
  #endif

  //
  // PIDTEMP
  //
  raw_pidcf_t hotendPID[HOTENDS];                       // M301 En PIDCF / M303 En U
  int16_t lpq_len;                                      // M301 L

  //
  // PIDTEMPBED
  //
  raw_pid_t bedPID;                                     // M304 PID / M303 E-1 U

  //
  // PIDTEMPCHAMBER
  //
  raw_pid_t chamberPID;                                 // M309 PID / M303 E-2 U

  //
  // User-defined Thermistors
  //
  #if HAS_USER_THERMISTORS
    user_thermistor_t user_thermistor[USER_THERMISTORS]; // M305 P0 R4700 T100000 B3950
  #endif

  //
  // Power monitor
  //
  uint8_t power_monitor_flags;                          // M430 I V W

  //
  // HAS_LCD_CONTRAST
  //
  uint8_t lcd_contrast;                                 // M250 C

  //
  // HAS_LCD_BRIGHTNESS
  //
  uint8_t lcd_brightness;                               // M256 B

  //
  // Display Sleep
  //
  #if ENABLED(EDITABLE_DISPLAY_TIMEOUT)
    #if HAS_BACKLIGHT_TIMEOUT
      uint8_t backlight_timeout_minutes;                // M255 S
    #elif HAS_DISPLAY_SLEEP
      uint8_t sleep_timeout_minutes;                    // M255 S
    #endif
  #endif

  //
  // Controller fan settings
  //
  controllerFan_settings_t controllerFan_settings;      // M710

  //
  // POWER_LOSS_RECOVERY
  //
  bool recovery_enabled;                                // M413 S
  celsius_t bed_temp_threshold;                         // M413 B

  //
  // FWRETRACT
  //
  fwretract_settings_t fwretract_settings;              // M207 S F Z W, M208 S F W R
  bool autoretract_enabled;                             // M209 S

  //
  // EDITABLE_HOMING_FEEDRATE
  //
  #if ENABLED(EDITABLE_HOMING_FEEDRATE)
    xyz_feedrate_t homing_feedrate_mm_m;                // M210 X Y Z I J K U V W
  #endif

  //
  // !NO_VOLUMETRIC
  //
  bool parser_volumetric_enabled;                       // M200 S  parser.volumetric_enabled
  float planner_filament_size[EXTRUDERS];               // M200 T D  planner.filament_size[]
  float planner_volumetric_extruder_limit[EXTRUDERS];   // M200 T L  planner.volumetric_extruder_limit[]

  //
  // HAS_TRINAMIC_CONFIG
  //
  per_stepper_uint16_t tmc_stepper_current;             // M906 X Y Z...
  per_stepper_uint32_t tmc_hybrid_threshold;            // M913 X Y Z...
  mot_stepper_int16_t tmc_sgt;                          // M914 X Y Z...
  per_stepper_bool_t tmc_stealth_enabled;               // M569 X Y Z...

  //
  // LIN_ADVANCE
  //
  float planner_extruder_advance_K[DISTINCT_E]; // M900 K  planner.extruder_advance_K

  //
  // HAS_MOTOR_CURRENT_PWM
  //
  #ifndef MOTOR_CURRENT_COUNT
    #if HAS_MOTOR_CURRENT_PWM
      #define MOTOR_CURRENT_COUNT 3
    #elif HAS_MOTOR_CURRENT_DAC
      #define MOTOR_CURRENT_COUNT LOGICAL_AXES
    #elif HAS_MOTOR_CURRENT_I2C
      #define MOTOR_CURRENT_COUNT DIGIPOT_I2C_NUM_CHANNELS
    #else // HAS_MOTOR_CURRENT_SPI
      #define MOTOR_CURRENT_COUNT DISTINCT_AXES
    #endif
  #endif
  uint32_t motor_current_setting[MOTOR_CURRENT_COUNT];  // M907 X Z E ...

  //
  // Adaptive Step Smoothing state
  //
  #if ENABLED(ADAPTIVE_STEP_SMOOTHING_TOGGLE)
    bool adaptive_step_smoothing_enabled;               // G-code pending
  #endif

  //
  // CNC_COORDINATE_SYSTEMS
  //
  #if NUM_AXES
    xyz_pos_t coordinate_system[MAX_COORDINATE_SYSTEMS]; // G54-G59.3
  #endif

  //
  // SKEW_CORRECTION
  //
  #if ENABLED(SKEW_CORRECTION)
    skew_factor_t planner_skew_factor;                  // M852 I J K
  #endif

  //
  // ADVANCED_PAUSE_FEATURE
  //
  #if ENABLED(CONFIGURE_FILAMENT_CHANGE)
    fil_change_settings_t fc_settings[EXTRUDERS];       // M603 T U L
  #endif

  //
  // Tool-change settings
  //
  #if HAS_MULTI_EXTRUDER
    toolchange_settings_t toolchange_settings;          // M217 S P R
  #endif

  //
  // BACKLASH_COMPENSATION
  //
  #if NUM_AXES
    xyz_float_t backlash_distance_mm;                   // M425 X Y Z
    uint8_t backlash_correction;                        // M425 F
    float backlash_smoothing_mm;                        // M425 S
  #endif

  //
  // EXTENSIBLE_UI
  //
  #if ENABLED(EXTENSIBLE_UI)
    uint8_t extui_data[ExtUI::eeprom_data_size];
  #endif

  //
  // Ender-3 V2 DWIN
  //
  #if ENABLED(DWIN_CREALITY_LCD_JYERSUI)
    uint8_t dwin_settings[jyersDWIN.eeprom_data_size];
  #endif

  //
  // CASELIGHT_USES_BRIGHTNESS
  //
  #if CASELIGHT_USES_BRIGHTNESS
    uint8_t caselight_brightness;                        // M355 P
  #endif

  //
  // PASSWORD_FEATURE
  //
  #if ENABLED(PASSWORD_FEATURE)
    bool password_is_set;
    uint32_t password_value;
  #endif

  //
  // TOUCH_SCREEN_CALIBRATION
  //
  #if ENABLED(TOUCH_SCREEN_CALIBRATION)
    touch_calibration_t touch_calibration_data;
  #endif

  // Ethernet settings
  #if HAS_ETHERNET
    bool ethernet_hardware_enabled;                     // M552 S
    uint32_t ethernet_ip,                               // M552 P
             ethernet_dns,
             ethernet_gateway,                          // M553 P
             ethernet_subnet;                           // M554 P
  #endif

  //
  // Buzzer enable/disable
  //
  #if ENABLED(SOUND_MENU_ITEM)
    bool sound_on;
  #endif

  //
  // Fan tachometer check
  //
  #if HAS_FANCHECK
    bool fan_check_enabled;
  #endif

  //
  // MKS UI controller
  //
  #if DGUS_LCD_UI_MKS
    MKS_Language mks_language_index;                    // Display Language
    xy_int_t mks_corner_offsets[5];                     // Bed Tramming
    xyz_int_t mks_park_pos;                             // Custom Parking (without NOZZLE_PARK)
    celsius_t mks_min_extrusion_temp;                   // Min E Temp (shadow M302 value)
  #endif

  #if HAS_MULTI_LANGUAGE
    uint8_t ui_language;                                // M414 S
  #endif

  //
  // Model predictive control
  //
  #if ENABLED(MPCTEMP)
    MPC_t mpc_constants[HOTENDS];                       // M306
  #endif

  //
  // Fixed-Time Motion
  //
  #if ENABLED(FT_MOTION)
    ft_config_t ftMotion_cfg;                          // M493
  #endif

  //
  // Input Shaping
  //
  #if ENABLED(INPUT_SHAPING_X)
    float shaping_x_frequency,                          // M593 X F
          shaping_x_zeta;                               // M593 X D
  #endif
  #if ENABLED(INPUT_SHAPING_Y)
    float shaping_y_frequency,                          // M593 Y F
          shaping_y_zeta;                               // M593 Y D
  #endif
  #if ENABLED(INPUT_SHAPING_Z)
    float shaping_z_frequency,                          // M593 Z F
          shaping_z_zeta;                               // M593 Z D
  #endif

  //
  // HOTEND_IDLE_TIMEOUT
  //
  #if ENABLED(HOTEND_IDLE_TIMEOUT)
    hotend_idle_settings_t hotend_idle_config;          // M86 S T E B
  #endif

  //
  // Nonlinear Extrusion
  //
  #if ENABLED(NONLINEAR_EXTRUSION)
    ne_coeff_t stepper_ne;                              // M592 A B C
  #endif

  //
  // MMU3
  //
  #if HAS_PRUSA_MMU3
    bool spool_join_enabled;      // EEPROM_SPOOL_JOIN
    uint16_t fail_total_num;      // EEPROM_MMU_FAIL_TOT
    uint8_t fail_num;             // EEPROM_MMU_FAIL
    uint16_t load_fail_total_num; // EEPROM_MMU_LOAD_FAIL_TOT
    uint8_t load_fail_num;        // EEPROM_MMU_LOAD_FAIL
    uint16_t tool_change_counter;
    uint32_t tool_change_total_counter; // EEPROM_MMU_MATERIAL_CHANGES
    uint8_t cutter_mode;          // EEPROM_MMU_CUTTER_ENABLED
    uint8_t stealth_mode;         // EEPROM_MMU_STEALTH
    bool mmu_hw_enabled;          // EEPROM_MMU_ENABLED
    // uint32_t material_changes
  #endif

} SettingsData;

//static_assert(sizeof(SettingsData) <= MARLIN_EEPROM_SIZE, "EEPROM too small to contain SettingsData!");

MarlinSettings settings;

uint16_t MarlinSettings::datasize() { return sizeof(SettingsData); }

/**
 * Post-process after Retrieve or Reset
 */

#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
  float new_z_fade_height;
#endif

void MarlinSettings::postprocess() {
  xyze_pos_t oldpos = current_position;

  // steps per s2 needs to be updated to agree with units per s2
  planner.refresh_acceleration_rates();

  // Make sure delta kinematics are updated before refreshing the
  // planner position so the stepper counts will be set correctly.
  TERN_(DELTA, recalc_delta_settings());

  TERN_(PIDTEMP, thermalManager.updatePID());

  #if DISABLED(NO_VOLUMETRICS)
    planner.calculate_volumetric_multipliers();
  #elif EXTRUDERS
    for (uint8_t i = COUNT(planner.e_factor); i--;)
      planner.refresh_e_factor(i);
  #endif

  // Software endstops depend on home_offset
  LOOP_NUM_AXES(i) update_software_endstops((AxisEnum)i);

  TERN_(ENABLE_LEVELING_FADE_HEIGHT, set_z_fade_height(new_z_fade_height, false)); // false = no report

  TERN_(AUTO_BED_LEVELING_BILINEAR, bedlevel.refresh_bed_level());

  TERN_(HAS_MOTOR_CURRENT_PWM, stepper.refresh_motor_power());

  TERN_(FWRETRACT, fwretract.refresh_autoretract());

  TERN_(HAS_LINEAR_E_JERK, planner.recalculate_max_e_jerk());

  TERN_(CASELIGHT_USES_BRIGHTNESS, caselight.update_brightness());

  TERN_(EXTENSIBLE_UI, ExtUI::onPostprocessSettings());

  // Refresh mm_per_step with the reciprocal of axis_steps_per_mm
  // and init stepper.count[], planner.position[] with current_position
  planner.refresh_positioning();

  // Various factors can change the current position
  if (oldpos != current_position)
    report_current_position();

  // Moved as last update due to interference with NeoPixel init
  TERN_(HAS_LCD_CONTRAST, ui.refresh_contrast());
  TERN_(HAS_LCD_BRIGHTNESS, ui.refresh_brightness());
  TERN_(HAS_BACKLIGHT_TIMEOUT, ui.refresh_backlight_timeout());
  TERN_(HAS_DISPLAY_SLEEP, ui.refresh_screen_timeout());
}

#if ALL(PRINTCOUNTER, EEPROM_SETTINGS)
  #include "printcounter.h"
  static_assert(
    !WITHIN(STATS_EEPROM_ADDRESS, EEPROM_OFFSET, EEPROM_OFFSET + sizeof(SettingsData)) &&
    !WITHIN(STATS_EEPROM_ADDRESS + sizeof(printStatistics), EEPROM_OFFSET, EEPROM_OFFSET + sizeof(SettingsData)),
    "STATS_EEPROM_ADDRESS collides with EEPROM settings storage."
  );
#endif

#if ENABLED(SD_FIRMWARE_UPDATE)

  #if ENABLED(EEPROM_SETTINGS)
    static_assert(
      !WITHIN(SD_FIRMWARE_UPDATE_EEPROM_ADDR, EEPROM_OFFSET, EEPROM_OFFSET + sizeof(SettingsData)),
      "SD_FIRMWARE_UPDATE_EEPROM_ADDR collides with EEPROM settings storage."
    );
  #endif

  bool MarlinSettings::sd_update_status() {
    uint8_t val;
    int pos = SD_FIRMWARE_UPDATE_EEPROM_ADDR;
    persistentStore.read_data(pos, &val);
    return (val == SD_FIRMWARE_UPDATE_ACTIVE_VALUE);
  }

  bool MarlinSettings::set_sd_update_status(const bool enable) {
    if (enable != sd_update_status())
      persistentStore.write_data(
        SD_FIRMWARE_UPDATE_EEPROM_ADDR,
        enable ? SD_FIRMWARE_UPDATE_ACTIVE_VALUE : SD_FIRMWARE_UPDATE_INACTIVE_VALUE
      );
    return true;
  }

#endif // SD_FIRMWARE_UPDATE

#ifdef ARCHIM2_SPI_FLASH_EEPROM_BACKUP_SIZE
  static_assert(EEPROM_OFFSET + sizeof(SettingsData) < ARCHIM2_SPI_FLASH_EEPROM_BACKUP_SIZE,
                "ARCHIM2_SPI_FLASH_EEPROM_BACKUP_SIZE is insufficient to capture all EEPROM data.");
#endif

//
// This file simply uses the DEBUG_ECHO macros to implement EEPROM_CHITCHAT.
// For deeper debugging of EEPROM issues enable DEBUG_EEPROM_READWRITE.
//
#define DEBUG_OUT ANY(EEPROM_CHITCHAT, DEBUG_LEVELING_FEATURE)
#include "../core/debug_out.h"

#if ALL(EEPROM_CHITCHAT, HOST_PROMPT_SUPPORT)
  #define HOST_EEPROM_CHITCHAT 1
#endif

#if ENABLED(EEPROM_SETTINGS)

  #define EEPROM_ASSERT(TST,ERR)  do{ if (!(TST)) { SERIAL_WARN_MSG(ERR); eeprom_error = ERR_EEPROM_SIZE; } }while(0)

  #define TWO_BYTE_HASH(A,B) uint16_t((uint16_t(A ^ 0xC3) << 4) ^ (uint16_t(B ^ 0xC3) << 12))

  #define EEPROM_OFFSETOF(FIELD) (EEPROM_OFFSET + offsetof(SettingsData, FIELD))

  #if ENABLED(DEBUG_EEPROM_READWRITE)
    #define _FIELD_TEST(FIELD) \
      SERIAL_ECHOLNPGM("Field: " STRINGIFY(FIELD)); \
      EEPROM_ASSERT( \
        eeprom_error || eeprom_index == EEPROM_OFFSETOF(FIELD), \
        "Field " STRINGIFY(FIELD) " mismatch." \
      )
  #else
    #define _FIELD_TEST(FIELD) NOOP
  #endif

  #if ENABLED(DEBUG_EEPROM_OBSERVE)
    #define EEPROM_READ(V...)        do{ SERIAL_ECHOLNPGM("READ: ", F(STRINGIFY(FIRST(V)))); EEPROM_READ_(V); }while(0)
    #define EEPROM_READ_ALWAYS(V...) do{ SERIAL_ECHOLNPGM("READ: ", F(STRINGIFY(FIRST(V)))); EEPROM_READ_ALWAYS_(V); }while(0)
  #else
    #define EEPROM_READ(V...)        EEPROM_READ_(V)
    #define EEPROM_READ_ALWAYS(V...) EEPROM_READ_ALWAYS_(V)
  #endif

  constexpr char version_str[4] = EEPROM_VERSION;

  #if ENABLED(EEPROM_INIT_NOW)
    constexpr uint32_t strhash32(const char *s, const uint32_t h=0) {
      return *s ? strhash32(s + 1, ((h + *s) << (*s & 3)) ^ *s) : h;
    }
    constexpr uint32_t build_hash = strhash32(__DATE__ __TIME__);
  #endif

  bool MarlinSettings::validating;
  int MarlinSettings::eeprom_index;
  uint16_t MarlinSettings::working_crc;

  EEPROM_Error MarlinSettings::size_error(const uint16_t size) {
    if (size != datasize()) {
      DEBUG_WARN_MSG("EEPROM datasize error."
        #if ENABLED(MARLIN_DEV_MODE)
          " (Actual:", size, " Expected:", datasize(), ")"
        #endif
      );
      return ERR_EEPROM_SIZE;
    }
    return ERR_EEPROM_NOERR;
  }

  /**
   * M500 - Store Configuration
   */
  bool MarlinSettings::save() {
    float dummyf = 0;

    if (!EEPROM_START(EEPROM_OFFSET)) return false;

    EEPROM_Error eeprom_error = ERR_EEPROM_NOERR;

    // Write or Skip version. (Flash doesn't allow rewrite without erase.)
    constexpr char dummy_version[] = "ERR";
    TERN(FLASH_EEPROM_EMULATION, EEPROM_SKIP, EEPROM_WRITE)(dummy_version);

    #if ENABLED(EEPROM_INIT_NOW)
      EEPROM_SKIP(build_hash);  // Skip the hash slot which will be written later
    #endif

    EEPROM_SKIP(working_crc);   // Skip the checksum slot

    //
    // Clear after skipping CRC and before writing the CRC'ed data
    //
    working_crc = 0;

    // Write the size of the data structure for use in validation
    const uint16_t data_size = datasize();
    EEPROM_WRITE(data_size);

    const uint8_t e_factors = DISTINCT_AXES - (NUM_AXES);
    _FIELD_TEST(e_factors);
    EEPROM_WRITE(e_factors);

    //
    // Planner Motion
    //
    {
      EEPROM_WRITE(planner.settings);

      #if ENABLED(CLASSIC_JERK)
        EEPROM_WRITE(planner.max_jerk);
        #if HAS_LINEAR_E_JERK
          dummyf = float(DEFAULT_EJERK);
          EEPROM_WRITE(dummyf);
        #endif
      #else
        const xyze_pos_t planner_max_jerk = LOGICAL_AXIS_ARRAY(5, 10, 10, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4);
        EEPROM_WRITE(planner_max_jerk);
      #endif

      TERN_(CLASSIC_JERK, dummyf = 0.02f);
      EEPROM_WRITE(TERN(CLASSIC_JERK, dummyf, planner.junction_deviation_mm));
    }

    //
    // Home Offset
    //
    #if NUM_AXES
    {
      _FIELD_TEST(home_offset);

      #if HAS_SCARA_OFFSET
        EEPROM_WRITE(scara_home_offset);
      #else
        #if !HAS_HOME_OFFSET
          const xyz_pos_t home_offset{0};
        #endif
        EEPROM_WRITE(home_offset);
      #endif
    }
    #endif // NUM_AXES

    //
    // Hotend Offsets, if any
    //
    {
      #if HAS_HOTEND_OFFSET
        // Skip hotend 0 which must be 0
        for (uint8_t e = 1; e < HOTENDS; ++e)
          EEPROM_WRITE(hotend_offset[e]);
      #endif
    }

    //
    // Filament Runout Sensor
    //
    {
      #if HAS_FILAMENT_SENSOR
        const bool &runout_sensor_enabled = runout.enabled;
      #else
        constexpr int8_t runout_sensor_enabled = -1;
      #endif
      _FIELD_TEST(runout_sensor_enabled);
      EEPROM_WRITE(runout_sensor_enabled);

      #if HAS_FILAMENT_RUNOUT_DISTANCE
        const float &runout_distance_mm = runout.runout_distance();
      #else
        constexpr float runout_distance_mm = 0;
      #endif
      EEPROM_WRITE(runout_distance_mm);
    }

    //
    // Global Leveling
    //
    {
      #ifndef DEFAULT_LEVELING_FADE_HEIGHT
        #define DEFAULT_LEVELING_FADE_HEIGHT 0
      #endif
      const float zfh = TERN(ENABLE_LEVELING_FADE_HEIGHT, planner.z_fade_height, DEFAULT_LEVELING_FADE_HEIGHT);
      EEPROM_WRITE(zfh);
    }

    //
    // AUTOTEMP
    //
    #if ENABLED(AUTOTEMP)
      _FIELD_TEST(planner_autotemp_max);
      EEPROM_WRITE(planner.autotemp.max);
      EEPROM_WRITE(planner.autotemp.min);
      EEPROM_WRITE(planner.autotemp.factor);
    #endif

    //
    // Mesh Bed Leveling
    //
    {
      #if ENABLED(MESH_BED_LEVELING)
        static_assert(
          sizeof(bedlevel.z_values) == GRID_MAX_POINTS * sizeof(bedlevel.z_values[0][0]),
          "MBL Z array is the wrong size."
        );
      #else
        dummyf = 0;
      #endif

      const uint8_t mesh_num_x = TERN(MESH_BED_LEVELING, GRID_MAX_POINTS_X, 3),
                    mesh_num_y = TERN(MESH_BED_LEVELING, GRID_MAX_POINTS_Y, 3);

      EEPROM_WRITE(TERN(MESH_BED_LEVELING, bedlevel.z_offset, dummyf));
      EEPROM_WRITE(mesh_num_x);
      EEPROM_WRITE(mesh_num_y);

      // Check value for the X/Y values
      const uint16_t mesh_check = TWO_BYTE_HASH(mesh_num_x, mesh_num_y);
      EEPROM_WRITE(mesh_check);

      #if ENABLED(MESH_BED_LEVELING)
        EEPROM_WRITE(bedlevel.z_values);
      #else
        for (uint8_t q = mesh_num_x * mesh_num_y; q--;) EEPROM_WRITE(dummyf);
      #endif
    }

    //
    // Probe XYZ Offsets
    //
    #if NUM_AXES
    {
      _FIELD_TEST(probe_offset);
      #if HAS_BED_PROBE
        const xyz_pos_t &zpo = probe.offset;
      #else
        constexpr xyz_pos_t zpo{0};
      #endif
      EEPROM_WRITE(zpo);
    }
    #endif

    //
    // Planar Bed Leveling matrix
    //
    {
      #if ABL_PLANAR
        EEPROM_WRITE(planner.bed_level_matrix);
      #else
        dummyf = 0;
        for (uint8_t q = 9; q--;) EEPROM_WRITE(dummyf);
      #endif
    }

    //
    // Bilinear Auto Bed Leveling
    //
    {
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        static_assert(
          sizeof(bedlevel.z_values) == GRID_MAX_POINTS * sizeof(bedlevel.z_values[0][0]),
          "Bilinear Z array is the wrong size."
        );
      #endif

      const uint8_t grid_max_x = TERN(AUTO_BED_LEVELING_BILINEAR, GRID_MAX_POINTS_X, 3),
                    grid_max_y = TERN(AUTO_BED_LEVELING_BILINEAR, GRID_MAX_POINTS_Y, 3);
      EEPROM_WRITE(grid_max_x);
      EEPROM_WRITE(grid_max_y);

      // Check value for the X/Y values
      const uint16_t grid_check = TWO_BYTE_HASH(grid_max_x, grid_max_y);
      EEPROM_WRITE(grid_check);

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        EEPROM_WRITE(bedlevel.grid_spacing);
        EEPROM_WRITE(bedlevel.grid_start);
      #else
        const xy_pos_t bilinear_grid_spacing{0}, bilinear_start{0};
        EEPROM_WRITE(bilinear_grid_spacing);
        EEPROM_WRITE(bilinear_start);
      #endif

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        EEPROM_WRITE(bedlevel.z_values);              // 9-256 floats
      #else
        dummyf = 0;
        for (uint16_t q = grid_max_x * grid_max_y; q--;) EEPROM_WRITE(dummyf);
      #endif
    }

    //
    // X Axis Twist Compensation
    //
    #if ENABLED(X_AXIS_TWIST_COMPENSATION)
      _FIELD_TEST(xatc_spacing);
      EEPROM_WRITE(xatc.spacing);
      EEPROM_WRITE(xatc.start);
      EEPROM_WRITE(xatc.z_offset);
    #endif

    //
    // Unified Bed Leveling
    //
    {
      _FIELD_TEST(planner_leveling_active);
      const bool ubl_active = TERN(AUTO_BED_LEVELING_UBL, planner.leveling_active, false);
      const int8_t storage_slot = TERN(AUTO_BED_LEVELING_UBL, bedlevel.storage_slot, -1);
      EEPROM_WRITE(ubl_active);
      EEPROM_WRITE(storage_slot);
    }

    //
    // Servo Angles
    //
    #if HAS_SERVO_ANGLES
    {
      _FIELD_TEST(servo_angles);
      EEPROM_WRITE(servo_angles);
    }
    #endif

    //
    // Thermal first layer compensation values
    //
    #if HAS_PTC
      #if ENABLED(PTC_PROBE)
        EEPROM_WRITE(ptc.z_offsets_probe);
      #endif
      #if ENABLED(PTC_BED)
        EEPROM_WRITE(ptc.z_offsets_bed);
      #endif
      #if ENABLED(PTC_HOTEND)
        EEPROM_WRITE(ptc.z_offsets_hotend);
      #endif
    #else
      // No placeholder data for this feature
    #endif

    //
    // BLTOUCH
    //
    {
      _FIELD_TEST(bltouch_od_5v_mode);
      const bool bltouch_od_5v_mode = TERN0(BLTOUCH, bltouch.od_5v_mode);
      EEPROM_WRITE(bltouch_od_5v_mode);

      #if HAS_BLTOUCH_HS_MODE
        _FIELD_TEST(bltouch_high_speed_mode);
        const bool bltouch_high_speed_mode = TERN0(BLTOUCH, bltouch.high_speed_mode);
        EEPROM_WRITE(bltouch_high_speed_mode);
      #endif
    }

    //
    // Kinematic Settings (Delta, SCARA, TPARA, Polargraph...)
    //
    #if IS_KINEMATIC
    {
      EEPROM_WRITE(segments_per_second);
      #if ENABLED(DELTA)
        _FIELD_TEST(delta_height);
        EEPROM_WRITE(delta_height);              // 1 float
        EEPROM_WRITE(delta_endstop_adj);         // 3 floats
        EEPROM_WRITE(delta_radius);              // 1 float
        EEPROM_WRITE(delta_diagonal_rod);        // 1 float
        EEPROM_WRITE(delta_tower_angle_trim);    // 3 floats
        EEPROM_WRITE(delta_diagonal_rod_trim);   // 3 floats
      #elif ENABLED(POLARGRAPH)
        _FIELD_TEST(draw_area_min);
        EEPROM_WRITE(draw_area_min);             // 2 floats
        EEPROM_WRITE(draw_area_max);             // 2 floats
        EEPROM_WRITE(polargraph_max_belt_len);   // 1 float
      #endif
    }
    #endif

    //
    // Extra Endstops offsets
    //
    #if HAS_EXTRA_ENDSTOPS
    {
      _FIELD_TEST(x2_endstop_adj);

      // Write dual endstops in X, Y, Z order. Unused = 0.0
      dummyf = 0;
      EEPROM_WRITE(TERN(X_DUAL_ENDSTOPS, endstops.x2_endstop_adj, dummyf));   // 1 float
      EEPROM_WRITE(TERN(Y_DUAL_ENDSTOPS, endstops.y2_endstop_adj, dummyf));   // 1 float
      EEPROM_WRITE(TERN(Z_MULTI_ENDSTOPS, endstops.z2_endstop_adj, dummyf));  // 1 float

      #if ENABLED(Z_MULTI_ENDSTOPS) && NUM_Z_STEPPERS >= 3
        EEPROM_WRITE(endstops.z3_endstop_adj);   // 1 float
      #else
        EEPROM_WRITE(dummyf);
      #endif

      #if ENABLED(Z_MULTI_ENDSTOPS) && NUM_Z_STEPPERS >= 4
        EEPROM_WRITE(endstops.z4_endstop_adj);   // 1 float
      #else
        EEPROM_WRITE(dummyf);
      #endif
    }
    #endif

    #if ENABLED(Z_STEPPER_AUTO_ALIGN)
      EEPROM_WRITE(z_stepper_align.xy);
      #if HAS_Z_STEPPER_ALIGN_STEPPER_XY
        EEPROM_WRITE(z_stepper_align.stepper_xy);
      #endif
    #endif

    //
    // LCD Preheat settings
    //
    #if HAS_PREHEAT
      _FIELD_TEST(ui_material_preset);
      EEPROM_WRITE(ui.material_preset);
    #endif

    //
    // PIDTEMP
    //
    {
      _FIELD_TEST(hotendPID);
      #if DISABLED(PIDTEMP)
        raw_pidcf_t pidcf = { NAN, NAN, NAN, NAN, NAN };
      #endif
      HOTEND_LOOP() {
        #if ENABLED(PIDTEMP)
          const hotend_pid_t &pid = thermalManager.temp_hotend[e].pid;
          raw_pidcf_t pidcf = { pid.p(), pid.i(), pid.d(), pid.c(), pid.f() };
        #endif
        EEPROM_WRITE(pidcf);
      }

      _FIELD_TEST(lpq_len);
      const int16_t lpq_len = TERN(PID_EXTRUSION_SCALING, thermalManager.lpq_len, 20);
      EEPROM_WRITE(lpq_len);
    }

    //
    // PIDTEMPBED
    //
    {
      _FIELD_TEST(bedPID);
      #if ENABLED(PIDTEMPBED)
        const auto &pid = thermalManager.temp_bed.pid;
        const raw_pid_t bed_pid = { pid.p(), pid.i(), pid.d() };
      #else
        const raw_pid_t bed_pid = { NAN, NAN, NAN };
      #endif
      EEPROM_WRITE(bed_pid);
    }

    //
    // PIDTEMPCHAMBER
    //
    {
      _FIELD_TEST(chamberPID);
      #if ENABLED(PIDTEMPCHAMBER)
        const auto &pid = thermalManager.temp_chamber.pid;
        const raw_pid_t chamber_pid = { pid.p(), pid.i(), pid.d() };
      #else
        const raw_pid_t chamber_pid = { NAN, NAN, NAN };
      #endif
      EEPROM_WRITE(chamber_pid);
    }

    //
    // User-defined Thermistors
    //
    #if HAS_USER_THERMISTORS
      _FIELD_TEST(user_thermistor);
      EEPROM_WRITE(thermalManager.user_thermistor);
    #endif

    //
    // Power monitor
    //
    {
      #if HAS_POWER_MONITOR
        const uint8_t &power_monitor_flags = power_monitor.flags;
      #else
        constexpr uint8_t power_monitor_flags = 0x00;
      #endif
      _FIELD_TEST(power_monitor_flags);
      EEPROM_WRITE(power_monitor_flags);
    }

    //
    // LCD Contrast
    //
    {
      _FIELD_TEST(lcd_contrast);
      const uint8_t lcd_contrast = TERN(HAS_LCD_CONTRAST, ui.contrast, 127);
      EEPROM_WRITE(lcd_contrast);
    }

    //
    // LCD Brightness
    //
    {
      _FIELD_TEST(lcd_brightness);
      const uint8_t lcd_brightness = TERN(HAS_LCD_BRIGHTNESS, ui.brightness, 255);
      EEPROM_WRITE(lcd_brightness);
    }

    //
    // LCD Backlight / Sleep Timeout
    //
    #if ENABLED(EDITABLE_DISPLAY_TIMEOUT)
      #if HAS_BACKLIGHT_TIMEOUT
        EEPROM_WRITE(ui.backlight_timeout_minutes);
      #elif HAS_DISPLAY_SLEEP
        EEPROM_WRITE(ui.sleep_timeout_minutes);
      #endif
    #endif

    //
    // Controller Fan
    //
    {
      _FIELD_TEST(controllerFan_settings);
      #if ENABLED(USE_CONTROLLER_FAN)
        const controllerFan_settings_t &cfs = controllerFan.settings;
      #else
        constexpr controllerFan_settings_t cfs = controllerFan_defaults;
      #endif
      EEPROM_WRITE(cfs);
    }

    //
    // Power-Loss Recovery
    //
    {
      _FIELD_TEST(recovery_enabled);
      const bool recovery_enabled = TERN0(POWER_LOSS_RECOVERY, recovery.enabled);
      const celsius_t bed_temp_threshold = TERN0(HAS_PLR_BED_THRESHOLD, recovery.bed_temp_threshold);
      EEPROM_WRITE(recovery_enabled);
      EEPROM_WRITE(bed_temp_threshold);
    }

    //
    // Firmware Retraction
    //
    {
      _FIELD_TEST(fwretract_settings);
      #if DISABLED(FWRETRACT)
        const fwretract_settings_t autoretract_defaults = { 3, 45, 0, 0, 0, 13, 0, 8 };
      #endif
      EEPROM_WRITE(TERN(FWRETRACT, fwretract.settings, autoretract_defaults));

      #if DISABLED(FWRETRACT_AUTORETRACT)
        const bool autoretract_enabled = false;
      #endif
      EEPROM_WRITE(TERN(FWRETRACT_AUTORETRACT, fwretract.autoretract_enabled, autoretract_enabled));
    }

    //
    // Homing Feedrate
    //
    #if ENABLED(EDITABLE_HOMING_FEEDRATE)
      _FIELD_TEST(homing_feedrate_mm_m);
      EEPROM_WRITE(homing_feedrate_mm_m);
    #endif

    //
    // Volumetric & Filament Size
    //
    {
      _FIELD_TEST(parser_volumetric_enabled);

      #if DISABLED(NO_VOLUMETRICS)

        EEPROM_WRITE(parser.volumetric_enabled);
        EEPROM_WRITE(planner.filament_size);
        #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
          EEPROM_WRITE(planner.volumetric_extruder_limit);
        #else
          dummyf = 0.0;
          for (uint8_t q = EXTRUDERS; q--;) EEPROM_WRITE(dummyf);
        #endif

      #else

        const bool volumetric_enabled = false;
        EEPROM_WRITE(volumetric_enabled);
        dummyf = DEFAULT_NOMINAL_FILAMENT_DIA;
        for (uint8_t q = EXTRUDERS; q--;) EEPROM_WRITE(dummyf);
        dummyf = 0.0;
        for (uint8_t q = EXTRUDERS; q--;) EEPROM_WRITE(dummyf);

      #endif
    }

    //
    // TMC Configuration
    //
    {
      _FIELD_TEST(tmc_stepper_current);

      per_stepper_uint16_t tmc_stepper_current{0};

      #if HAS_TRINAMIC_CONFIG
        #if AXIS_IS_TMC(X)
          tmc_stepper_current.X = stepperX.getMilliamps();
        #endif
        #if AXIS_IS_TMC(Y)
          tmc_stepper_current.Y = stepperY.getMilliamps();
        #endif
        #if AXIS_IS_TMC(Z)
          tmc_stepper_current.Z = stepperZ.getMilliamps();
        #endif
        #if AXIS_IS_TMC(I)
          tmc_stepper_current.I = stepperI.getMilliamps();
        #endif
        #if AXIS_IS_TMC(J)
          tmc_stepper_current.J = stepperJ.getMilliamps();
        #endif
        #if AXIS_IS_TMC(K)
          tmc_stepper_current.K = stepperK.getMilliamps();
        #endif
        #if AXIS_IS_TMC(U)
          tmc_stepper_current.U = stepperU.getMilliamps();
        #endif
        #if AXIS_IS_TMC(V)
          tmc_stepper_current.V = stepperV.getMilliamps();
        #endif
        #if AXIS_IS_TMC(W)
          tmc_stepper_current.W = stepperW.getMilliamps();
        #endif
        #if AXIS_IS_TMC(X2)
          tmc_stepper_current.X2 = stepperX2.getMilliamps();
        #endif
        #if AXIS_IS_TMC(Y2)
          tmc_stepper_current.Y2 = stepperY2.getMilliamps();
        #endif
        #if AXIS_IS_TMC(Z2)
          tmc_stepper_current.Z2 = stepperZ2.getMilliamps();
        #endif
        #if AXIS_IS_TMC(Z3)
          tmc_stepper_current.Z3 = stepperZ3.getMilliamps();
        #endif
        #if AXIS_IS_TMC(Z4)
          tmc_stepper_current.Z4 = stepperZ4.getMilliamps();
        #endif
        #if AXIS_IS_TMC(E0)
          tmc_stepper_current.E0 = stepperE0.getMilliamps();
        #endif
        #if AXIS_IS_TMC(E1)
          tmc_stepper_current.E1 = stepperE1.getMilliamps();
        #endif
        #if AXIS_IS_TMC(E2)
          tmc_stepper_current.E2 = stepperE2.getMilliamps();
        #endif
        #if AXIS_IS_TMC(E3)
          tmc_stepper_current.E3 = stepperE3.getMilliamps();
        #endif
        #if AXIS_IS_TMC(E4)
          tmc_stepper_current.E4 = stepperE4.getMilliamps();
        #endif
        #if AXIS_IS_TMC(E5)
          tmc_stepper_current.E5 = stepperE5.getMilliamps();
        #endif
        #if AXIS_IS_TMC(E6)
          tmc_stepper_current.E6 = stepperE6.getMilliamps();
        #endif
        #if AXIS_IS_TMC(E7)
          tmc_stepper_current.E7 = stepperE7.getMilliamps();
        #endif
      #endif
      EEPROM_WRITE(tmc_stepper_current);
    }

    //
    // TMC Hybrid Threshold, and placeholder values
    //
    {
      _FIELD_TEST(tmc_hybrid_threshold);

      #if ENABLED(HYBRID_THRESHOLD)
        per_stepper_uint32_t tmc_hybrid_threshold{0};
        TERN_(X_HAS_STEALTHCHOP,  tmc_hybrid_threshold.X =  stepperX.get_pwm_thrs());
        TERN_(Y_HAS_STEALTHCHOP,  tmc_hybrid_threshold.Y =  stepperY.get_pwm_thrs());
        TERN_(Z_HAS_STEALTHCHOP,  tmc_hybrid_threshold.Z =  stepperZ.get_pwm_thrs());
        TERN_(I_HAS_STEALTHCHOP,  tmc_hybrid_threshold.I =  stepperI.get_pwm_thrs());
        TERN_(J_HAS_STEALTHCHOP,  tmc_hybrid_threshold.J =  stepperJ.get_pwm_thrs());
        TERN_(K_HAS_STEALTHCHOP,  tmc_hybrid_threshold.K =  stepperK.get_pwm_thrs());
        TERN_(U_HAS_STEALTHCHOP,  tmc_hybrid_threshold.U =  stepperU.get_pwm_thrs());
        TERN_(V_HAS_STEALTHCHOP,  tmc_hybrid_threshold.V =  stepperV.get_pwm_thrs());
        TERN_(W_HAS_STEALTHCHOP,  tmc_hybrid_threshold.W =  stepperW.get_pwm_thrs());
        TERN_(X2_HAS_STEALTHCHOP, tmc_hybrid_threshold.X2 = stepperX2.get_pwm_thrs());
        TERN_(Y2_HAS_STEALTHCHOP, tmc_hybrid_threshold.Y2 = stepperY2.get_pwm_thrs());
        TERN_(Z2_HAS_STEALTHCHOP, tmc_hybrid_threshold.Z2 = stepperZ2.get_pwm_thrs());
        TERN_(Z3_HAS_STEALTHCHOP, tmc_hybrid_threshold.Z3 = stepperZ3.get_pwm_thrs());
        TERN_(Z4_HAS_STEALTHCHOP, tmc_hybrid_threshold.Z4 = stepperZ4.get_pwm_thrs());
        TERN_(E0_HAS_STEALTHCHOP, tmc_hybrid_threshold.E0 = stepperE0.get_pwm_thrs());
        TERN_(E1_HAS_STEALTHCHOP, tmc_hybrid_threshold.E1 = stepperE1.get_pwm_thrs());
        TERN_(E2_HAS_STEALTHCHOP, tmc_hybrid_threshold.E2 = stepperE2.get_pwm_thrs());
        TERN_(E3_HAS_STEALTHCHOP, tmc_hybrid_threshold.E3 = stepperE3.get_pwm_thrs());
        TERN_(E4_HAS_STEALTHCHOP, tmc_hybrid_threshold.E4 = stepperE4.get_pwm_thrs());
        TERN_(E5_HAS_STEALTHCHOP, tmc_hybrid_threshold.E5 = stepperE5.get_pwm_thrs());
        TERN_(E6_HAS_STEALTHCHOP, tmc_hybrid_threshold.E6 = stepperE6.get_pwm_thrs());
        TERN_(E7_HAS_STEALTHCHOP, tmc_hybrid_threshold.E7 = stepperE7.get_pwm_thrs());
      #else
        #define _EN_ITEM(N) , .E##N =  30
        const per_stepper_uint32_t tmc_hybrid_threshold = {
          NUM_AXIS_LIST_(.X = 100, .Y = 100, .Z = 3, .I = 3, .J = 3, .K = 3, .U = 3, .V = 3, .W = 3)
          .X2 = 100, .Y2 = 100, .Z2 = 3, .Z3 = 3, .Z4 = 3
          REPEAT(E_STEPPERS, _EN_ITEM)
        };
        #undef _EN_ITEM
      #endif
      EEPROM_WRITE(tmc_hybrid_threshold);
    }

    //
    // TMC StallGuard threshold
    //
    {
      mot_stepper_int16_t tmc_sgt{0};
      #if USE_SENSORLESS
        NUM_AXIS_CODE(
          TERN_(X_SENSORLESS, tmc_sgt.X = stepperX.homing_threshold()),
          TERN_(Y_SENSORLESS, tmc_sgt.Y = stepperY.homing_threshold()),
          TERN_(Z_SENSORLESS, tmc_sgt.Z = stepperZ.homing_threshold()),
          TERN_(I_SENSORLESS, tmc_sgt.I = stepperI.homing_threshold()),
          TERN_(J_SENSORLESS, tmc_sgt.J = stepperJ.homing_threshold()),
          TERN_(K_SENSORLESS, tmc_sgt.K = stepperK.homing_threshold()),
          TERN_(U_SENSORLESS, tmc_sgt.U = stepperU.homing_threshold()),
          TERN_(V_SENSORLESS, tmc_sgt.V = stepperV.homing_threshold()),
          TERN_(W_SENSORLESS, tmc_sgt.W = stepperW.homing_threshold())
        );
        TERN_(X2_SENSORLESS, tmc_sgt.X2 = stepperX2.homing_threshold());
        TERN_(Y2_SENSORLESS, tmc_sgt.Y2 = stepperY2.homing_threshold());
        TERN_(Z2_SENSORLESS, tmc_sgt.Z2 = stepperZ2.homing_threshold());
        TERN_(Z3_SENSORLESS, tmc_sgt.Z3 = stepperZ3.homing_threshold());
        TERN_(Z4_SENSORLESS, tmc_sgt.Z4 = stepperZ4.homing_threshold());
      #endif
      EEPROM_WRITE(tmc_sgt);
    }

    //
    // TMC stepping mode
    //
    {
      _FIELD_TEST(tmc_stealth_enabled);

      per_stepper_bool_t tmc_stealth_enabled = { false };
      TERN_(X_HAS_STEALTHCHOP,  tmc_stealth_enabled.X  = stepperX.get_stored_stealthChop());
      TERN_(Y_HAS_STEALTHCHOP,  tmc_stealth_enabled.Y  = stepperY.get_stored_stealthChop());
      TERN_(Z_HAS_STEALTHCHOP,  tmc_stealth_enabled.Z  = stepperZ.get_stored_stealthChop());
      TERN_(I_HAS_STEALTHCHOP,  tmc_stealth_enabled.I  = stepperI.get_stored_stealthChop());
      TERN_(J_HAS_STEALTHCHOP,  tmc_stealth_enabled.J  = stepperJ.get_stored_stealthChop());
      TERN_(K_HAS_STEALTHCHOP,  tmc_stealth_enabled.K  = stepperK.get_stored_stealthChop());
      TERN_(U_HAS_STEALTHCHOP,  tmc_stealth_enabled.U  = stepperU.get_stored_stealthChop());
      TERN_(V_HAS_STEALTHCHOP,  tmc_stealth_enabled.V  = stepperV.get_stored_stealthChop());
      TERN_(W_HAS_STEALTHCHOP,  tmc_stealth_enabled.W  = stepperW.get_stored_stealthChop());
      TERN_(X2_HAS_STEALTHCHOP, tmc_stealth_enabled.X2 = stepperX2.get_stored_stealthChop());
      TERN_(Y2_HAS_STEALTHCHOP, tmc_stealth_enabled.Y2 = stepperY2.get_stored_stealthChop());
      TERN_(Z2_HAS_STEALTHCHOP, tmc_stealth_enabled.Z2 = stepperZ2.get_stored_stealthChop());
      TERN_(Z3_HAS_STEALTHCHOP, tmc_stealth_enabled.Z3 = stepperZ3.get_stored_stealthChop());
      TERN_(Z4_HAS_STEALTHCHOP, tmc_stealth_enabled.Z4 = stepperZ4.get_stored_stealthChop());
      TERN_(E0_HAS_STEALTHCHOP, tmc_stealth_enabled.E0 = stepperE0.get_stored_stealthChop());
      TERN_(E1_HAS_STEALTHCHOP, tmc_stealth_enabled.E1 = stepperE1.get_stored_stealthChop());
      TERN_(E2_HAS_STEALTHCHOP, tmc_stealth_enabled.E2 = stepperE2.get_stored_stealthChop());
      TERN_(E3_HAS_STEALTHCHOP, tmc_stealth_enabled.E3 = stepperE3.get_stored_stealthChop());
      TERN_(E4_HAS_STEALTHCHOP, tmc_stealth_enabled.E4 = stepperE4.get_stored_stealthChop());
      TERN_(E5_HAS_STEALTHCHOP, tmc_stealth_enabled.E5 = stepperE5.get_stored_stealthChop());
      TERN_(E6_HAS_STEALTHCHOP, tmc_stealth_enabled.E6 = stepperE6.get_stored_stealthChop());
      TERN_(E7_HAS_STEALTHCHOP, tmc_stealth_enabled.E7 = stepperE7.get_stored_stealthChop());
      EEPROM_WRITE(tmc_stealth_enabled);
    }

    //
    // Linear Advance
    //
    {
      _FIELD_TEST(planner_extruder_advance_K);

      #if ENABLED(LIN_ADVANCE)
        EEPROM_WRITE(planner.extruder_advance_K);
      #else
        dummyf = 0;
        for (uint8_t q = DISTINCT_E; q--;) EEPROM_WRITE(dummyf);
      #endif
    }

    //
    // Motor Current PWM
    //
    {
      _FIELD_TEST(motor_current_setting);

      #if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM
        EEPROM_WRITE(stepper.motor_current_setting);
      #else
        const uint32_t no_current[MOTOR_CURRENT_COUNT] = { 0 };
        EEPROM_WRITE(no_current);
      #endif
    }

    //
    // Adaptive Step Smoothing state
    //
    #if ENABLED(ADAPTIVE_STEP_SMOOTHING_TOGGLE)
      _FIELD_TEST(adaptive_step_smoothing_enabled);
      EEPROM_WRITE(stepper.adaptive_step_smoothing_enabled);
    #endif

    //
    // CNC Coordinate Systems
    //
    #if NUM_AXES
      _FIELD_TEST(coordinate_system);
      #if DISABLED(CNC_COORDINATE_SYSTEMS)
        const xyz_pos_t coordinate_system[MAX_COORDINATE_SYSTEMS] = { { 0 } };
      #endif
      EEPROM_WRITE(TERN(CNC_COORDINATE_SYSTEMS, gcode.coordinate_system, coordinate_system));
    #endif

    //
    // Skew correction factors
    //
    #if ENABLED(SKEW_CORRECTION)
      _FIELD_TEST(planner_skew_factor);
      EEPROM_WRITE(planner.skew_factor);
    #endif

    //
    // Advanced Pause filament load & unload lengths
    //
    #if ENABLED(CONFIGURE_FILAMENT_CHANGE)
    {
      _FIELD_TEST(fc_settings);
      EEPROM_WRITE(fc_settings);
    }
    #endif

    //
    // Multiple Extruders
    //

    #if HAS_MULTI_EXTRUDER
      _FIELD_TEST(toolchange_settings);
      EEPROM_WRITE(toolchange_settings);
    #endif

    //
    // Backlash Compensation
    //
    #if NUM_AXES
    {
      #if ENABLED(BACKLASH_GCODE)
        xyz_float_t backlash_distance_mm;
        LOOP_NUM_AXES(axis) backlash_distance_mm[axis] = backlash.get_distance_mm((AxisEnum)axis);
        const uint8_t backlash_correction = backlash.get_correction_uint8();
      #else
        const xyz_float_t backlash_distance_mm{0};
        const uint8_t backlash_correction = 0;
      #endif
      #if ENABLED(BACKLASH_GCODE) && defined(BACKLASH_SMOOTHING_MM)
        const float backlash_smoothing_mm = backlash.get_smoothing_mm();
      #else
        const float backlash_smoothing_mm = 3;
      #endif
      _FIELD_TEST(backlash_distance_mm);
      EEPROM_WRITE(backlash_distance_mm);
      EEPROM_WRITE(backlash_correction);
      EEPROM_WRITE(backlash_smoothing_mm);
    }
    #endif // NUM_AXES

    //
    // Extensible UI User Data
    //
    #if ENABLED(EXTENSIBLE_UI)
    {
      char extui_data[ExtUI::eeprom_data_size] = { 0 };
      ExtUI::onStoreSettings(extui_data);
      _FIELD_TEST(extui_data);
      EEPROM_WRITE(extui_data);
    }
    #endif

    //
    // JyersUI DWIN User Data
    //
    #if ENABLED(DWIN_CREALITY_LCD_JYERSUI)
    {
      _FIELD_TEST(dwin_settings);
      char dwin_settings[jyersDWIN.eeprom_data_size] = { 0 };
      jyersDWIN.saveSettings(dwin_settings);
      EEPROM_WRITE(dwin_settings);
    }
    #endif

    //
    // Case Light Brightness
    //
    #if CASELIGHT_USES_BRIGHTNESS
      EEPROM_WRITE(caselight.brightness);
    #endif

    //
    // Password feature
    //
    #if ENABLED(PASSWORD_FEATURE)
      EEPROM_WRITE(password.is_set);
      EEPROM_WRITE(password.value);
    #endif

    //
    // TOUCH_SCREEN_CALIBRATION
    //
    #if ENABLED(TOUCH_SCREEN_CALIBRATION)
      EEPROM_WRITE(touch_calibration.calibration);
    #endif

    //
    // Ethernet network info
    //
    #if HAS_ETHERNET
    {
      _FIELD_TEST(ethernet_hardware_enabled);
      const bool ethernet_hardware_enabled = ethernet.hardware_enabled;
      const uint32_t ethernet_ip      = ethernet.ip,
                     ethernet_dns     = ethernet.myDns,
                     ethernet_gateway = ethernet.gateway,
                     ethernet_subnet  = ethernet.subnet;
      EEPROM_WRITE(ethernet_hardware_enabled);
      EEPROM_WRITE(ethernet_ip);
      EEPROM_WRITE(ethernet_dns);
      EEPROM_WRITE(ethernet_gateway);
      EEPROM_WRITE(ethernet_subnet);
    }
    #endif

    //
    // Buzzer enable/disable
    //
    #if ENABLED(SOUND_MENU_ITEM)
      EEPROM_WRITE(ui.sound_on);
    #endif

    //
    // Fan tachometer check
    //
    #if HAS_FANCHECK
      EEPROM_WRITE(fan_check.enabled);
    #endif

    //
    // MKS UI controller
    //
    #if DGUS_LCD_UI_MKS
      EEPROM_WRITE(mks_language_index);
      EEPROM_WRITE(mks_corner_offsets);
      EEPROM_WRITE(mks_park_pos);
      EEPROM_WRITE(mks_min_extrusion_temp);
    #endif

    //
    // Selected LCD language
    //
    #if HAS_MULTI_LANGUAGE
      EEPROM_WRITE(ui.language);
    #endif

    //
    // Model predictive control
    //
    #if ENABLED(MPCTEMP)
      HOTEND_LOOP() EEPROM_WRITE(thermalManager.temp_hotend[e].mpc);
    #endif

    //
    // Fixed-Time Motion
    //
    #if ENABLED(FT_MOTION)
      _FIELD_TEST(ftMotion_cfg);
      EEPROM_WRITE(ftMotion.cfg);
    #endif

    //
    // Input Shaping
    //
    #if HAS_ZV_SHAPING
      #if ENABLED(INPUT_SHAPING_X)
        EEPROM_WRITE(stepper.get_shaping_frequency(X_AXIS));
        EEPROM_WRITE(stepper.get_shaping_damping_ratio(X_AXIS));
      #endif
      #if ENABLED(INPUT_SHAPING_Y)
        EEPROM_WRITE(stepper.get_shaping_frequency(Y_AXIS));
        EEPROM_WRITE(stepper.get_shaping_damping_ratio(Y_AXIS));
      #endif
      #if ENABLED(INPUT_SHAPING_Z)
        EEPROM_WRITE(stepper.get_shaping_frequency(Z_AXIS));
        EEPROM_WRITE(stepper.get_shaping_damping_ratio(Z_AXIS));
      #endif
    #endif

    //
    // HOTEND_IDLE_TIMEOUT
    //
    #if ENABLED(HOTEND_IDLE_TIMEOUT)
      EEPROM_WRITE(hotend_idle.cfg);
    #endif

    //
    // Nonlinear Extrusion
    //
    #if ENABLED(NONLINEAR_EXTRUSION)
      EEPROM_WRITE(stepper.ne);
    #endif

    //
    // MMU3
    //
    #if HAS_PRUSA_MMU3
      EEPROM_WRITE(spooljoin.enabled); // EEPROM_SPOOL_JOIN
      // for testing purposes fill with default values
      EEPROM_WRITE(MMU3::operation_statistics.fail_total_num); //EEPROM_MMU_FAIL_TOT
      EEPROM_WRITE(MMU3::operation_statistics.fail_num); // EEPROM_MMU_FAIL
      EEPROM_WRITE(MMU3::operation_statistics.load_fail_total_num); // EEPROM_MMU_LOAD_FAIL_TOT
      EEPROM_WRITE(MMU3::operation_statistics.load_fail_num); // EEPROM_MMU_LOAD_FAIL
      EEPROM_WRITE(MMU3::operation_statistics.tool_change_counter);
      EEPROM_WRITE(MMU3::operation_statistics.tool_change_total_counter); // EEPROM_MMU_MATERIAL_CHANGES
      EEPROM_WRITE(mmu3.cutter_mode); // EEPROM_MMU_CUTTER_ENABLED
      EEPROM_WRITE(mmu3.stealth_mode); // EEPROM_MMU_STEALTH
      EEPROM_WRITE(mmu3.mmu_hw_enabled); // EEPROM_MMU_ENABLED
    #endif

    //
    // Report final CRC and Data Size
    //
    if (eeprom_error == ERR_EEPROM_NOERR) {
      const uint16_t eeprom_size = eeprom_index - (EEPROM_OFFSET),
                     final_crc = working_crc;

      // Write the EEPROM header
      eeprom_index = EEPROM_OFFSET;

      EEPROM_WRITE(version_str);
      #if ENABLED(EEPROM_INIT_NOW)
        EEPROM_WRITE(build_hash);
      #endif
      EEPROM_WRITE(final_crc);

      // Report storage size
      DEBUG_ECHO_MSG("Settings Stored (", eeprom_size, " bytes; crc ", (uint32_t)final_crc, ")");

      eeprom_error = size_error(eeprom_size);
    }
    EEPROM_FINISH();

    //
    // UBL Mesh
    //
    #if ENABLED(UBL_SAVE_ACTIVE_ON_M500)
      if (bedlevel.storage_slot >= 0)
        store_mesh(bedlevel.storage_slot);
    #endif

    const bool success = (eeprom_error == ERR_EEPROM_NOERR);
    if (success) {
      LCD_MESSAGE(MSG_SETTINGS_STORED);
      TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_SETTINGS_STORED)));
    }

    TERN_(EXTENSIBLE_UI, ExtUI::onSettingsStored(success));

    return success;
  }

  EEPROM_Error MarlinSettings::check_version() {
    if (!EEPROM_START(EEPROM_OFFSET)) return ERR_EEPROM_NOPROM;
    char stored_ver[4];
    EEPROM_READ_ALWAYS(stored_ver);

    // Version has to match or defaults are used
    if (strncmp(stored_ver, version_str, 3) != 0) {
      if (stored_ver[3] != '\0') {
        stored_ver[0] = '?';
        stored_ver[1] = '\0';
      }
      DEBUG_ECHO_MSG("EEPROM version mismatch (EEPROM=", stored_ver, " Marlin=" EEPROM_VERSION ")");
      return ERR_EEPROM_VERSION;
    }
    return ERR_EEPROM_NOERR;
  }

  /**
   * M501 - Retrieve Configuration
   */
  EEPROM_Error MarlinSettings::_load() {
    EEPROM_Error eeprom_error = ERR_EEPROM_NOERR;

    const EEPROM_Error check = check_version();
    if (check == ERR_EEPROM_NOPROM) return eeprom_error;

    uint16_t stored_crc;

    do { // A block to break out of on error

      // Version has to match or defaults are used
      if (check == ERR_EEPROM_VERSION) {
        eeprom_error = check;
        break;
      }

      //
      // Optionally reset on first boot after flashing
      //
      #if ENABLED(EEPROM_INIT_NOW)
        uint32_t stored_hash;
        EEPROM_READ_ALWAYS(stored_hash);
        if (stored_hash != build_hash) {
          eeprom_error = ERR_EEPROM_CORRUPT;
          break;
        }
      #endif

      //
      // Get the stored CRC to compare at the end
      //
      EEPROM_READ_ALWAYS(stored_crc);

      //
      // A temporary float for safe storage
      //
      float dummyf = 0;

      //
      // Init to 0. Accumulated by EEPROM_READ
      //
      working_crc = 0;

      //
      // Validate the stored size against the current data structure size
      //
      uint16_t stored_size;
      EEPROM_READ_ALWAYS(stored_size);
      if ((eeprom_error = size_error(stored_size))) break;

      //
      // Extruder Parameter Count
      // Number of e_factors may change
      //
      _FIELD_TEST(e_factors);
      uint8_t e_factors;
      EEPROM_READ_ALWAYS(e_factors);

      //
      // Planner Motion
      //
      {
        // Get only the number of E stepper parameters previously stored
        // Any steppers added later are set to their defaults
        uint32_t tmp1[NUM_AXES + e_factors];
        EEPROM_READ((uint8_t *)tmp1, sizeof(tmp1)); // max_acceleration_mm_per_s2

        EEPROM_READ(planner.settings.min_segment_time_us);

        #if ENABLED(EDITABLE_STEPS_PER_UNIT)
          float tmp2[NUM_AXES + e_factors];
          EEPROM_READ((uint8_t *)tmp2, sizeof(tmp2)); // axis_steps_per_mm
        #endif

        feedRate_t tmp3[NUM_AXES + e_factors];
        EEPROM_READ((uint8_t *)tmp3, sizeof(tmp3)); // max_feedrate_mm_s

        if (!validating) LOOP_DISTINCT_AXES(i) {
          const bool in = (i < e_factors + NUM_AXES);
          planner.settings.max_acceleration_mm_per_s2[i] = in ? tmp1[i] : pgm_read_dword(&_DMA[ALIM(i, _DMA)]);
          #if ENABLED(EDITABLE_STEPS_PER_UNIT)
            planner.settings.axis_steps_per_mm[i]        = in ? tmp2[i] : pgm_read_float(&_DASU[ALIM(i, _DASU)]);
          #endif
          planner.settings.max_feedrate_mm_s[i]          = in ? tmp3[i] : pgm_read_float(&_DMF[ALIM(i, _DMF)]);
        }

        EEPROM_READ(planner.settings.acceleration);
        EEPROM_READ(planner.settings.retract_acceleration);
        EEPROM_READ(planner.settings.travel_acceleration);
        EEPROM_READ(planner.settings.min_feedrate_mm_s);
        EEPROM_READ(planner.settings.min_travel_feedrate_mm_s);

        #if ENABLED(CLASSIC_JERK)
          EEPROM_READ(planner.max_jerk);
          #if HAS_LINEAR_E_JERK
            EEPROM_READ(dummyf);
          #endif
        #else
          for (uint8_t q = LOGICAL_AXES; q--;) EEPROM_READ(dummyf);
        #endif

        EEPROM_READ(TERN(CLASSIC_JERK, dummyf, planner.junction_deviation_mm));
      }

      //
      // Home Offset (M206 / M665)
      //
      #if NUM_AXES
      {
        _FIELD_TEST(home_offset);

        #if HAS_SCARA_OFFSET
          EEPROM_READ(scara_home_offset);
        #else
          #if !HAS_HOME_OFFSET
            xyz_pos_t home_offset;
          #endif
          EEPROM_READ(home_offset);
        #endif
      }
      #endif // NUM_AXES

      //
      // Hotend Offsets, if any
      //
      {
        #if HAS_HOTEND_OFFSET
          // Skip hotend 0 which must be 0
          for (uint8_t e = 1; e < HOTENDS; ++e)
            EEPROM_READ(hotend_offset[e]);
        #endif
      }

      //
      // Filament Runout Sensor
      //
      {
        int8_t runout_sensor_enabled;
        _FIELD_TEST(runout_sensor_enabled);
        EEPROM_READ(runout_sensor_enabled);
        #if HAS_FILAMENT_SENSOR
        if (!validating) runout.enabled = runout_sensor_enabled < 0 ? FIL_RUNOUT_ENABLED_DEFAULT : runout_sensor_enabled;
        #endif

        TERN_(HAS_FILAMENT_SENSOR, if (runout.enabled) runout.reset());

        float runout_distance_mm;
        EEPROM_READ(runout_distance_mm);
        #if HAS_FILAMENT_RUNOUT_DISTANCE
          if (!validating) runout.set_runout_distance(runout_distance_mm);
        #endif
      }

      //
      // Global Leveling
      //
      EEPROM_READ(TERN(ENABLE_LEVELING_FADE_HEIGHT, new_z_fade_height, dummyf));

      //
      // AUTOTEMP
      //
      #if ENABLED(AUTOTEMP)
        EEPROM_READ(planner.autotemp.max);
        EEPROM_READ(planner.autotemp.min);
        EEPROM_READ(planner.autotemp.factor);
      #endif

      //
      // Mesh (Manual) Bed Leveling
      //
      {
        uint8_t mesh_num_x, mesh_num_y;
        uint16_t mesh_check;
        EEPROM_READ(dummyf);
        EEPROM_READ_ALWAYS(mesh_num_x);
        EEPROM_READ_ALWAYS(mesh_num_y);

        // Check value must correspond to the X/Y values
        EEPROM_READ_ALWAYS(mesh_check);
        if (mesh_check != TWO_BYTE_HASH(mesh_num_x, mesh_num_y)) {
          eeprom_error = ERR_EEPROM_CORRUPT;
          break;
        }

        #if ENABLED(MESH_BED_LEVELING)
          if (!validating) bedlevel.z_offset = dummyf;
          if (mesh_num_x == (GRID_MAX_POINTS_X) && mesh_num_y == (GRID_MAX_POINTS_Y)) {
            // EEPROM data fits the current mesh
            EEPROM_READ(bedlevel.z_values);
          }
          else if (mesh_num_x > (GRID_MAX_POINTS_X) || mesh_num_y > (GRID_MAX_POINTS_Y)) {
            eeprom_error = ERR_EEPROM_CORRUPT;
            break;
          }
          else {
            // EEPROM data is stale
            if (!validating) bedlevel.reset();
            for (uint16_t q = mesh_num_x * mesh_num_y; q--;) EEPROM_READ(dummyf);
          }
        #else
          // MBL is disabled - skip the stored data
          for (uint16_t q = mesh_num_x * mesh_num_y; q--;) EEPROM_READ(dummyf);
        #endif
      }

      //
      // Probe Z Offset
      //
      #if NUM_AXES
      {
        _FIELD_TEST(probe_offset);
        #if HAS_BED_PROBE
          const xyz_pos_t &zpo = probe.offset;
        #else
          xyz_pos_t zpo;
        #endif
        EEPROM_READ(zpo);
      }
      #endif

      //
      // Planar Bed Leveling matrix
      //
      {
        #if ABL_PLANAR
          EEPROM_READ(planner.bed_level_matrix);
        #else
          for (uint8_t q = 9; q--;) EEPROM_READ(dummyf);
        #endif
      }

      //
      // Bilinear Auto Bed Leveling
      //
      {
        uint8_t grid_max_x, grid_max_y;
        EEPROM_READ_ALWAYS(grid_max_x);                // 1 byte
        EEPROM_READ_ALWAYS(grid_max_y);                // 1 byte

        // Check value must correspond to the X/Y values
        uint16_t grid_check;
        EEPROM_READ_ALWAYS(grid_check);
        if (grid_check != TWO_BYTE_HASH(grid_max_x, grid_max_y)) {
          eeprom_error = ERR_EEPROM_CORRUPT;
          break;
        }

        xy_pos_t spacing, start;
        EEPROM_READ(spacing);                          // 2 ints
        EEPROM_READ(start);                            // 2 ints
        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          if (grid_max_x == (GRID_MAX_POINTS_X) && grid_max_y == (GRID_MAX_POINTS_Y)) {
            if (!validating) set_bed_leveling_enabled(false);
            bedlevel.set_grid(spacing, start);
            EEPROM_READ(bedlevel.z_values);            // 9 to 256 floats
          }
          else if (grid_max_x > (GRID_MAX_POINTS_X) || grid_max_y > (GRID_MAX_POINTS_Y)) {
            eeprom_error = ERR_EEPROM_CORRUPT;
            break;
          }
          else // EEPROM data is stale
        #endif // AUTO_BED_LEVELING_BILINEAR
          {
            // Skip past disabled (or stale) Bilinear Grid data
            for (uint16_t q = grid_max_x * grid_max_y; q--;) EEPROM_READ(dummyf);
          }
      }

      //
      // X Axis Twist Compensation
      //
      #if ENABLED(X_AXIS_TWIST_COMPENSATION)
        _FIELD_TEST(xatc_spacing);
        EEPROM_READ(xatc.spacing);
        EEPROM_READ(xatc.start);
        EEPROM_READ(xatc.z_offset);
      #endif

      //
      // Unified Bed Leveling active state
      //
      {
        _FIELD_TEST(planner_leveling_active);
        #if ENABLED(AUTO_BED_LEVELING_UBL)
          const bool &planner_leveling_active = planner.leveling_active;
          const int8_t &ubl_storage_slot = bedlevel.storage_slot;
        #else
          bool planner_leveling_active;
          int8_t ubl_storage_slot;
        #endif
        EEPROM_READ(planner_leveling_active);
        EEPROM_READ(ubl_storage_slot);
      }

      //
      // SERVO_ANGLES
      //
      #if HAS_SERVO_ANGLES
      {
        _FIELD_TEST(servo_angles);
        #if ENABLED(EDITABLE_SERVO_ANGLES)
          uint16_t (&servo_angles_arr)[NUM_SERVOS][2] = servo_angles;
        #else
          uint16_t servo_angles_arr[NUM_SERVOS][2];
        #endif
        EEPROM_READ(servo_angles_arr);
      }
      #endif

      //
      // Thermal first layer compensation values
      //
      #if HAS_PTC
        #if ENABLED(PTC_PROBE)
          EEPROM_READ(ptc.z_offsets_probe);
        #endif
        # if ENABLED(PTC_BED)
          EEPROM_READ(ptc.z_offsets_bed);
        #endif
        #if ENABLED(PTC_HOTEND)
          EEPROM_READ(ptc.z_offsets_hotend);
        #endif
        if (!validating) ptc.reset_index();
      #else
        // No placeholder data for this feature
      #endif

      //
      // BLTOUCH
      //
      {
        _FIELD_TEST(bltouch_od_5v_mode);
        #if ENABLED(BLTOUCH)
          const bool &bltouch_od_5v_mode = bltouch.od_5v_mode;
        #else
          bool bltouch_od_5v_mode;
        #endif
        EEPROM_READ(bltouch_od_5v_mode);

        #if HAS_BLTOUCH_HS_MODE
          _FIELD_TEST(bltouch_high_speed_mode);
          #if ENABLED(BLTOUCH)
            const bool &bltouch_high_speed_mode = bltouch.high_speed_mode;
          #else
            bool bltouch_high_speed_mode;
          #endif
          EEPROM_READ(bltouch_high_speed_mode);
        #endif
      }

      //
      // Kinematic Settings (Delta, SCARA, TPARA, Polargraph...)
      //
      #if IS_KINEMATIC
      {
        EEPROM_READ(segments_per_second);
        #if ENABLED(DELTA)
          _FIELD_TEST(delta_height);
          EEPROM_READ(delta_height);              // 1 float
          EEPROM_READ(delta_endstop_adj);         // 3 floats
          EEPROM_READ(delta_radius);              // 1 float
          EEPROM_READ(delta_diagonal_rod);        // 1 float
          EEPROM_READ(delta_tower_angle_trim);    // 3 floats
          EEPROM_READ(delta_diagonal_rod_trim);   // 3 floats
        #elif ENABLED(POLARGRAPH)
          _FIELD_TEST(draw_area_min);
          EEPROM_READ(draw_area_min);             // 2 floats
          EEPROM_READ(draw_area_max);             // 2 floats
          EEPROM_READ(polargraph_max_belt_len);   // 1 float
        #endif
      }
      #endif

      //
      // Extra Endstops offsets
      //
      #if HAS_EXTRA_ENDSTOPS
      {
        _FIELD_TEST(x2_endstop_adj);

        EEPROM_READ(TERN(X_DUAL_ENDSTOPS, endstops.x2_endstop_adj, dummyf));  // 1 float
        EEPROM_READ(TERN(Y_DUAL_ENDSTOPS, endstops.y2_endstop_adj, dummyf));  // 1 float
        EEPROM_READ(TERN(Z_MULTI_ENDSTOPS, endstops.z2_endstop_adj, dummyf)); // 1 float

        #if ENABLED(Z_MULTI_ENDSTOPS) && NUM_Z_STEPPERS >= 3
          EEPROM_READ(endstops.z3_endstop_adj); // 1 float
        #else
          EEPROM_READ(dummyf);
        #endif
        #if ENABLED(Z_MULTI_ENDSTOPS) && NUM_Z_STEPPERS >= 4
          EEPROM_READ(endstops.z4_endstop_adj); // 1 float
        #else
          EEPROM_READ(dummyf);
        #endif
      }
      #endif

      #if ENABLED(Z_STEPPER_AUTO_ALIGN)
        EEPROM_READ(z_stepper_align.xy);
        #if HAS_Z_STEPPER_ALIGN_STEPPER_XY
          EEPROM_READ(z_stepper_align.stepper_xy);
        #endif
      #endif

      //
      // LCD Preheat settings
      //
      #if HAS_PREHEAT
        _FIELD_TEST(ui_material_preset);
        EEPROM_READ(ui.material_preset);
      #endif

      //
      // Hotend PID
      //
      {
        HOTEND_LOOP() {
          raw_pidcf_t pidcf;
          EEPROM_READ(pidcf);
          #if ENABLED(PIDTEMP)
            if (!validating && !isnan(pidcf.p))
              thermalManager.temp_hotend[e].pid.set(pidcf);
          #endif
        }
      }

      //
      // PID Extrusion Scaling
      //
      {
        _FIELD_TEST(lpq_len);
        #if ENABLED(PID_EXTRUSION_SCALING)
          const int16_t &lpq_len = thermalManager.lpq_len;
        #else
          int16_t lpq_len;
        #endif
        EEPROM_READ(lpq_len);
      }

      //
      // Heated Bed PID
      //
      {
        raw_pid_t pid;
        EEPROM_READ(pid);
        #if ENABLED(PIDTEMPBED)
          if (!validating && !isnan(pid.p))
            thermalManager.temp_bed.pid.set(pid);
        #endif
      }

      //
      // Heated Chamber PID
      //
      {
        raw_pid_t pid;
        EEPROM_READ(pid);
        #if ENABLED(PIDTEMPCHAMBER)
          if (!validating && !isnan(pid.p))
            thermalManager.temp_chamber.pid.set(pid);
        #endif
      }

      //
      // User-defined Thermistors
      //
      #if HAS_USER_THERMISTORS
      {
        user_thermistor_t user_thermistor[USER_THERMISTORS];
        _FIELD_TEST(user_thermistor);
        EEPROM_READ(user_thermistor);
        if (!validating) COPY(thermalManager.user_thermistor, user_thermistor);
      }
      #endif

      //
      // Power monitor
      //
      {
        uint8_t power_monitor_flags;
        _FIELD_TEST(power_monitor_flags);
        EEPROM_READ(power_monitor_flags);
        TERN_(HAS_POWER_MONITOR, if (!validating) power_monitor.flags = power_monitor_flags);
      }

      //
      // LCD Contrast
      //
      {
        uint8_t lcd_contrast;
        _FIELD_TEST(lcd_contrast);
        EEPROM_READ(lcd_contrast);
        TERN_(HAS_LCD_CONTRAST, if (!validating) ui.contrast = lcd_contrast);
      }

      //
      // LCD Brightness
      //
      {
        uint8_t lcd_brightness;
        _FIELD_TEST(lcd_brightness);
        EEPROM_READ(lcd_brightness);
        TERN_(HAS_LCD_BRIGHTNESS, if (!validating) ui.brightness = lcd_brightness);
      }

      //
      // LCD Backlight / Sleep Timeout
      //
      #if ENABLED(EDITABLE_DISPLAY_TIMEOUT)
        #if HAS_BACKLIGHT_TIMEOUT
          EEPROM_READ(ui.backlight_timeout_minutes);
        #elif HAS_DISPLAY_SLEEP
          EEPROM_READ(ui.sleep_timeout_minutes);
        #endif
      #endif

      //
      // Controller Fan
      //
      {
        controllerFan_settings_t cfs = { 0 };
        _FIELD_TEST(controllerFan_settings);
        EEPROM_READ(cfs);
        TERN_(CONTROLLER_FAN_EDITABLE, if (!validating) controllerFan.settings = cfs);
      }

      //
      // Power-Loss Recovery
      //
      {
        _FIELD_TEST(recovery_enabled);
        bool recovery_enabled;
        celsius_t bed_temp_threshold;
        EEPROM_READ(recovery_enabled);
        EEPROM_READ(bed_temp_threshold);
        if (!validating) {
          TERN_(POWER_LOSS_RECOVERY, recovery.enabled = recovery_enabled);
          TERN_(HAS_PLR_BED_THRESHOLD, recovery.bed_temp_threshold = bed_temp_threshold);
        }
      }

      //
      // Firmware Retraction
      //
      {
        fwretract_settings_t fwretract_settings;
        bool autoretract_enabled;
        _FIELD_TEST(fwretract_settings);
        EEPROM_READ(fwretract_settings);
        EEPROM_READ(autoretract_enabled);

        #if ENABLED(FWRETRACT)
          if (!validating) {
            fwretract.settings = fwretract_settings;
            TERN_(FWRETRACT_AUTORETRACT, fwretract.autoretract_enabled = autoretract_enabled);
          }
        #endif
      }

      //
      // Homing Feedrate
      //
      #if ENABLED(EDITABLE_HOMING_FEEDRATE)
        _FIELD_TEST(homing_feedrate_mm_m);
        EEPROM_READ(homing_feedrate_mm_m);
      #endif

      //
      // Volumetric & Filament Size
      //
      {
        struct {
          bool volumetric_enabled;
          float filament_size[EXTRUDERS];
          float volumetric_extruder_limit[EXTRUDERS];
        } storage;

        _FIELD_TEST(parser_volumetric_enabled);
        EEPROM_READ(storage);

        #if DISABLED(NO_VOLUMETRICS)
          if (!validating) {
            parser.volumetric_enabled = storage.volumetric_enabled;
            COPY(planner.filament_size, storage.filament_size);
            #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
              COPY(planner.volumetric_extruder_limit, storage.volumetric_extruder_limit);
            #endif
          }
        #endif
      }

      //
      // TMC Stepper Settings
      //

      if (!validating) reset_stepper_drivers();

      // TMC Stepper Current
      {
        _FIELD_TEST(tmc_stepper_current);

        per_stepper_uint16_t currents;
        EEPROM_READ(currents);

        #if HAS_TRINAMIC_CONFIG

          #define SET_CURR(Q) stepper##Q.rms_current(currents.Q ? currents.Q : Q##_CURRENT)
          if (!validating) {
            #if AXIS_IS_TMC(X)
              SET_CURR(X);
            #endif
            #if AXIS_IS_TMC(Y)
              SET_CURR(Y);
            #endif
            #if AXIS_IS_TMC(Z)
              SET_CURR(Z);
            #endif
            #if AXIS_IS_TMC(X2)
              SET_CURR(X2);
            #endif
            #if AXIS_IS_TMC(Y2)
              SET_CURR(Y2);
            #endif
            #if AXIS_IS_TMC(Z2)
              SET_CURR(Z2);
            #endif
            #if AXIS_IS_TMC(Z3)
              SET_CURR(Z3);
            #endif
            #if AXIS_IS_TMC(Z4)
              SET_CURR(Z4);
            #endif
            #if AXIS_IS_TMC(I)
              SET_CURR(I);
            #endif
            #if AXIS_IS_TMC(J)
              SET_CURR(J);
            #endif
            #if AXIS_IS_TMC(K)
              SET_CURR(K);
            #endif
            #if AXIS_IS_TMC(U)
              SET_CURR(U);
            #endif
            #if AXIS_IS_TMC(V)
              SET_CURR(V);
            #endif
            #if AXIS_IS_TMC(W)
              SET_CURR(W);
            #endif
            #if AXIS_IS_TMC(E0)
              SET_CURR(E0);
            #endif
            #if AXIS_IS_TMC(E1)
              SET_CURR(E1);
            #endif
            #if AXIS_IS_TMC(E2)
              SET_CURR(E2);
            #endif
            #if AXIS_IS_TMC(E3)
              SET_CURR(E3);
            #endif
            #if AXIS_IS_TMC(E4)
              SET_CURR(E4);
            #endif
            #if AXIS_IS_TMC(E5)
              SET_CURR(E5);
            #endif
            #if AXIS_IS_TMC(E6)
              SET_CURR(E6);
            #endif
            #if AXIS_IS_TMC(E7)
              SET_CURR(E7);
            #endif
          }
        #endif
      }

      // TMC Hybrid Threshold
      {
        per_stepper_uint32_t tmc_hybrid_threshold;
        _FIELD_TEST(tmc_hybrid_threshold);
        EEPROM_READ(tmc_hybrid_threshold);

        #if ENABLED(HYBRID_THRESHOLD)
          if (!validating) {
            TERN_(X_HAS_STEALTHCHOP,  stepperX.set_pwm_thrs(tmc_hybrid_threshold.X));
            TERN_(Y_HAS_STEALTHCHOP,  stepperY.set_pwm_thrs(tmc_hybrid_threshold.Y));
            TERN_(Z_HAS_STEALTHCHOP,  stepperZ.set_pwm_thrs(tmc_hybrid_threshold.Z));
            TERN_(X2_HAS_STEALTHCHOP, stepperX2.set_pwm_thrs(tmc_hybrid_threshold.X2));
            TERN_(Y2_HAS_STEALTHCHOP, stepperY2.set_pwm_thrs(tmc_hybrid_threshold.Y2));
            TERN_(Z2_HAS_STEALTHCHOP, stepperZ2.set_pwm_thrs(tmc_hybrid_threshold.Z2));
            TERN_(Z3_HAS_STEALTHCHOP, stepperZ3.set_pwm_thrs(tmc_hybrid_threshold.Z3));
            TERN_(Z4_HAS_STEALTHCHOP, stepperZ4.set_pwm_thrs(tmc_hybrid_threshold.Z4));
            TERN_(I_HAS_STEALTHCHOP,  stepperI.set_pwm_thrs(tmc_hybrid_threshold.I));
            TERN_(J_HAS_STEALTHCHOP,  stepperJ.set_pwm_thrs(tmc_hybrid_threshold.J));
            TERN_(K_HAS_STEALTHCHOP,  stepperK.set_pwm_thrs(tmc_hybrid_threshold.K));
            TERN_(U_HAS_STEALTHCHOP,  stepperU.set_pwm_thrs(tmc_hybrid_threshold.U));
            TERN_(V_HAS_STEALTHCHOP,  stepperV.set_pwm_thrs(tmc_hybrid_threshold.V));
            TERN_(W_HAS_STEALTHCHOP,  stepperW.set_pwm_thrs(tmc_hybrid_threshold.W));
            TERN_(E0_HAS_STEALTHCHOP, stepperE0.set_pwm_thrs(tmc_hybrid_threshold.E0));
            TERN_(E1_HAS_STEALTHCHOP, stepperE1.set_pwm_thrs(tmc_hybrid_threshold.E1));
            TERN_(E2_HAS_STEALTHCHOP, stepperE2.set_pwm_thrs(tmc_hybrid_threshold.E2));
            TERN_(E3_HAS_STEALTHCHOP, stepperE3.set_pwm_thrs(tmc_hybrid_threshold.E3));
            TERN_(E4_HAS_STEALTHCHOP, stepperE4.set_pwm_thrs(tmc_hybrid_threshold.E4));
            TERN_(E5_HAS_STEALTHCHOP, stepperE5.set_pwm_thrs(tmc_hybrid_threshold.E5));
            TERN_(E6_HAS_STEALTHCHOP, stepperE6.set_pwm_thrs(tmc_hybrid_threshold.E6));
            TERN_(E7_HAS_STEALTHCHOP, stepperE7.set_pwm_thrs(tmc_hybrid_threshold.E7));
          }
        #endif
      }

      //
      // TMC StallGuard threshold.
      //
      {
        mot_stepper_int16_t tmc_sgt;
        _FIELD_TEST(tmc_sgt);
        EEPROM_READ(tmc_sgt);
        #if USE_SENSORLESS
          if (!validating) {
            NUM_AXIS_CODE(
              TERN_(X_SENSORLESS, stepperX.homing_threshold(tmc_sgt.X)),
              TERN_(Y_SENSORLESS, stepperY.homing_threshold(tmc_sgt.Y)),
              TERN_(Z_SENSORLESS, stepperZ.homing_threshold(tmc_sgt.Z)),
              TERN_(I_SENSORLESS, stepperI.homing_threshold(tmc_sgt.I)),
              TERN_(J_SENSORLESS, stepperJ.homing_threshold(tmc_sgt.J)),
              TERN_(K_SENSORLESS, stepperK.homing_threshold(tmc_sgt.K)),
              TERN_(U_SENSORLESS, stepperU.homing_threshold(tmc_sgt.U)),
              TERN_(V_SENSORLESS, stepperV.homing_threshold(tmc_sgt.V)),
              TERN_(W_SENSORLESS, stepperW.homing_threshold(tmc_sgt.W))
            );
            TERN_(X2_SENSORLESS, stepperX2.homing_threshold(tmc_sgt.X2));
            TERN_(Y2_SENSORLESS, stepperY2.homing_threshold(tmc_sgt.Y2));
            TERN_(Z2_SENSORLESS, stepperZ2.homing_threshold(tmc_sgt.Z2));
            TERN_(Z3_SENSORLESS, stepperZ3.homing_threshold(tmc_sgt.Z3));
            TERN_(Z4_SENSORLESS, stepperZ4.homing_threshold(tmc_sgt.Z4));
          }
        #endif
      }

      // TMC stepping mode
      {
        _FIELD_TEST(tmc_stealth_enabled);

        per_stepper_bool_t tmc_stealth_enabled;
        EEPROM_READ(tmc_stealth_enabled);

        #if HAS_TRINAMIC_CONFIG

          #define SET_STEPPING_MODE(ST) stepper##ST.stored.stealthChop_enabled = tmc_stealth_enabled.ST; stepper##ST.refresh_stepping_mode();
          if (!validating) {
            TERN_(X_HAS_STEALTHCHOP,  SET_STEPPING_MODE(X));
            TERN_(Y_HAS_STEALTHCHOP,  SET_STEPPING_MODE(Y));
            TERN_(Z_HAS_STEALTHCHOP,  SET_STEPPING_MODE(Z));
            TERN_(I_HAS_STEALTHCHOP,  SET_STEPPING_MODE(I));
            TERN_(J_HAS_STEALTHCHOP,  SET_STEPPING_MODE(J));
            TERN_(K_HAS_STEALTHCHOP,  SET_STEPPING_MODE(K));
            TERN_(U_HAS_STEALTHCHOP,  SET_STEPPING_MODE(U));
            TERN_(V_HAS_STEALTHCHOP,  SET_STEPPING_MODE(V));
            TERN_(W_HAS_STEALTHCHOP,  SET_STEPPING_MODE(W));
            TERN_(X2_HAS_STEALTHCHOP, SET_STEPPING_MODE(X2));
            TERN_(Y2_HAS_STEALTHCHOP, SET_STEPPING_MODE(Y2));
            TERN_(Z2_HAS_STEALTHCHOP, SET_STEPPING_MODE(Z2));
            TERN_(Z3_HAS_STEALTHCHOP, SET_STEPPING_MODE(Z3));
            TERN_(Z4_HAS_STEALTHCHOP, SET_STEPPING_MODE(Z4));
            TERN_(E0_HAS_STEALTHCHOP, SET_STEPPING_MODE(E0));
            TERN_(E1_HAS_STEALTHCHOP, SET_STEPPING_MODE(E1));
            TERN_(E2_HAS_STEALTHCHOP, SET_STEPPING_MODE(E2));
            TERN_(E3_HAS_STEALTHCHOP, SET_STEPPING_MODE(E3));
            TERN_(E4_HAS_STEALTHCHOP, SET_STEPPING_MODE(E4));
            TERN_(E5_HAS_STEALTHCHOP, SET_STEPPING_MODE(E5));
            TERN_(E6_HAS_STEALTHCHOP, SET_STEPPING_MODE(E6));
            TERN_(E7_HAS_STEALTHCHOP, SET_STEPPING_MODE(E7));
          }
        #endif
      }

      //
      // Linear Advance
      //
      {
        float extruder_advance_K[DISTINCT_E];
        _FIELD_TEST(planner_extruder_advance_K);
        EEPROM_READ(extruder_advance_K);
        #if ENABLED(LIN_ADVANCE)
          if (!validating)
            COPY(planner.extruder_advance_K, extruder_advance_K);
        #endif
      }

      //
      // Motor Current PWM
      //
      {
        _FIELD_TEST(motor_current_setting);
        uint32_t motor_current_setting[MOTOR_CURRENT_COUNT]
          #if HAS_MOTOR_CURRENT_SPI
             = DIGIPOT_MOTOR_CURRENT
          #endif
        ;
        #if HAS_MOTOR_CURRENT_SPI
          DEBUG_ECHO_MSG("DIGIPOTS Loading");
        #endif
        EEPROM_READ(motor_current_setting);
        #if HAS_MOTOR_CURRENT_SPI
          DEBUG_ECHO_MSG("DIGIPOTS Loaded");
        #endif
        #if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM
          if (!validating)
            COPY(stepper.motor_current_setting, motor_current_setting);
        #endif
      }

      //
      // Adaptive Step Smoothing state
      //
      #if ENABLED(ADAPTIVE_STEP_SMOOTHING_TOGGLE)
        EEPROM_READ(stepper.adaptive_step_smoothing_enabled);
      #endif

      //
      // CNC Coordinate System
      //
      #if NUM_AXES
      {
        _FIELD_TEST(coordinate_system);
        #if ENABLED(CNC_COORDINATE_SYSTEMS)
          if (!validating) (void)gcode.select_coordinate_system(-1); // Go back to machine space
          EEPROM_READ(gcode.coordinate_system);
        #else
          xyz_pos_t coordinate_system[MAX_COORDINATE_SYSTEMS];
          EEPROM_READ(coordinate_system);
        #endif
      }
      #endif

      //
      // Skew correction factors
      //
      #if ENABLED(SKEW_CORRECTION)
      {
        skew_factor_t skew_factor;
        _FIELD_TEST(planner_skew_factor);
        EEPROM_READ(skew_factor);
        #if ENABLED(SKEW_CORRECTION_GCODE)
          if (!validating) {
            planner.skew_factor.xy = skew_factor.xy;
            #if ENABLED(SKEW_CORRECTION_FOR_Z)
              planner.skew_factor.xz = skew_factor.xz;
              planner.skew_factor.yz = skew_factor.yz;
            #endif
          }
        #endif
      }
      #endif

      //
      // Advanced Pause filament load & unload lengths
      //
      #if ENABLED(CONFIGURE_FILAMENT_CHANGE)
      {
        _FIELD_TEST(fc_settings);
        EEPROM_READ(fc_settings);
      }
      #endif

      //
      // Tool-change settings
      //
      #if HAS_MULTI_EXTRUDER
        _FIELD_TEST(toolchange_settings);
        EEPROM_READ(toolchange_settings);
      #endif

      //
      // Backlash Compensation
      //
      #if NUM_AXES
      {
        xyz_float_t backlash_distance_mm;
        uint8_t backlash_correction;
        float backlash_smoothing_mm;

        _FIELD_TEST(backlash_distance_mm);
        EEPROM_READ(backlash_distance_mm);
        EEPROM_READ(backlash_correction);
        EEPROM_READ(backlash_smoothing_mm);

        #if ENABLED(BACKLASH_GCODE)
        if (!validating) {
          LOOP_NUM_AXES(axis) backlash.set_distance_mm((AxisEnum)axis, backlash_distance_mm[axis]);
          backlash.set_correction_uint8(backlash_correction);
          #ifdef BACKLASH_SMOOTHING_MM
            backlash.set_smoothing_mm(backlash_smoothing_mm);
          #endif
        }
        #endif
      }
      #endif // NUM_AXES

      //
      // Extensible UI User Data
      //
      #if ENABLED(EXTENSIBLE_UI)
      { // This is a significant hardware change; don't reserve EEPROM space when not present
        const char extui_data[ExtUI::eeprom_data_size] = { 0 };
        _FIELD_TEST(extui_data);
        EEPROM_READ(extui_data);
        if (!validating) ExtUI::onLoadSettings(extui_data);
      }
      #endif

      //
      // JyersUI User Data
      //
      #if ENABLED(DWIN_CREALITY_LCD_JYERSUI)
      {
        const char dwin_settings[jyersDWIN.eeprom_data_size] = { 0 };
        _FIELD_TEST(dwin_settings);
        EEPROM_READ(dwin_settings);
        if (!validating) jyersDWIN.loadSettings(dwin_settings);
      }
      #endif

      //
      // Case Light Brightness
      //
      #if CASELIGHT_USES_BRIGHTNESS
        _FIELD_TEST(caselight_brightness);
        EEPROM_READ(caselight.brightness);
      #endif

      //
      // Password feature
      //
      #if ENABLED(PASSWORD_FEATURE)
        _FIELD_TEST(password_is_set);
        EEPROM_READ(password.is_set);
        EEPROM_READ(password.value);
      #endif

      //
      // TOUCH_SCREEN_CALIBRATION
      //
      #if ENABLED(TOUCH_SCREEN_CALIBRATION)
        _FIELD_TEST(touch_calibration_data);
        EEPROM_READ(touch_calibration.calibration);
      #endif

      //
      // Ethernet network info
      //
      #if HAS_ETHERNET
        _FIELD_TEST(ethernet_hardware_enabled);
        uint32_t ethernet_ip, ethernet_dns, ethernet_gateway, ethernet_subnet;
        EEPROM_READ(ethernet.hardware_enabled);
        EEPROM_READ(ethernet_ip);      ethernet.ip      = ethernet_ip;
        EEPROM_READ(ethernet_dns);     ethernet.myDns   = ethernet_dns;
        EEPROM_READ(ethernet_gateway); ethernet.gateway = ethernet_gateway;
        EEPROM_READ(ethernet_subnet);  ethernet.subnet  = ethernet_subnet;
      #endif

      //
      // Buzzer enable/disable
      //
      #if ENABLED(SOUND_MENU_ITEM)
        _FIELD_TEST(sound_on);
        EEPROM_READ(ui.sound_on);
      #endif

      //
      // Fan tachometer check
      //
      #if HAS_FANCHECK
        _FIELD_TEST(fan_check_enabled);
        EEPROM_READ(fan_check.enabled);
      #endif

      //
      // MKS UI controller
      //
      #if DGUS_LCD_UI_MKS
        _FIELD_TEST(mks_language_index);
        EEPROM_READ(mks_language_index);
        EEPROM_READ(mks_corner_offsets);
        EEPROM_READ(mks_park_pos);
        EEPROM_READ(mks_min_extrusion_temp);
      #endif

      //
      // Selected LCD language
      //
      #if HAS_MULTI_LANGUAGE
      {
        uint8_t ui_language;
        EEPROM_READ(ui_language);
        if (ui_language >= NUM_LANGUAGES) ui_language = 0;
        if (!validating) ui.set_language(ui_language);
      }
      #endif

      //
      // Model predictive control
      //
      #if ENABLED(MPCTEMP)
        HOTEND_LOOP() EEPROM_READ(thermalManager.temp_hotend[e].mpc);
      #endif

      //
      // Fixed-Time Motion
      //
      #if ENABLED(FT_MOTION)
        _FIELD_TEST(ftMotion_cfg);
        EEPROM_READ(ftMotion.cfg);
      #endif

      //
      // Input Shaping
      //
      #if ENABLED(INPUT_SHAPING_X)
      {
        struct { float freq, damp; } _data;
        EEPROM_READ(_data);
        if (!validating) {
          stepper.set_shaping_frequency(X_AXIS, _data.freq);
          stepper.set_shaping_damping_ratio(X_AXIS, _data.damp);
        }
      }
      #endif

      #if ENABLED(INPUT_SHAPING_Y)
      {
        struct { float freq, damp; } _data;
        EEPROM_READ(_data);
        if (!validating) {
          stepper.set_shaping_frequency(Y_AXIS, _data.freq);
          stepper.set_shaping_damping_ratio(Y_AXIS, _data.damp);
        }
      }
      #endif

      #if ENABLED(INPUT_SHAPING_Z)
      {
        struct { float freq, damp; } _data;
        EEPROM_READ(_data);
        if (!validating) {
          stepper.set_shaping_frequency(Z_AXIS, _data.freq);
          stepper.set_shaping_damping_ratio(Z_AXIS, _data.damp);
        }
      }
      #endif

      //
      // HOTEND_IDLE_TIMEOUT
      //
      #if ENABLED(HOTEND_IDLE_TIMEOUT)
        EEPROM_READ(hotend_idle.cfg);
      #endif

      //
      // Nonlinear Extrusion
      //
      #if ENABLED(NONLINEAR_EXTRUSION)
        EEPROM_READ(stepper.ne);
      #endif

      //
      // MMU3
      //
      #if HAS_PRUSA_MMU3
        spooljoin.epprom_addr = eeprom_index;
        EEPROM_READ(spooljoin.enabled);  // EEPROM_SPOOL_JOIN

        MMU3::operation_statistics.fail_total_num_addr = eeprom_index;
        EEPROM_READ(MMU3::operation_statistics.fail_total_num); //EEPROM_MMU_FAIL_TOT

        MMU3::operation_statistics.fail_num_addr = eeprom_index;
        EEPROM_READ(MMU3::operation_statistics.fail_num); // EEPROM_MMU_FAIL;

        MMU3::operation_statistics.load_fail_total_num_addr = eeprom_index;
        EEPROM_READ(MMU3::operation_statistics.load_fail_total_num); // EEPROM_MMU_LOAD_FAIL_TOT

        MMU3::operation_statistics.load_fail_num_addr = eeprom_index;
        EEPROM_READ(MMU3::operation_statistics.load_fail_num); // EEPROM_MMU_LOAD_FAIL

        MMU3::operation_statistics.tool_change_counter_addr = eeprom_index;
        EEPROM_READ(MMU3::operation_statistics.tool_change_counter);

        MMU3::operation_statistics.tool_change_total_counter_addr = eeprom_index;
        EEPROM_READ(MMU3::operation_statistics.tool_change_total_counter); // EEPROM_MMU_MATERIAL_CHANGES

        mmu3.cutter_mode_addr = eeprom_index;
        EEPROM_READ(mmu3.cutter_mode); // EEPROM_MMU_CUTTER_ENABLED

        mmu3.stealth_mode_addr = eeprom_index;
        EEPROM_READ(mmu3.stealth_mode); // EEPROM_MMU_STEALTH

        mmu3.mmu_hw_enabled_addr = eeprom_index;
        EEPROM_READ(mmu3.mmu_hw_enabled); // EEPROM_MMU_ENABLED
      #endif

      //
      // Validate Final Size and CRC
      //
      const uint16_t eeprom_total = eeprom_index - (EEPROM_OFFSET);
      if ((eeprom_error = size_error(eeprom_total))) {
        // Handle below and on return
        break;
      }
      else if (working_crc != stored_crc) {
        eeprom_error = ERR_EEPROM_CRC;
        break;
      }
      else if (!validating) {
        DEBUG_ECHO_START();
        DEBUG_ECHOLN(version_str, F(" stored settings retrieved ("), eeprom_total, F(" bytes; crc "), working_crc, C(')'));
        TERN_(HOST_EEPROM_CHITCHAT, hostui.notify(F("Stored settings retrieved")));
      }

      #if ENABLED(AUTO_BED_LEVELING_UBL)
        if (!validating) {
          bedlevel.report_state();

          if (!bedlevel.sanity_check()) {
            #if ALL(EEPROM_CHITCHAT, DEBUG_LEVELING_FEATURE)
              bedlevel.echo_name();
              DEBUG_ECHOLNPGM(" initialized.\n");
            #endif
          }
          else {
            eeprom_error = ERR_EEPROM_CORRUPT;
            #if ALL(EEPROM_CHITCHAT, DEBUG_LEVELING_FEATURE)
              DEBUG_ECHOPGM("?Can't enable ");
              bedlevel.echo_name();
              DEBUG_ECHOLNPGM(".");
            #endif
            bedlevel.reset();
          }

          if (bedlevel.storage_slot >= 0) {
            load_mesh(bedlevel.storage_slot);
            DEBUG_ECHOLNPGM("Mesh ", bedlevel.storage_slot, " loaded from storage.");
          }
          else {
            bedlevel.reset();
            DEBUG_ECHOLNPGM("UBL reset");
          }
        }
      #endif

    } while(0);

    EEPROM_FINISH();

    switch (eeprom_error) {
      case ERR_EEPROM_NOERR:
        if (!validating) postprocess();
        break;
      case ERR_EEPROM_SIZE:
        DEBUG_ECHO_MSG("Index: ", eeprom_index - (EEPROM_OFFSET), " Size: ", datasize());
        break;
      case ERR_EEPROM_CORRUPT:
        DEBUG_WARN_MSG(STR_ERR_EEPROM_CORRUPT);
        break;
      case ERR_EEPROM_CRC:
        DEBUG_WARN_MSG("EEPROM CRC mismatch - (stored) ", stored_crc, " != ", working_crc, " (calculated)!");
        TERN_(HOST_EEPROM_CHITCHAT, hostui.notify(GET_TEXT_F(MSG_ERR_EEPROM_CRC)));
        break;
      default: break;
    }

    #if ENABLED(EEPROM_CHITCHAT) && DISABLED(DISABLE_M503)
      // Report the EEPROM settings
      if (!validating && TERN1(EEPROM_BOOT_SILENT, IsRunning())) report();
    #endif

    return eeprom_error;
  }

  #ifdef ARCHIM2_SPI_FLASH_EEPROM_BACKUP_SIZE
    extern bool restoreEEPROM();
  #endif

  bool MarlinSettings::validate() {
    validating = true;
    #ifdef ARCHIM2_SPI_FLASH_EEPROM_BACKUP_SIZE
      EEPROM_Error err = _load();
      if (err != ERR_EEPROM_NOERR && restoreEEPROM()) {
        SERIAL_ECHOLNPGM("Recovered backup EEPROM settings from SPI Flash");
        err = _load();
      }
    #else
      const EEPROM_Error err = _load();
    #endif
    validating = false;

    if (err) ui.eeprom_alert(err);

    return (err == ERR_EEPROM_NOERR);
  }

  #if HAS_EARLY_LCD_SETTINGS

    #if HAS_LCD_CONTRAST
      void MarlinSettings::load_contrast() {
        uint8_t lcd_contrast; EEPROM_START(EEPROM_OFFSETOF(lcd_contrast)); EEPROM_READ(lcd_contrast);
        DEBUG_ECHOLNPGM("LCD Contrast: ", lcd_contrast);
        ui.contrast = lcd_contrast;
      }
    #endif

    #if HAS_LCD_BRIGHTNESS
      void MarlinSettings::load_brightness() {
        uint8_t lcd_brightness; EEPROM_START(EEPROM_OFFSETOF(lcd_brightness)); EEPROM_READ(lcd_brightness);
        DEBUG_ECHOLNPGM("LCD Brightness: ", lcd_brightness);
        ui.brightness = lcd_brightness;
      }
    #endif

  #endif // HAS_EARLY_LCD_SETTINGS

  bool MarlinSettings::load() {
    // If the EEPROM data is valid load it
    if (validate()) {
      const EEPROM_Error err = _load();
      const bool success = (err == ERR_EEPROM_NOERR);
      TERN_(EXTENSIBLE_UI, ExtUI::onSettingsLoaded(success));
      return success;
    }

    // Otherwise reset settings to default "factory settings"
    reset();

    // Options to overwrite the EEPROM on error
    #if ANY(EEPROM_AUTO_INIT, EEPROM_INIT_NOW)
      (void)init_eeprom();
      LCD_MESSAGE(MSG_EEPROM_INITIALIZED);
      SERIAL_ECHO_MSG(STR_EEPROM_INITIALIZED);
    #endif

    return false;
  }

  #if ENABLED(AUTO_BED_LEVELING_UBL)

    inline void ubl_invalid_slot(const int s) {
      DEBUG_ECHOLNPGM("?Invalid slot.\n", s, " mesh slots available.");
      UNUSED(s);
    }

    // 128 (+1 because of the change to capacity rather than last valid address)
    // is a placeholder for the size of the MAT; the MAT will always
    // live at the very end of the eeprom
    const uint16_t MarlinSettings::meshes_end = persistentStore.capacity() - 129;

    uint16_t MarlinSettings::meshes_start_index() {
      // Pad the end of configuration data so it can float up
      // or down a little bit without disrupting the mesh data
      return (datasize() + EEPROM_OFFSET + 32) & 0xFFF8;
    }

    #define MESH_STORE_SIZE sizeof(TERN(OPTIMIZED_MESH_STORAGE, mesh_store_t, bedlevel.z_values))

    uint16_t MarlinSettings::calc_num_meshes() {
      return (meshes_end - meshes_start_index()) / MESH_STORE_SIZE;
    }

    int MarlinSettings::mesh_slot_offset(const int8_t slot) {
      return meshes_end - (slot + 1) * MESH_STORE_SIZE;
    }

    void MarlinSettings::store_mesh(const int8_t slot) {

      #if ENABLED(AUTO_BED_LEVELING_UBL)
        const int16_t a = calc_num_meshes();
        if (!WITHIN(slot, 0, a - 1)) {
          ubl_invalid_slot(a);
          DEBUG_ECHOLNPGM("E2END=", persistentStore.capacity() - 1, " meshes_end=", meshes_end, " slot=", slot);
          DEBUG_EOL();
          return;
        }

        int pos = mesh_slot_offset(slot);
        uint16_t crc = 0;

        #if ENABLED(OPTIMIZED_MESH_STORAGE)
          int16_t z_mesh_store[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
          bedlevel.set_store_from_mesh(bedlevel.z_values, z_mesh_store);
          uint8_t * const src = (uint8_t*)&z_mesh_store;
        #else
          uint8_t * const src = (uint8_t*)&bedlevel.z_values;
        #endif

        // Write crc to MAT along with other data, or just tack on to the beginning or end
        persistentStore.access_start();
        const bool status = persistentStore.write_data(pos, src, MESH_STORE_SIZE, &crc);
        persistentStore.access_finish();

        if (status) SERIAL_ECHOLNPGM("?Unable to save mesh data.");
        else        DEBUG_ECHOLNPGM("Mesh saved in slot ", slot);

      #else

        // Other mesh types

      #endif
    }

    void MarlinSettings::load_mesh(const int8_t slot, void * const into/*=nullptr*/) {

      #if ENABLED(AUTO_BED_LEVELING_UBL)

        const int16_t a = settings.calc_num_meshes();

        if (!WITHIN(slot, 0, a - 1)) {
          ubl_invalid_slot(a);
          return;
        }

        int pos = mesh_slot_offset(slot);
        uint16_t crc = 0;
        #if ENABLED(OPTIMIZED_MESH_STORAGE)
          int16_t z_mesh_store[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
          uint8_t * const dest = (uint8_t*)&z_mesh_store;
        #else
          uint8_t * const dest = into ? (uint8_t*)into : (uint8_t*)&bedlevel.z_values;
        #endif

        persistentStore.access_start();
        uint16_t status = persistentStore.read_data(pos, dest, MESH_STORE_SIZE, &crc);
        persistentStore.access_finish();

        #if ENABLED(OPTIMIZED_MESH_STORAGE)
          if (into) {
            float z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
            bedlevel.set_mesh_from_store(z_mesh_store, z_values);
            memcpy(into, z_values, sizeof(z_values));
          }
          else
            bedlevel.set_mesh_from_store(z_mesh_store, bedlevel.z_values);
        #endif

        #if ENABLED(DWIN_LCD_PROUI)
          status = !bedLevelTools.meshValidate();
          if (status) {
            bedlevel.invalidate();
            LCD_MESSAGE(MSG_UBL_MESH_INVALID);
          }
          else
            ui.status_printf(0, GET_TEXT_F(MSG_MESH_LOADED), bedlevel.storage_slot);
        #endif

        if (status) SERIAL_ECHOLNPGM("?Unable to load mesh data.");
        else        DEBUG_ECHOLNPGM("Mesh loaded from slot ", slot);

        EEPROM_FINISH();

      #else

        // Other mesh types

      #endif
    }

    //void MarlinSettings::delete_mesh() { return; }
    //void MarlinSettings::defrag_meshes() { return; }

  #endif // AUTO_BED_LEVELING_UBL

#else // !EEPROM_SETTINGS

  bool MarlinSettings::save() {
    DEBUG_WARN_MSG("EEPROM disabled");
    return false;
  }

#endif // !EEPROM_SETTINGS

#if HAS_EARLY_LCD_SETTINGS

  void MarlinSettings::load_lcd_state() {
    if (TERN0(EEPROM_SETTINGS, check_version() == ERR_EEPROM_NOERR)) {
      #if ENABLED(EEPROM_SETTINGS)
        TERN_(HAS_LCD_CONTRAST, load_contrast());
        TERN_(HAS_LCD_BRIGHTNESS, load_brightness());
      #endif
    }
    else {
      TERN_(HAS_LCD_CONTRAST, ui.contrast = LCD_CONTRAST_DEFAULT);
      TERN_(HAS_LCD_BRIGHTNESS, ui.brightness = LCD_BRIGHTNESS_DEFAULT);
    }
    TERN_(HAS_LCD_CONTRAST, ui.refresh_contrast());
    TERN_(HAS_LCD_BRIGHTNESS, ui.refresh_brightness());
  }

#endif // HAS_EARLY_LCD_SETTINGS

/**
 * M502 - Reset Configuration
 */
void MarlinSettings::reset() {
  LOOP_DISTINCT_AXES(i) {
    planner.settings.max_acceleration_mm_per_s2[i] = pgm_read_dword(&_DMA[ALIM(i, _DMA)]);
    #if ENABLED(EDITABLE_STEPS_PER_UNIT)
      planner.settings.axis_steps_per_mm[i] = pgm_read_float(&_DASU[ALIM(i, _DASU)]);
    #endif
    planner.settings.max_feedrate_mm_s[i] = pgm_read_float(&_DMF[ALIM(i, _DMF)]);
  }

  planner.settings.min_segment_time_us = DEFAULT_MINSEGMENTTIME;
  planner.settings.acceleration = DEFAULT_ACCELERATION;
  planner.settings.retract_acceleration = DEFAULT_RETRACT_ACCELERATION;
  planner.settings.travel_acceleration = DEFAULT_TRAVEL_ACCELERATION;
  planner.settings.min_feedrate_mm_s = feedRate_t(DEFAULT_MINIMUMFEEDRATE);
  planner.settings.min_travel_feedrate_mm_s = feedRate_t(DEFAULT_MINTRAVELFEEDRATE);

  #if ENABLED(CLASSIC_JERK)
    #if HAS_X_AXIS && !defined(DEFAULT_XJERK)
      #define DEFAULT_XJERK 0
    #endif
    #if HAS_Y_AXIS && !defined(DEFAULT_YJERK)
      #define DEFAULT_YJERK 0
    #endif
    #if HAS_Z_AXIS && !defined(DEFAULT_ZJERK)
      #define DEFAULT_ZJERK 0
    #endif
    #if HAS_I_AXIS && !defined(DEFAULT_IJERK)
      #define DEFAULT_IJERK 0
    #endif
    #if HAS_J_AXIS && !defined(DEFAULT_JJERK)
      #define DEFAULT_JJERK 0
    #endif
    #if HAS_K_AXIS && !defined(DEFAULT_KJERK)
      #define DEFAULT_KJERK 0
    #endif
    #if HAS_U_AXIS && !defined(DEFAULT_UJERK)
      #define DEFAULT_UJERK 0
    #endif
    #if HAS_V_AXIS && !defined(DEFAULT_VJERK)
      #define DEFAULT_VJERK 0
    #endif
    #if HAS_W_AXIS && !defined(DEFAULT_WJERK)
      #define DEFAULT_WJERK 0
    #endif
    planner.max_jerk.set(
      NUM_AXIS_LIST(DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_IJERK, DEFAULT_JJERK, DEFAULT_KJERK, DEFAULT_UJERK, DEFAULT_VJERK, DEFAULT_WJERK)
    );
    TERN_(HAS_CLASSIC_E_JERK, planner.max_jerk.e = DEFAULT_EJERK);
  #endif

  TERN_(HAS_JUNCTION_DEVIATION, planner.junction_deviation_mm = float(JUNCTION_DEVIATION_MM));

  #if HAS_SCARA_OFFSET
    scara_home_offset.reset();
  #elif HAS_HOME_OFFSET
    home_offset.reset();
  #endif

  TERN_(HAS_HOTEND_OFFSET, reset_hotend_offsets());

  //
  // Filament Runout Sensor
  //

  #if HAS_FILAMENT_SENSOR
    runout.enabled = FIL_RUNOUT_ENABLED_DEFAULT;
    runout.reset();
    TERN_(HAS_FILAMENT_RUNOUT_DISTANCE, runout.set_runout_distance(FILAMENT_RUNOUT_DISTANCE_MM));
  #endif

  //
  // Tool-change Settings
  //

  #if HAS_MULTI_EXTRUDER
    #if ENABLED(TOOLCHANGE_FILAMENT_SWAP)
      toolchange_settings.swap_length     = TOOLCHANGE_FS_LENGTH;
      toolchange_settings.extra_resume    = TOOLCHANGE_FS_EXTRA_RESUME_LENGTH;
      toolchange_settings.retract_speed   = TOOLCHANGE_FS_RETRACT_SPEED;
      toolchange_settings.unretract_speed = TOOLCHANGE_FS_UNRETRACT_SPEED;
      toolchange_settings.extra_prime     = TOOLCHANGE_FS_EXTRA_PRIME;
      toolchange_settings.prime_speed     = TOOLCHANGE_FS_PRIME_SPEED;
      toolchange_settings.wipe_retract    = TOOLCHANGE_FS_WIPE_RETRACT;
      toolchange_settings.fan_speed       = TOOLCHANGE_FS_FAN_SPEED;
      toolchange_settings.fan_time        = TOOLCHANGE_FS_FAN_TIME;
    #endif

    #if ENABLED(TOOLCHANGE_FS_PRIME_FIRST_USED)
      enable_first_prime = false;
    #endif

    #if ENABLED(TOOLCHANGE_PARK)
      constexpr xyz_pos_t tpxy = TOOLCHANGE_PARK_XY;
      toolchange_settings.enable_park = true;
      toolchange_settings.change_point = tpxy;
    #endif

    toolchange_settings.z_raise = TOOLCHANGE_ZRAISE;

    #if ENABLED(TOOLCHANGE_MIGRATION_FEATURE)
      migration = migration_defaults;
    #endif

  #endif

  #if ENABLED(BACKLASH_GCODE)
    backlash.set_correction(BACKLASH_CORRECTION);
    constexpr xyz_float_t tmp = BACKLASH_DISTANCE_MM;
    LOOP_NUM_AXES(axis) backlash.set_distance_mm((AxisEnum)axis, tmp[axis]);
    #ifdef BACKLASH_SMOOTHING_MM
      backlash.set_smoothing_mm(BACKLASH_SMOOTHING_MM);
    #endif
  #endif

  TERN_(DWIN_CREALITY_LCD_JYERSUI, jyersDWIN.resetSettings());

  //
  // Case Light Brightness
  //
  TERN_(CASELIGHT_USES_BRIGHTNESS, caselight.brightness = CASE_LIGHT_DEFAULT_BRIGHTNESS);

  //
  // TOUCH_SCREEN_CALIBRATION
  //
  TERN_(TOUCH_SCREEN_CALIBRATION, touch_calibration.calibration_reset());

  //
  // Buzzer enable/disable
  //
  #if ENABLED(SOUND_MENU_ITEM)
    ui.sound_on = ENABLED(SOUND_ON_DEFAULT);
  #endif

  //
  // Magnetic Parking Extruder
  //
  TERN_(MAGNETIC_PARKING_EXTRUDER, mpe_settings_init());

  //
  // Global Leveling
  //
  TERN_(ENABLE_LEVELING_FADE_HEIGHT, new_z_fade_height = DEFAULT_LEVELING_FADE_HEIGHT);
  TERN_(HAS_LEVELING, reset_bed_level());

  //
  // AUTOTEMP
  //
  #if ENABLED(AUTOTEMP)
    planner.autotemp.max = AUTOTEMP_MAX;
    planner.autotemp.min = AUTOTEMP_MIN;
    planner.autotemp.factor = AUTOTEMP_FACTOR;
  #endif

  //
  // X Axis Twist Compensation
  //
  TERN_(X_AXIS_TWIST_COMPENSATION, xatc.reset());

  //
  // Nozzle-to-probe Offset
  //
  #if HAS_BED_PROBE
    constexpr float dpo[] = NOZZLE_TO_PROBE_OFFSET;
    static_assert(COUNT(dpo) == NUM_AXES, "NOZZLE_TO_PROBE_OFFSET must contain offsets for each linear axis X, Y, Z....");
    #if HAS_PROBE_XY_OFFSET
      LOOP_NUM_AXES(a) probe.offset[a] = dpo[a];
    #else
      probe.offset.set(NUM_AXIS_LIST(0, 0, dpo[Z_AXIS], 0, 0, 0, 0, 0, 0));
    #endif
  #endif

  //
  // Z Stepper Auto-alignment points
  //
  TERN_(Z_STEPPER_AUTO_ALIGN, z_stepper_align.reset_to_default());

  //
  // Servo Angles
  //
  TERN_(EDITABLE_SERVO_ANGLES, COPY(servo_angles, base_servo_angles)); // When not editable only one copy of servo angles exists

  //
  // Probe Temperature Compensation
  //
  TERN_(HAS_PTC, ptc.reset());

  //
  // BLTouch
  //
  TERN_(HAS_BLTOUCH_HS_MODE, bltouch.high_speed_mode = BLTOUCH_HS_MODE);

  //
  // Kinematic Settings (Delta, SCARA, TPARA, Polargraph...)
  //

  #if IS_KINEMATIC
    segments_per_second = DEFAULT_SEGMENTS_PER_SECOND;
    #if ENABLED(DELTA)
      const abc_float_t adj = DELTA_ENDSTOP_ADJ, dta = DELTA_TOWER_ANGLE_TRIM, ddr = DELTA_DIAGONAL_ROD_TRIM_TOWER;
      delta_height = DELTA_HEIGHT;
      delta_endstop_adj = adj;
      delta_radius = DELTA_RADIUS;
      delta_diagonal_rod = DELTA_DIAGONAL_ROD;
      delta_tower_angle_trim = dta;
      delta_diagonal_rod_trim = ddr;
    #elif ENABLED(POLARGRAPH)
      draw_area_min.set(X_MIN_POS, Y_MIN_POS);
      draw_area_max.set(X_MAX_POS, Y_MAX_POS);
      polargraph_max_belt_len = POLARGRAPH_MAX_BELT_LEN;
    #endif
  #endif

  //
  // Endstop Adjustments
  //

  #if ENABLED(X_DUAL_ENDSTOPS)
    #ifndef X2_ENDSTOP_ADJUSTMENT
      #define X2_ENDSTOP_ADJUSTMENT 0
    #endif
    endstops.x2_endstop_adj = X2_ENDSTOP_ADJUSTMENT;
  #endif

  #if ENABLED(Y_DUAL_ENDSTOPS)
    #ifndef Y2_ENDSTOP_ADJUSTMENT
      #define Y2_ENDSTOP_ADJUSTMENT 0
    #endif
    endstops.y2_endstop_adj = Y2_ENDSTOP_ADJUSTMENT;
  #endif

  #if ENABLED(Z_MULTI_ENDSTOPS)
    #ifndef Z2_ENDSTOP_ADJUSTMENT
      #define Z2_ENDSTOP_ADJUSTMENT 0
    #endif
    endstops.z2_endstop_adj = Z2_ENDSTOP_ADJUSTMENT;
    #if NUM_Z_STEPPERS >= 3
      #ifndef Z3_ENDSTOP_ADJUSTMENT
        #define Z3_ENDSTOP_ADJUSTMENT 0
      #endif
      endstops.z3_endstop_adj = Z3_ENDSTOP_ADJUSTMENT;
    #endif
    #if NUM_Z_STEPPERS >= 4
      #ifndef Z4_ENDSTOP_ADJUSTMENT
        #define Z4_ENDSTOP_ADJUSTMENT 0
      #endif
      endstops.z4_endstop_adj = Z4_ENDSTOP_ADJUSTMENT;
    #endif
  #endif

  //
  // Preheat parameters
  //
  #if HAS_PREHEAT
    #define _PITEM(N,T) PREHEAT_##N##_##T,
    #if HAS_HOTEND
      constexpr uint16_t hpre[] = { REPEAT2_S(1, INCREMENT(PREHEAT_COUNT), _PITEM, TEMP_HOTEND) };
    #endif
    #if HAS_HEATED_BED
      constexpr uint16_t bpre[] = { REPEAT2_S(1, INCREMENT(PREHEAT_COUNT), _PITEM, TEMP_BED) };
    #endif
    #if HAS_FAN
      constexpr uint8_t fpre[] = { REPEAT2_S(1, INCREMENT(PREHEAT_COUNT), _PITEM, FAN_SPEED) };
    #endif
    for (uint8_t i = 0; i < PREHEAT_COUNT; ++i) {
      TERN_(HAS_HOTEND,     ui.material_preset[i].hotend_temp = hpre[i]);
      TERN_(HAS_HEATED_BED, ui.material_preset[i].bed_temp = bpre[i]);
      TERN_(HAS_FAN,        ui.material_preset[i].fan_speed = fpre[i]);
    }
  #endif

  //
  // Hotend PID
  //

  #if ENABLED(PIDTEMP)
    #if ENABLED(PID_PARAMS_PER_HOTEND)
      constexpr float defKp[] =
        #ifdef DEFAULT_Kp_LIST
          DEFAULT_Kp_LIST
        #else
          ARRAY_BY_HOTENDS1(DEFAULT_Kp)
        #endif
      , defKi[] =
        #ifdef DEFAULT_Ki_LIST
          DEFAULT_Ki_LIST
        #else
          ARRAY_BY_HOTENDS1(DEFAULT_Ki)
        #endif
      , defKd[] =
        #ifdef DEFAULT_Kd_LIST
          DEFAULT_Kd_LIST
        #else
          ARRAY_BY_HOTENDS1(DEFAULT_Kd)
        #endif
      ;
      static_assert(WITHIN(COUNT(defKp), 1, HOTENDS), "DEFAULT_Kp_LIST must have between 1 and HOTENDS items.");
      static_assert(WITHIN(COUNT(defKi), 1, HOTENDS), "DEFAULT_Ki_LIST must have between 1 and HOTENDS items.");
      static_assert(WITHIN(COUNT(defKd), 1, HOTENDS), "DEFAULT_Kd_LIST must have between 1 and HOTENDS items.");
      #if ENABLED(PID_EXTRUSION_SCALING)
        constexpr float defKc[] =
          #ifdef DEFAULT_Kc_LIST
            DEFAULT_Kc_LIST
          #else
            ARRAY_BY_HOTENDS1(DEFAULT_Kc)
          #endif
        ;
        static_assert(WITHIN(COUNT(defKc), 1, HOTENDS), "DEFAULT_Kc_LIST must have between 1 and HOTENDS items.");
      #endif
      #if ENABLED(PID_FAN_SCALING)
        constexpr float defKf[] =
          #ifdef DEFAULT_Kf_LIST
            DEFAULT_Kf_LIST
          #else
            ARRAY_BY_HOTENDS1(DEFAULT_Kf)
          #endif
        ;
        static_assert(WITHIN(COUNT(defKf), 1, HOTENDS), "DEFAULT_Kf_LIST must have between 1 and HOTENDS items.");
      #endif
      #define PID_DEFAULT(N,E) def##N[E]
    #else
      #define PID_DEFAULT(N,E) DEFAULT_##N
    #endif
    HOTEND_LOOP() {
      thermalManager.temp_hotend[e].pid.set(
        PID_DEFAULT(Kp, ALIM(e, defKp)),
        PID_DEFAULT(Ki, ALIM(e, defKi)),
        PID_DEFAULT(Kd, ALIM(e, defKd))
        OPTARG(PID_EXTRUSION_SCALING, PID_DEFAULT(Kc, ALIM(e, defKc)))
        OPTARG(PID_FAN_SCALING, PID_DEFAULT(Kf, ALIM(e, defKf)))
      );
    }
  #endif

  //
  // PID Extrusion Scaling
  //
  TERN_(PID_EXTRUSION_SCALING, thermalManager.lpq_len = 20); // Default last-position-queue size

  //
  // Heated Bed PID
  //
  #if ENABLED(PIDTEMPBED)
    thermalManager.temp_bed.pid.set(DEFAULT_bedKp, DEFAULT_bedKi, DEFAULT_bedKd);
  #endif

  //
  // Heated Chamber PID
  //
  #if ENABLED(PIDTEMPCHAMBER)
    thermalManager.temp_chamber.pid.set(DEFAULT_chamberKp, DEFAULT_chamberKi, DEFAULT_chamberKd);
  #endif

  //
  // User-Defined Thermistors
  //
  TERN_(HAS_USER_THERMISTORS, thermalManager.reset_user_thermistors());

  //
  // Power Monitor
  //
  TERN_(POWER_MONITOR, power_monitor.reset());

  //
  // LCD Contrast
  //
  TERN_(HAS_LCD_CONTRAST, ui.contrast = LCD_CONTRAST_DEFAULT);

  //
  // LCD Brightness
  //
  TERN_(HAS_LCD_BRIGHTNESS, ui.brightness = LCD_BRIGHTNESS_DEFAULT);

  //
  // LCD Backlight / Sleep Timeout
  //
  #if ENABLED(EDITABLE_DISPLAY_TIMEOUT)
    #if HAS_BACKLIGHT_TIMEOUT
      ui.backlight_timeout_minutes = LCD_BACKLIGHT_TIMEOUT_MINS;
    #elif HAS_DISPLAY_SLEEP
      ui.sleep_timeout_minutes = DISPLAY_SLEEP_MINUTES;
    #endif
  #endif

  //
  // Controller Fan
  //
  TERN_(USE_CONTROLLER_FAN, controllerFan.reset());

  //
  // Power-Loss Recovery
  //
  #if ENABLED(POWER_LOSS_RECOVERY)
    recovery.enable(ENABLED(PLR_ENABLED_DEFAULT));
    TERN_(HAS_PLR_BED_THRESHOLD, recovery.bed_temp_threshold = PLR_BED_THRESHOLD);
  #endif

  //
  // Firmware Retraction
  //
  TERN_(FWRETRACT, fwretract.reset());

  //
  // Homing Feedrate
  //
  TERN_(EDITABLE_HOMING_FEEDRATE, homing_feedrate_mm_m = xyz_feedrate_t(HOMING_FEEDRATE_MM_M));

  //
  // Volumetric & Filament Size
  //
  #if DISABLED(NO_VOLUMETRICS)
    parser.volumetric_enabled = ENABLED(VOLUMETRIC_DEFAULT_ON);
    for (uint8_t q = 0; q < COUNT(planner.filament_size); ++q)
      planner.filament_size[q] = DEFAULT_NOMINAL_FILAMENT_DIA;
    #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
      for (uint8_t q = 0; q < COUNT(planner.volumetric_extruder_limit); ++q)
        planner.volumetric_extruder_limit[q] = DEFAULT_VOLUMETRIC_EXTRUDER_LIMIT;
    #endif
  #endif

  endstops.enable_globally(ENABLED(ENDSTOPS_ALWAYS_ON_DEFAULT));

  reset_stepper_drivers();

  //
  // Linear Advance
  //
  #if ENABLED(LIN_ADVANCE)
    #if ENABLED(DISTINCT_E_FACTORS)
      constexpr float linAdvanceK[] = ADVANCE_K;
      EXTRUDER_LOOP() {
        const float a = linAdvanceK[_MAX(uint8_t(e), COUNT(linAdvanceK) - 1)];
        planner.extruder_advance_K[e] = a;
        TERN_(ADVANCE_K_EXTRA, other_extruder_advance_K[e] = a);
      }
    #else
      planner.extruder_advance_K[0] = ADVANCE_K;
    #endif
  #endif

  //
  // Motor Current PWM
  //

  #if HAS_MOTOR_CURRENT_PWM
    constexpr uint32_t tmp_motor_current_setting[MOTOR_CURRENT_COUNT] = PWM_MOTOR_CURRENT;
    for (uint8_t q = 0; q < MOTOR_CURRENT_COUNT; ++q)
      stepper.set_digipot_current(q, (stepper.motor_current_setting[q] = tmp_motor_current_setting[q]));
  #endif

  //
  // DIGIPOTS
  //
  #if HAS_MOTOR_CURRENT_SPI
    static constexpr uint32_t tmp_motor_current_setting[] = DIGIPOT_MOTOR_CURRENT;
    DEBUG_ECHOLNPGM("Writing Digipot");
    for (uint8_t q = 0; q < COUNT(tmp_motor_current_setting); ++q)
      stepper.set_digipot_current(q, tmp_motor_current_setting[q]);
    DEBUG_ECHOLNPGM("Digipot Written");
  #endif

  //
  // Adaptive Step Smoothing state
  //
  #if ENABLED(ADAPTIVE_STEP_SMOOTHING_TOGGLE)
    stepper.adaptive_step_smoothing_enabled = true;
  #endif

  //
  // CNC Coordinate System
  //
  TERN_(CNC_COORDINATE_SYSTEMS, (void)gcode.select_coordinate_system(-1)); // Go back to machine space

  //
  // Skew Correction
  //
  #if ENABLED(SKEW_CORRECTION_GCODE)
    planner.skew_factor.xy = XY_SKEW_FACTOR;
    #if ENABLED(SKEW_CORRECTION_FOR_Z)
      planner.skew_factor.xz = XZ_SKEW_FACTOR;
      planner.skew_factor.yz = YZ_SKEW_FACTOR;
    #endif
  #endif

  //
  // Advanced Pause filament load & unload lengths
  //
  #if ENABLED(CONFIGURE_FILAMENT_CHANGE)
    EXTRUDER_LOOP() {
      fc_settings[e].unload_length = FILAMENT_CHANGE_UNLOAD_LENGTH;
      fc_settings[e].load_length = FILAMENT_CHANGE_FAST_LOAD_LENGTH;
    }
  #endif

  #if ENABLED(PASSWORD_FEATURE)
    #ifdef PASSWORD_DEFAULT_VALUE
      password.is_set = true;
      password.value = PASSWORD_DEFAULT_VALUE;
    #else
      password.is_set = false;
    #endif
  #endif

  //
  // Fan tachometer check
  //
  TERN_(HAS_FANCHECK, fan_check.enabled = true);

  //
  // MKS UI controller
  //
  TERN_(DGUS_LCD_UI_MKS, MKS_reset_settings());

  //
  // Model predictive control
  //
  #if ENABLED(MPCTEMP)
    constexpr float _mpc_heater_power[] = MPC_HEATER_POWER;
    constexpr float _mpc_block_heat_capacity[] = MPC_BLOCK_HEAT_CAPACITY;
    constexpr float _mpc_sensor_responsiveness[] = MPC_SENSOR_RESPONSIVENESS;
    constexpr float _mpc_ambient_xfer_coeff[] = MPC_AMBIENT_XFER_COEFF;
    #if ENABLED(MPC_INCLUDE_FAN)
      constexpr float _mpc_ambient_xfer_coeff_fan255[] = MPC_AMBIENT_XFER_COEFF_FAN255;
    #endif
    constexpr float _filament_heat_capacity_permm[] = FILAMENT_HEAT_CAPACITY_PERMM;

    static_assert(COUNT(_mpc_heater_power) == HOTENDS, "MPC_HEATER_POWER must have HOTENDS items.");
    static_assert(COUNT(_mpc_block_heat_capacity) == HOTENDS, "MPC_BLOCK_HEAT_CAPACITY must have HOTENDS items.");
    static_assert(COUNT(_mpc_sensor_responsiveness) == HOTENDS, "MPC_SENSOR_RESPONSIVENESS must have HOTENDS items.");
    static_assert(COUNT(_mpc_ambient_xfer_coeff) == HOTENDS, "MPC_AMBIENT_XFER_COEFF must have HOTENDS items.");
    #if ENABLED(MPC_INCLUDE_FAN)
      static_assert(COUNT(_mpc_ambient_xfer_coeff_fan255) == HOTENDS, "MPC_AMBIENT_XFER_COEFF_FAN255 must have HOTENDS items.");
    #endif
    static_assert(COUNT(_filament_heat_capacity_permm) == HOTENDS, "FILAMENT_HEAT_CAPACITY_PERMM must have HOTENDS items.");

    HOTEND_LOOP() {
      MPC_t &mpc = thermalManager.temp_hotend[e].mpc;
      mpc.heater_power = _mpc_heater_power[e];
      mpc.block_heat_capacity = _mpc_block_heat_capacity[e];
      mpc.sensor_responsiveness = _mpc_sensor_responsiveness[e];
      mpc.ambient_xfer_coeff_fan0 = _mpc_ambient_xfer_coeff[e];
      #if ENABLED(MPC_INCLUDE_FAN)
        mpc.fan255_adjustment = _mpc_ambient_xfer_coeff_fan255[e] - _mpc_ambient_xfer_coeff[e];
      #endif
      mpc.filament_heat_capacity_permm = _filament_heat_capacity_permm[e];
    }
  #endif

  //
  // Fixed-Time Motion
  //
  TERN_(FT_MOTION, ftMotion.set_defaults());

  //
  // Nonlinear Extrusion
  //
  TERN_(NONLINEAR_EXTRUSION, stepper.ne.reset());

  //
  // Input Shaping
  //
  #if HAS_ZV_SHAPING
    #if ENABLED(INPUT_SHAPING_X)
      stepper.set_shaping_frequency(X_AXIS, SHAPING_FREQ_X);
      stepper.set_shaping_damping_ratio(X_AXIS, SHAPING_ZETA_X);
    #endif
    #if ENABLED(INPUT_SHAPING_Y)
      stepper.set_shaping_frequency(Y_AXIS, SHAPING_FREQ_Y);
      stepper.set_shaping_damping_ratio(Y_AXIS, SHAPING_ZETA_Y);
    #endif
    #if ENABLED(INPUT_SHAPING_Z)
      stepper.set_shaping_frequency(Z_AXIS, SHAPING_FREQ_Z);
      stepper.set_shaping_damping_ratio(Z_AXIS, SHAPING_ZETA_Z);
    #endif
  #endif

  //
  // MMU Settings
  //
  #if HAS_PRUSA_MMU3
    spooljoin.enabled = false;
    MMU3::operation_statistics.reset_stats();
    mmu3.cutter_mode = 0;
    mmu3.stealth_mode = 0;
    mmu3.mmu_hw_enabled = true;
  #endif

  //
  // Hotend Idle Timeout
  //
  TERN_(HOTEND_IDLE_TIMEOUT, hotend_idle.cfg.set_defaults());

  postprocess();

  #if ANY(EEPROM_CHITCHAT, DEBUG_LEVELING_FEATURE)
    FSTR_P const hdsl = F("Hardcoded Default Settings Loaded");
    TERN_(HOST_EEPROM_CHITCHAT, hostui.notify(hdsl));
    DEBUG_ECHO_START(); DEBUG_ECHOLN(hdsl);
  #endif

  TERN_(EXTENSIBLE_UI, ExtUI::onFactoryReset());
}

#if DISABLED(DISABLE_M503)

  #define CONFIG_ECHO_START()       gcode.report_echo_start(forReplay)
  #define CONFIG_ECHO_MSG(V...)     do{ CONFIG_ECHO_START(); SERIAL_ECHOLNPGM(V); }while(0)
  #define CONFIG_ECHO_MSG_P(V...)   do{ CONFIG_ECHO_START(); SERIAL_ECHOLNPGM_P(V); }while(0)
  #define CONFIG_ECHO_HEADING(STR)  gcode.report_heading(forReplay, F(STR))

  #if ENABLED(EDITABLE_STEPS_PER_UNIT)
    void M92_report(const bool echo=true, const int8_t e=-1);
  #endif

  /**
   * M503 - Report current settings in RAM
   *
   * Unless specifically disabled, M503 is available even without EEPROM
   */
  void MarlinSettings::report(const bool forReplay) {
    //
    // Announce current units, in case inches are being displayed
    //
    CONFIG_ECHO_HEADING("Linear Units");
    CONFIG_ECHO_START();
    #if ENABLED(INCH_MODE_SUPPORT)
      SERIAL_ECHOPGM("  G2", AS_DIGIT(parser.linear_unit_factor == 1.0), " ;");
    #else
      SERIAL_ECHOPGM("  G21 ;");
    #endif
    gcode.say_units(); // " (in/mm)"

    //
    // M149 Temperature units
    //
    #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
      gcode.M149_report(forReplay);
    #else
      CONFIG_ECHO_HEADING(STR_TEMPERATURE_UNITS);
      CONFIG_ECHO_MSG("  M149 C ; Units in Celsius");
    #endif

    //
    // M200 Volumetric Extrusion
    //
    IF_DISABLED(NO_VOLUMETRICS, gcode.M200_report(forReplay));

    //
    // M92 Steps per Unit
    //
    TERN_(EDITABLE_STEPS_PER_UNIT, gcode.M92_report(forReplay));

    //
    // M203 Maximum feedrates (units/s)
    //
    gcode.M203_report(forReplay);

    //
    // M201 Maximum Acceleration (units/s2)
    //
    gcode.M201_report(forReplay);

    //
    // M204 Acceleration (units/s2)
    //
    gcode.M204_report(forReplay);

    //
    // M205 "Advanced" Settings
    //
    gcode.M205_report(forReplay);

    //
    // M206 Home Offset
    //
    TERN_(HAS_HOME_OFFSET, gcode.M206_report(forReplay));

    //
    // M218 Hotend offsets
    //
    TERN_(HAS_HOTEND_OFFSET, gcode.M218_report(forReplay));

    //
    // Bed Leveling
    //
    #if HAS_LEVELING

      gcode.M420_report(forReplay);

      #if ENABLED(MESH_BED_LEVELING)

        if (leveling_is_valid()) {
          for (uint8_t py = 0; py < GRID_MAX_POINTS_Y; ++py) {
            for (uint8_t px = 0; px < GRID_MAX_POINTS_X; ++px) {
              CONFIG_ECHO_START();
              SERIAL_ECHOLN(F("  G29 S3 I"), px, F(" J"), py, FPSTR(SP_Z_STR), p_float_t(LINEAR_UNIT(bedlevel.z_values[px][py]), 5));
            }
          }
          CONFIG_ECHO_START();
          SERIAL_ECHOLNPGM("  G29 S4 Z", p_float_t(LINEAR_UNIT(bedlevel.z_offset), 5));
        }

      #elif ENABLED(AUTO_BED_LEVELING_UBL)

        if (!forReplay) {
          SERIAL_EOL();
          bedlevel.report_state();
          SERIAL_ECHO_MSG("Active Mesh Slot ", bedlevel.storage_slot);
          SERIAL_ECHO_MSG("EEPROM can hold ", calc_num_meshes(), " meshes.\n");
        }

       //bedlevel.report_current_mesh();   // This is too verbose for large meshes. A better (more terse)
                                           // solution needs to be found.

      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

        if (leveling_is_valid()) {
          for (uint8_t py = 0; py < GRID_MAX_POINTS_Y; ++py) {
            for (uint8_t px = 0; px < GRID_MAX_POINTS_X; ++px) {
              CONFIG_ECHO_START();
              SERIAL_ECHOLN(F("  G29 W I"), px, F(" J"), py, FPSTR(SP_Z_STR), p_float_t(LINEAR_UNIT(bedlevel.z_values[px][py]), 5));
            }
          }
        }

      #endif

    #endif // HAS_LEVELING

    //
    // X Axis Twist Compensation
    //
    TERN_(X_AXIS_TWIST_COMPENSATION, gcode.M423_report(forReplay));

    //
    // Editable Servo Angles
    //
    TERN_(EDITABLE_SERVO_ANGLES, gcode.M281_report(forReplay));

    //
    // Kinematic Settings
    //
    TERN_(IS_KINEMATIC, gcode.M665_report(forReplay));

    //
    // M666 Endstops Adjustment
    //
    #if ANY(DELTA, HAS_EXTRA_ENDSTOPS)
      gcode.M666_report(forReplay);
    #endif

    //
    // Z Auto-Align
    //
    TERN_(Z_STEPPER_AUTO_ALIGN, gcode.M422_report(forReplay));

    //
    // LCD Preheat Settings
    //
    TERN_(HAS_PREHEAT, gcode.M145_report(forReplay));

    //
    // PID
    //
    TERN_(PIDTEMP,        gcode.M301_report(forReplay));
    TERN_(PIDTEMPBED,     gcode.M304_report(forReplay));
    TERN_(PIDTEMPCHAMBER, gcode.M309_report(forReplay));

    #if HAS_USER_THERMISTORS
      for (uint8_t i = 0; i < USER_THERMISTORS; ++i)
        thermalManager.M305_report(i, forReplay);
    #endif

    //
    // LCD Contrast
    //
    TERN_(HAS_LCD_CONTRAST, gcode.M250_report(forReplay));

    //
    // Display Sleep
    //
    TERN_(EDITABLE_DISPLAY_TIMEOUT, gcode.M255_report(forReplay));

    //
    // LCD Brightness
    //
    TERN_(HAS_LCD_BRIGHTNESS, gcode.M256_report(forReplay));

    //
    // Controller Fan
    //
    TERN_(CONTROLLER_FAN_EDITABLE, gcode.M710_report(forReplay));

    //
    // Power-Loss Recovery
    //
    TERN_(POWER_LOSS_RECOVERY, gcode.M413_report(forReplay));

    //
    // Firmware Retraction
    //
    #if ENABLED(FWRETRACT)
      gcode.M207_report(forReplay);
      gcode.M208_report(forReplay);
      TERN_(FWRETRACT_AUTORETRACT, gcode.M209_report(forReplay));
    #endif

    //
    // Homing Feedrate
    //
    TERN_(EDITABLE_HOMING_FEEDRATE, gcode.M210_report(forReplay));

    //
    // Probe Offset
    //
    TERN_(HAS_BED_PROBE, gcode.M851_report(forReplay));

    //
    // Bed Skew Correction
    //
    TERN_(SKEW_CORRECTION_GCODE, gcode.M852_report(forReplay));

    #if HAS_TRINAMIC_CONFIG
      //
      // TMC Stepper driver current
      //
      gcode.M906_report(forReplay);

      //
      // TMC Hybrid Threshold
      //
      TERN_(HYBRID_THRESHOLD, gcode.M913_report(forReplay));

      //
      // TMC Sensorless homing thresholds
      //
      TERN_(USE_SENSORLESS, gcode.M914_report(forReplay));
    #endif

    //
    // TMC stepping mode
    //
    TERN_(HAS_STEALTHCHOP, gcode.M569_report(forReplay));

    //
    // Fixed-Time Motion
    //
    TERN_(FT_MOTION, gcode.M493_report(forReplay));

    //
    // Nonlinear Extrusion
    //
    TERN_(NONLINEAR_EXTRUSION, gcode.M592_report(forReplay));

    //
    // Input Shaping
    //
    TERN_(HAS_ZV_SHAPING, gcode.M593_report(forReplay));

    //
    // Hotend Idle Timeout
    //
    TERN_(HOTEND_IDLE_TIMEOUT, gcode.M86_report(forReplay));

    //
    // Linear Advance
    //
    TERN_(LIN_ADVANCE, gcode.M900_report(forReplay));

    //
    // Motor Current (SPI or PWM)
    //
    #if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM
      gcode.M907_report(forReplay);
    #endif

    //
    // Advanced Pause filament load & unload lengths
    //
    TERN_(CONFIGURE_FILAMENT_CHANGE, gcode.M603_report(forReplay));

    //
    // Tool-changing Parameters
    //
    E_TERN_(gcode.M217_report(forReplay));

    //
    // Backlash Compensation
    //
    TERN_(BACKLASH_GCODE, gcode.M425_report(forReplay));

    //
    // Filament Runout Sensor
    //
    TERN_(HAS_FILAMENT_SENSOR, gcode.M412_report(forReplay));

    #if HAS_ETHERNET
      CONFIG_ECHO_HEADING("Ethernet");
      if (!forReplay) ETH0_report();
      CONFIG_ECHO_START(); SERIAL_ECHO_SP(2); MAC_report();
      CONFIG_ECHO_START(); SERIAL_ECHO_SP(2); gcode.M552_report();
      CONFIG_ECHO_START(); SERIAL_ECHO_SP(2); gcode.M553_report();
      CONFIG_ECHO_START(); SERIAL_ECHO_SP(2); gcode.M554_report();
    #endif

    TERN_(HAS_MULTI_LANGUAGE, gcode.M414_report(forReplay));

    //
    // Model predictive control
    //
    TERN_(MPCTEMP, gcode.M306_report(forReplay));

    //
    // MMU3
    //
    TERN_(HAS_PRUSA_MMU3, gcode.MMU3_report(forReplay));
  }

#endif // !DISABLE_M503

#pragma pack(pop)
