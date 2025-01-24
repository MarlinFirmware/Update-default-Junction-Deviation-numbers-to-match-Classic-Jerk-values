/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Changes.h
 *
 * Alert about Configuration changes at compile-time.
 */

/**
 * Warnings for old configurations
 */
#ifdef GITHUB_ACTION
  // Skip change alerts during CI Test
#elif WATCH_TEMP_PERIOD > 500
  #error "WATCH_TEMP_PERIOD now uses seconds instead of milliseconds."
#elif DISABLED(THERMAL_PROTECTION_HOTENDS) && (defined(WATCH_TEMP_PERIOD) || defined(THERMAL_PROTECTION_PERIOD))
  #error "Thermal Runaway Protection for hotends is now enabled with THERMAL_PROTECTION_HOTENDS."
#elif DISABLED(THERMAL_PROTECTION_BED) && defined(THERMAL_PROTECTION_BED_PERIOD)
  #error "Thermal Runaway Protection for the bed is now enabled with THERMAL_PROTECTION_BED."
#elif defined(NO_FAN_SLOWING_IN_PID_TUNING)
  #error "NO_FAN_SLOWING_IN_PID_TUNING is now TEMP_TUNING_MAINTAIN_FAN."
#elif (CORE_IS_XZ || CORE_IS_YZ) && ENABLED(Z_LATE_ENABLE)
  #error "Z_LATE_ENABLE can't be used with COREXZ, COREZX, COREYZ, or COREZY."
#elif defined(X_HOME_RETRACT_MM)
  #error "[XYZ]_HOME_RETRACT_MM settings have been renamed [XYZ]_HOME_BUMP_MM."
#elif defined(SDCARDDETECTINVERTED)
  #error "SDCARDDETECTINVERTED is now SD_DETECT_STATE (HIGH)."
#elif defined(SD_DETECT_INVERTED)
  #error "SD_DETECT_INVERTED is now SD_DETECT_STATE (HIGH)."
#elif defined(BTENABLED)
  #error "BTENABLED is now BLUETOOTH."
#elif defined(CUSTOM_MENDEL_NAME)
  #error "CUSTOM_MENDEL_NAME is now CUSTOM_MACHINE_NAME."
#elif defined(HAS_AUTOMATIC_VERSIONING)
  #error "HAS_AUTOMATIC_VERSIONING is now CUSTOM_VERSION_FILE."
#elif defined(USE_AUTOMATIC_VERSIONING)
  #error "USE_AUTOMATIC_VERSIONING is now CUSTOM_VERSION_FILE."
#elif defined(SDSLOW)
  #error "SDSLOW deprecated. Set SD_SPI_SPEED to SPI_HALF_SPEED instead."
#elif defined(SDEXTRASLOW)
  #error "SDEXTRASLOW deprecated. Set SD_SPI_SPEED to SPI_QUARTER_SPEED instead."
#elif defined(FILAMENT_SENSOR)
  #error "FILAMENT_SENSOR is now FILAMENT_WIDTH_SENSOR."
#elif defined(ENDSTOPPULLUP_FIL_RUNOUT)
  #error "ENDSTOPPULLUP_FIL_RUNOUT is now FIL_RUNOUT_PULLUP."
#elif defined(DISABLE_MAX_ENDSTOPS) || defined(DISABLE_MIN_ENDSTOPS)
  #error "DISABLE_MAX_ENDSTOPS and DISABLE_MIN_ENDSTOPS deprecated. Endstops are automatically determined."
#elif defined(LANGUAGE_INCLUDE)
  #error "LANGUAGE_INCLUDE has been replaced by LCD_LANGUAGE."
#elif defined(EXTRUDER_OFFSET_X) || defined(EXTRUDER_OFFSET_Y)
  #error "EXTRUDER_OFFSET_[XY] is deprecated. Use HOTEND_OFFSET_[XY] instead."
#elif defined(PID_PARAMS_PER_EXTRUDER)
  #error "PID_PARAMS_PER_EXTRUDER is deprecated. Use PID_PARAMS_PER_HOTEND instead."
#elif defined(EXTRUDER_WATTS) || defined(BED_WATTS)
  #error "EXTRUDER_WATTS and BED_WATTS are deprecated and should be removed."
#elif defined(SERVO_ENDSTOP_ANGLES)
  #error "SERVO_ENDSTOP_ANGLES is deprecated. Use Z_SERVO_ANGLES instead."
#elif defined(X_ENDSTOP_SERVO_NR) || defined(Y_ENDSTOP_SERVO_NR)
  #error "X_ENDSTOP_SERVO_NR and Y_ENDSTOP_SERVO_NR are deprecated and should be removed."
#elif defined(Z_ENDSTOP_SERVO_NR)
  #error "Z_ENDSTOP_SERVO_NR is now Z_PROBE_SERVO_NR."
#elif defined(DEFAULT_XYJERK)
  #error "DEFAULT_XYJERK is deprecated. Use DEFAULT_XJERK and DEFAULT_YJERK instead."
#elif defined(ALLOW_LOW_EJERK)
  #error "ALLOW_LOW_EJERK is deprecated and should be removed."
#elif defined(XY_TRAVEL_SPEED)
  #error "XY_TRAVEL_SPEED is now XY_PROBE_FEEDRATE."
#elif defined(XY_PROBE_SPEED)
  #error "XY_PROBE_SPEED is now XY_PROBE_FEEDRATE."
#elif defined(Z_PROBE_SPEED_FAST)
  #error "Z_PROBE_SPEED_FAST is now Z_PROBE_FEEDRATE_FAST."
#elif defined(Z_PROBE_SPEED_SLOW)
  #error "Z_PROBE_SPEED_SLOW is now Z_PROBE_FEEDRATE_SLOW."
#elif defined(PROBE_SERVO_DEACTIVATION_DELAY)
  #error "PROBE_SERVO_DEACTIVATION_DELAY is deprecated. Use DEACTIVATE_SERVOS_AFTER_MOVE instead."
#elif defined(SERVO_DEACTIVATION_DELAY)
  #error "SERVO_DEACTIVATION_DELAY is now SERVO_DELAY."
#elif ENABLED(FILAMENTCHANGEENABLE)
  #error "FILAMENTCHANGEENABLE is now ADVANCED_PAUSE_FEATURE."
#elif ENABLED(FILAMENT_CHANGE_FEATURE)
  #error "FILAMENT_CHANGE_FEATURE is now ADVANCED_PAUSE_FEATURE."
#elif defined(FILAMENT_CHANGE_X_POS) || defined(FILAMENT_CHANGE_Y_POS)
  #error "FILAMENT_CHANGE_[XY]_POS is now set with NOZZLE_PARK_POINT."
#elif defined(FILAMENT_CHANGE_Z_ADD)
  #error "FILAMENT_CHANGE_Z_ADD is now set with NOZZLE_PARK_POINT."
#elif defined(FILAMENT_CHANGE_XY_FEEDRATE)
  #error "FILAMENT_CHANGE_XY_FEEDRATE is now NOZZLE_PARK_XY_FEEDRATE."
#elif defined(FILAMENT_CHANGE_Z_FEEDRATE)
  #error "FILAMENT_CHANGE_Z_FEEDRATE is now NOZZLE_PARK_Z_FEEDRATE."
#elif defined(PAUSE_PARK_X_POS) || defined(PAUSE_PARK_Y_POS)
  #error "PAUSE_PARK_[XY]_POS is now set with NOZZLE_PARK_POINT."
#elif defined(PAUSE_PARK_Z_ADD)
  #error "PAUSE_PARK_Z_ADD is now set with NOZZLE_PARK_POINT."
#elif defined(PAUSE_PARK_XY_FEEDRATE)
  #error "PAUSE_PARK_XY_FEEDRATE is now NOZZLE_PARK_XY_FEEDRATE."
#elif defined(PAUSE_PARK_Z_FEEDRATE)
  #error "PAUSE_PARK_Z_FEEDRATE is now NOZZLE_PARK_Z_FEEDRATE."
#elif defined(FILAMENT_CHANGE_RETRACT_FEEDRATE)
  #error "FILAMENT_CHANGE_RETRACT_FEEDRATE is now PAUSE_PARK_RETRACT_FEEDRATE."
#elif defined(FILAMENT_CHANGE_RETRACT_LENGTH)
  #error "FILAMENT_CHANGE_RETRACT_LENGTH is now PAUSE_PARK_RETRACT_LENGTH."
#elif defined(FILAMENT_CHANGE_EXTRUDE_FEEDRATE)
  #error "FILAMENT_CHANGE_EXTRUDE_FEEDRATE is now ADVANCED_PAUSE_PURGE_FEEDRATE."
#elif defined(ADVANCED_PAUSE_EXTRUDE_FEEDRATE)
  #error "ADVANCED_PAUSE_EXTRUDE_FEEDRATE is now ADVANCED_PAUSE_PURGE_FEEDRATE."
#elif defined(FILAMENT_CHANGE_EXTRUDE_LENGTH)
  #error "FILAMENT_CHANGE_EXTRUDE_LENGTH is now ADVANCED_PAUSE_PURGE_LENGTH."
#elif defined(ADVANCED_PAUSE_EXTRUDE_LENGTH)
  #error "ADVANCED_PAUSE_EXTRUDE_LENGTH is now ADVANCED_PAUSE_PURGE_LENGTH."
#elif defined(FILAMENT_CHANGE_NOZZLE_TIMEOUT)
  #error "FILAMENT_CHANGE_NOZZLE_TIMEOUT is now PAUSE_PARK_NOZZLE_TIMEOUT."
#elif defined(FILAMENT_CHANGE_NUMBER_OF_ALERT_BEEPS)
  #error "FILAMENT_CHANGE_NUMBER_OF_ALERT_BEEPS is now FILAMENT_CHANGE_ALERT_BEEPS."
#elif defined(FILAMENT_CHANGE_NO_STEPPER_TIMEOUT)
  #error "FILAMENT_CHANGE_NO_STEPPER_TIMEOUT is now PAUSE_PARK_NO_STEPPER_TIMEOUT."
#elif defined(PLA_PREHEAT_HOTEND_TEMP)
  #error "PLA_PREHEAT_HOTEND_TEMP is now PREHEAT_1_TEMP_HOTEND."
#elif defined(PLA_PREHEAT_HPB_TEMP)
  #error "PLA_PREHEAT_HPB_TEMP is now PREHEAT_1_TEMP_BED."
#elif defined(PLA_PREHEAT_FAN_SPEED)
  #error "PLA_PREHEAT_FAN_SPEED is now PREHEAT_1_FAN_SPEED."
#elif defined(ABS_PREHEAT_HOTEND_TEMP)
  #error "ABS_PREHEAT_HOTEND_TEMP is now PREHEAT_2_TEMP_HOTEND."
#elif defined(ABS_PREHEAT_HPB_TEMP)
  #error "ABS_PREHEAT_HPB_TEMP is now PREHEAT_2_TEMP_BED."
#elif defined(ABS_PREHEAT_FAN_SPEED)
  #error "ABS_PREHEAT_FAN_SPEED is now PREHEAT_2_FAN_SPEED."
#elif defined(ENDSTOPS_ONLY_FOR_HOMING)
  #error "ENDSTOPS_ONLY_FOR_HOMING is deprecated. Use (disable) ENDSTOPS_ALWAYS_ON_DEFAULT instead."
#elif defined(HOMING_FEEDRATE)
  #error "HOMING_FEEDRATE is now set using the HOMING_FEEDRATE_MM_M array instead."
#elif (defined(HOMING_FEEDRATE_XY) || defined(HOMING_FEEDRATE_Z)) && !defined(HOMING_FEEDRATE_MM_M)
  #error "HOMING_FEEDRATE_XY and HOMING_FEEDRATE_Z are now set using the HOMING_FEEDRATE_MM_M array instead."
#elif defined(MANUAL_HOME_POSITIONS)
  #error "MANUAL_HOME_POSITIONS is deprecated. Set MANUAL_[XYZ]_HOME_POS as-needed instead."
#elif defined(PID_ADD_EXTRUSION_RATE)
  #error "PID_ADD_EXTRUSION_RATE is now PID_EXTRUSION_SCALING and is DISABLED by default."
#elif defined(Z_RAISE_BEFORE_HOMING)
  #error "Z_RAISE_BEFORE_HOMING is now Z_CLEARANCE_FOR_HOMING."
#elif defined(MIN_Z_HEIGHT_FOR_HOMING)
  #error "MIN_Z_HEIGHT_FOR_HOMING is now Z_CLEARANCE_FOR_HOMING."
#elif defined(Z_HOMING_HEIGHT)
  #error "Z_HOMING_HEIGHT is now Z_CLEARANCE_FOR_HOMING."
#elif defined(Z_RAISE_BEFORE_PROBING) || defined(Z_RAISE_AFTER_PROBING)
  #error "Z_RAISE_(BEFORE|AFTER)_PROBING are deprecated. Use Z_CLEARANCE_DEPLOY_PROBE and Z_AFTER_PROBING instead."
#elif defined(Z_RAISE_PROBE_DEPLOY_STOW) || defined(Z_RAISE_BETWEEN_PROBINGS)
  #error "Z_RAISE_PROBE_DEPLOY_STOW and Z_RAISE_BETWEEN_PROBINGS are now Z_CLEARANCE_DEPLOY_PROBE and Z_CLEARANCE_BETWEEN_PROBES."
#elif defined(Z_PROBE_DEPLOY_HEIGHT) || defined(Z_PROBE_TRAVEL_HEIGHT)
  #error "Z_PROBE_DEPLOY_HEIGHT and Z_PROBE_TRAVEL_HEIGHT are now Z_CLEARANCE_DEPLOY_PROBE and Z_CLEARANCE_BETWEEN_PROBES."
#elif defined(MANUAL_BED_LEVELING)
  #error "MANUAL_BED_LEVELING is now LCD_BED_LEVELING."
#elif defined(MESH_HOME_SEARCH_Z)
  #error "MESH_HOME_SEARCH_Z is now LCD_PROBE_Z_RANGE."
#elif defined(MANUAL_PROBE_Z_RANGE)
  #error "MANUAL_PROBE_Z_RANGE is now LCD_PROBE_Z_RANGE."
#elif !defined(MIN_STEPS_PER_SEGMENT)
  #error "Please replace 'const int dropsegments' with '#define MIN_STEPS_PER_SEGMENT' (and increase by 1)."
#elif MIN_STEPS_PER_SEGMENT <= 0
  #error "MIN_STEPS_PER_SEGMENT must be at least 1."
#elif defined(PREVENT_DANGEROUS_EXTRUDE)
  #error "PREVENT_DANGEROUS_EXTRUDE is now PREVENT_COLD_EXTRUSION."
#elif defined(SCARA)
  #error "SCARA is now MORGAN_SCARA."
#elif defined(ENABLE_AUTO_BED_LEVELING)
  #error "ENABLE_AUTO_BED_LEVELING is deprecated. Specify AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_BILINEAR, or AUTO_BED_LEVELING_3POINT."
#elif defined(AUTO_BED_LEVELING_FEATURE)
  #error "AUTO_BED_LEVELING_FEATURE is deprecated. Specify AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_BILINEAR, or AUTO_BED_LEVELING_3POINT."
#elif defined(ABL_GRID_POINTS)
  #error "ABL_GRID_POINTS is now GRID_MAX_POINTS_X and GRID_MAX_POINTS_Y."
#elif defined(ABL_GRID_POINTS_X) || defined(ABL_GRID_POINTS_Y)
  #error "ABL_GRID_POINTS_[XY] is now GRID_MAX_POINTS_[XY]."
#elif defined(ABL_GRID_MAX_POINTS_X) || defined(ABL_GRID_MAX_POINTS_Y)
  #error "ABL_GRID_MAX_POINTS_[XY] is now GRID_MAX_POINTS_[XY]."
#elif defined(MESH_NUM_X_POINTS) || defined(MESH_NUM_Y_POINTS)
  #error "MESH_NUM_[XY]_POINTS is now GRID_MAX_POINTS_[XY]."
#elif defined(UBL_MESH_NUM_X_POINTS) || defined(UBL_MESH_NUM_Y_POINTS)
  #error "UBL_MESH_NUM_[XY]_POINTS is now GRID_MAX_POINTS_[XY]."
#elif defined(UBL_G26_MESH_VALIDATION)
  #error "UBL_G26_MESH_VALIDATION is now G26_MESH_VALIDATION."
#elif defined(UBL_MESH_EDIT_ENABLED)
  #error "UBL_MESH_EDIT_ENABLED is now G26_MESH_VALIDATION."
#elif defined(UBL_MESH_EDITING)
  #error "UBL_MESH_EDITING is now G26_MESH_VALIDATION."
#elif defined(BLTOUCH_HEATERS_OFF)
  #error "BLTOUCH_HEATERS_OFF is now PROBING_HEATERS_OFF."
#elif defined(BLTOUCH_V3)
  #error "BLTOUCH_V3 is obsolete."
#elif defined(BLTOUCH_FORCE_OPEN_DRAIN_MODE)
  #error "BLTOUCH_FORCE_OPEN_DRAIN_MODE is obsolete."
#elif defined(BEEPER)
  #error "BEEPER is now BEEPER_PIN."
#elif defined(SDCARDDETECT)
  #error "SDCARDDETECT is now SD_DETECT_PIN."
#elif defined(STAT_LED_RED) || defined(STAT_LED_BLUE)
  #error "STAT_LED_RED/STAT_LED_BLUE are now STAT_LED_RED_PIN/STAT_LED_BLUE_PIN."
#elif defined(LCD_PIN_BL)
  #error "LCD_PIN_BL is now LCD_BACKLIGHT_PIN."
#elif defined(LCD_PIN_RESET)
  #error "LCD_PIN_RESET is now LCD_RESET_PIN."
#elif defined(EXTRUDER_0_AUTO_FAN_PIN) || defined(EXTRUDER_1_AUTO_FAN_PIN) || defined(EXTRUDER_2_AUTO_FAN_PIN) || defined(EXTRUDER_3_AUTO_FAN_PIN)
  #error "EXTRUDER_[0123]_AUTO_FAN_PIN is now E[0123]_AUTO_FAN_PIN."
#elif defined(min_software_endstops) || defined(max_software_endstops)
  #error "(min|max)_software_endstops are now (MIN|MAX)_SOFTWARE_ENDSTOPS."
#elif ENABLED(Z_PROBE_SLED) && defined(SLED_PIN)
  #error "Replace SLED_PIN with SOL1_PIN (applies to both Z_PROBE_SLED and SOLENOID_PROBE)."
#elif defined(CONTROLLERFAN_PIN)
  #error "CONTROLLERFAN_PIN is now CONTROLLER_FAN_PIN, enabled with USE_CONTROLLER_FAN."
#elif defined(CONTROLLERFAN_SPEED)
  #error "CONTROLLERFAN_SPEED is now CONTROLLERFAN_SPEED_ACTIVE."
#elif defined(CONTROLLERFAN_SECS)
  #error "CONTROLLERFAN_SECS is now CONTROLLERFAN_IDLE_TIME."
#elif defined(MIN_RETRACT)
  #error "MIN_RETRACT is now MIN_AUTORETRACT and MAX_AUTORETRACT."
#elif defined(ADVANCE)
  #error "ADVANCE is now LIN_ADVANCE."
#elif defined(LIN_ADVANCE_E_D_RATIO)
  #error "LIN_ADVANCE (1.5) no longer uses LIN_ADVANCE_E_D_RATIO."
#elif defined(NEOPIXEL_RGBW_LED)
  #error "NEOPIXEL_RGBW_LED is now NEOPIXEL_LED."
#elif ENABLED(DELTA) && defined(DELTA_PROBEABLE_RADIUS)
  #error "Remove DELTA_PROBEABLE_RADIUS and use PROBING_MARGIN to inset the probe area instead."
#elif ENABLED(DELTA) && defined(DELTA_CALIBRATION_RADIUS)
  #error "Remove DELTA_CALIBRATION_RADIUS and use PROBING_MARGIN to inset the probe area instead."
#elif defined(UBL_MESH_INSET)
  #error "UBL_MESH_INSET is now just MESH_INSET."
#elif defined(UBL_MESH_MIN_X) || defined(UBL_MESH_MIN_Y) || defined(UBL_MESH_MAX_X) || defined(UBL_MESH_MAX_Y)
  #error "UBL_MESH_(MIN|MAX)_[XY] is now just MESH_(MIN|MAX)_[XY]."
#elif defined(ABL_PROBE_PT_1_X) || defined(ABL_PROBE_PT_1_Y) || defined(ABL_PROBE_PT_2_X) || defined(ABL_PROBE_PT_2_Y) || defined(ABL_PROBE_PT_3_X) || defined(ABL_PROBE_PT_3_Y)
  #error "ABL_PROBE_PT_[123]_[XY] is no longer required. Please remove it."
#elif defined(UBL_PROBE_PT_1_X) || defined(UBL_PROBE_PT_1_Y) || defined(UBL_PROBE_PT_2_X) || defined(UBL_PROBE_PT_2_Y) || defined(UBL_PROBE_PT_3_X) || defined(UBL_PROBE_PT_3_Y)
  #error "UBL_PROBE_PT_[123]_[XY] is no longer required. Please remove it."
#elif defined(MIN_PROBE_EDGE)
  #error "MIN_PROBE_EDGE is now called PROBING_MARGIN."
#elif defined(MIN_PROBE_EDGE_LEFT)
  #error "MIN_PROBE_EDGE_LEFT is now called PROBING_MARGIN_LEFT."
#elif defined(MIN_PROBE_EDGE_RIGHT)
  #error "MIN_PROBE_EDGE_RIGHT is now called PROBING_MARGIN_RIGHT."
#elif defined(MIN_PROBE_EDGE_FRONT)
  #error "MIN_PROBE_EDGE_FRONT is now called PROBING_MARGIN_FRONT."
#elif defined(MIN_PROBE_EDGE_BACK)
  #error "MIN_PROBE_EDGE_BACK is now called PROBING_MARGIN_BACK."
#elif defined(LEFT_PROBE_BED_POSITION)
  #error "LEFT_PROBE_BED_POSITION is obsolete. Set a margin with PROBING_MARGIN or PROBING_MARGIN_LEFT instead."
#elif defined(RIGHT_PROBE_BED_POSITION)
  #error "RIGHT_PROBE_BED_POSITION is obsolete. Set a margin with PROBING_MARGIN or PROBING_MARGIN_RIGHT instead."
#elif defined(FRONT_PROBE_BED_POSITION)
  #error "FRONT_PROBE_BED_POSITION is obsolete. Set a margin with PROBING_MARGIN or PROBING_MARGIN_FRONT instead."
#elif defined(BACK_PROBE_BED_POSITION)
  #error "BACK_PROBE_BED_POSITION is obsolete. Set a margin with PROBING_MARGIN or PROBING_MARGIN_BACK instead."
#elif defined(ENABLE_MESH_EDIT_GFX_OVERLAY)
  #error "ENABLE_MESH_EDIT_GFX_OVERLAY is now MESH_EDIT_GFX_OVERLAY."
#elif defined(BABYSTEP_ZPROBE_GFX_REVERSE)
  #error "BABYSTEP_ZPROBE_GFX_REVERSE is now set by OVERLAY_GFX_REVERSE."
#elif defined(UBL_GRANULAR_SEGMENTATION_FOR_CARTESIAN)
  #error "UBL_GRANULAR_SEGMENTATION_FOR_CARTESIAN is now SEGMENT_LEVELED_MOVES."
#elif HAS_PID_HEATING && (defined(K1) || !defined(PID_K1))
  #error "K1 is now PID_K1."
#elif defined(PROBE_DOUBLE_TOUCH)
  #error "PROBE_DOUBLE_TOUCH is now MULTIPLE_PROBING."
#elif defined(ANET_KEYPAD_LCD)
  #error "ANET_KEYPAD_LCD is now ZONESTAR_LCD."
#elif defined(LCD_I2C_SAINSMART_YWROBOT)
  #error "LCD_I2C_SAINSMART_YWROBOT is now LCD_SAINSMART_I2C_(1602|2004)."
#elif defined(MEASURED_LOWER_LIMIT) || defined(MEASURED_UPPER_LIMIT)
  #error "MEASURED_(UPPER|LOWER)_LIMIT is now FILWIDTH_ERROR_MARGIN."
#elif defined(HAVE_TMCDRIVER)
  #error "HAVE_TMCDRIVER is obsolete."
#elif defined(STEALTHCHOP)
  #error "STEALTHCHOP is now STEALTHCHOP_(XY|Z|E)."
#elif defined(HAVE_TMC26X)
  #error "HAVE_TMC26X is now obsolete."
#elif defined(HAVE_TMC2130)
  #error "HAVE_TMC2130 is now [AXIS]_DRIVER_TYPE TMC2130."
#elif defined(HAVE_TMC2208)
  #error "HAVE_TMC2208 is now [AXIS]_DRIVER_TYPE TMC2208."
#elif defined(HAVE_L6470DRIVER)
  #error "HAVE_L6470DRIVER is obsolete. L64xx stepper drivers are no longer supported in Marlin."
#elif defined(X_IS_TMC) || defined(X2_IS_TMC) || defined(Y_IS_TMC) || defined(Y2_IS_TMC) || defined(Z_IS_TMC) || defined(Z2_IS_TMC) || defined(Z3_IS_TMC) \
   || defined(E0_IS_TMC) || defined(E1_IS_TMC) || defined(E2_IS_TMC) || defined(E3_IS_TMC) || defined(E4_IS_TMC) || defined(E5_IS_TMC) || defined(E6_IS_TMC) || defined(E7_IS_TMC)
  #error "[AXIS]_IS_TMC is now obsolete."
#elif defined(X_IS_TMC26X) || defined(X2_IS_TMC26X) || defined(Y_IS_TMC26X) || defined(Y2_IS_TMC26X) || defined(Z_IS_TMC26X) || defined(Z2_IS_TMC26X) || defined(Z3_IS_TMC26X) \
   || defined(E0_IS_TMC26X) || defined(E1_IS_TMC26X) || defined(E2_IS_TMC26X) || defined(E3_IS_TMC26X) || defined(E4_IS_TMC26X) || defined(E5_IS_TMC26X) || defined(E6_IS_TMC26X) || defined(E7_IS_TMC26X)
  #error "[AXIS]_IS_TMC26X is now obsolete."
#elif defined(X_IS_TMC2130) || defined(X2_IS_TMC2130) || defined(Y_IS_TMC2130) || defined(Y2_IS_TMC2130) || defined(Z_IS_TMC2130) || defined(Z2_IS_TMC2130) || defined(Z3_IS_TMC2130) \
   || defined(E0_IS_TMC2130) || defined(E1_IS_TMC2130) || defined(E2_IS_TMC2130) || defined(E3_IS_TMC2130) || defined(E4_IS_TMC2130) || defined(E5_IS_TMC2130) || defined(E6_IS_TMC2130) || defined(E7_IS_TMC2130)
  #error "[AXIS]_IS_TMC2130 is now [AXIS]_DRIVER_TYPE TMC2130."
#elif defined(X_IS_TMC2208) || defined(X2_IS_TMC2208) || defined(Y_IS_TMC2208) || defined(Y2_IS_TMC2208) || defined(Z_IS_TMC2208) || defined(Z2_IS_TMC2208) || defined(Z3_IS_TMC2208) \
   || defined(E0_IS_TMC2208) || defined(E1_IS_TMC2208) || defined(E2_IS_TMC2208) || defined(E3_IS_TMC2208) || defined(E4_IS_TMC2208) || defined(E5_IS_TMC2208) || defined(E6_IS_TMC2208) || defined(E7_IS_TMC2208)
  #error "[AXIS]_IS_TMC2208 is now [AXIS]_DRIVER_TYPE TMC2208."
#elif defined(AUTOMATIC_CURRENT_CONTROL)
  #error "AUTOMATIC_CURRENT_CONTROL is now MONITOR_DRIVER_STATUS."
#elif defined(FILAMENT_CHANGE_LOAD_LENGTH)
  #error "FILAMENT_CHANGE_LOAD_LENGTH is now FILAMENT_CHANGE_FAST_LOAD_LENGTH."
#elif defined(LEVEL_CORNERS_INSET)
  #error "LEVEL_CORNERS_INSET is now BED_TRAMMING_INSET_LFRB."
#elif defined(BEZIER_JERK_CONTROL)
  #error "BEZIER_JERK_CONTROL is now S_CURVE_ACCELERATION."
#elif HAS_JUNCTION_DEVIATION && defined(JUNCTION_DEVIATION_FACTOR)
  #error "JUNCTION_DEVIATION_FACTOR is now JUNCTION_DEVIATION_MM."
#elif defined(JUNCTION_ACCELERATION_FACTOR)
  #error "JUNCTION_ACCELERATION_FACTOR is obsolete. Delete it from Configuration_adv.h."
#elif defined(JUNCTION_ACCELERATION)
  #error "JUNCTION_ACCELERATION is obsolete. Delete it from Configuration_adv.h."
#elif defined(MAX7219_DEBUG_STEPPER_HEAD)
  #error "MAX7219_DEBUG_STEPPER_HEAD is now MAX7219_DEBUG_PLANNER_HEAD."
#elif defined(MAX7219_DEBUG_STEPPER_TAIL)
  #error "MAX7219_DEBUG_STEPPER_TAIL is now MAX7219_DEBUG_PLANNER_TAIL."
#elif defined(MAX7219_DEBUG_STEPPER_QUEUE)
  #error "MAX7219_DEBUG_STEPPER_QUEUE is now MAX7219_DEBUG_PLANNER_QUEUE."
#elif defined(ENDSTOP_NOISE_FILTER)
  #error "ENDSTOP_NOISE_FILTER is now ENDSTOP_NOISE_THRESHOLD [2-7]."
#elif defined(RETRACT_ZLIFT)
  #error "RETRACT_ZLIFT is now RETRACT_ZRAISE."
#elif defined(TOOLCHANGE_FS_INIT_BEFORE_SWAP)
  #error "TOOLCHANGE_FS_INIT_BEFORE_SWAP is now TOOLCHANGE_FS_SLOW_FIRST_PRIME."
#elif defined(TOOLCHANGE_PARK_ZLIFT) || defined(TOOLCHANGE_UNPARK_ZLIFT)
  #error "TOOLCHANGE_PARK_ZLIFT and TOOLCHANGE_UNPARK_ZLIFT are now TOOLCHANGE_ZRAISE."
#elif defined(SINGLENOZZLE_TOOLCHANGE_ZRAISE)
  #error "SINGLENOZZLE_TOOLCHANGE_ZRAISE is now TOOLCHANGE_ZRAISE."
#elif defined(SINGLENOZZLE_SWAP_LENGTH)
  #error "SINGLENOZZLE_SWAP_LENGTH is now TOOLCHANGE_FIL_SWAP_LENGTH."
#elif defined(SINGLENOZZLE_SWAP_RETRACT_SPEED)
  #error "SINGLENOZZLE_SWAP_RETRACT_SPEED is now TOOLCHANGE_FIL_SWAP_RETRACT_SPEED."
#elif defined(SINGLENOZZLE_SWAP_PRIME_SPEED)
  #error "SINGLENOZZLE_SWAP_PRIME_SPEED is now TOOLCHANGE_FIL_SWAP_PRIME_SPEED."
#elif defined(SINGLENOZZLE_SWAP_PARK)
  #error "SINGLENOZZLE_SWAP_PARK is now TOOLCHANGE_PARK."
#elif defined(SINGLENOZZLE_TOOLCHANGE_XY)
  #error "SINGLENOZZLE_TOOLCHANGE_XY is now TOOLCHANGE_PARK_XY."
#elif defined(SINGLENOZZLE_PARK_XY_FEEDRATE)
  #error "SINGLENOZZLE_PARK_XY_FEEDRATE is now TOOLCHANGE_PARK_XY_FEEDRATE."
#elif defined(PARKING_EXTRUDER_SECURITY_RAISE)
  #error "PARKING_EXTRUDER_SECURITY_RAISE is now TOOLCHANGE_ZRAISE."
#elif defined(SWITCHING_TOOLHEAD_SECURITY_RAISE)
  #error "SWITCHING_TOOLHEAD_SECURITY_RAISE is now TOOLCHANGE_ZRAISE."
#elif defined(G0_FEEDRATE) && G0_FEEDRATE == 0
  #error "G0_FEEDRATE is now used to set the G0 feedrate."
#elif defined(MBL_Z_STEP)
  #error "MBL_Z_STEP is now MESH_EDIT_Z_STEP."
#elif defined(CHDK)
  #error "CHDK is now CHDK_PIN."
#elif ANY_PIN( \
        MAX6675_SS, MAX6675_SS2, MAX6675_SS3, MAX6675_CS, MAX6675_CS2, MAX6675_CS3,\
        MAX31855_SS, MAX31855_SS2, MAX31855_SS3, MAX31855_CS, MAX31855_CS2, MAX31855_CS3, \
        MAX31865_SS, MAX31865_SS2, MAX31865_SS3, MAX31865_CS, MAX31865_CS2, MAX31865_CS3)
  #warning "MAX*_SS_PIN, MAX*_SS2_PIN, MAX*_SS3_PIN, MAX*_CS_PIN, MAX*_CS2_PIN, and MAX*_CS3_PIN, are obsolete. Please use TEMP_0_CS_PIN/TEMP_1_CS_PIN/TEMP_2_CS_PIN instead."
#elif ANY_PIN(MAX6675_SCK, MAX31855_SCK, MAX31865_SCK)
  #warning "MAX*_SCK_PIN is obsolete. Please use TEMP_0_SCK_PIN/TEMP_1_SCK_PIN/TEMP_2_SCK_PIN instead."
#elif ANY_PIN(MAX6675_MISO, MAX6675_DO, MAX31855_MISO, MAX31855_DO, MAX31865_MISO, MAX31865_DO)
  #warning "MAX*_MISO_PIN and MAX*_DO_PIN are obsolete. Please use TEMP_0_MISO_PIN/TEMP_1_MISO_PIN/TEMP_2_MISO_PIN instead."
#elif PIN_EXISTS(MAX31865_MOSI)
  #warning "MAX31865_MOSI_PIN is obsolete. Please use TEMP_0_MOSI_PIN/TEMP_1_MOSI_PIN/TEMP_2_MOSI_PIN instead."
#elif ANY_PIN(THERMO_CS1_PIN, THERMO_CS2_PIN, THERMO_CS3_PIN, THERMO_DO_PIN, THERMO_SCK_PIN)
  #error "THERMO_*_PIN is now TEMP_n_CS_PIN, TEMP_n_SCK_PIN, TEMP_n_MOSI_PIN, TEMP_n_MISO_PIN."
#elif defined(MAX31865_SENSOR_OHMS)
  #error "MAX31865_SENSOR_OHMS is now MAX31865_SENSOR_OHMS_0."
#elif defined(MAX31865_CALIBRATION_OHMS)
  #error "MAX31865_CALIBRATION_OHMS is now MAX31865_CALIBRATION_OHMS_0."
#elif defined(SPINDLE_LASER_ENABLE)
  #error "SPINDLE_LASER_ENABLE is now SPINDLE_FEATURE or LASER_FEATURE."
#elif defined(SPINDLE_LASER_ENABLE_PIN)
  #error "SPINDLE_LASER_ENABLE_PIN is now SPINDLE_LASER_ENA_PIN."
#elif defined(SPINDLE_DIR_CHANGE)
  #error "SPINDLE_DIR_CHANGE is now SPINDLE_CHANGE_DIR."
#elif defined(SPINDLE_STOP_ON_DIR_CHANGE)
  #error "SPINDLE_STOP_ON_DIR_CHANGE is now SPINDLE_CHANGE_DIR_STOP."
#elif defined(SPINDLE_LASER_ACTIVE_HIGH)
  #error "SPINDLE_LASER_ACTIVE_HIGH is now SPINDLE_LASER_ACTIVE_STATE."
#elif defined(SPINDLE_LASER_ENABLE_INVERT)
  #error "SPINDLE_LASER_ENABLE_INVERT is now SPINDLE_LASER_ACTIVE_STATE."
#elif defined(LASER_POWER_INLINE)
  #error "LASER_POWER_INLINE is not required, inline mode is enabled with 'M3 I' and disabled with 'M5 I'."
#elif defined(LASER_POWER_INLINE_TRAPEZOID)
  #error "LASER_POWER_INLINE_TRAPEZOID is now LASER_POWER_TRAP."
#elif defined(LASER_POWER_INLINE_TRAPEZOID_CONT)
  #error "LASER_POWER_INLINE_TRAPEZOID_CONT is replaced with LASER_POWER_TRAP."
#elif defined(LASER_POWER_INLINE_TRAPEZOID_PER)
  #error "LASER_POWER_INLINE_TRAPEZOID_CONT_PER  replaced with LASER_POWER_TRAP."
#elif defined(LASER_POWER_INLINE_CONTINUOUS)
  #error "LASER_POWER_INLINE_CONTINUOUS is not required, inline mode is enabled with 'M3 I' and disabled with 'M5 I'."
#elif defined(CUTTER_POWER_DISPLAY)
  #error "CUTTER_POWER_DISPLAY is now CUTTER_POWER_UNIT."
#elif defined(CHAMBER_HEATER_PIN)
  #error "CHAMBER_HEATER_PIN is now HEATER_CHAMBER_PIN."
#elif defined(TMC_Z_CALIBRATION)
  #error "TMC_Z_CALIBRATION has been deprecated in favor of MECHANICAL_GANTRY_CALIBRATION."
#elif defined(Z_MIN_PROBE_ENDSTOP)
  #error "Z_MIN_PROBE_ENDSTOP is no longer required. Please remove it."
#elif defined(DUAL_NOZZLE_DUPLICATION_MODE)
  #error "DUAL_NOZZLE_DUPLICATION_MODE is now MULTI_NOZZLE_DUPLICATION."
#elif defined(MENU_ITEM_CASE_LIGHT)
  #error "MENU_ITEM_CASE_LIGHT is now CASE_LIGHT_MENU."
#elif defined(CASE_LIGHT_NEOPIXEL_COLOR)
  #error "CASE_LIGHT_NEOPIXEL_COLOR is now CASE_LIGHT_DEFAULT_COLOR."
#elif defined(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  #error "ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED is now SD_ABORT_ON_ENDSTOP_HIT."
#elif defined(LPC_SD_LCD) || defined(LPC_SD_ONBOARD) || defined(LPC_SD_CUSTOM_CABLE)
  #error "LPC_SD_(LCD|ONBOARD|CUSTOM_CABLE) are now SDCARD_CONNECTION."
#elif defined(USB_SD_DISABLED)
  #error "USB_SD_DISABLED is now NO_SD_HOST_DRIVE."
#elif defined(USB_SD_ONBOARD)
  #error "USB_SD_ONBOARD is obsolete. Disable NO_SD_HOST_DRIVE instead."
#elif defined(PSU_ACTIVE_HIGH)
  #error "PSU_ACTIVE_HIGH is now PSU_ACTIVE_STATE."
#elif POWER_SUPPLY == 1
  #error "Replace POWER_SUPPLY 1 by enabling PSU_CONTROL and setting PSU_ACTIVE_STATE to 'LOW'."
#elif POWER_SUPPLY == 2
  #error "Replace POWER_SUPPLY 2 by enabling PSU_CONTROL and setting PSU_ACTIVE_STATE to 'HIGH'."
#elif defined(POWER_SUPPLY)
  #error "POWER_SUPPLY is now obsolete. Please remove it."
#elif defined(MKS_ROBIN_TFT)
  #error "MKS_ROBIN_TFT is now FSMC_GRAPHICAL_TFT."
#elif defined(SDPOWER)
  #error "SDPOWER is now SDPOWER_PIN."
#elif defined(STRING_SPLASH_LINE1) || defined(STRING_SPLASH_LINE2)
  #error "STRING_SPLASH_LINE[12] are now obsolete. Please remove them."
#elif defined(Z_PROBE_ALLEN_KEY_DEPLOY_1_X) || defined(Z_PROBE_ALLEN_KEY_STOW_1_X)
  #error "Z_PROBE_ALLEN_KEY_(DEPLOY|STOW) coordinates are now a single setting."
#elif defined(X_PROBE_OFFSET_FROM_EXTRUDER) || defined(Y_PROBE_OFFSET_FROM_EXTRUDER) || defined(Z_PROBE_OFFSET_FROM_EXTRUDER)
  #error "[XYZ]_PROBE_OFFSET_FROM_EXTRUDER is now NOZZLE_TO_PROBE_OFFSET."
#elif defined(MIN_PROBE_X) || defined(MIN_PROBE_Y) || defined(MAX_PROBE_X) || defined(MAX_PROBE_Y)
  #error "(MIN|MAX)_PROBE_[XY] are now calculated at runtime. Please remove them."
#elif defined(Z_STEPPER_ALIGN_X) || defined(Z_STEPPER_ALIGN_X)
  #error "Z_STEPPER_ALIGN_X and Z_STEPPER_ALIGN_Y are now combined as Z_STEPPER_ALIGN_XY."
#elif defined(JUNCTION_DEVIATION)
  #error "JUNCTION_DEVIATION is no longer required. (See CLASSIC_JERK). Please remove it."
#elif defined(BABYSTEP_MULTIPLICATOR)
  #error "BABYSTEP_MULTIPLICATOR is now BABYSTEP_MULTIPLICATOR_[XY|Z]."
#elif defined(LULZBOT_TOUCH_UI)
  #error "LULZBOT_TOUCH_UI is now TOUCH_UI_FTDI_EVE."
#elif defined(PS_DEFAULT_OFF)
  #error "PS_DEFAULT_OFF is now PSU_DEFAULT_OFF."
#elif defined(FILAMENT_UNLOAD_RETRACT_LENGTH)
  #error "FILAMENT_UNLOAD_RETRACT_LENGTH is now FILAMENT_UNLOAD_PURGE_RETRACT."
#elif defined(FILAMENT_UNLOAD_DELAY)
  #error "FILAMENT_UNLOAD_DELAY is now FILAMENT_UNLOAD_PURGE_DELAY."
#elif defined(HOME_USING_SPREADCYCLE)
  #error "HOME_USING_SPREADCYCLE is now obsolete. Please remove it."
#elif defined(DGUS_LCD)
  #error "DGUS_LCD is now DGUS_LCD_UI ORIGIN|FYSETC|HIPRECY)."
#elif defined(DGUS_SERIAL_PORT)
  #error "DGUS_SERIAL_PORT is now LCD_SERIAL_PORT."
#elif defined(DGUS_BAUDRATE)
  #error "DGUS_BAUDRATE is now LCD_BAUDRATE."
#elif defined(DGUS_STATS_RX_BUFFER_OVERRUNS)
  #error "DGUS_STATS_RX_BUFFER_OVERRUNS is now STATS_RX_BUFFER_OVERRUNS."
#elif defined(ANYCUBIC_LCD_SERIAL_PORT)
  #error "ANYCUBIC_LCD_SERIAL_PORT is now LCD_SERIAL_PORT."
#elif defined(INTERNAL_SERIAL_PORT)
  #error "INTERNAL_SERIAL_PORT is now MMU_SERIAL_PORT."
#elif defined(X_DUAL_ENDSTOPS_ADJUSTMENT) || defined(Y_DUAL_ENDSTOPS_ADJUSTMENT) || defined(Z_DUAL_ENDSTOPS_ADJUSTMENT)
  #error "[XYZ]_DUAL_ENDSTOPS_ADJUSTMENT is now [XYZ]2_ENDSTOP_ADJUSTMENT."
#elif defined(Z_TRIPLE_ENDSTOPS_ADJUSTMENT2) || defined(Z_TRIPLE_ENDSTOPS_ADJUSTMENT3)
  #error "Z_TRIPLE_ENDSTOPS_ADJUSTMENT[23] is now Z[23]_ENDSTOP_ADJUSTMENT."
#elif defined(Z_QUAD_ENDSTOPS_ADJUSTMENT2) || defined(Z_QUAD_ENDSTOPS_ADJUSTMENT3) || defined(Z_QUAD_ENDSTOPS_ADJUSTMENT4)
  #error "Z_QUAD_ENDSTOPS_ADJUSTMENT[234] is now Z[234]_ENDSTOP_ADJUSTMENT."
#elif defined(Z_DUAL_STEPPER_DRIVERS)
  #error "Z_DUAL_STEPPER_DRIVERS is no longer needed and should be removed."
#elif defined(Z_TRIPLE_STEPPER_DRIVERS)
  #error "Z_TRIPLE_STEPPER_DRIVERS is no longer needed and should be removed."
#elif defined(Z_QUAD_STEPPER_DRIVERS)
  #error "Z_QUAD_STEPPER_DRIVERS is no longer needed and should be removed."
#elif defined(Z_DUAL_ENDSTOPS) || defined(Z_TRIPLE_ENDSTOPS) || defined(Z_QUAD_ENDSTOPS)
  #error "Z_(DUAL|TRIPLE|QUAD)_ENDSTOPS is now Z_MULTI_ENDSTOPS."
#elif defined(DUGS_UI_MOVE_DIS_OPTION)
  #error "DUGS_UI_MOVE_DIS_OPTION is spelled DGUS_UI_MOVE_DIS_OPTION."
#elif defined(ORIG_E0_AUTO_FAN_PIN) || defined(ORIG_E1_AUTO_FAN_PIN) || defined(ORIG_E2_AUTO_FAN_PIN) || defined(ORIG_E3_AUTO_FAN_PIN) || defined(ORIG_E4_AUTO_FAN_PIN) || defined(ORIG_E5_AUTO_FAN_PIN) || defined(ORIG_E6_AUTO_FAN_PIN) || defined(ORIG_E7_AUTO_FAN_PIN)
  #error "ORIG_Ex_AUTO_FAN_PIN is now just Ex_AUTO_FAN_PIN."
#elif defined(ORIG_CHAMBER_AUTO_FAN_PIN)
  #error "ORIG_CHAMBER_AUTO_FAN_PIN is now just CHAMBER_AUTO_FAN_PIN."
#elif defined(HOMING_BACKOFF_MM)
  #error "HOMING_BACKOFF_MM is now HOMING_BACKOFF_POST_MM."
#elif defined(X_HOME_BUMP_MM) || defined(Y_HOME_BUMP_MM) || defined(Z_HOME_BUMP_MM)
  #error "[XYZ]_HOME_BUMP_MM is now HOMING_BUMP_MM."
#elif defined(DIGIPOT_I2C)
  #error "DIGIPOT_I2C is now DIGIPOT_MCP4451 (or DIGIPOT_MCP4018)."
#elif defined(TOUCH_BUTTONS)
  #error "TOUCH_BUTTONS is now TOUCH_SCREEN."
#elif defined(DISABLE_ENCODER)
  #error "DISABLE_ENCODER is now NO_BACK_MENU_ITEM."
#elif defined(LCD_FULL_PIXEL_HEIGHT) || defined(LCD_FULL_PIXEL_WIDTH)
  #error "LCD_FULL_PIXEL_(WIDTH|HEIGHT) is deprecated and should be removed."
#elif defined(FSMC_UPSCALE)
  #error "FSMC_UPSCALE is now GRAPHICAL_TFT_UPSCALE."
#elif defined(ANYCUBIC_TFT_MODEL)
  #error "ANYCUBIC_TFT_MODEL is now ANYCUBIC_LCD_I3MEGA."
#elif defined(EVENT_GCODE_SD_STOP)
  #error "EVENT_GCODE_SD_STOP is now EVENT_GCODE_SD_ABORT."
#elif defined(GRAPHICAL_TFT_ROTATE_180)
  #error "GRAPHICAL_TFT_ROTATE_180 is now TFT_ROTATION set to TFT_ROTATE_180."
#elif defined(PROBE_OFFSET_START)
  #error "PROBE_OFFSET_START is now PROBE_OFFSET_WIZARD_START_Z."
#elif defined(POWER_LOSS_PULL)
  #error "POWER_LOSS_PULL is now specifically POWER_LOSS_PULL(UP|DOWN)."
#elif defined(SHORT_MANUAL_Z_MOVE)
  #error "SHORT_MANUAL_Z_MOVE is now FINE_MANUAL_MOVE, applying to Z on most printers."
#elif defined(FIL_RUNOUT_INVERTING)
  #if FIL_RUNOUT_INVERTING
    #error "FIL_RUNOUT_INVERTING true is now FIL_RUNOUT_STATE HIGH."
  #else
    #error "FIL_RUNOUT_INVERTING false is now FIL_RUNOUT_STATE LOW."
  #endif
#elif defined(ASSISTED_TRAMMING_MENU_ITEM)
  #error "ASSISTED_TRAMMING_MENU_ITEM is deprecated and should be removed."
#elif defined(UNKNOWN_Z_NO_RAISE)
  #error "UNKNOWN_Z_NO_RAISE is replaced by setting Z_IDLE_HEIGHT to Z_MAX_POS."
#elif defined(Z_AFTER_DEACTIVATE)
  #error "Z_AFTER_DEACTIVATE is replaced by Z_IDLE_HEIGHT."
#elif defined(MEATPACK)
  #error "MEATPACK is now enabled with MEATPACK_ON_SERIAL_PORT_1, MEATPACK_ON_SERIAL_PORT_2, etc."
#elif defined(CUSTOM_USER_MENUS)
  #error "CUSTOM_USER_MENUS has been replaced by CUSTOM_MENU_MAIN and CUSTOM_MENU_CONFIG."
#elif defined(MKS_LCD12864)
  #error "MKS_LCD12864 is now MKS_LCD12864A or MKS_LCD12864B."
#elif defined(DOGM_SD_PERCENT)
  #error "DOGM_SD_PERCENT is now SHOW_PROGRESS_PERCENT."
#elif defined(NEOPIXEL_BKGD_LED_INDEX)
  #error "NEOPIXEL_BKGD_LED_INDEX is now NEOPIXEL_BKGD_INDEX_FIRST."
#elif defined(TEMP_SENSOR_1_AS_REDUNDANT)
  #error "TEMP_SENSOR_1_AS_REDUNDANT is now TEMP_SENSOR_REDUNDANT, with associated TEMP_SENSOR_REDUNDANT_* config."
#elif defined(MAX_REDUNDANT_TEMP_SENSOR_DIFF)
  #error "MAX_REDUNDANT_TEMP_SENSOR_DIFF is now TEMP_SENSOR_REDUNDANT_MAX_DIFF"
#elif defined(LCD_ALEPHOBJECTS_CLCD_UI)
  #error "LCD_ALEPHOBJECTS_CLCD_UI is now LCD_LULZBOT_CLCD_UI."
#elif defined(MIN_ARC_SEGMENTS)
  #error "MIN_ARC_SEGMENTS is now MIN_CIRCLE_SEGMENTS."
#elif defined(ARC_SEGMENTS_PER_R)
  #error "ARC_SUPPORT no longer uses ARC_SEGMENTS_PER_R."
#elif ENABLED(ARC_SUPPORT) && (!defined(MIN_ARC_SEGMENT_MM) || !defined(MAX_ARC_SEGMENT_MM))
  #error "ARC_SUPPORT now requires MIN_ARC_SEGMENT_MM and MAX_ARC_SEGMENT_MM."
#elif defined(LASER_POWER_INLINE)
  #error "LASER_POWER_INLINE is obsolete."
#elif defined(SPINDLE_LASER_PWM)
  #error "SPINDLE_LASER_PWM (true) is now set with SPINDLE_LASER_USE_PWM (enabled)."
#elif ANY(IS_RAMPS_EEB, IS_RAMPS_EEF, IS_RAMPS_EFB, IS_RAMPS_EFF, IS_RAMPS_SF)
  #error "The IS_RAMPS_* conditionals (for heater/fan/bed pins) are now called FET_ORDER_*."
#elif defined(PROBE_TEMP_COMPENSATION)
  #error "PROBE_TEMP_COMPENSATION is now set using the PTC_PROBE, PTC_BED, PTC_HOTEND options."
#elif defined(BTC_PROBE_TEMP)
  #error "BTC_PROBE_TEMP is now PTC_PROBE_TEMP."
#elif defined(LCD_SCREEN_ROT_90)
  #error "LCD_SCREEN_ROT_90 is now LCD_SCREEN_ROTATE with a value of 90."
#elif defined(LCD_SCREEN_ROT_180)
  #error "LCD_SCREEN_ROT_180 is now LCD_SCREEN_ROTATE with a value of 180."
#elif defined(LCD_SCREEN_ROT_270)
  #error "LCD_SCREEN_ROT_270 is now LCD_SCREEN_ROTATE with a value of 270."
#elif defined(DEFAULT_LCD_BRIGHTNESS)
  #error "DEFAULT_LCD_BRIGHTNESS is now LCD_BRIGHTNESS_DEFAULT."
#elif defined(NOZZLE_PARK_X_ONLY)
  #error "NOZZLE_PARK_X_ONLY is now NOZZLE_PARK_MOVE 1."
#elif defined(NOZZLE_PARK_Y_ONLY)
  #error "NOZZLE_PARK_Y_ONLY is now NOZZLE_PARK_MOVE 2."
#elif defined(Z_STEPPER_ALIGN_KNOWN_STEPPER_POSITIONS)
  #error "Z_STEPPER_ALIGN_KNOWN_STEPPER_POSITIONS is now just Z_STEPPER_ALIGN_STEPPER_XY."
#elif defined(DWIN_CREALITY_LCD_ENHANCED)
  #error "DWIN_CREALITY_LCD_ENHANCED is now DWIN_LCD_PROUI."
#elif defined(LINEAR_AXES)
  #error "LINEAR_AXES is now NUM_AXES (to account for rotational axes)."
#elif defined(X_DUAL_STEPPER_DRIVERS)
  #error "X_DUAL_STEPPER_DRIVERS is no longer needed and should be removed."
#elif defined(Y_DUAL_STEPPER_DRIVERS)
  #error "Y_DUAL_STEPPER_DRIVERS is no longer needed and should be removed."
#elif defined(NUM_Z_STEPPER_DRIVERS)
  #error "NUM_Z_STEPPER_DRIVERS is no longer needed and should be removed."
#elif defined(SNMM)
  #error "SNMM is obsolete. Define MMU_MODEL as PRUSA_MMU1 instead."
#elif defined(MK2_MULTIPLEXER)
  #error "MK2_MULTIPLEXER is obsolete. Define MMU_MODEL as PRUSA_MMU1 instead."
#elif defined(PRUSA_MMU2)
  #error "PRUSA_MMU2 is obsolete. Define MMU_MODEL as PRUSA_MMU2 instead."
#elif defined(PRUSA_MMU2_S_MODE)
  #error "PRUSA_MMU2_S_MODE is obsolete. Define MMU_MODEL as PRUSA_MMU2S instead."
#elif defined(SMUFF_EMU_MMU2)
  #error "SMUFF_EMU_MMU2 is obsolete. Define MMU_MODEL as EXTENDABLE_EMU_MMU2 instead."
#elif defined(SMUFF_EMU_MMU2S)
  #error "SMUFF_EMU_MMU2S is obsolete. Define MMU_MODEL as EXTENDABLE_EMU_MMU2S instead."
#elif defined(LEVEL_BED_CORNERS)
  #error "LEVEL_BED_CORNERS is now LCD_BED_TRAMMING."
#elif defined(LEVEL_CORNERS_INSET_LFRB) || defined(LEVEL_CORNERS_HEIGHT) || defined(LEVEL_CORNERS_Z_HOP) || defined(LEVEL_CORNERS_USE_PROBE) || defined(LEVEL_CORNERS_PROBE_TOLERANCE) || defined(LEVEL_CORNERS_VERIFY_RAISED) || defined(LEVEL_CORNERS_AUDIO_FEEDBACK)
  #error "LEVEL_CORNERS_* settings have been renamed BED_TRAMMING_*."
#elif defined(LEVEL_CENTER_TOO)
  #error "LEVEL_CENTER_TOO is now BED_TRAMMING_INCLUDE_CENTER."
#elif defined(TOUCH_IDLE_SLEEP)
  #error "TOUCH_IDLE_SLEEP (seconds) is now DISPLAY_SLEEP_MINUTES (minutes)."
#elif defined(TOUCH_IDLE_SLEEP_MINS)
  #error "TOUCH_IDLE_SLEEP_MINS is now DISPLAY_SLEEP_MINUTES."
#elif defined(LCD_BACKLIGHT_TIMEOUT)
  #error "LCD_BACKLIGHT_TIMEOUT (seconds) is now LCD_BACKLIGHT_TIMEOUT_MINS (minutes)."
#elif defined(LCD_SET_PROGRESS_MANUALLY)
  #error "LCD_SET_PROGRESS_MANUALLY is now SET_PROGRESS_MANUALLY."
#elif defined(USE_M73_REMAINING_TIME)
  #error "USE_M73_REMAINING_TIME is now SET_REMAINING_TIME."
#elif defined(SHOW_SD_PERCENT)
  #error "SHOW_SD_PERCENT is now SHOW_PROGRESS_PERCENT."
#elif defined(LIN_ADVANCE_K)
  #error "LIN_ADVANCE_K is now ADVANCE_K."
#elif defined(EXTRA_LIN_ADVANCE_K)
  #error "EXTRA_LIN_ADVANCE_K is now ADVANCE_K_EXTRA."
#elif defined(POLAR_SEGMENTS_PER_SECOND) || defined(DELTA_SEGMENTS_PER_SECOND)  || defined(SCARA_SEGMENTS_PER_SECOND) || defined(TPARA_SEGMENTS_PER_SECOND)
  #error "(POLAR|DELTA|SCARA|TPARA)_SEGMENTS_PER_SECOND is now DEFAULT_SEGMENTS_PER_SECOND."
#elif defined(TMC_SW_MOSI) || defined(TMC_SW_MISO) || defined(TMC_SW_SCK)
  #error "TMC_SW_(MOSI|MISO|SCK) is now TMC_SPI_(MOSI|MISO|SCK)."
#elif ANY(DGUS_LCD_UI_ORIGIN, DGUS_LCD_UI_FYSETC, DGUS_LCD_UI_HIPRECY, DGUS_LCD_UI_MKS, DGUS_LCD_UI_RELOADED) && !defined(DGUS_LCD_UI)
  #error "DGUS_LCD_UI_[TYPE] is now set using DGUS_LCD_UI TYPE."
#elif defined(DELTA_PRINTABLE_RADIUS)
  #error "DELTA_PRINTABLE_RADIUS is now PRINTABLE_RADIUS."
#elif defined(SCARA_PRINTABLE_RADIUS)
  #error "SCARA_PRINTABLE_RADIUS is now PRINTABLE_RADIUS."
#elif defined(SCARA_FEEDRATE_SCALING)
  #error "SCARA_FEEDRATE_SCALING is now FEEDRATE_SCALING."
#elif defined(MILLISECONDS_PREHEAT_TIME)
  #error "MILLISECONDS_PREHEAT_TIME is now PREHEAT_TIME_HOTEND_MS."
#elif defined(EXPERIMENTAL_SCURVE)
  #error "EXPERIMENTAL_SCURVE is no longer needed and should be removed."
#elif defined(BABYSTEP_ZPROBE_GFX_OVERLAY)
  #error "BABYSTEP_ZPROBE_GFX_OVERLAY is now BABYSTEP_GFX_OVERLAY."
#elif defined(DISABLE_INACTIVE_EXTRUDER)
  #error "DISABLE_INACTIVE_EXTRUDER is now DISABLE_OTHER_EXTRUDERS."
#elif defined(INVERT_X_STEP_PIN) || defined(INVERT_Y_STEP_PIN) || defined(INVERT_Z_STEP_PIN) || defined(INVERT_I_STEP_PIN) || defined(INVERT_J_STEP_PIN) || defined(INVERT_K_STEP_PIN) || defined(INVERT_U_STEP_PIN) || defined(INVERT_V_STEP_PIN) || defined(INVERT_W_STEP_PIN) || defined(INVERT_E_STEP_PIN)
  #error "INVERT_*_STEP_PIN true is now STEP_STATE_* LOW, and INVERT_*_STEP_PIN false is now STEP_STATE_* HIGH."
#elif defined(PROBE_PT_1_X) || defined(PROBE_PT_1_Y) || defined(PROBE_PT_2_X) || defined(PROBE_PT_2_Y) || defined(PROBE_PT_3_X) || defined(PROBE_PT_3_Y)
  #error "PROBE_PT_[123]_[XY] is now defined using PROBE_PT_[123] with an array { x, y }."
#elif defined(SQUARE_WAVE_STEPPING)
  #error "SQUARE_WAVE_STEPPING is now EDGE_STEPPING."
#elif defined(MINIMUM_STEPPER_PULSE)
  #error "MINIMUM_STEPPER_PULSE (in µs) is now MINIMUM_STEPPER_PULSE_NS. Multiply old MINIMUM_STEPPER_PULSE x 1000!"
#elif defined(FAN_PIN)
  #error "FAN_PIN is now FAN0_PIN."
#elif defined(X_MIN_ENDSTOP_INVERTING) || defined(Y_MIN_ENDSTOP_INVERTING) || defined(Z_MIN_ENDSTOP_INVERTING) \
   || defined(I_MIN_ENDSTOP_INVERTING) || defined(J_MIN_ENDSTOP_INVERTING) || defined(K_MIN_ENDSTOP_INVERTING) \
   || defined(U_MIN_ENDSTOP_INVERTING) || defined(V_MIN_ENDSTOP_INVERTING) || defined(W_MIN_ENDSTOP_INVERTING) \
   || defined(X_MAX_ENDSTOP_INVERTING) || defined(Y_MAX_ENDSTOP_INVERTING) || defined(Z_MAX_ENDSTOP_INVERTING) \
   || defined(I_MAX_ENDSTOP_INVERTING) || defined(J_MAX_ENDSTOP_INVERTING) || defined(K_MAX_ENDSTOP_INVERTING) \
   || defined(U_MAX_ENDSTOP_INVERTING) || defined(V_MAX_ENDSTOP_INVERTING) || defined(W_MAX_ENDSTOP_INVERTING) \
   || defined(Z_MIN_PROBE_ENDSTOP_INVERTING)
  #error "*_ENDSTOP_INVERTING false/true is now set with *_ENDSTOP_HIT_STATE HIGH/LOW."
#elif defined(DISABLE_INACTIVE_X) || defined(DISABLE_INACTIVE_Y) || defined(DISABLE_INACTIVE_Z) \
   || defined(DISABLE_INACTIVE_I) || defined(DISABLE_INACTIVE_J) || defined(DISABLE_INACTIVE_K) \
   || defined(DISABLE_INACTIVE_U) || defined(DISABLE_INACTIVE_V) || defined(DISABLE_INACTIVE_W) || defined(DISABLE_INACTIVE_E)
  #error "DISABLE_INACTIVE_[XYZIJKUVWE] is now DISABLE_IDLE_[XYZIJKUVWE]."
#elif defined(DEFAULT_STEPPER_DEACTIVE_TIME)
  #error "DEFAULT_STEPPER_DEACTIVE_TIME is now DEFAULT_STEPPER_TIMEOUT_SEC."
#elif defined(TFT_SHARED_SPI)
  #error "TFT_SHARED_SPI is now TFT_SHARED_IO."
#elif defined(LCD_PINS_ENABLE)
  #error "LCD_PINS_ENABLE is now LCD_PINS_EN."
#elif ANY(USE_XMIN_PLUG, USE_XMAX_PLUG, USE_YMIN_PLUG, USE_YMAX_PLUG, USE_ZMIN_PLUG, USE_ZMAX_PLUG, \
          USE_IMIN_PLUG, USE_IMAX_PLUG, USE_JMIN_PLUG, USE_JMAX_PLUG, USE_KMIN_PLUG, USE_KMAX_PLUG, \
          USE_UMIN_PLUG, USE_UMAX_PLUG, USE_VMIN_PLUG, USE_VMAX_PLUG, USE_WMIN_PLUG, USE_WMAX_PLUG)
  #error "USE_*_PLUG settings are no longer needed and should be removed."
#elif defined(X2_USE_ENDSTOP)
  #error "X2_USE_ENDSTOP is obsolete. Instead set X2_STOP_PIN directly. (e.g., 'X2_USE_ENDSTOP _XMAX_' becomes 'X2_STOP_PIN X_MAX_PIN')"
#elif defined(Y2_USE_ENDSTOP)
  #error "Y2_USE_ENDSTOP is obsolete. Instead set Y2_STOP_PIN directly. (e.g., 'Y2_USE_ENDSTOP _YMAX_' becomes 'Y2_STOP_PIN Y_MAX_PIN')"
#elif defined(Z2_USE_ENDSTOP)
  #error "Z2_USE_ENDSTOP is obsolete. Instead set Z2_STOP_PIN directly. (e.g., 'Z2_USE_ENDSTOP _ZMAX_' becomes 'Z2_STOP_PIN Z_MAX_PIN')"
#elif defined(Z3_USE_ENDSTOP)
  #error "Z3_USE_ENDSTOP is obsolete. Instead set Z3_STOP_PIN directly. (e.g., 'Z3_USE_ENDSTOP _ZMAX_' becomes 'Z3_STOP_PIN Z_MAX_PIN')"
#elif defined(Z4_USE_ENDSTOP)
  #error "Z4_USE_ENDSTOP is obsolete. Instead set Z4_STOP_PIN directly. (e.g., 'Z4_USE_ENDSTOP _ZMAX_' becomes 'Z4_STOP_PIN Z_MAX_PIN')"
#elif defined(INTEGRATED_BABYSTEPPING)
  #error "INTEGRATED_BABYSTEPPING is no longer needed and should be removed."
#elif defined(FOLDER_SORTING)
  #error "FOLDER_SORTING is now SDSORT_FOLDERS."
#elif defined(BTT_MINI_12864_V1)
  #error "BTT_MINI_12864_V1 is now BTT_MINI_12864."
#elif defined(Z_PROBE_OFFSET_RANGE_MIN) || defined(Z_PROBE_OFFSET_RANGE_MAX)
  #error "Z_PROBE_OFFSET_RANGE_(MIN|MAX) is now PROBE_OFFSET_Z(MIN|MAX)."
#elif defined(LARGE_MOVE_ITEMS)
  #error "LARGE_MOVE_ITEMS is obsolete. Instead define MANUAL_MOVE_DISTANCE_MM and MANUAL_MOVE_DISTANCE_IN."
#elif defined(SDIO_SUPPORT)
  #error "SDIO_SUPPORT is now ONBOARD_SDIO."
#elif defined(ANET_FULL_GRAPHICS_LCD_ALT_WIRING)
  #error "ANET_FULL_GRAPHICS_LCD_ALT_WIRING is now CTC_A10S_A13."
#elif defined(Z_PROBE_END_SCRIPT)
  #error "Z_PROBE_END_SCRIPT is now EVENT_GCODE_AFTER_G29."
#elif defined(WIFI_SERIAL)
  #error "WIFI_SERIAL is now WIFI_SERIAL_PORT."
#elif defined(CALIBRATION_MEASUREMENT_RESOLUTION)
  #error "CALIBRATION_MEASUREMENT_RESOLUTION is no longer needed and should be removed."
#elif defined(MMU2_MENUS)
  #error "MMU2_MENUS is now MMU_MENUS."
#elif defined(MMU2_SERIAL_PORT)
  #error "MMU2_SERIAL_PORT is now MMU_SERIAL_PORT."
#elif defined(MMU2_RST_PIN)
  #error "MMU2_RST_PIN is now MMU_RST_PIN."
#elif defined(MMU2_MAX_RETRIES)
  #error "MMU2_MAX_RETRIES is now MMU3_MAX_RETRIES."
#elif defined(MMU2_FILAMENT_SENSOR_POSITION)
  #error "MMU2_FILAMENT_SENSOR_POSITION is now MMU3_FILAMENT_SENSOR_POSITION."
#elif defined(MMU2_TOOL_CHANGE_LOAD_LENGTH)
  #error "MMU2_TOOL_CHANGE_LOAD_LENGTH is now MMU3_TOOL_CHANGE_LOAD_LENGTH."
#elif defined(MMU2_LOAD_TO_NOZZLE_FEED_RATE)
  #error "MMU2_LOAD_TO_NOZZLE_FEED_RATE is now MMU3_LOAD_TO_NOZZLE_FEED_RATE."
#elif defined(MMU2_UNLOAD_TO_FINDA_FEED_RATE)
  #error "MMU2_UNLOAD_TO_FINDA_FEED_RATE is no longer needed and should be removed."
#elif defined(MMU2_VERIFY_LOAD_TO_NOZZLE_FEED_RATE)
  #error "MMU2_VERIFY_LOAD_TO_NOZZLE_FEED_RATE is now MMU3_VERIFY_LOAD_TO_NOZZLE_FEED_RATE."
#elif defined(MMU2_RETRY_UNLOAD_TO_FINDA_LENGTH)
  #error "MMU2_RETRY_UNLOAD_TO_FINDA_LENGTH is now MMU3_RETRY_UNLOAD_TO_FINDA_LENGTH."
#elif defined(MMU2_RETRY_UNLOAD_TO_FINDA_FEED_RATE)
  #error "MMU2_RETRY_UNLOAD_TO_FINDA_FEED_RATE is now MMU3_RETRY_UNLOAD_TO_FINDA_FEED_RATE."
#elif defined(MMU2_FILAMENT_SENSOR_POSITION)
  #error "MMU2_FILAMENT_SENSOR_POSITION is now MMU3_FILAMENT_SENSOR_E_POSITION."
#elif defined(MMU2_CHECK_FILAMENT_PRESENCE_EXTRUSION_LENGTH)
  #error "MMU2_CHECK_FILAMENT_PRESENCE_EXTRUSION_LENGTH is now MMU3_CHECK_FILAMENT_PRESENCE_EXTRUSION_LENGTH."
#elif defined(MMU_HAS_CUTTER)
  #error "MMU_HAS_CUTTER is now MMU3_HAS_CUTTER."
#elif defined(MMU_FORCE_STEALTH_MODE)
  #error "MMU_FORCE_STEALTH_MODE is obsolete and should be removed."
#elif defined(MMU_SPOOL_JOIN_CONSUMES_ALL_FILAMENT)
  #error "MMU_SPOOL_JOIN_CONSUMES_ALL_FILAMENT is now MMU3_SPOOL_JOIN_CONSUMES_ALL_FILAMENT."
#elif HAS_PRUSA_MMU3 && defined(MMU2_LOAD_TO_NOZZLE_SEQUENCE)
  #error "MMU2_LOAD_TO_NOZZLE_SEQUENCE is now MMU3_LOAD_TO_NOZZLE_SEQUENCE for MMU_MODEL MMU3."
#elif HAS_PRUSA_MMU3 && defined(MMU2_RAMMING_SEQUENCE)
  #error "MMU2_RAMMING_SEQUENCE is now MMU3_RAMMING_SEQUENCE for MMU_MODEL MMU3."
#elif defined(MMU_EXTRUDER_SENSOR)
  #error "MMU_EXTRUDER_SENSOR is now MMU2_EXTRUDER_SENSOR."
#elif defined(MMU_LOADING_ATTEMPTS_NR)
  #error "MMU_LOADING_ATTEMPTS_NR is now MMU2_LOADING_ATTEMPTS_NR."
#elif defined(MMU2_DEBUG)
  #error "MMU2_DEBUG is now MMU_DEBUG."
#elif defined(FTM_SHAPING_DEFAULT_X_FREQ) || defined(FTM_SHAPING_DEFAULT_Y_FREQ)
  #error "FTM_SHAPING_DEFAULT_[XY]_FREQ is now FTM_SHAPING_DEFAULT_FREQ_[XY]."
#elif defined(SDSS)
  #error "SDSS is now SD_SS_PIN."
#endif

// SDSS renamed to SD_SS_PIN
#undef SDSS
#define SDSS 8675309
#if USB_CS_PIN == SDSS
  #error "SDSS is now SD_SS_PIN."
#endif
#undef SDSS

// Changes to Probe Temp Compensation (#17392)
#if HAS_PTC && TEMP_SENSOR_PROBE && TEMP_SENSOR_BED
  #if defined(PTC_PARK_POS_X) || defined(PTC_PARK_POS_Y) || defined(PTC_PARK_POS_Z)
    #error "PTC_PARK_POS_[XYZ] is now PTC_PARK_POS (array)."
  #elif defined(PTC_PROBE_POS_X) || defined(PTC_PROBE_POS_Y)
    #error "PTC_PROBE_POS_[XY] is now PTC_PROBE_POS (array)."
  #endif
#endif

// Consolidate TMC26X, validate migration (#24373)
#define _ISMAX_1(A) defined(A##_MAX_CURRENT)
#define _ISSNS_1(A) defined(A##_SENSE_RESISTOR)
#if DO(ISMAX,||,ALL_AXIS_NAMES)
  #error "*_MAX_CURRENT is now set with *_CURRENT."
#elif DO(ISSNS,||,ALL_AXIS_NAMES)
  #error "*_SENSE_RESISTOR (in Milli-Ohms) is now set with *_RSENSE (in Ohms), so you must divide values by 1000."
#endif
#undef _ISMAX_1
#undef _ISSNS_1

// L64xx stepper drivers have been removed
#define _L6470              0x6470
#define _L6474              0x6474
#define _L6480              0x6480
#define _POWERSTEP01        0xF00D
#define _TMC26X             0x2600
#define _TMC26X_STANDALONE  0x2601
#if HAS_DRIVER(L6470)
  #error "L6470 stepper drivers are no longer supported in Marlin."
#elif HAS_DRIVER(L6474)
  #error "L6474 stepper drivers are no longer supported in Marlin."
#elif HAS_DRIVER(L6480)
  #error "L6480 stepper drivers are no longer supported in Marlin."
#elif HAS_DRIVER(POWERSTEP01)
  #error "POWERSTEP01 stepper drivers are no longer supported in Marlin."
#elif HAS_DRIVER(TMC26X) || HAS_DRIVER(TMC26X_STANDALONE)
  #error "TMC26X stepper drivers are no longer supported in Marlin."
#endif
#undef _L6470
#undef _L6474
#undef _L6480
#undef _POWERSTEP01
#undef _TMC26X
#undef _TMC26X_STANDALONE
