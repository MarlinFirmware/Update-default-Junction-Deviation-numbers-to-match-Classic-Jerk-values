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

/************
 * ui_api.h *
 ************/

/****************************************************************************
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

#include "../../inc/MarlinConfig.h"

#include "../marlinui.h"
#include "../../gcode/gcode.h"
#include "../../module/temperature.h"

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "../../module/motion.h"
  #include "../../feature/pause.h"
#endif

namespace ExtUI {

  // The ExtUI implementation can store up to this many bytes
  // in the EEPROM when the methods onStoreSettings and
  // onLoadSettings are called.

  #ifndef EXTUI_EEPROM_DATA_SIZE
    #define EXTUI_EEPROM_DATA_SIZE 48
  #endif
  static constexpr size_t eeprom_data_size = EXTUI_EEPROM_DATA_SIZE;

  enum axis_t     : uint8_t { X, Y, Z, I, J, K, U, V, W, X2, Y2, Z2, Z3, Z4 };
  enum extruder_t : uint8_t { E0, E1, E2, E3, E4, E5, E6, E7 };
  enum heater_t   : uint8_t { H0, H1, H2, H3, H4, H5, BED, CHAMBER, COOLER };
  enum fan_t      : uint8_t { FAN0, FAN1, FAN2, FAN3, FAN4, FAN5, FAN6, FAN7 };
  #if HAS_PID_HEATING
    enum pidresult_t : uint8_t { PID_STARTED, PID_BED_STARTED, PID_CHAMBER_STARTED, PID_BAD_HEATER_ID, PID_TEMP_TOO_HIGH, PID_TUNING_TIMEOUT, PID_DONE };
  #endif
  #if ENABLED(MPC_AUTOTUNE)
    enum mpcresult_t : uint8_t { MPC_STARTED, MPC_TEMP_ERROR, MPC_INTERRUPTED, MPC_DONE };
  #endif
  struct probe_limits_t { float xmin, ymin, xmax, ymax; };

  constexpr uint8_t extruderCount = EXTRUDERS;
  constexpr uint8_t hotendCount   = HOTENDS;
  constexpr uint8_t fanCount      = FAN_COUNT;

  inline const axis_t axis_to_axis_t(const AxisEnum a) {
    switch (a) {
      TERN_(HAS_X_AXIS, case X_AXIS:)
      default: return X;
      OPTCODE(HAS_Y_AXIS, case Y_AXIS: return Y)
      OPTCODE(HAS_Z_AXIS, case Z_AXIS: return Z)
      OPTCODE(HAS_I_AXIS, case I_AXIS: return I)
      OPTCODE(HAS_J_AXIS, case J_AXIS: return J)
      OPTCODE(HAS_K_AXIS, case K_AXIS: return K)
      OPTCODE(HAS_U_AXIS, case U_AXIS: return U)
      OPTCODE(HAS_V_AXIS, case V_AXIS: return V)
      OPTCODE(HAS_W_AXIS, case W_AXIS: return W)
    }
  }

  #if HAS_MESH
    typedef float bed_mesh_t[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
  #endif

  /**
   * The Extensible UI API is a utility class that can be used to implement:
   * - An LCD view that responds to standard events, e.g., onMediaMounted(...)
   * - An LCD that polls firmware states and settings in a standard manner.
   *   (e.g., With tool indexes and extruder indexes).
   * - Standard hooks to send data to a serial-based controller.
   *
   * ExtUI is best used when:
   * - The display handles LCD touch / buttons so the firmware doesn't see these events.
   * - Commands and value edits are sent over serial to Marlin as G-codes.
   * - The display can get data from Marlin, but is not necessarily drawn by Marlin.
   * - The display cannot implement a MarlinUI menu.
   * - The display is implemented with code callbacks alongside ExtUI callbacks.
   *
   * Building an ExtUI layer:
   * - Start by making an lcd/extui subfolder. Copy 'example' or another display.
   * - Many of these methods are optional. Implement them according to your UI needs.
   * - If your display needs information from Marlin, add an accessor to ExtUI.
   * - If some addition seems like it should be standard part of ExtUI, submit a PR with the new
   *   methods added to this API. Implement in the ExtUI example and/or with some existing displays.
   */

  bool isMoving();
  bool isAxisPositionKnown(const axis_t);
  bool isAxisPositionKnown(const extruder_t);
  bool isPositionKnown(); // Axis position guaranteed, steppers active since homing
  bool isMachineHomed(); // Axis position most likely correct, steppers may have deactivated
  bool canMove(const axis_t);
  bool canMove(const extruder_t);
  void injectCommands_P(PGM_P const);
  inline void injectCommands(FSTR_P const fstr) { injectCommands_P(FTOP(fstr)); }
  void injectCommands(char * const);
  bool commandsInQueue();

  #if ENABLED(HOST_KEEPALIVE_FEATURE)
    GcodeSuite::MarlinBusyState getHostKeepaliveState();
    bool getHostKeepaliveIsPaused();
  #endif

  #if ENABLED(JOYSTICK)
    void jog(const xyz_float_t &dir);
    void _joystick_update(xyz_float_t &norm_jog);
  #endif

  /**
   * Getters and setters
   * Use to query or change Marlin's state.
   */
  PGM_P getFirmwareName_str();

  #if HAS_SOFTWARE_ENDSTOPS
    bool getSoftEndstopState();
    void setSoftEndstopState(const bool);
  #endif

  // Trinamic Current / Bump Sensitivity
  #if HAS_TRINAMIC_CONFIG
    float getAxisCurrent_mA(const axis_t);
    float getAxisCurrent_mA(const extruder_t);
    void  setAxisCurrent_mA(const_float_t, const axis_t);
    void  setAxisCurrent_mA(const_float_t, const extruder_t);

     int getTMCBumpSensitivity(const axis_t);
    void setTMCBumpSensitivity(const_float_t, const axis_t);
  #endif

  // Actual and target accessors, by Heater ID, Extruder ID, Fan ID
  void enableHeater(const heater_t);
  void enableHeater(const extruder_t);
  bool isHeaterIdle(const heater_t);
  bool isHeaterIdle(const extruder_t);
  celsius_float_t getActualTemp_celsius(const heater_t);
  celsius_float_t getActualTemp_celsius(const extruder_t);
  celsius_t getTargetTemp_celsius(const heater_t);
  celsius_t getTargetTemp_celsius(const extruder_t);
  uint8_t getActualFan_percent(const fan_t);
  uint8_t getTargetFan_percent(const fan_t);

  // High level positions, by Axis ID, Extruder ID
  float getAxisPosition_mm(const axis_t);
  float getAxisPosition_mm(const extruder_t);
  // Axis steps-per-mm, by Axis ID, Extruder ID
  float getAxisSteps_per_mm(const axis_t);
  float getAxisSteps_per_mm(const extruder_t);
  // Speed and acceleration limits, per Axis ID or Extruder ID
  feedRate_t getAxisMaxFeedrate_mm_s(const axis_t);
  feedRate_t getAxisMaxFeedrate_mm_s(const extruder_t);
  float getAxisMaxAcceleration_mm_s2(const axis_t);
  float getAxisMaxAcceleration_mm_s2(const extruder_t);
  // Standard speeds, as set in the planner
  feedRate_t getMinFeedrate_mm_s();
  feedRate_t getMinTravelFeedrate_mm_s();
  feedRate_t getFeedrate_mm_s();
  // Standard accelerations, as set in the planner
  float getPrintingAcceleration_mm_s2();
  float getRetractAcceleration_mm_s2();
  float getTravelAcceleration_mm_s2();
  // A speed multiplier for overall printing
  float getFeedrate_percent();
  // The flow percentage of an extruder
  int16_t getFlow_percent(const extruder_t);

  // Progress / Elapsed Time
  inline uint8_t getProgress_percent() { return ui.get_progress_percent(); }
  #if HAS_PRINT_PROGRESS_PERMYRIAD
    inline uint16_t getProgress_permyriad() { return ui.get_progress_permyriad(); }
  #endif
  uint32_t getProgress_seconds_elapsed();

  // Material Preheat Presets
  #if HAS_PREHEAT
    uint16_t getMaterial_preset_E(const uint16_t);
    #if HAS_HEATED_BED
      uint16_t getMaterial_preset_B(const uint16_t);
    #endif
  #endif

  // IDEX Machine Mode
  #if ENABLED(DUAL_X_CARRIAGE)
    uint8_t getIDEX_Mode();
  #endif

  #if ENABLED(SHOW_REMAINING_TIME)
    inline uint32_t getProgress_seconds_remaining() { return ui.get_remaining_time(); }
  #endif
  #if ENABLED(SHOW_INTERACTION_TIME)
    inline uint32_t getInteraction_seconds_remaining() { return ui.interaction_time; }
  #endif

  #if HAS_LEVELING
    // Global leveling state, events
    bool getLevelingActive();
    void setLevelingActive(const bool);
    bool getLevelingIsValid();
    void onLevelingStart();
    void onLevelingDone();
    #if ENABLED(PREHEAT_BEFORE_LEVELING)
      celsius_t getLevelingBedTemp();
    #endif
    #if HAS_MESH
      // Mesh data, utilities, events
      bed_mesh_t& getMeshArray();
      float getMeshPoint(const xy_uint8_t &pos);
      void setMeshPoint(const xy_uint8_t &pos, const_float_t zval);
      void moveToMeshPoint(const xy_uint8_t &pos, const_float_t z);
      void onMeshUpdate(const int8_t xpos, const int8_t ypos, const_float_t zval);
      inline void onMeshUpdate(const xy_int8_t &pos, const_float_t zval) { onMeshUpdate(pos.x, pos.y, zval); }

      typedef enum : uint8_t {
        G29_START,        // Prior to start of probe
        G29_FINISH,       // Following probe of all points
        G29_POINT_START,  // Beginning probe of grid location
        G29_POINT_FINISH, // Finished probe of grid location
        G26_START,
        G26_FINISH,
        G26_POINT_START,
        G26_POINT_FINISH
      } probe_state_t;
      void onMeshUpdate(const int8_t xpos, const int8_t ypos, probe_state_t state);
      inline void onMeshUpdate(const xy_int8_t &pos, probe_state_t state) { onMeshUpdate(pos.x, pos.y, state); }
    #endif
  #endif

  // Send an 'M876 S' host response
  #if ENABLED(HOST_PROMPT_SUPPORT)
    void setHostResponse(const uint8_t);
  #endif

  // Provide a simulated click to MarlinUI
  inline void simulateUserClick() {
    #if ANY(HAS_MARLINUI_MENU, EXTENSIBLE_UI)
      ui.lcd_clicked = true;
    #endif
  }

  #if ENABLED(PRINTCOUNTER)
    // Printcounter strings (See nextion_tft.cpp)
    char* getFailedPrints_str(char buffer[21]);
    char* getTotalPrints_str(char buffer[21]);
    char* getFinishedPrints_str(char buffer[21]);
    char* getTotalPrintTime_str(char buffer[21]);
    char* getLongestPrint_str(char buffer[21]);
    char* getFilamentUsed_str(char buffer[21]);
  #endif

  // Temperature Control
  void setTargetTemp_celsius(const_float_t, const heater_t);
  void setTargetTemp_celsius(const_float_t, const extruder_t);
  void setTargetFan_percent(const_float_t, const fan_t);
  void coolDown();

  // Motion Control
  void setAxisPosition_mm(const_float_t, const axis_t, const feedRate_t=0);
  void setAxisPosition_mm(const_float_t, const extruder_t, const feedRate_t=0);

  // Planner Control
  #if ENABLED(EDITABLE_STEPS_PER_UNIT)
    void setAxisSteps_per_mm(const_float_t, const axis_t);
    void setAxisSteps_per_mm(const_float_t, const extruder_t);
  #endif
  void setAxisMaxFeedrate_mm_s(const feedRate_t, const axis_t);
  void setAxisMaxFeedrate_mm_s(const feedRate_t, const extruder_t);
  void setAxisMaxAcceleration_mm_s2(const_float_t, const axis_t);
  void setAxisMaxAcceleration_mm_s2(const_float_t, const extruder_t);
  void setFeedrate_mm_s(const feedRate_t);
  void setMinFeedrate_mm_s(const feedRate_t);
  void setMinTravelFeedrate_mm_s(const feedRate_t);
  void setPrintingAcceleration_mm_s2(const_float_t);
  void setRetractAcceleration_mm_s2(const_float_t);
  void setTravelAcceleration_mm_s2(const_float_t);
  void setFeedrate_percent(const_float_t);
  void setFlow_percent(const int16_t, const extruder_t);

  // Waiting for User Interaction
  bool awaitingUserConfirm();
  void setUserConfirmed();

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    // "Purge More" has a control screen
    void setPauseMenuResponse(PauseMenuResponse);
    PauseMode getPauseMode();
  #endif

  #if ENABLED(LIN_ADVANCE)
    // Linear Advance Control
    float getLinearAdvance_mm_mm_s(const extruder_t);
    void setLinearAdvance_mm_mm_s(const_float_t, const extruder_t);
  #endif

  #if HAS_SHAPING
    float getShapingZeta(const axis_t);
    void setShapingZeta(const float, const axis_t);
    float getShapingFrequency(const axis_t);
    void setShapingFrequency(const float, const axis_t);
  #endif

  // JD or Jerk Control
  #if HAS_JUNCTION_DEVIATION
    float getJunctionDeviation_mm();
    void setJunctionDeviation_mm(const_float_t);
  #else
    float getAxisMaxJerk_mm_s(const axis_t);
    float getAxisMaxJerk_mm_s(const extruder_t);
    void setAxisMaxJerk_mm_s(const_float_t, const axis_t);
    void setAxisMaxJerk_mm_s(const_float_t, const extruder_t);
  #endif

  // Tool Changing
  extruder_t getTool(const uint8_t extruder);
  extruder_t getActiveTool();
  void setActiveTool(const extruder_t, bool no_move);

  // Babystepping (axis, probe offset)
  #if ENABLED(BABYSTEPPING)
    int16_t mmToWholeSteps(const_float_t mm, const axis_t axis);
    float mmFromWholeSteps(int16_t steps, const axis_t axis);

    bool babystepAxis_steps(const int16_t steps, const axis_t axis);
    void smartAdjustAxis_steps(const int16_t steps, const axis_t axis, bool linked_nozzles);
  #endif

  // Hotend Offsets
  #if HAS_HOTEND_OFFSET
    float getNozzleOffset_mm(const axis_t, const extruder_t);
    void setNozzleOffset_mm(const_float_t, const axis_t, const extruder_t);
    void normalizeNozzleOffset(const axis_t axis);
  #endif

  // The Probe Z Offset
  float getZOffset_mm();
  void setZOffset_mm(const_float_t);

  // The Probe XYZ Offset
  #if HAS_BED_PROBE
    float getProbeOffset_mm(const axis_t);
    void setProbeOffset_mm(const_float_t, const axis_t);
    probe_limits_t getBedProbeLimits();
  #endif

  // Backlash Control
  #if ENABLED(BACKLASH_GCODE)
    float getAxisBacklash_mm(const axis_t);
    void setAxisBacklash_mm(const_float_t, const axis_t);

    float getBacklashCorrection_percent();
    void setBacklashCorrection_percent(const_float_t);

    #ifdef BACKLASH_SMOOTHING_MM
      float getBacklashSmoothing_mm();
      void setBacklashSmoothing_mm(const_float_t);
    #endif
  #endif

  // Filament Runout Sensor
  #if HAS_FILAMENT_SENSOR
    bool getFilamentRunoutEnabled();
    void setFilamentRunoutEnabled(const bool);
    bool getFilamentRunoutState();
    void setFilamentRunoutState(const bool);

    #if HAS_FILAMENT_RUNOUT_DISTANCE
      float getFilamentRunoutDistance_mm();
      void setFilamentRunoutDistance_mm(const_float_t);
    #endif
  #endif

  // Case Light Control
  #if ENABLED(CASE_LIGHT_ENABLE)
    bool getCaseLightState();
    void setCaseLightState(const bool);

    #if DISABLED(CASE_LIGHT_NO_BRIGHTNESS)
      float getCaseLightBrightness_percent();
      void setCaseLightBrightness_percent(const_float_t);
    #endif
  #endif

  // Power-Loss Recovery
  #if ENABLED(POWER_LOSS_RECOVERY)
    bool getPowerLossRecoveryEnabled();
    void setPowerLossRecoveryEnabled(const bool);
  #endif

  // Hotend PID
  #if ENABLED(PIDTEMP)
    float getPID_Kp(const extruder_t);
    float getPID_Ki(const extruder_t);
    float getPID_Kd(const extruder_t);
    void setPID(const_float_t, const_float_t , const_float_t , extruder_t);
    void startPIDTune(const celsius_t, extruder_t);
  #endif

  // Bed PID
  #if ENABLED(PIDTEMPBED)
    float getBedPID_Kp();
    float getBedPID_Ki();
    float getBedPID_Kd();
    void setBedPID(const_float_t, const_float_t , const_float_t);
    void startBedPIDTune(const celsius_t);
  #endif

  /**
   * Delay and timing routines
   * Should be used by the EXTENSIBLE_UI to safely pause or measure time
   * safe_millis must be called at least every 1 sec to guarantee time
   * yield should be called within lengthy loops
   */
  #ifdef __SAM3X8E__
    uint32_t safe_millis();
  #else
    FORCE_INLINE uint32_t safe_millis() { return millis(); } // TODO: Implement for AVR
  #endif

  void delay_us(uint32_t us);
  void delay_ms(uint32_t ms);
  void yield();

  /**
   * Media access routines
   * Use these to operate on files
   */
  bool isMediaMounted();
  bool isPrintingFromMediaPaused();
  bool isPrintingFromMedia();
  bool isPrinting();
  bool isPrintingPaused();
  bool isOngoingPrintJob();

  void printFile(const char *filename);
  void stopPrint();
  void pausePrint();
  void resumePrint();

  class FileList {
    public:
      FileList();
      void refresh();
      bool seek(const uint16_t, const bool skip_range_check = false);

      const char *longFilename();
      const char *shortFilename();
      const char *filename();
      bool isDir();

      void changeDir(const char * const dirname);
      void upDir();
      bool isAtRootDir();
      uint16_t count();
  };

  /**
   * Event callback routines
   * Must be defined, and will be called by Marlin as needed
   */
  void onStartup();
  void onIdle();

  void onMediaMounted();
  void onMediaError();
  void onMediaRemoved();

  void onHeatingError(const heater_id_t header_id);
  void onMinTempError(const heater_id_t header_id);
  void onMaxTempError(const heater_id_t header_id);

  void onPlayTone(const uint16_t frequency, const uint16_t duration=0);

  void onPrinterKilled(FSTR_P const error, FSTR_P const component);
  void onSurviveInKilled();

  void onPrintTimerStarted();
  void onPrintTimerPaused();
  void onPrintTimerStopped();
  void onPrintDone();

  void onFilamentRunout(const extruder_t extruder);

  void onUserConfirmRequired(const char * const msg);
  void onUserConfirmRequired(FSTR_P const fstr);

  // For fancy LCDs include an icon ID, message, and translated button title
  void onUserConfirmRequired(const int icon, const char * const cstr, FSTR_P const fBtn);
  void onUserConfirmRequired(const int icon, FSTR_P const fstr, FSTR_P const fBtn);

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    // Standard stdOnPauseMode sets pauseModeStatus and calls onUserConfirmRequired
    extern PauseMessage pauseModeStatus;
    void stdOnPauseMode(const PauseMessage message, const PauseMode mode=PAUSE_MODE_SAME, const uint8_t extruder=active_extruder);
    void onPauseMode(const PauseMessage message, const PauseMode mode=PAUSE_MODE_SAME, const uint8_t extruder=active_extruder);
  #endif

  void onStatusChanged(const char * const msg);
  void onStatusChanged(FSTR_P const fstr);

  void onHomingStart();
  void onHomingDone();

  void onSteppersDisabled();
  void onSteppersEnabled();
  void onAxisDisabled(const axis_t axis);
  void onAxisEnabled(const axis_t axis);

  void onFactoryReset();
  void onStoreSettings(char *);
  void onLoadSettings(const char *);
  void onPostprocessSettings();
  void onSettingsStored(const bool success);
  void onSettingsLoaded(const bool success);

  #if ENABLED(PREVENT_COLD_EXTRUSION)
    void onSetMinExtrusionTemp(const celsius_t t);
  #endif
  #if ENABLED(POWER_LOSS_RECOVERY)
    void onSetPowerLoss(const bool onoff);
    void onPowerLoss();
    void onPowerLossResume();
  #endif
  #if HAS_PID_HEATING
    void onPIDTuning(const pidresult_t rst);
    void onStartM303(const int count, const heater_id_t hid, const celsius_t temp);
  #endif
  #if ENABLED(MPC_AUTOTUNE)
    void onMPCTuning(const mpcresult_t rst);
  #endif
  #if ENABLED(PLATFORM_M997_SUPPORT)
    void onFirmwareFlash();
  #endif
};

/**
 * Helper macros to increment or decrement a value. For example:
 *
 *   UI_INCREMENT_BY(TargetTemp_celsius, 10, E0)
 *
 * Expands to:
 *
 *   setTargetTemp_celsius(getTargetTemp_celsius(E0) + 10, E0);
 *
 * Or, in the case where a constant increment is desired:
 *
 *   constexpr float increment = 10;
 *
 *   UI_INCREMENT(TargetTemp_celsius, E0)
 */
#define UI_INCREMENT_BY(method, inc, ...) ExtUI::set ## method(ExtUI::get ## method (__VA_ARGS__) + inc, ##__VA_ARGS__)
#define UI_DECREMENT_BY(method, inc, ...) ExtUI::set ## method(ExtUI::get ## method (__VA_ARGS__) - inc, ##__VA_ARGS__)

#define UI_INCREMENT(method, ...) UI_INCREMENT_BY(method, increment, ##__VA_ARGS__)
#define UI_DECREMENT(method, ...) UI_DECREMENT_BY(method, increment, ##__VA_ARGS__)
