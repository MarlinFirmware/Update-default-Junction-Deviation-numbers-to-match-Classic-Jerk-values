/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * DWIN Enhanced implementation for PRO UI
 * Based on the original work of: Miguel Risco-Castillo (MRISCOC)
 * https://github.com/mriscoc/Ender3V2S1
 * Version: 3.25.3
 * Date: 2023/05/18
 */

#include "../../../inc/MarlinConfig.h"

#include "dwin_defines.h"
#include "dwinui.h"
#include "../common/encoder.h"
#include "../common/limits.h"
#include "../../../libs/BL24CXX.h"
#if ENABLED(LED_CONTROL_MENU)
  #include "../../../feature/leds/leds.h"
#endif

namespace GET_LANG(LCD_LANGUAGE) {
  #define _MSG_PREHEAT(N) \
    LSTR MSG_PREHEAT_##N                  = _UxGT("Preheat ") PREHEAT_## N ##_LABEL; \
    LSTR MSG_PREHEAT_## N ##_SETTINGS     = _UxGT("Preheat ") PREHEAT_## N ##_LABEL _UxGT(" Conf");
  #if PREHEAT_COUNT > 1
    REPEAT_S(2, INCREMENT(PREHEAT_COUNT), _MSG_PREHEAT)
  #endif
}

enum processID : uint8_t {
  // Process ID
  ID_MainMenu,
  ID_Menu,
  ID_SetInt,
  ID_SetFloat,
  ID_SetPInt,
  ID_SetPFloat,
  ID_SetIntNoDraw,
  ID_PrintProcess,
  ID_Popup,
  ID_Leveling,
  ID_Locked,
  ID_ESDiagProcess,
  ID_PrintDone,
  ID_WaitResponse,
  ID_Homing,
  ID_PIDProcess,
  ID_PlotProcess,
  ID_MPCProcess,
  ID_NothingToDo
};

#if ANY(HAS_PID_HEATING, MPC_AUTOTUNE)

  enum TempControl {
    AUTOTUNE_DONE,
    #if HAS_PID_HEATING
      #if ENABLED(PIDTEMP)
        PIDTEMP_START,
      #endif
      #if ENABLED(PIDTEMPBED)
        PIDTEMPBED_START,
      #endif
      #if ENABLED(PIDTEMPCHAMBER)
        PIDTEMPCHAMBER_START,
      #endif
      PID_BAD_HEATER_ID,
      PID_TEMP_TOO_HIGH,
      PID_TUNING_TIMEOUT,
    #endif
    #if ENABLED(MPC_AUTOTUNE)
      MPC_STARTED,
      MPC_TEMP_ERROR,
      MPC_INTERRUPTED,
    #endif
    TEMPCONTROL_COUNT
  };

  typedef bits_t(TEMPCONTROL_COUNT) tempcontrol_t;

#endif

#define DWIN_CHINESE 123
#define DWIN_ENGLISH 0

typedef struct {
  // Color settings
  uint16_t colorBackground;
  uint16_t colorCursor;
  uint16_t colorTitleBg;
  uint16_t colorTitleTxt;
  uint16_t colorText;
  uint16_t colorSelected;
  uint16_t colorSplitLine;
  uint16_t colorHighlight;
  uint16_t colorStatusBg;
  uint16_t colorStatusTxt;
  uint16_t colorPopupBg;
  uint16_t colorPopupTxt;
  uint16_t colorAlertBg;
  uint16_t colorAlertTxt;
  uint16_t colorPercentTxt;
  uint16_t colorBarfill;
  uint16_t colorIndicator;
  uint16_t colorCoordinate;

  // Temperatures
  #if HAS_PID_HEATING
    int16_t pidCycles = DEF_PIDCYCLES;
    #if ENABLED(PIDTEMP)
      celsius_t hotendPIDT = DEF_HOTENDPIDT;
    #endif
    #if ENABLED(PIDTEMPBED)
      celsius_t bedPIDT = DEF_BEDPIDT;
    #endif
    #if ENABLED(PIDTEMPCHAMBER)
      celsius_t chamberPIDT = DEF_CHAMBERPIDT;
    #endif
  #endif
  #if ENABLED(PREVENT_COLD_EXTRUSION)
    celsius_t extMinT = EXTRUDE_MINTEMP;
  #endif
  #if ENABLED(PREHEAT_BEFORE_LEVELING)
    celsius_t bedLevT = LEVELING_BED_TEMP;
  #endif
  #if ENABLED(BAUD_RATE_GCODE)
    bool baud115K = false;
  #endif
  #if ALL(LCD_BED_TRAMMING, HAS_BED_PROBE)
    bool fullManualTramming = false;
  #endif
  #if ENABLED(PROUI_MEDIASORT)
    bool mediaSort = true;
  #endif
  bool mediaAutoMount = ENABLED(HAS_SD_EXTENDER);
  #if ALL(INDIVIDUAL_AXIS_HOMING_SUBMENU, MESH_BED_LEVELING)
    uint8_t zAfterHoming = DEF_Z_AFTER_HOMING;
    #define Z_POST_CLEARANCE hmiData.zAfterHoming
  #endif
  #if ALL(LED_CONTROL_MENU, HAS_COLOR_LEDS)
    LEDColor ledColor = defColorLeds;
  #endif
  #if HAS_GCODE_PREVIEW
    bool enablePreview = true;
  #endif
} hmi_data_t;

extern hmi_data_t hmiData;

#define EXTUI_EEPROM_DATA_SIZE sizeof(hmi_data_t)

typedef struct {
  int8_t r, g, b;
  void set(int8_t _r, int8_t _g, int8_t _b) { r = _r; g = _g; b = _b; }
  int8_t& operator[](const int i) {
    switch (i) {
      default:
      case 0: return r;
      case 1: return g;
      case 2: return b;
    }
  }
} rgb_t;

typedef struct {
  rgb_t color;                        // Color
  #if ANY(HAS_PID_HEATING, MPCTEMP)
    tempcontrol_t tempControl = AUTOTUNE_DONE;
  #endif
  uint8_t select = 0;                 // Auxiliary selector variable
  AxisEnum axis = X_AXIS;             // Axis Select
} hmi_value_t;

extern hmi_value_t hmiValue;

typedef struct {
  uint8_t language;
  bool printing_flag:1; // sd or host printing
  bool abort_flag:1;    // sd or host was aborted
  bool pause_flag:1;    // printing is paused
  bool select_flag:1;   // Popup button selected
  bool home_flag:1;     // homing in course
} hmi_flag_t;

extern hmi_flag_t hmiFlag;
extern uint8_t checkkey;

// Popups
#if HAS_HOTEND || HAS_HEATED_BED || HAS_HEATED_CHAMBER
  void dwinPopupTemperature(const int_fast8_t heater_id, const uint8_t state);
#endif
#if ENABLED(POWER_LOSS_RECOVERY)
  void popupPowerLossRecovery();
#endif

// Tool Functions
uint32_t getHash(char * str);
#if ENABLED(EEPROM_SETTINGS)
  void writeEEPROM();
  void readEEPROM();
  void resetEEPROM();
  #if HAS_MESH
    void saveMesh();
  #endif
#endif
void rebootPrinter();
void disableMotors();
void autoLevel();
void autoHome();
#if HAS_PREHEAT
  #define _DOPREHEAT(N) void DoPreheat##N();
  REPEAT_1(PREHEAT_COUNT, _DOPREHEAT)
#endif
void doCoolDown();
#if ENABLED(BAUD_RATE_GCODE)
  void hmiSetBaudRate();
  void setBaud115K();
  void setBaud250K();
#endif
#if HAS_LCD_BRIGHTNESS
  void turnOffBacklight();
#endif
#if ENABLED(PREVENT_COLD_EXTRUSION)
  void applyExtMinT();
#endif
#if ENABLED(NOZZLE_PARK_FEATURE)
  void parkHead();
#endif
#if HAS_ONESTEP_LEVELING
  void trammingwizard();
#endif
#if ALL(LED_CONTROL_MENU, HAS_COLOR_LEDS)
  void applyLEDColor();
#endif
#if ENABLED(AUTO_BED_LEVELING_UBL)
  void ublMeshTilt();
  void ublMeshSave();
  void ublMeshLoad();
#endif
#if DISABLED(HAS_BED_PROBE)
  void homeZAndDisable();
#endif

// Other
void gotoPrintProcess();
void gotoMainMenu();
void gotoInfoMenu();
void gotoPowerLossRecovery();
void gotoConfirmToPrint();
void dwinDrawDashboard(); // Status Area
void drawMainArea();      // Redraw main area
void dwinDrawStatusLine(const char *text = ""); // Draw simple status text
void dwinRedrawDash();     // Redraw Dash and Status line
void dwinRedrawScreen();   // Redraw all screen elements
void hmiMainMenu();        // Main process screen
void hmiPrinting();        // Print page
void hmiReturnScreen();    // Return to previous screen before popups
void hmiWaitForUser();
void hmiSaveProcessID(const uint8_t id);
void hmiSDCardUpdate();
void eachMomentUpdate();
void updateVariable();
void dwinInitScreen();
void dwinHandleScreen();
void dwinCheckStatusMessage();
void dwinHomingStart();
void dwinHomingDone();
#if HAS_MESH
  void dwinMeshUpdate(const int8_t cpos, const int8_t tpos, const_float_t zval);
#endif
void dwinLevelingStart();
void dwinLevelingDone();
void dwinPrintStarted();
void dwinPrintPause();
void dwinPrintResume();
void dwinPrintFinished();
void dwinPrintAborted();
#if HAS_FILAMENT_SENSOR
  void dwinFilamentRunout(const uint8_t extruder);
#endif
void dwinPrintHeader(const char * const cstr=nullptr);
void dwinSetColorDefaults();
void dwinCopySettingsTo(char * const buff);
void dwinCopySettingsFrom(const char * const buff);
void dwinSetDataDefaults();
void dwinRebootScreen();

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  void dwinPopupPause(FSTR_P const fmsg, uint8_t button=0);
  void drawPopupFilamentPurge();
  void gotoFilamentPurge();
#endif

// Utility and extensions
#if HAS_LOCKSCREEN
  void dwinLockScreen();
  void dwinUnLockScreen();
  void hmiLockScreen();
#endif
#if HAS_MESH
  void dwinMeshViewer();
#endif
#if HAS_ESDIAG
  void drawEndStopDiag();
#endif
#if ALL(PROUI_TUNING_GRAPH, PROUI_ITEM_PLOT)
  void dwinDrawPlot(tempcontrol_t result);
  void drawHPlot();
  void drawBPlot();
#endif

// Menu drawing functions
void drawPrintFileMenu();
void drawControlMenu();
void drawAdvancedSettingsMenu();
void drawPrepareMenu();
void drawMoveMenu();
void drawTrammingMenu();
#if HAS_HOME_OFFSET
  void drawHomeOffsetMenu();
#endif
#if HAS_BED_PROBE
  void drawProbeSetMenu();
#endif
void drawFilSetMenu();
#if ALL(CASE_LIGHT_MENU, CASELIGHT_USES_BRIGHTNESS)
  void drawCaseLightMenu();
#endif
#if ENABLED(LED_CONTROL_MENU)
  void drawLedControlMenu();
#endif
void drawTuneMenu();
void drawMotionMenu();
void drawFilamentManMenu();
#if ENABLED(MESH_BED_LEVELING)
  void drawManualMeshMenu();
#endif
void drawTemperatureMenu();
void drawMaxSpeedMenu();
void drawMaxAccelMenu();
#if ENABLED(CLASSIC_JERK)
  void drawMaxJerkMenu();
#endif
#if ENABLED(EDITABLE_STEPS_PER_UNIT)
  void drawStepsMenu();
#endif
#if ANY(HAS_BED_PROBE, BABYSTEPPING)
  void drawZOffsetWizMenu();
#endif
#if ENABLED(INDIVIDUAL_AXIS_HOMING_SUBMENU)
  void drawHomingMenu();
#endif
#if ENABLED(EDITABLE_HOMING_FEEDRATE)
  void drawHomingFRMenu();
#endif
#if ENABLED(FWRETRACT)
  void drawFWRetractMenu();
#endif
#if HAS_MESH
  void drawMeshSetMenu();
  #if ENABLED(PROUI_MESH_EDIT)
    void drawEditMeshMenu();
  #endif
#endif
#if HAS_TRINAMIC_CONFIG
  void drawTrinamicConfigMenu();
#endif

// Custom colors editing
#if HAS_CUSTOM_COLORS
  void dwinApplyColor();
  void drawSelectColorsMenu();
  void drawGetColorMenu();
#endif

// PID
#if HAS_PID_HEATING
  #include "../../../module/temperature.h"
  void dwinStartM303(const int count, const heater_id_t hid, const celsius_t temp);
  void dwinPIDTuning(tempcontrol_t result);
  #if ANY(PID_AUTOTUNE_MENU, PID_EDIT_MENU)
    #if ENABLED(PIDTEMP)
      #if ENABLED(PID_AUTOTUNE_MENU)
        void hotendPID();
      #endif
      void drawHotendPIDMenu();
    #endif
    #if ENABLED(PIDTEMPBED)
      #if ENABLED(PID_AUTOTUNE_MENU)
        void bedPID();
      #endif
      void drawBedPIDMenu();
    #endif
    #if ENABLED(PIDTEMPCHAMBER)
      #if ENABLED(PID_AUTOTUNE_MENU)
        void chamberPID();
      #endif
      void drawChamberPIDMenu();
    #endif
  #endif
#endif

// MPC
#if ENABLED(MPCTEMP)
  #if ANY(MPC_EDIT_MENU, MPC_AUTOTUNE_MENU)
    void drawHotendMPCMenu();
  #endif
  #if ENABLED(MPC_AUTOTUNE)
    void dwinMPCTuning(tempcontrol_t result);
  #endif
#endif

#if PROUI_TUNING_GRAPH
  void dwinDrawPIDMPCPopup();
#endif
