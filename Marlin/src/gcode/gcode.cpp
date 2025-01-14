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
 * gcode.cpp - Temporary container for all G-code handlers
 *             Most will migrate to classes, by feature.
 */

#include "gcode.h"
GcodeSuite gcode;

#if ENABLED(WIFI_CUSTOM_COMMAND)
  extern bool wifi_custom_command(char * const command_ptr);
#endif

#include "parser.h"
#include "queue.h"
#include "../module/motion.h"

#if ENABLED(PRINTCOUNTER)
  #include "../module/printcounter.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../feature/host_actions.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../sd/cardreader.h"
  #include "../feature/powerloss.h"
#endif

#if ENABLED(CANCEL_OBJECTS)
  #include "../feature/cancel_object.h"
#endif

#if ENABLED(LASER_FEATURE)
  #include "../feature/spindle_laser.h"
#endif

#if ENABLED(FLOWMETER_SAFETY)
  #include "../feature/cooler.h"
#endif

#if ENABLED(PASSWORD_FEATURE)
  #include "../feature/password/password.h"
#endif

#if HAS_FANCHECK
  #include "../feature/fancheck.h"
#endif

#include "../MarlinCore.h" // for idle, kill

// Inactivity shutdown
millis_t GcodeSuite::previous_move_ms = 0,
         GcodeSuite::max_inactive_time = 0;

#if HAS_DISABLE_IDLE_AXES
  millis_t GcodeSuite::stepper_inactive_time = SEC_TO_MS(DEFAULT_STEPPER_TIMEOUT_SEC);
#endif

// Relative motion mode for each logical axis
relative_t GcodeSuite::axis_relative; // Init in constructor

#if ANY(HAS_AUTO_REPORTING, HOST_KEEPALIVE_FEATURE)
  bool GcodeSuite::autoreport_paused; // = false
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  GcodeSuite::MarlinBusyState GcodeSuite::busy_state = NOT_BUSY;
  uint8_t GcodeSuite::host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
#endif

#if ENABLED(CNC_WORKSPACE_PLANES)
  GcodeSuite::WorkspacePlane GcodeSuite::workspace_plane = PLANE_XY;
#endif

#if ENABLED(CNC_COORDINATE_SYSTEMS)
  int8_t GcodeSuite::active_coordinate_system = -1; // machine space
  xyz_pos_t GcodeSuite::coordinate_system[MAX_COORDINATE_SYSTEMS];
#endif

void GcodeSuite::report_echo_start(const bool forReplay) { if (!forReplay) SERIAL_ECHO_START(); }
void GcodeSuite::report_heading(const bool forReplay, FSTR_P const fstr, const bool eol/*=true*/) {
  if (forReplay) return;
  if (fstr) {
    SERIAL_ECHO_START();
    SERIAL_ECHO(F("; "), fstr);
  }
  if (eol) { SERIAL_CHAR(':'); SERIAL_EOL(); }
}

void GcodeSuite::say_units() {
  SERIAL_ECHOLNPGM_P(
    TERN_(INCH_MODE_SUPPORT, parser.linear_unit_factor != 1.0 ? PSTR(" (in)") :)
    PSTR(" (mm)")
  );
}

/**
 * Get the target extruder from the T parameter or the active_extruder
 * Return -1 if the T parameter is out of range
 */
int8_t GcodeSuite::get_target_extruder_from_command() {
  #if HAS_TOOLCHANGE
    if (parser.seenval('T')) {
      const int8_t e = parser.value_byte();
      if (e < EXTRUDERS) return e;
      SERIAL_ECHO_START();
      SERIAL_CHAR('M'); SERIAL_ECHO(parser.codenum);
      SERIAL_ECHOLNPGM(" " STR_INVALID_EXTRUDER " ", e);
      return -1;
    }
  #endif
  return active_extruder;
}

/**
 * Get the target E stepper from the 'T' parameter.
 * If there is no 'T' parameter then dval will be substituted.
 * Returns -1 if the resulting E stepper index is out of range.
 */
int8_t GcodeSuite::get_target_e_stepper_from_command(const int8_t dval/*=-1*/) {
  const int8_t e = parser.intval('T', dval);
  if (WITHIN(e, 0, E_STEPPERS - 1)) return e;
  if (dval == -2) return dval;

  SERIAL_ECHO_START();
  SERIAL_CHAR('M'); SERIAL_ECHO(parser.codenum);
  if (e == -1)
    SERIAL_ECHOLNPGM(" " STR_E_STEPPER_NOT_SPECIFIED);
  else
    SERIAL_ECHOLNPGM(" " STR_INVALID_E_STEPPER " ", e);
  return -1;
}

/**
 * Set XYZ...E destination and feedrate from the current G-Code command
 *
 *  - Set destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the feedrate, if included
 */
void GcodeSuite::get_destination_from_command() {
  xyze_bool_t seen{false};

  #if ENABLED(CANCEL_OBJECTS)
    const bool &skip_move = cancelable.state.skipping;
  #else
    constexpr bool skip_move = false;
  #endif

  // Get new XYZ position, whether absolute or relative
  LOOP_NUM_AXES(i) {
    if ( (seen[i] = parser.seenval(AXIS_CHAR(i))) ) {
      const float v = parser.value_axis_units((AxisEnum)i);
      if (skip_move)
        destination[i] = current_position[i];
      else
        destination[i] = axis_is_relative(AxisEnum(i)) ? current_position[i] + v : LOGICAL_TO_NATIVE(v, i);
    }
    else
      destination[i] = current_position[i];
  }

  #if HAS_EXTRUDERS
    // Get new E position, whether absolute or relative
    if ( (seen.e = parser.seenval('E')) ) {
      const float v = parser.value_axis_units(E_AXIS);
      destination.e = axis_is_relative(E_AXIS) ? current_position.e + v : v;
    }
    else
      destination.e = current_position.e;
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY) && !PIN_EXISTS(POWER_LOSS)
    // Only update power loss recovery on moves with E
    if (recovery.enabled && IS_SD_PRINTING() && seen.e && (seen.x || seen.y))
      recovery.save();
  #endif

  if (parser.floatval('F') > 0) {
    const float fr_mm_min = parser.value_linear_units();
    feedrate_mm_s = MMM_TO_MMS(fr_mm_min);
    // Update the cutter feed rate for use by M4 I set inline moves.
    TERN_(LASER_FEATURE, cutter.feedrate_mm_m = fr_mm_min);
  }

  #if ALL(PRINTCOUNTER, HAS_EXTRUDERS)
    if (!DEBUGGING(DRYRUN) && !skip_move)
      print_job_timer.incFilamentUsed(destination.e - current_position.e);
  #endif

  // Get ABCDHI mixing factors
  #if ALL(MIXING_EXTRUDER, DIRECT_MIXING_IN_G1)
    M165();
  #endif

  #if ENABLED(LASER_FEATURE)
    if (cutter.cutter_mode == CUTTER_MODE_CONTINUOUS || cutter.cutter_mode == CUTTER_MODE_DYNAMIC) {
      // Set the cutter power in the planner to configure this move
      cutter.last_feedrate_mm_m = 0;
      if (WITHIN(parser.codenum, 1, TERN(ARC_SUPPORT, 3, 1)) || TERN0(BEZIER_CURVE_SUPPORT, parser.codenum == 5)) {
        planner.laser_inline.status.isPowered = true;
        if (parser.seen('I')) cutter.set_enabled(true);       // This is set for backward LightBurn compatibility.
        if (parser.seenval('S')) {
          const float v = parser.value_float(),
                      u = TERN(LASER_POWER_TRAP, v, cutter.power_to_range(v));
          cutter.menuPower = cutter.unitPower = u;
          cutter.inline_power(TERN(SPINDLE_LASER_USE_PWM, cutter.upower_to_ocr(u), u > 0 ? 255 : 0));
        }
      }
      else if (parser.codenum == 0) {
        // For dynamic mode we need to flag isPowered off, dynamic power is calculated in the stepper based on feedrate.
        if (cutter.cutter_mode == CUTTER_MODE_DYNAMIC) planner.laser_inline.status.isPowered = false;
        cutter.inline_power(0); // This is planner-based so only set power and do not disable inline control flags.
      }
    }
    else if (parser.codenum == 0)
      cutter.apply_power(0);
  #endif // LASER_FEATURE
}

/**
 * Dwell waits immediately. It does not synchronize. Use M400 instead of G4
 */
void GcodeSuite::dwell(millis_t time) {
  time += millis();
  while (PENDING(millis(), time)) idle();
}

/**
 * When G29_RETRY_AND_RECOVER is enabled, call G29() in
 * a loop with recovery and retry handling.
 */
#if ENABLED(G29_RETRY_AND_RECOVER)

  void GcodeSuite::event_probe_recover() {
    TERN_(HOST_PROMPT_SUPPORT, hostui.prompt_do(PROMPT_INFO, F("G29 Retrying"), FPSTR(DISMISS_STR)));
    #ifdef ACTION_ON_G29_RECOVER
      hostui.g29_recover();
    #endif
    #ifdef G29_RECOVER_COMMANDS
      process_subcommands_now(F(G29_RECOVER_COMMANDS));
    #endif
  }

  #if ENABLED(G29_HALT_ON_FAILURE)
    #include "../lcd/marlinui.h"
  #endif

  void GcodeSuite::event_probe_failure() {
    #ifdef ACTION_ON_G29_FAILURE
      hostui.g29_failure();
    #endif
    #ifdef G29_FAILURE_COMMANDS
      process_subcommands_now(F(G29_FAILURE_COMMANDS));
    #endif
    #if ENABLED(G29_HALT_ON_FAILURE)
      #ifdef ACTION_ON_CANCEL
        hostui.cancel();
      #endif
      kill(GET_TEXT_F(MSG_LCD_PROBING_FAILED));
    #endif
  }

  #ifndef G29_MAX_RETRIES
    #define G29_MAX_RETRIES 0
  #endif

  void GcodeSuite::G29_with_retry() {
    uint8_t retries = G29_MAX_RETRIES;
    bool fail = false;
    for (;;) {
      fail = G29(); // G29 should return true for failed probes ONLY
      if (!fail || !retries) break;
      event_probe_recover();
      --retries;
    }

    if (fail) {
      event_probe_failure();
      TERN_(G29_HALT_ON_FAILURE, return);
    }
    else {
      TERN_(HOST_PROMPT_SUPPORT, hostui.prompt_end());
      #ifdef G29_SUCCESS_COMMANDS
        process_subcommands_now(F(G29_SUCCESS_COMMANDS));
      #endif
    }

    TERN_(HAS_DWIN_E3V2_BASIC, dwinLevelingDone());
    TERN_(EXTENSIBLE_UI, ExtUI::onLevelingDone());
  }

#endif // G29_RETRY_AND_RECOVER

/**
 * Process the parsed command and dispatch it to its handler
 */
void GcodeSuite::process_parsed_command(const bool no_ok/*=false*/) {
  TERN_(HAS_FANCHECK, fan_check.check_deferred_error());

  KEEPALIVE_STATE(IN_HANDLER);

 /**
  * Block all Gcodes except M511 Unlock Printer, if printer is locked
  * Will still block Gcodes if M511 is disabled, in which case the printer should be unlocked via LCD Menu
  */
  #if ENABLED(PASSWORD_FEATURE)
    if (password.is_locked && !parser.is_command('M', 511)) {
      SERIAL_ECHO_MSG(STR_PRINTER_LOCKED);
      if (!no_ok) queue.ok_to_send();
      return;
    }
  #endif

  #if ENABLED(FLOWMETER_SAFETY)
    if (cooler.flowfault) {
      SERIAL_ECHO_MSG(STR_FLOWMETER_FAULT);
      return;
    }
  #endif

  // Handle a known command or reply "unknown command"

  switch (parser.command_letter) {

    case 'G': switch (parser.codenum) {

      case 0: case 1:                                             // G0: Fast Move, G1: Linear Move
        G0_G1(TERN_(HAS_FAST_MOVES, parser.codenum == 0)); break;

      #if ENABLED(ARC_SUPPORT)
        case 2: case 3: G2_G3(parser.codenum == 2); break;        // G2: CW ARC, G3: CCW ARC
      #endif

      case 4: G4(); break;                                        // G4: Dwell

      #if ENABLED(BEZIER_CURVE_SUPPORT)
        case 5: G5(); break;                                      // G5: Cubic B_spline
      #endif

      #if ENABLED(DIRECT_STEPPING)
        case 6: G6(); break;                                      // G6: Direct Stepper Move
      #endif

      #if ENABLED(FWRETRACT)
        case 10: G10(); break;                                    // G10: Retract / Swap Retract
        case 11: G11(); break;                                    // G11: Recover / Swap Recover
      #endif

      #if ENABLED(NOZZLE_CLEAN_FEATURE)
        case 12: G12(); break;                                    // G12: Nozzle Clean
      #endif

      #if ENABLED(CNC_WORKSPACE_PLANES)
        case 17: G17(); break;                                    // G17: Select Plane XY
        case 18: G18(); break;                                    // G18: Select Plane ZX
        case 19: G19(); break;                                    // G19: Select Plane YZ
      #endif

      #if ENABLED(INCH_MODE_SUPPORT)
        case 20: G20(); break;                                    // G20: Inch Mode
        case 21: G21(); break;                                    // G21: MM Mode
      #else
        case 21: NOOP; break;                                     // No error on unknown G21
      #endif

      #if ENABLED(G26_MESH_VALIDATION)
        case 26: G26(); break;                                    // G26: Mesh Validation Pattern generation
      #endif

      #if ENABLED(NOZZLE_PARK_FEATURE)
        case 27: G27(); break;                                    // G27: Nozzle Park
      #endif

      case 28: G28(); break;                                      // G28: Home one or more axes

      #if HAS_LEVELING
        case 29:                                                  // G29: Bed leveling calibration
          TERN(G29_RETRY_AND_RECOVER, G29_with_retry, G29)();
          break;
      #endif

      #if HAS_BED_PROBE
        case 30: G30(); break;                                    // G30: Single Z probe
        #if ENABLED(Z_PROBE_SLED)
          case 31: G31(); break;                                  // G31: dock the sled
          case 32: G32(); break;                                  // G32: undock the sled
        #endif
      #endif

      #if ENABLED(DELTA_AUTO_CALIBRATION)
        case 33: G33(); break;                                    // G33: Delta Auto-Calibration
      #endif

      #if ANY(Z_MULTI_ENDSTOPS, Z_STEPPER_AUTO_ALIGN, MECHANICAL_GANTRY_CALIBRATION)
        case 34: G34(); break;                                    // G34: Z Stepper automatic alignment using probe
      #endif

      #if ENABLED(ASSISTED_TRAMMING)
        case 35: G35(); break;                                    // G35: Read four bed corners to help adjust bed screws
      #endif

      #if ENABLED(G38_PROBE_TARGET)
        case 38:                                                  // G38.2, G38.3: Probe towards target
          if (WITHIN(parser.subcode, 2, TERN(G38_PROBE_AWAY, 5, 3)))
            G38(parser.subcode);                                  // G38.4, G38.5: Probe away from target
          break;
      #endif

      #if HAS_MESH
        case 42: G42(); break;                                    // G42: Coordinated move to a mesh point
      #endif

      #if ENABLED(CNC_COORDINATE_SYSTEMS)
        case 53: G53(); break;                                    // G53: (prefix) Apply native workspace
        case 54: G54(); break;                                    // G54: Switch to Workspace 1
        case 55: G55(); break;                                    // G55: Switch to Workspace 2
        case 56: G56(); break;                                    // G56: Switch to Workspace 3
        case 57: G57(); break;                                    // G57: Switch to Workspace 4
        case 58: G58(); break;                                    // G58: Switch to Workspace 5
        case 59: G59(); break;                                    // G59.0 - G59.3: Switch to Workspace 6-9
      #endif

      #if SAVED_POSITIONS
        case 60: G60(); break;                                    // G60:  save current position
        case 61: G61(); break;                                    // G61:  Apply/restore saved coordinates.
      #endif

      #if ALL(PTC_PROBE, PTC_BED)
        case 76: G76(); break;                                    // G76: Calibrate first layer compensation values
      #endif

      #if ENABLED(GCODE_MOTION_MODES)
        case 80: G80(); break;                                    // G80: Reset the current motion mode
      #endif

      case 90: set_relative_mode(false); break;                   // G90: Absolute Mode
      case 91: set_relative_mode(true);  break;                   // G91: Relative Mode

      case 92: G92(); break;                                      // G92: Set current axis position(s)

      #if ENABLED(CALIBRATION_GCODE)
        case 425: G425(); break;                                  // G425: Perform calibration with calibration cube
      #endif

      #if ENABLED(DEBUG_GCODE_PARSER)
        case 800: parser.debug(); break;                          // G800: G-Code Parser Test for G
      #endif

      default: parser.unknown_command_warning(); break;
    }
    break;

    case 'M': switch (parser.codenum) {

      #if HAS_RESUME_CONTINUE
        case 0:                                                   // M0: Unconditional stop - Wait for user button press on LCD
        case 1: M0_M1(); break;                                   // M1: Conditional stop - Wait for user button press on LCD
      #endif

      #if HAS_CUTTER
        case 3: M3_M4(false); break;                              // M3: Turn ON Laser | Spindle (clockwise), set Power | Speed
        case 4: M3_M4(true ); break;                              // M4: Turn ON Laser | Spindle (counter-clockwise), set Power | Speed
        case 5: M5(); break;                                      // M5: Turn OFF Laser | Spindle
      #endif

      #if ENABLED(COOLANT_MIST)
        case 7: M7(); break;                                      // M7: Coolant Mist ON
      #endif

      #if ANY(AIR_ASSIST, COOLANT_FLOOD)
        case 8: M8(); break;                                      // M8: Air Assist / Coolant Flood ON
      #endif

      #if ANY(AIR_ASSIST, COOLANT_CONTROL)
        case 9: M9(); break;                                      // M9: Air Assist / Coolant OFF
      #endif

      #if ENABLED(AIR_EVACUATION)
        case 10: M10(); break;                                    // M10: Vacuum or Blower motor ON
        case 11: M11(); break;                                    // M11: Vacuum or Blower motor OFF
      #endif

      #if ENABLED(EXTERNAL_CLOSED_LOOP_CONTROLLER)
        case 12: M12(); break;                                    // M12: Synchronize and optionally force a CLC set
      #endif

      #if ENABLED(EXPECTED_PRINTER_CHECK)
        case 16: M16(); break;                                    // M16: Expected printer check
      #endif

      case 17: M17(); break;                                      // M17: Enable all stepper motors

      #if HAS_MEDIA
        case 20: M20(); break;                                    // M20: List SD card
        case 21: M21(); break;                                    // M21: Init SD card
        case 22: M22(); break;                                    // M22: Release SD card
        case 23: M23(); break;                                    // M23: Select file
        case 24: M24(); break;                                    // M24: Start SD print
        case 25: M25(); break;                                    // M25: Pause SD print
        case 26: M26(); break;                                    // M26: Set SD index
        case 27: M27(); break;                                    // M27: Get SD status
        case 28: M28(); break;                                    // M28: Start SD write
        case 29: M29(); break;                                    // M29: Stop SD write
        case 30: M30(); break;                                    // M30 <filename> Delete File

        #if HAS_MEDIA_SUBCALLS
          case 32: M32(); break;                                  // M32: Select file and start SD print
        #endif

        #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
          case 33: M33(); break;                                  // M33: Get the long full path to a file or folder
        #endif

        #if ALL(SDCARD_SORT_ALPHA, SDSORT_GCODE)
          case 34: M34(); break;                                  // M34: Set SD card sorting options
        #endif

        case 928: M928(); break;                                  // M928: Start SD write
      #endif // HAS_MEDIA

      case 31: M31(); break;                                      // M31: Report time since the start of SD print or last M109

      #if ENABLED(DIRECT_PIN_CONTROL)
        case 42: M42(); break;                                    // M42: Change pin state
      #endif

      #if ENABLED(PINS_DEBUGGING)
        case 43: M43(); break;                                    // M43: Read pin state
      #endif

      #if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
        case 48: M48(); break;                                    // M48: Z probe repeatability test
      #endif

      #if ENABLED(SET_PROGRESS_MANUALLY)
        case 73: M73(); break;                                    // M73: Set progress percentage
      #endif

      case 75: M75(); break;                                      // M75: Start print timer
      case 76: M76(); break;                                      // M76: Pause print timer
      case 77: M77(); break;                                      // M77: Stop print timer

      #if ENABLED(PRINTCOUNTER)
        case 78: M78(); break;                                    // M78: Show print statistics
      #endif

      #if ENABLED(M100_FREE_MEMORY_WATCHER)
        case 100: M100(); break;                                  // M100: Free Memory Report
      #endif

      #if ENABLED(BD_SENSOR)
        case 102: M102(); break;                                  // M102: Configure Bed Distance Sensor
      #endif

      #if HAS_HOTEND
        case 104: M104(); break;                                  // M104: Set hot end temperature
        case 109: M109(); break;                                  // M109: Wait for hotend temperature to reach target
      #endif

      case 105: M105(); return;                                   // M105: Report Temperatures (and say "ok")

      #if HAS_FAN
        case 106: M106(); break;                                  // M106: Fan On
        case 107: M107(); break;                                  // M107: Fan Off
      #endif

      case 110: M110(); break;                                    // M110: Set Current Line Number
      case 111: M111(); break;                                    // M111: Set debug level

      #if DISABLED(EMERGENCY_PARSER)
        case 108: M108(); break;                                  // M108: Cancel Waiting
        case 112: M112(); break;                                  // M112: Full Shutdown
        case 410: M410(); break;                                  // M410: Quickstop - Abort all the planned moves.
        #if ENABLED(HOST_PROMPT_SUPPORT)
          case 876: M876(); break;                                // M876: Handle Host prompt responses
        #endif
      #else
        case 108: case 112: case 410:
        TERN_(HOST_PROMPT_SUPPORT, case 876:)
        break;
      #endif

      #if ENABLED(HOST_KEEPALIVE_FEATURE)
        case 113: M113(); break;                                  // M113: Set Host Keepalive interval
      #endif

      #if HAS_FANCHECK
        case 123: M123(); break;                                  // M123: Report fan states or set fans auto-report interval
      #endif

      #if HAS_HEATED_BED
        case 140: M140(); break;                                  // M140: Set bed temperature
        case 190: M190(); break;                                  // M190: Wait for bed temperature to reach target
      #endif

      #if HAS_HEATED_CHAMBER
        case 141: M141(); break;                                  // M141: Set chamber temperature
        case 191: M191(); break;                                  // M191: Wait for chamber temperature to reach target
      #endif

      #if HAS_TEMP_PROBE
        case 192: M192(); break;                                  // M192: Wait for probe temp
      #endif

      #if HAS_COOLER
        case 143: M143(); break;                                  // M143: Set cooler temperature
        case 193: M193(); break;                                  // M193: Wait for cooler temperature to reach target
      #endif

      #if ENABLED(AUTO_REPORT_POSITION)
        case 154: M154(); break;                                  // M154: Set position auto-report interval
      #endif

      #if ALL(AUTO_REPORT_TEMPERATURES, HAS_TEMP_SENSOR)
        case 155: M155(); break;                                  // M155: Set temperature auto-report interval
      #endif

      #if ENABLED(PARK_HEAD_ON_PAUSE)
        case 125: M125(); break;                                  // M125: Store current position and move to filament change position
      #endif

      #if ENABLED(BARICUDA)
        // PWM for HEATER_1_PIN
        #if HAS_HEATER_1
          case 126: M126(); break;                                // M126: valve open
          case 127: M127(); break;                                // M127: valve closed
        #endif

        // PWM for HEATER_2_PIN
        #if HAS_HEATER_2
          case 128: M128(); break;                                // M128: valve open
          case 129: M129(); break;                                // M129: valve closed
        #endif
      #endif // BARICUDA

      #if ENABLED(PSU_CONTROL)
        case 80: M80(); break;                                    // M80: Turn on Power Supply
      #endif
      case 81: M81(); break;                                      // M81: Turn off Power, including Power Supply, if possible

      #if HAS_EXTRUDERS
        case 82: M82(); break;                                    // M82: Set E axis normal mode (same as other axes)
        case 83: M83(); break;                                    // M83: Set E axis relative mode
      #endif

      case 18: case 84: M18_M84(); break;                         // M18/M84: Disable Steppers / Set Timeout
      case 85: M85(); break;                                      // M85: Set inactivity stepper shutdown timeout

      #if ENABLED(HOTEND_IDLE_TIMEOUT)
        case 86: M86(); break;                                    // M86: Set Hotend Idle Timeout
        case 87: M87(); break;                                    // M87: Cancel Hotend Idle Timeout
      #endif

      #if ENABLED(EDITABLE_STEPS_PER_UNIT)
        case 92: M92(); break;                                    // M92: Set the steps-per-unit for one or more axes
      #endif

      case 114: M114(); break;                                    // M114: Report current position

      #if ENABLED(CAPABILITIES_REPORT)
        case 115: M115(); break;                                  // M115: Report capabilities
      #endif

      case 117: TERN_(HAS_STATUS_MESSAGE, M117()); break;         // M117: Set LCD message text, if possible

      case 118: M118(); break;                                    // M118: Display a message in the host console
      case 119: M119(); break;                                    // M119: Report endstop states
      case 120: M120(); break;                                    // M120: Enable endstops
      case 121: M121(); break;                                    // M121: Disable endstops

      #if HAS_PREHEAT
        case 145: M145(); break;                                  // M145: Set material heatup parameters
      #endif

      #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
        case 149: M149(); break;                                  // M149: Set temperature units
      #endif

      #if HAS_COLOR_LEDS
        case 150: M150(); break;                                  // M150: Set Status LED Color
      #endif

      #if ENABLED(MIXING_EXTRUDER)
        case 163: M163(); break;                                  // M163: Set a component weight for mixing extruder
        case 164: M164(); break;                                  // M164: Save current mix as a virtual extruder
        #if ENABLED(DIRECT_MIXING_IN_G1)
          case 165: M165(); break;                                // M165: Set multiple mix weights
        #endif
        #if ENABLED(GRADIENT_MIX)
          case 166: M166(); break;                                // M166: Set Gradient Mix
        #endif
      #endif

      #if DISABLED(NO_VOLUMETRICS)
        case 200: M200(); break;                                  // M200: Set filament diameter, E to cubic units
      #endif

      case 201: M201(); break;                                    // M201: Set max acceleration for print moves (units/s^2)

      #if 0
        case 202: M202(); break;                                  // M202: Not used for Sprinter/grbl gen6
      #endif

      case 203: M203(); break;                                    // M203: Set max feedrate (units/sec)
      case 204: M204(); break;                                    // M204: Set acceleration
      case 205: M205(); break;                                    // M205: Set advanced settings

      #if HAS_HOME_OFFSET
        case 206: M206(); break;                                  // M206: Set home offsets
      #endif

      #if ENABLED(FWRETRACT)
        case 207: M207(); break;                                  // M207: Set Retract Length, Feedrate, and Z lift
        case 208: M208(); break;                                  // M208: Set Recover (unretract) Additional Length and Feedrate
        #if ENABLED(FWRETRACT_AUTORETRACT)
          case 209:
            if (MIN_AUTORETRACT <= MAX_AUTORETRACT) M209();       // M209: Turn Automatic Retract Detection on/off
            break;
        #endif
      #endif

      #if ENABLED(EDITABLE_HOMING_FEEDRATE)
        case 210: M210(); break;                                  // M210: Set the homing feedrate
      #endif

      #if HAS_SOFTWARE_ENDSTOPS
        case 211: M211(); break;                                  // M211: Enable, Disable, and/or Report software endstops
      #endif

      #if HAS_MULTI_EXTRUDER
        case 217: M217(); break;                                  // M217: Set filament swap parameters
      #endif

      #if HAS_HOTEND_OFFSET
        case 218: M218(); break;                                  // M218: Set a tool offset
      #endif

      case 220: M220(); break;                                    // M220: Set Feedrate Percentage: S<percent> ("FR" on your LCD)

      #if HAS_EXTRUDERS
        case 221: M221(); break;                                  // M221: Set Flow Percentage
      #endif

      #if ENABLED(DIRECT_PIN_CONTROL)
        case 226: M226(); break;                                  // M226: Wait until a pin reaches a state
      #endif

      #if HAS_SERVOS
        case 280: M280(); break;                                  // M280: Set servo position absolute
        #if ENABLED(EDITABLE_SERVO_ANGLES)
          case 281: M281(); break;                                // M281: Set servo angles
        #endif
        #if ENABLED(SERVO_DETACH_GCODE)
          case 282: M282(); break;                                // M282: Detach servo
        #endif
      #endif

      #if ENABLED(BABYSTEPPING)
        case 290: M290(); break;                                  // M290: Babystepping
        #if ENABLED(EP_BABYSTEPPING)
          case 293: IF_DISABLED(EMERGENCY_PARSER, M293()); break; // M293: Babystep up
          case 294: IF_DISABLED(EMERGENCY_PARSER, M294()); break; // M294: Babystep down
        #endif
      #endif

      #if HAS_SOUND
        case 300: M300(); break;                                  // M300: Play beep tone
      #endif

      #if ENABLED(PIDTEMP)
        case 301: M301(); break;                                  // M301: Set hotend PID parameters
      #endif

      #if ENABLED(PIDTEMPBED)
        case 304: M304(); break;                                  // M304: Set bed PID parameters
      #endif

      #if ENABLED(PIDTEMPCHAMBER)
        case 309: M309(); break;                                  // M309: Set chamber PID parameters
      #endif

      #if ENABLED(PHOTO_GCODE)
        case 240: M240(); break;                                  // M240: Trigger a camera
      #endif

      #if HAS_LCD_CONTRAST
        case 250: M250(); break;                                  // M250: Set LCD contrast
      #endif

      #if ENABLED(EDITABLE_DISPLAY_TIMEOUT)
        case 255: M255(); break;                                  // M255: Set LCD Sleep/Backlight Timeout (Minutes)
      #endif

      #if HAS_LCD_BRIGHTNESS
        case 256: M256(); break;                                  // M256: Set LCD brightness
      #endif

      #if ENABLED(EXPERIMENTAL_I2CBUS)
        case 260: M260(); break;                                  // M260: Send data to an i2c slave
        case 261: M261(); break;                                  // M261: Request data from an i2c slave
      #endif

      #if ENABLED(PREVENT_COLD_EXTRUSION)
        case 302: M302(); break;                                  // M302: Allow cold extrudes (set the minimum extrude temperature)
      #endif

      #if HAS_PID_HEATING
        case 303: M303(); break;                                  // M303: PID autotune
      #endif

      #if HAS_USER_THERMISTORS
        case 305: M305(); break;                                  // M305: Set user thermistor parameters
      #endif

      #if ENABLED(MPCTEMP)
        case 306: M306(); break;                                  // M306: MPC autotune
      #endif

      #if ENABLED(REPETIER_GCODE_M360)
        case 360: M360(); break;                                  // M360: Firmware settings
      #endif

      #if ENABLED(MORGAN_SCARA)
        case 360: if (M360()) return; break;                      // M360: SCARA Theta pos1
        case 361: if (M361()) return; break;                      // M361: SCARA Theta pos2
        case 362: if (M362()) return; break;                      // M362: SCARA Psi pos1
        case 363: if (M363()) return; break;                      // M363: SCARA Psi pos2
        case 364: if (M364()) return; break;                      // M364: SCARA Psi pos3 (90 deg to Theta)
      #endif

      #if ANY(EXT_SOLENOID, MANUAL_SOLENOID_CONTROL)
        case 380: M380(); break;                                  // M380: Activate solenoid on active (or specified) extruder
        case 381: M381(); break;                                  // M381: Disable all solenoids or, if MANUAL_SOLENOID_CONTROL, active (or specified) solenoid
      #endif

      case 400: M400(); break;                                    // M400: Finish all moves

      #if HAS_BED_PROBE
        case 401: M401(); break;                                  // M401: Deploy probe
        case 402: M402(); break;                                  // M402: Stow probe
      #endif

      #if HAS_PRUSA_MMU2 || HAS_PRUSA_MMU3
        case 403: M403(); break;
      #endif

      #if ENABLED(FILAMENT_WIDTH_SENSOR)
        case 404: M404(); break;                                  // M404: Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width
        case 405: M405(); break;                                  // M405: Turn on filament sensor for control
        case 406: M406(); break;                                  // M406: Turn off filament sensor for control
        case 407: M407(); break;                                  // M407: Display measured filament diameter
      #endif

      #if HAS_FILAMENT_SENSOR
        case 412: M412(); break;                                  // M412: Enable/Disable filament runout detection
      #endif

      #if HAS_MULTI_LANGUAGE
        case 414: M414(); break;                                  // M414: Select multi language menu
      #endif

      #if HAS_LEVELING
        case 420: M420(); break;                                  // M420: Enable/Disable Bed Leveling
      #endif

      #if HAS_MESH
        case 421: M421(); break;                                  // M421: Set a Mesh Bed Leveling Z coordinate
      #endif

      #if ENABLED(X_AXIS_TWIST_COMPENSATION)
        case 423: M423(); break;                                  // M423: Reset, modify, or report X-Twist Compensation data
      #endif

      #if ENABLED(BACKLASH_GCODE)
        case 425: M425(); break;                                  // M425: Tune backlash compensation
      #endif

      #if HAS_HOME_OFFSET
        case 428: M428(); break;                                  // M428: Apply current_position to home_offset
      #endif

      #if HAS_POWER_MONITOR
        case 430: M430(); break;                                  // M430: Read the system current (A), voltage (V), and power (W)
      #endif

      #if HAS_RS485_SERIAL
        case 485: M485(); break;                                  // M485: Send RS485 packets
      #endif

      #if ENABLED(CANCEL_OBJECTS)
        case 486: M486(); break;                                  // M486: Identify and cancel objects
      #endif

      #if ENABLED(FT_MOTION)
        case 493: M493(); break;                                  // M493: Fixed-Time Motion control
      #endif

      case 500: M500(); break;                                    // M500: Store settings in EEPROM
      case 501: M501(); break;                                    // M501: Read settings from EEPROM
      case 502: M502(); break;                                    // M502: Revert to default settings
      #if DISABLED(DISABLE_M503)
        case 503: M503(); break;                                  // M503: print settings currently in memory
      #endif
      #if ENABLED(EEPROM_SETTINGS)
        case 504: M504(); break;                                  // M504: Validate EEPROM contents
      #endif

      #if ENABLED(PASSWORD_FEATURE)
        case 510: M510(); break;                                  // M510: Lock Printer
        #if ENABLED(PASSWORD_UNLOCK_GCODE)
          case 511: M511(); break;                                // M511: Unlock Printer
        #endif
        #if ENABLED(PASSWORD_CHANGE_GCODE)
          case 512: M512(); break;                                // M512: Set/Change/Remove Password
        #endif
      #endif

      #if HAS_MEDIA
        case 524: M524(); break;                                  // M524: Abort the current SD print job
      #endif

      #if ENABLED(SD_ABORT_ON_ENDSTOP_HIT)
        case 540: M540(); break;                                  // M540: Set abort on endstop hit for SD printing
      #endif

      #if HAS_ETHERNET
        case 552: M552(); break;                                  // M552: Set IP address
        case 553: M553(); break;                                  // M553: Set gateway
        case 554: M554(); break;                                  // M554: Set netmask
      #endif

      #if ENABLED(BAUD_RATE_GCODE)
        case 575: M575(); break;                                  // M575: Set serial baudrate
      #endif

      #if ENABLED(NONLINEAR_EXTRUSION)
        case 592: M592(); break;                                  // M592: Nonlinear Extrusion control
      #endif

      #if HAS_ZV_SHAPING
        case 593: M593(); break;                                  // M593: Input Shaping control
      #endif

      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        case 600: M600(); break;                                  // M600: Pause for Filament Change
        #if ENABLED(CONFIGURE_FILAMENT_CHANGE)
          case 603: M603(); break;                                  // M603: Configure Filament Change
        #endif
      #endif

      #if HAS_DUPLICATION_MODE
        case 605: M605(); break;                                  // M605: Set Dual X Carriage movement mode
      #endif

      #if IS_KINEMATIC
        case 665: M665(); break;                                  // M665: Set Kinematics parameters
      #endif

      #if ANY(DELTA, HAS_EXTRA_ENDSTOPS)
        case 666: M666(); break;                                  // M666: Set delta or multiple endstop adjustment
      #endif

      #if ENABLED(DUET_SMART_EFFECTOR) && PIN_EXISTS(SMART_EFFECTOR_MOD)
        case 672: M672(); break;                                  // M672: Set/clear Duet Smart Effector sensitivity
      #endif

      #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
        case 701: M701(); break;                                  // M701: Load Filament
        case 702: M702(); break;                                  // M702: Unload Filament
      #endif

      #if HAS_PRUSA_MMU3
        case 704: M704(); break;                                  // M704: Preload to MMU
        case 705: M705(); break;                                  // M705: Eject filament
        case 706: M706(); break;                                  // M706: Cut filament
        case 707: M707(); break;                                  // M707: Read from MMU register
        case 708: M708(); break;                                  // M708: Write to MMU register
        case 709: M709(); break;                                  // M709: MMU power & reset
      #endif

      #if ENABLED(CONTROLLER_FAN_EDITABLE)
        case 710: M710(); break;                                  // M710: Set Controller Fan settings
      #endif

      #if ENABLED(GCODE_MACROS)
        case 810: case 811: case 812: case 813: case 814:
        case 815: case 816: case 817: case 818: case 819:
        M810_819(); break;                                        // M810-M819: Define/execute G-code macro
        case 820: M820(); break;                                  // M820: Report macros to serial output
      #endif

      #if HAS_BED_PROBE
        case 851: M851(); break;                                  // M851: Set Z Probe Z Offset
      #endif

      #if ENABLED(SKEW_CORRECTION_GCODE)
        case 852: M852(); break;                                  // M852: Set Skew factors
      #endif

      #if HAS_PTC
        case 871: M871(); break;                                  // M871: Print/reset/clear first layer temperature offset values
      #endif

      #if ENABLED(LIN_ADVANCE)
        case 900: M900(); break;                                  // M900: Set advance K factor.
      #endif

      #if ANY(HAS_MOTOR_CURRENT_SPI, HAS_MOTOR_CURRENT_PWM, HAS_MOTOR_CURRENT_I2C, HAS_MOTOR_CURRENT_DAC)
        case 907: M907(); break;                                  // M907: Set digital trimpot motor current using axis codes.
        #if ANY(HAS_MOTOR_CURRENT_SPI, HAS_MOTOR_CURRENT_DAC)
          case 908: M908(); break;                                // M908: Control digital trimpot directly.
          #if HAS_MOTOR_CURRENT_DAC
            case 909: M909(); break;                              // M909: Print digipot/DAC current value
            case 910: M910(); break;                              // M910: Commit digipot/DAC value to external EEPROM
          #endif
        #endif
      #endif

      #if HAS_TRINAMIC_CONFIG
        case 122: M122(); break;                                  // M122: Report driver configuration and status
        case 906: M906(); break;                                  // M906: Set motor current in milliamps using axis codes X, Y, Z, E
        #if HAS_STEALTHCHOP
          case 569: M569(); break;                                // M569: Enable stealthChop on an axis.
        #endif
        #if ENABLED(MONITOR_DRIVER_STATUS)
          case 911: M911(); break;                                // M911: Report TMC2130 prewarn triggered flags
          case 912: M912(); break;                                // M912: Clear TMC2130 prewarn triggered flags
        #endif
        #if ENABLED(HYBRID_THRESHOLD)
          case 913: M913(); break;                                // M913: Set HYBRID_THRESHOLD speed.
        #endif
        #if USE_SENSORLESS
          case 914: M914(); break;                                // M914: Set StallGuard sensitivity.
        #endif
        case 919: M919(); break;                                  // M919: Set stepper Chopper Times
      #endif

      #if HAS_MICROSTEPS
        case 350: M350(); break;                                  // M350: Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
        case 351: M351(); break;                                  // M351: Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
      #endif

      #if ENABLED(CASE_LIGHT_ENABLE)
        case 355: M355(); break;                                  // M355: Set case light brightness
      #endif

      #if ENABLED(DEBUG_GCODE_PARSER)
        case 800: parser.debug(); break;                          // M800: G-Code Parser Test for M
      #endif

      #if ENABLED(GCODE_REPEAT_MARKERS)
        case 808: M808(); break;                                  // M808: Set / Goto repeat markers
      #endif

      #if ENABLED(I2C_POSITION_ENCODERS)
        case 860: M860(); break;                                  // M860: Report encoder module position
        case 861: M861(); break;                                  // M861: Report encoder module status
        case 862: M862(); break;                                  // M862: Perform axis test
        case 863: M863(); break;                                  // M863: Calibrate steps/mm
        case 864: M864(); break;                                  // M864: Change module address
        case 865: M865(); break;                                  // M865: Check module firmware version
        case 866: M866(); break;                                  // M866: Report axis error count
        case 867: M867(); break;                                  // M867: Toggle error correction
        case 868: M868(); break;                                  // M868: Set error correction threshold
        case 869: M869(); break;                                  // M869: Report axis error
      #endif

      #if ENABLED(MAGNETIC_PARKING_EXTRUDER)
        case 951: M951(); break;                                  // M951: Set Magnetic Parking Extruder parameters
      #endif

      #if ENABLED(Z_STEPPER_AUTO_ALIGN)
        case 422: M422(); break;                                  // M422: Set Z Stepper automatic alignment position using probe
      #endif

      #if ENABLED(OTA_FIRMWARE_UPDATE)
        case 936: M936(); break;                                  // M936: OTA update firmware.
      #endif

      #if SPI_FLASH_BACKUP
        case 993: M993(); break;                                  // M993: Backup SPI Flash to SD
        case 994: M994(); break;                                  // M994: Load a Backup from SD to SPI Flash
      #endif

      #if ENABLED(TOUCH_SCREEN_CALIBRATION)
        case 995: M995(); break;                                  // M995: Touch screen calibration for TFT display
      #endif

      #if ENABLED(PLATFORM_M997_SUPPORT)
        case 997: M997(); break;                                  // M997: Perform in-application firmware update
      #endif

      case 999: M999(); break;                                    // M999: Restart after being Stopped

      #if ENABLED(POWER_LOSS_RECOVERY)
        case 413: M413(); break;                                  // M413: Enable/disable/query Power-Loss Recovery
        case 1000: M1000(); break;                                // M1000: [INTERNAL] Resume from power-loss
      #endif

      #if HAS_MEDIA
        case 1001: M1001(); break;                                // M1001: [INTERNAL] Handle SD completion
      #endif

      #if DGUS_LCD_UI_MKS
        case 1002: M1002(); break;                                // M1002: [INTERNAL] Tool-change and Relative E Move
      #endif

      #if ENABLED(ONE_CLICK_PRINT)
        case 1003: M1003(); break;                                // M1003: [INTERNAL] Set the current dir to /
      #endif

      #if ENABLED(UBL_MESH_WIZARD)
        case 1004: M1004(); break;                                // M1004: UBL Mesh Wizard
      #endif

      #if ENABLED(MAX7219_GCODE)
        case 7219: M7219(); break;                                // M7219: Set LEDs, columns, and rows
      #endif

      #if ENABLED(HAS_MCP3426_ADC)
        case 3426: M3426(); break;                                // M3426: Read MCP3426 ADC (over i2c)
      #endif

      default: parser.unknown_command_warning(); break;
    }
    break;

    case 'T': T(parser.codenum); break;                           // Tn: Tool Change

    #if ENABLED(MARLIN_DEV_MODE)
      case 'D': D(parser.codenum); break;                         // Dn: Debug codes
    #endif

    #if ENABLED(REALTIME_REPORTING_COMMANDS)
      case 'S': case 'P': case 'R': break;                        // Invalid S, P, R commands already filtered
    #endif

    default:
      #if ENABLED(WIFI_CUSTOM_COMMAND)
        if (wifi_custom_command(parser.command_ptr)) break;
      #endif
      parser.unknown_command_warning();
  }

  if (!no_ok) queue.ok_to_send();

  SERIAL_IMPL.msgDone(); // Call the msgDone serial hook to signal command processing done
}

#if ENABLED(M100_FREE_MEMORY_DUMPER)
  void M100_dump_routine(FSTR_P const title, const char * const start, const uintptr_t size);
#endif

/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void GcodeSuite::process_next_command() {
  GCodeQueue::CommandLine &command = queue.ring_buffer.peek_next_command();

  PORT_REDIRECT(SERIAL_PORTMASK(command.port));

  TERN_(POWER_LOSS_RECOVERY, recovery.queue_index_r = queue.ring_buffer.index_r);

  if (DEBUGGING(ECHO)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOLN(command.buffer);
    #if ENABLED(M100_FREE_MEMORY_DUMPER)
      SERIAL_ECHOPGM("slot:", queue.ring_buffer.index_r);
      M100_dump_routine(F("   Command Queue:"), (const char*)&queue.ring_buffer, sizeof(queue.ring_buffer));
    #endif
  }

  // Parse the next command in the queue
  parser.parse(command.buffer);
  process_parsed_command();
}

#pragma GCC diagnostic push
#if GCC_VERSION >= 80000
  #pragma GCC diagnostic ignored "-Wstringop-truncation"
#endif

/**
 * Run a series of commands, bypassing the command queue to allow
 * G-code "macros" to be called from within other G-code handlers.
 */
void GcodeSuite::process_subcommands_now(FSTR_P fgcode) {
  PGM_P pgcode = FTOP(fgcode);
  char * const saved_cmd = parser.command_ptr;        // Save the parser state
  for (;;) {
    PGM_P const delim = strchr_P(pgcode, '\n');       // Get address of next newline
    const size_t len = delim ? delim - pgcode : strlen_P(pgcode); // Get the command length
    parser.parse(MString<MAX_CMD_SIZE>().setn_P(pgcode, len));    // Parse the command
    process_parsed_command(true);                     // Process it (no "ok")
    if (!delim) break;                                // Last command?
    pgcode = delim + 1;                               // Get the next command
  }
  parser.parse(saved_cmd);                            // Restore the parser state
}

#pragma GCC diagnostic pop

void GcodeSuite::process_subcommands_now(char * gcode) {
  char * const saved_cmd = parser.command_ptr;        // Save the parser state
  for (;;) {
    char * const delim = strchr(gcode, '\n');         // Get address of next newline
    if (delim) *delim = '\0';                         // Replace with nul
    parser.parse(gcode);                              // Parse the current command
    process_parsed_command(true);                     // Process it (no "ok")
    if (!delim) break;                                // Last command?
    *delim = '\n';                                    // Put back the newline
    gcode = delim + 1;                                // Get the next command
  }
  parser.parse(saved_cmd);                            // Restore the parser state
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)

  /**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting commands.
   */
  void GcodeSuite::host_keepalive() {
    const millis_t ms = millis();
    static millis_t next_busy_signal_ms = 0;
    if (!autoreport_paused && host_keepalive_interval && busy_state != NOT_BUSY) {
      if (PENDING(ms, next_busy_signal_ms)) return;
      PORT_REDIRECT(SerialMask::All);
      switch (busy_state) {
        case IN_HANDLER:
        case IN_PROCESS:
          SERIAL_ECHO_MSG(STR_BUSY_PROCESSING);
          TERN_(FULL_REPORT_TO_HOST_FEATURE, report_current_position_moving());
          break;
        case PAUSED_FOR_USER:
          SERIAL_ECHO_MSG(STR_BUSY_PAUSED_FOR_USER);
          TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_HOLD));
          break;
        case PAUSED_FOR_INPUT:
          SERIAL_ECHO_MSG(STR_BUSY_PAUSED_FOR_INPUT);
          TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_HOLD));
          break;
        default:
          break;
      }
    }
    next_busy_signal_ms = ms + SEC_TO_MS(host_keepalive_interval);
  }

#endif // HOST_KEEPALIVE_FEATURE
