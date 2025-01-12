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
 * feature/powerloss.cpp - Resume an SD print after power-loss
 */

#include "../inc/MarlinConfigPre.h"

#if ENABLED(POWER_LOSS_RECOVERY)

#include "../inc/MarlinConfig.h"

#include "powerloss.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extui/ui_api.h"
#endif

bool PrintJobRecovery::enabled; // Initialized by settings.load

#if HAS_PLR_BED_THRESHOLD
  celsius_t PrintJobRecovery::bed_temp_threshold; // Initialized by settings.load
#endif

MediaFile PrintJobRecovery::file;
job_recovery_info_t PrintJobRecovery::info;
const char PrintJobRecovery::filename[5] = "/PLR";
uint8_t PrintJobRecovery::queue_index_r;
uint32_t PrintJobRecovery::cmd_sdpos, // = 0
         PrintJobRecovery::sdpos[BUFSIZE];

#if HAS_PLR_UI_FLAG
  bool PrintJobRecovery::ui_flag_resume; // = false
#endif

#include "../sd/cardreader.h"
#include "../lcd/marlinui.h"
#include "../gcode/queue.h"
#include "../gcode/gcode.h"
#include "../module/motion.h"
#include "../module/planner.h"
#include "../module/printcounter.h"
#include "../module/temperature.h"

#if HOMING_Z_WITH_PROBE
  #include "../module/probe.h"
#endif

#if ENABLED(SOVOL_SV06_RTS)
  #include "../lcd/sovol_rts/sovol_rts.h"
#endif

#if ENABLED(FWRETRACT)
  #include "fwretract.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_POWER_LOSS_RECOVERY)
#include "../core/debug_out.h"

PrintJobRecovery recovery;

#if DISABLED(BACKUP_POWER_SUPPLY)
  #undef POWER_LOSS_RETRACT_LEN   // No retract at outage without backup power
#endif
#ifndef POWER_LOSS_RETRACT_LEN
  #define POWER_LOSS_RETRACT_LEN 0
#endif
#ifndef POWER_LOSS_PURGE_LEN
  #define POWER_LOSS_PURGE_LEN 0
#endif

// Allow power-loss recovery to be aborted
#define PLR_CAN_ABORT
#define PROCESS_SUBCOMMANDS_NOW(cmd) do{ \
    if (TERN0(PLR_CAN_ABORT, card.flag.abort_sd_printing)) return; \
    gcode.process_subcommands_now(cmd); \
  }while(0)

/**
 * Clear the recovery info
 */
void PrintJobRecovery::init() { info = {}; }

/**
 * Enable or disable then call changed()
 */
void PrintJobRecovery::enable(const bool onoff) {
  enabled = onoff;
  changed();
}

/**
 * The enabled state was changed:
 *  - Enabled: Purge the job recovery file
 *  - Disabled: Write the job recovery file
 */
void PrintJobRecovery::changed() {
  if (!enabled)
    purge();
  else if (IS_SD_PRINTING())
    save(true);
  TERN_(EXTENSIBLE_UI, ExtUI::onSetPowerLoss(enabled));
}

/**
 * Check for Print Job Recovery during setup()
 *
 * If a saved state exists send 'M1000 S' to initiate job recovery.
 */
bool PrintJobRecovery::check() {
  //if (!card.isMounted()) card.mount();
  bool success = false;
  if (card.isMounted()) {
    load();
    success = valid();
    if (!success)
      cancel();
    else
      queue.inject(F("M1000S"));
  }
  return success;
}

/**
 * Delete the recovery file and clear the recovery data
 */
void PrintJobRecovery::purge() {
  init();
  card.removeJobRecoveryFile();
}

/**
 * Load the recovery data, if it exists
 */
void PrintJobRecovery::load() {
  if (exists()) {
    open(true);
    (void)file.read(&info, sizeof(info));
    close();
  }
  debug(F("Load"));
}

/**
 * Set info fields that won't change
 */
void PrintJobRecovery::prepare() {
  card.getAbsFilenameInCWD(info.sd_filename);  // SD filename
  cmd_sdpos = 0;
}

/**
 * Save the current machine state to the power-loss recovery file
 */
void PrintJobRecovery::save(const bool force/*=false*/, const float zraise/*=POWER_LOSS_ZRAISE*/, const bool raised/*=false*/) {

  // We don't check IS_SD_PRINTING here so a save may occur during a pause

  #if SAVE_INFO_INTERVAL_MS > 0
    static millis_t next_save_ms; // = 0
    millis_t ms = millis();
  #endif

  #ifndef POWER_LOSS_MIN_Z_CHANGE
    #define POWER_LOSS_MIN_Z_CHANGE 0.05  // Vase-mode-friendly out of the box
  #endif

  // Did Z change since the last call?
  if (force
    #if DISABLED(SAVE_EACH_CMD_MODE)      // Always save state when enabled
      #if SAVE_INFO_INTERVAL_MS > 0       // Save if interval is elapsed
        || ELAPSED(ms, next_save_ms)
      #endif
      // Save if Z is above the last-saved position by some minimum height
      || current_position.z > info.current_position.z + POWER_LOSS_MIN_Z_CHANGE
    #endif
  ) {

    #if SAVE_INFO_INTERVAL_MS > 0
      next_save_ms = ms + SAVE_INFO_INTERVAL_MS;
    #endif

    // Set Head and Foot to matching non-zero values
    if (!++info.valid_head) ++info.valid_head; // non-zero in sequence
    //if (!IS_SD_PRINTING()) info.valid_head = 0;
    info.valid_foot = info.valid_head;

    // Machine state
    // info.sdpos and info.current_position are pre-filled from the Stepper ISR

    info.feedrate = uint16_t(MMS_TO_MMM(feedrate_mm_s));
    info.feedrate_percentage = feedrate_percentage;
    COPY(info.flow_percentage, planner.flow_percentage);

    info.zraise = zraise;
    info.flag.raised = raised;                      // Was Z raised before power-off?

    TERN_(CANCEL_OBJECTS, info.cancel_state = cancelable.state);
    TERN_(GCODE_REPEAT_MARKERS, info.stored_repeat = repeat);
    TERN_(HAS_HOME_OFFSET, info.home_offset = home_offset);
    TERN_(HAS_WORKSPACE_OFFSET, info.workspace_offset = workspace_offset);
    E_TERN_(info.active_extruder = active_extruder);

    #if DISABLED(NO_VOLUMETRICS)
      info.flag.volumetric_enabled = parser.volumetric_enabled;
      #if HAS_MULTI_EXTRUDER
        COPY(info.filament_size, planner.filament_size);
      #else
        if (parser.volumetric_enabled) info.filament_size[0] = planner.filament_size[active_extruder];
      #endif
    #endif

    #if HAS_HOTEND
      HOTEND_LOOP() info.target_temperature[e] = thermalManager.degTargetHotend(e);
    #endif

    TERN_(HAS_HEATED_BED, info.target_temperature_bed = thermalManager.degTargetBed());

    TERN_(HAS_HEATED_CHAMBER, info.target_temperature_chamber = thermalManager.degTargetChamber());

    TERN_(HAS_FAN, COPY(info.fan_speed, thermalManager.fan_speed));

    #if HAS_LEVELING
      info.flag.leveling = planner.leveling_active;
      info.fade = TERN0(ENABLE_LEVELING_FADE_HEIGHT, planner.z_fade_height);
    #endif

    TERN_(GRADIENT_MIX, memcpy(&info.gradient, &mixer.gradient, sizeof(info.gradient)));

    #if ENABLED(FWRETRACT)
      COPY(info.retract, fwretract.current_retract);
      info.retract_hop = fwretract.current_hop;
    #endif

    // Elapsed print job time
    info.print_job_elapsed = print_job_timer.duration();

    // Relative axis modes
    info.axis_relative = gcode.axis_relative;

    // Misc. Marlin flags
    info.flag.dryrun = !!(marlin_debug_flags & MARLIN_DEBUG_DRYRUN);
    info.flag.allow_cold_extrusion = TERN0(PREVENT_COLD_EXTRUSION, thermalManager.allow_cold_extrude);

    write();
  }
}

#if PIN_EXISTS(POWER_LOSS)

  #if ENABLED(BACKUP_POWER_SUPPLY)

    void PrintJobRecovery::retract_and_lift(const_float_t zraise) {
      #if POWER_LOSS_RETRACT_LEN || POWER_LOSS_ZRAISE

        gcode.set_relative_mode(true);  // Use relative coordinates

        #if POWER_LOSS_RETRACT_LEN
          // Retract filament now
          const uint16_t old_flow = planner.flow_percentage[active_extruder];
          planner.set_flow(active_extruder, 100);
          gcode.process_subcommands_now(F("G1F3000E-" STRINGIFY(POWER_LOSS_RETRACT_LEN)));
          planner.set_flow(active_extruder, old_flow);
        #endif

        #if POWER_LOSS_ZRAISE
          // Raise the Z axis now
          if (zraise)
            gcode.process_subcommands_now(TS(F("G0Z"), p_float_t(zraise, 3)));
        #else
          UNUSED(zraise);
        #endif

        //gcode.axis_relative = info.axis_relative;
        planner.synchronize();
      #endif
    }

  #endif

#endif // POWER_LOSS_PIN

#if PIN_EXISTS(POWER_LOSS) || ENABLED(DEBUG_POWER_LOSS_RECOVERY)

  /**
   * An outage was detected by a sensor pin.
   *  - If not SD printing, let the machine turn off on its own with no "KILL" screen
   *  - Disable all heaters first to save energy
   *  - Save the recovery data for the current instant
   *  - If backup power is available Retract E and Raise Z
   *  - Go to the KILL screen
   */
  void PrintJobRecovery::_outage(TERN_(DEBUG_POWER_LOSS_RECOVERY, const bool simulated/*=false*/)) {
    #if ENABLED(BACKUP_POWER_SUPPLY)
      static bool lock = false;
      if (lock) return; // No re-entrance from idle() during retract_and_lift()
      lock = true;
    #endif

    #if POWER_LOSS_ZRAISE
      // Get the limited Z-raise to do now or on resume
      const float zraise = _MAX(0, _MIN(current_position.z + POWER_LOSS_ZRAISE, Z_MAX_POS - 1) - current_position.z);
    #else
      constexpr float zraise = 0;
    #endif

    // Save the current position, distance that Z was (or should be) raised,
    // and a flag whether the raise was already done here.
    if (IS_SD_PRINTING()) save(true, zraise, ENABLED(BACKUP_POWER_SUPPLY));

    // Tell the LCD about the outage, even though it is about to die
    TERN_(EXTENSIBLE_UI, ExtUI::onPowerLoss());

    // Disable all heaters to reduce power loss
    thermalManager.disable_all_heaters();

    #if ENABLED(BACKUP_POWER_SUPPLY)
      // Do a hard-stop of the steppers (with possibly a loud thud)
      quickstop_stepper();
      // With backup power a retract and raise can be done now
      retract_and_lift(zraise);
    #endif

    if (TERN0(DEBUG_POWER_LOSS_RECOVERY, simulated)) {
      card.fileHasFinished();
      current_position.reset();
      sync_plan_position();
    }
    else
      kill(GET_TEXT_F(MSG_OUTAGE_RECOVERY));
  }

#endif // POWER_LOSS_PIN || DEBUG_POWER_LOSS_RECOVERY

/**
 * Save the recovery info the recovery file
 */
void PrintJobRecovery::write() {

  debug(F("Write"));

  open(false);
  file.seekSet(0);
  const int16_t ret = file.write(&info, sizeof(info));
  if (ret == -1) DEBUG_ECHOLNPGM("Power-loss file write failed.");
  if (!file.close()) DEBUG_ECHOLNPGM("Power-loss file close failed.");
}

/**
 * Resume the saved print job
 */
void PrintJobRecovery::resume() {
  // Get these fields before any moves because stepper.cpp overwrites them
  const xyze_pos_t resume_pos = info.current_position;
  const uint32_t resume_sdpos = info.sdpos;

  // Apply the dry-run flag if enabled
  if (info.flag.dryrun) marlin_debug_flags |= MARLIN_DEBUG_DRYRUN;

  #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
    struct OnExit {
      uint8_t old_flags;
      OnExit() {
        old_flags = marlin_debug_flags;
        marlin_debug_flags |= MARLIN_DEBUG_ECHO;
      }
      ~OnExit() { marlin_debug_flags = old_flags; }
    } on_exit;
  #endif

  // Restore cold extrusion permission
  TERN_(PREVENT_COLD_EXTRUSION, thermalManager.allow_cold_extrude = info.flag.allow_cold_extrusion);

  #if HAS_LEVELING
    // Make sure leveling is off before any G92 and G28
    PROCESS_SUBCOMMANDS_NOW(F("M420S0"));
  #endif

  #if HAS_HEATED_CHAMBER
    // Restore the chamber temperature
    const celsius_t ct = info.target_temperature_chamber;
    if (ct) PROCESS_SUBCOMMANDS_NOW(TS(F("M191S"), ct));
  #endif

  #if HAS_HEATED_BED
    // Restore the bed temperature
    const celsius_t bt = info.target_temperature_bed;
    if (bt) PROCESS_SUBCOMMANDS_NOW(TS(F("M190S"), bt));
  #endif

  // Heat hotend enough to soften material
  #if HAS_HOTEND
    HOTEND_LOOP() {
      const celsius_t et = _MAX(info.target_temperature[e], 180);
      if (et) {
        TERN_(HAS_MULTI_HOTEND, PROCESS_SUBCOMMANDS_NOW(TS('T', e, 'S')));
        PROCESS_SUBCOMMANDS_NOW(TS(F("M109S"), et));
      }
    }
  #endif

  // Interpret the saved Z according to flags
  const float z_print = resume_pos.z,
              z_raised = z_print + info.zraise;

  //
  // Home the axes that can safely be homed, and
  // establish the current position as best we can.
  //

  PROCESS_SUBCOMMANDS_NOW(F("G92.9E0")); // Reset E to 0

  #if Z_HOME_TO_MAX

    float z_now = z_raised;

    // If Z homing goes to max then just move back to the "raised" position
    PROCESS_SUBCOMMANDS_NOW(TS(
      F( "G28R0\n"    // Home all axes (no raise)
         "G1F1200Z")  // Move Z down to (raised) height
      , p_float_t(z_now, 3)
    ));

  #elif DISABLED(BELTPRINTER)

    #if ENABLED(POWER_LOSS_RECOVER_ZHOME) && defined(POWER_LOSS_ZHOME_POS)
      #define HOMING_Z_DOWN 1
    #endif

    float z_now = info.flag.raised ? z_raised : z_print;

    #if !HOMING_Z_DOWN
      // Set Z to the real position
      PROCESS_SUBCOMMANDS_NOW(TS(F("G92.9Z"), p_float_t(z_now, 3)));
    #endif

    // Does Z need to be raised now? It should be raised before homing XY.
    if (z_raised > z_now) {
      z_now = z_raised;
      PROCESS_SUBCOMMANDS_NOW(TS(F("G1F600Z"), p_float_t(z_now, 3)));
    }

    // Home XY with no Z raise
    PROCESS_SUBCOMMANDS_NOW(F("G28R0XY")); // No raise during G28

  #endif

  #if HOMING_Z_DOWN
    // Move to a safe XY position and home Z while avoiding the print.
    const xy_pos_t p = xy_pos_t(POWER_LOSS_ZHOME_POS) TERN_(HOMING_Z_WITH_PROBE, - probe.offset_xy);
    PROCESS_SUBCOMMANDS_NOW(TS(F("G1F1000X"), p_float_t(p.x, 3), 'Y', p_float_t(p.y, 3), F("\nG28HZ")));
  #endif

  // Mark all axes as having been homed (no effect on current_position)
  set_all_homed();

  #if HAS_LEVELING
    // Restore Z fade and possibly re-enable bed leveling compensation.
    // Leveling may already be enabled due to the ENABLE_LEVELING_AFTER_G28 option.
    // TODO: Add a G28 parameter to leave leveling disabled.
    PROCESS_SUBCOMMANDS_NOW(TS(F("M420S"), '0' + (char)info.flag.leveling, 'Z', p_float_t(info.fade, 1)));

    #if !HOMING_Z_DOWN
      // The physical Z was adjusted at power-off so undo the M420S1 correction to Z with G92.9.
      PROCESS_SUBCOMMANDS_NOW(TS(F("G92.9Z"), p_float_t(z_now, 1)));
    #endif
  #endif

  #if ENABLED(POWER_LOSS_RECOVER_ZHOME)
    // Z was homed down to the bed, so move up to the raised height.
    z_now = z_raised;
    PROCESS_SUBCOMMANDS_NOW(TS(F("G1F600Z"), p_float_t(z_now, 3)));
  #endif

  // Recover volumetric extrusion state
  #if DISABLED(NO_VOLUMETRICS)
    #if HAS_MULTI_EXTRUDER
      EXTRUDER_LOOP()
        PROCESS_SUBCOMMANDS_NOW(TS(F("M200T"), e, F("D"), p_float_t(info.filament_size[e], 3)));
      if (!info.flag.volumetric_enabled)
        PROCESS_SUBCOMMANDS_NOW(TS(F("M200D0T"), info.active_extruder));
    #else
      if (info.flag.volumetric_enabled)
        PROCESS_SUBCOMMANDS_NOW(TS(F("M200D"), p_float_t(info.filament_size[0], 3)));
    #endif
  #endif

  // Restore all hotend temperatures
  #if HAS_HOTEND
    HOTEND_LOOP() {
      const celsius_t et = info.target_temperature[e];
      if (et) {
        TERN_(HAS_MULTI_HOTEND, PROCESS_SUBCOMMANDS_NOW(TS('T', e, 'S')));
        PROCESS_SUBCOMMANDS_NOW(TS(F("M109S"), et));
      }
    }
  #endif

  // Restore the previously active tool (with no_move)
  #if HAS_MULTI_EXTRUDER || HAS_MULTI_HOTEND
    PROCESS_SUBCOMMANDS_NOW(TS('T', info.active_extruder, 'S'));
  #endif

  // Restore print cooling fan speeds
  #if HAS_FAN
    FANS_LOOP(i) {
      const int f = info.fan_speed[i];
      if (f) PROCESS_SUBCOMMANDS_NOW(TS(F("M106P"), i, 'S', f));
    }
  #endif

  // Restore retract and hop state from an active `G10` command
  #if ENABLED(FWRETRACT)
    EXTRUDER_LOOP() {
      if (info.retract[e] != 0.0) {
        fwretract.current_retract[e] = info.retract[e];
        fwretract.retracted.set(e);
      }
    }
    fwretract.current_hop = info.retract_hop;
  #endif

  #if ENABLED(GRADIENT_MIX)
    memcpy(&mixer.gradient, &info.gradient, sizeof(info.gradient));
  #endif

  // Un-retract if there was a retract at outage
  #if ENABLED(BACKUP_POWER_SUPPLY) && POWER_LOSS_RETRACT_LEN > 0
    PROCESS_SUBCOMMANDS_NOW(F("G1F3000E" STRINGIFY(POWER_LOSS_RETRACT_LEN)));
  #endif

  // Additional purge on resume if configured
  #if POWER_LOSS_PURGE_LEN
    PROCESS_SUBCOMMANDS_NOW(TS(F("G1F3000E"), (POWER_LOSS_PURGE_LEN) + (POWER_LOSS_RETRACT_LEN)));
  #endif

  #if ENABLED(NOZZLE_CLEAN_FEATURE)
    PROCESS_SUBCOMMANDS_NOW(F("G12"));
  #endif

  // Move back over to the saved XY
  PROCESS_SUBCOMMANDS_NOW(TS(
    F("G1F3000X"), p_float_t(resume_pos.x, 3), 'Y', p_float_t(resume_pos.y, 3)
  ));

  // Move back down to the saved Z for printing
  PROCESS_SUBCOMMANDS_NOW(TS(F("G1F600Z"), p_float_t(z_print, 3)));

  // Restore the feedrate and percentage
  PROCESS_SUBCOMMANDS_NOW(TS(F("G1F"), info.feedrate));
  feedrate_percentage = info.feedrate_percentage;

  // Flowrate percentage
  EXTRUDER_LOOP() planner.set_flow(e, info.flow_percentage[e]);

  // Restore E position with G92.9
  PROCESS_SUBCOMMANDS_NOW(TS(F("G92.9E"), p_float_t(resume_pos.e, 3)));

  #if ENABLED(CANCEL_OBJECTS)
    cancelable.state = info.cancel_state;
    cancelable.set_active_object(); // Sets the status message
  #endif

  TERN_(GCODE_REPEAT_MARKERS, repeat = info.stored_repeat);
  TERN_(HAS_HOME_OFFSET, home_offset = info.home_offset);
  TERN_(HAS_WORKSPACE_OFFSET, workspace_offset = info.workspace_offset);

  // Relative axis modes
  gcode.axis_relative = info.axis_relative;

  // Continue to apply PLR when a file is resumed!
  enable(true);

  // Resume the SD file from the last position
  PROCESS_SUBCOMMANDS_NOW(MString<MAX_CMD_SIZE>(F("M23 "), info.sd_filename));
  PROCESS_SUBCOMMANDS_NOW(TS(F("M24S"), resume_sdpos, 'T', info.print_job_elapsed));

  #if ENABLED(SOVOL_SV06_RTS)
    if (rts.print_state) rts.refreshTime();
    rts.start_print_flag = false;
  #endif
}

#if ENABLED(DEBUG_POWER_LOSS_RECOVERY)

  void PrintJobRecovery::debug(FSTR_P const prefix) {
    DEBUG_ECHOLN(prefix, F(" Job Recovery Info...\nvalid_head:"), info.valid_head, F(" valid_foot:"), info.valid_foot);
    if (info.valid_head) {
      if (info.valid_head == info.valid_foot) {
        DEBUG_ECHOPGM("current_position: ");
        LOOP_LOGICAL_AXES(i) {
          if (i) DEBUG_CHAR(',');
          DEBUG_ECHO(info.current_position[i]);
        }
        DEBUG_EOL();

        DEBUG_ECHOLN(F("feedrate: "), info.feedrate, F(" x "), info.feedrate_percentage, '%');
        EXTRUDER_LOOP() DEBUG_ECHOLN('E', e + 1, F(" flow %: "), info.flow_percentage[e]);

        DEBUG_ECHOLNPGM("zraise: ", info.zraise, " ", info.flag.raised ? "(before)" : "");

        #if ENABLED(CANCEL_OBJECTS)
          const cancel_state_t cs = info.cancel_state;
          DEBUG_ECHOPGM("Canceled:");
          for (int i = 0; i < cs.object_count; i++)
            if (TEST(cs.canceled, i)) { DEBUG_CHAR(' '); DEBUG_ECHO(i); }
          DEBUG_EOL();
        #endif

        #if ENABLED(GCODE_REPEAT_MARKERS)
          const uint8_t ind = info.stored_repeat.count();
          DEBUG_ECHOLNPGM("repeat markers: ", ind);
          for (uint8_t i = ind; i--;)
            DEBUG_ECHOLNPGM("...", i, " sdpos: ", info.stored_repeat.get_marker_sdpos(i), " count: ", info.stored_repeat.get_marker_counter(i));
        #endif

        #if HAS_HOME_OFFSET
          DEBUG_ECHOPGM("home_offset: ");
          LOOP_NUM_AXES(i) {
            if (i) DEBUG_CHAR(',');
            DEBUG_ECHO(info.home_offset[i]);
          }
          DEBUG_EOL();
        #endif

        #if HAS_WORKSPACE_OFFSET
          DEBUG_ECHOPGM("workspace_offset: ");
          LOOP_NUM_AXES(i) {
            if (i) DEBUG_CHAR(',');
            DEBUG_ECHO(info.workspace_offset[i]);
          }
          DEBUG_EOL();
        #endif

        #if HAS_MULTI_EXTRUDER
          DEBUG_ECHOLNPGM("active_extruder: ", info.active_extruder);
        #endif

        #if DISABLED(NO_VOLUMETRICS)
          DEBUG_ECHOPGM("filament_size:");
          EXTRUDER_LOOP() DEBUG_ECHOLNPGM(" ", info.filament_size[e]);
          DEBUG_EOL();
        #endif

        #if HAS_HOTEND
          DEBUG_ECHOPGM("target_temperature: ");
          HOTEND_LOOP() {
            DEBUG_ECHO(info.target_temperature[e]);
            if (e < HOTENDS - 1) DEBUG_CHAR(',');
          }
          DEBUG_EOL();
        #endif

        #if HAS_HEATED_BED
          DEBUG_ECHOLNPGM("target_temperature_bed: ", info.target_temperature_bed);
        #endif

        #if HAS_HEATED_CHAMBER
          DEBUG_ECHOLNPGM("target_temperature_chamber: ", info.target_temperature_chamber);
        #endif

        #if HAS_FAN
          DEBUG_ECHOPGM("fan_speed: ");
          FANS_LOOP(i) {
            DEBUG_ECHO(info.fan_speed[i]);
            if (i < FAN_COUNT - 1) DEBUG_CHAR(',');
          }
          DEBUG_EOL();
        #endif

        #if HAS_LEVELING
          DEBUG_ECHOLNPGM("leveling: ", info.flag.leveling ? "ON" : "OFF", "  fade: ", info.fade);
        #endif

        #if ENABLED(FWRETRACT)
          DEBUG_ECHOPGM("retract: ");
          EXTRUDER_LOOP() {
            DEBUG_ECHO(info.retract[e]);
            if (e < EXTRUDERS - 1) DEBUG_CHAR(',');
          }
          DEBUG_EOL();
          DEBUG_ECHOLNPGM("retract_hop: ", info.retract_hop);
        #endif

        // Mixing extruder and gradient
        #if ALL(MIXING_EXTRUDER, GRADIENT_MIX)
          DEBUG_ECHOLNPGM("gradient: ", info.gradient.enabled ? "ON" : "OFF");
        #endif

        DEBUG_ECHOLNPGM("sd_filename: ", info.sd_filename);
        DEBUG_ECHOLNPGM("sdpos: ", info.sdpos);
        DEBUG_ECHOLNPGM("print_job_elapsed: ", info.print_job_elapsed);

        DEBUG_ECHOPGM("axis_relative:");
        if (TEST(info.axis_relative, REL_X)) DEBUG_ECHOPGM(" REL_X");
        if (TEST(info.axis_relative, REL_Y)) DEBUG_ECHOPGM(" REL_Y");
        if (TEST(info.axis_relative, REL_Z)) DEBUG_ECHOPGM(" REL_Z");
        if (TEST(info.axis_relative, REL_E)) DEBUG_ECHOPGM(" REL_E");
        if (TEST(info.axis_relative, E_MODE_ABS)) DEBUG_ECHOPGM(" E_MODE_ABS");
        if (TEST(info.axis_relative, E_MODE_REL)) DEBUG_ECHOPGM(" E_MODE_REL");
        DEBUG_EOL();

        DEBUG_ECHOLNPGM("flag.dryrun: ", AS_DIGIT(info.flag.dryrun));
        DEBUG_ECHOLNPGM("flag.allow_cold_extrusion: ", AS_DIGIT(info.flag.allow_cold_extrusion));
        #if DISABLED(NO_VOLUMETRICS)
          DEBUG_ECHOLNPGM("flag.volumetric_enabled: ", AS_DIGIT(info.flag.volumetric_enabled));
        #endif
      }
      else
        DEBUG_ECHOLNPGM("INVALID DATA");
    }
    DEBUG_ECHOLNPGM("---");
  }

#endif // DEBUG_POWER_LOSS_RECOVERY

#endif // POWER_LOSS_RECOVERY
