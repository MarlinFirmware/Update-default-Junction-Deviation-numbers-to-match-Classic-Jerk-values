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
 * About Marlin
 *
 * This firmware is a mashup between Sprinter and grbl.
 *  - https://github.com/kliment/Sprinter
 *  - https://github.com/grbl/grbl
 */

#include "MarlinCore.h"

#include "HAL/shared/Delay.h"
#include "HAL/shared/esp_wifi.h"
#include "HAL/shared/cpu_exception/exception_hook.h"

#if ENABLED(WIFISUPPORT)
  #include "HAL/shared/esp_wifi.h"
#endif

#ifdef ARDUINO
  #include <pins_arduino.h>
#endif
#include <math.h>

#include "module/endstops.h"
#include "module/motion.h"
#include "module/planner.h"
#include "module/printcounter.h" // PrintCounter or Stopwatch
#include "module/settings.h"
#include "module/stepper.h"
#include "module/temperature.h"
#if ENABLED(FT_MOTION)
  #include "module/ft_motion.h"
#endif

#include "gcode/gcode.h"
#include "gcode/parser.h"
#include "gcode/queue.h"

#include "feature/pause.h"
#include "sd/cardreader.h"

#include "lcd/marlinui.h"
#if HAS_TOUCH_BUTTONS
  #include "lcd/touch/touch_buttons.h"
#endif

#if HAS_TFT_LVGL_UI
  #include "lcd/extui/mks_ui/tft_lvgl_configuration.h"
  #include "lcd/extui/mks_ui/draw_ui.h"
  #include "lcd/extui/mks_ui/mks_hardware.h"
  #include <lvgl.h>
#endif

#if HAS_DWIN_E3V2
  #include "lcd/e3v2/common/encoder.h"
  #if ENABLED(DWIN_CREALITY_LCD)
    #include "lcd/e3v2/creality/dwin.h"
  #elif ENABLED(DWIN_CREALITY_LCD_JYERSUI)
    #include "lcd/e3v2/jyersui/dwin.h"
  #elif ENABLED(SOVOL_SV06_RTS)
    #include "lcd/sovol_rts/sovol_rts.h"
  #endif
#endif

#if HAS_ETHERNET
  #include "feature/ethernet.h"
#endif

#if ENABLED(IIC_BL24CXX_EEPROM)
  #include "libs/BL24CXX.h"
#endif

#if ENABLED(DIRECT_STEPPING)
  #include "feature/direct_stepping.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "feature/host_actions.h"
#endif

#if HAS_BEEPER
  #include "libs/buzzer.h"
#endif

#if ENABLED(EXTERNAL_CLOSED_LOOP_CONTROLLER)
  #include "feature/closedloop.h"
#endif

#if HAS_MOTOR_CURRENT_I2C
  #include "feature/digipot/digipot.h"
#endif

#if ENABLED(MIXING_EXTRUDER)
  #include "feature/mixing.h"
#endif

#if ENABLED(MAX7219_DEBUG)
  #include "feature/max7219.h"
#endif

#if HAS_COLOR_LEDS
  #include "feature/leds/leds.h"
#endif

#if ENABLED(BLTOUCH)
  #include "feature/bltouch.h"
#endif

#if ENABLED(BD_SENSOR)
  #include "feature/bedlevel/bdl/bdl.h"
#endif

#if ENABLED(POLL_JOG)
  #include "feature/joystick.h"
#endif

#if HAS_SERVOS
  #include "module/servo.h"
#endif

#if HAS_MOTOR_CURRENT_DAC
  #include "feature/dac/stepper_dac.h"
#endif

#if ENABLED(EXPERIMENTAL_I2CBUS)
  #include "feature/twibus.h"
#endif

#if ENABLED(I2C_POSITION_ENCODERS)
  #include "feature/encoder_i2c.h"
#endif

#if (HAS_TRINAMIC_CONFIG || HAS_TMC_SPI) && DISABLED(PSU_DEFAULT_OFF)
  #include "feature/tmc_util.h"
#endif

#if HAS_CUTTER
  #include "feature/spindle_laser.h"
#endif

#if HAS_MEDIA
  CardReader card;
#endif

#if ENABLED(G38_PROBE_TARGET)
  uint8_t G38_move; // = 0
  bool G38_did_trigger; // = false
#endif

#if ENABLED(DELTA)
  #include "module/delta.h"
#elif ENABLED(POLARGRAPH)
  #include "module/polargraph.h"
#elif IS_SCARA
  #include "module/scara.h"
#elif ENABLED(POLAR)
  #include "module/polar.h"
#endif

#if HAS_LEVELING
  #include "feature/bedlevel/bedlevel.h"
#endif

#if ENABLED(GCODE_REPEAT_MARKERS)
  #include "feature/repeat.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "feature/powerloss.h"
#endif

#if ENABLED(CANCEL_OBJECTS)
  #include "feature/cancel_object.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "feature/runout.h"
#endif

#if ANY(PROBE_TARE, HAS_Z_SERVO_PROBE)
  #include "module/probe.h"
#endif

#if ENABLED(HOTEND_IDLE_TIMEOUT)
  #include "feature/hotend_idle.h"
#endif

#if ENABLED(TEMP_STAT_LEDS)
  #include "feature/leds/tempstat.h"
#endif

#if ENABLED(CASE_LIGHT_ENABLE)
  #include "feature/caselight.h"
#endif

#if HAS_FANMUX
  #include "feature/fanmux.h"
#endif

#if HAS_TOOLCHANGE
  #include "module/tool_change.h"
#endif

#if HAS_FANCHECK
  #include "feature/fancheck.h"
#endif

#if ENABLED(USE_CONTROLLER_FAN)
  #include "feature/controllerfan.h"
#endif

#if HAS_PRUSA_MMU3
  #include "feature/mmu3/mmu3.h"
  #include "feature/mmu3/mmu3_reporting.h"
  #include "feature/mmu3/SpoolJoin.h"
#elif HAS_PRUSA_MMU2
  #include "feature/mmu/mmu2.h"
#elif HAS_PRUSA_MMU1
  #include "feature/mmu/mmu.h"
#endif

#if ENABLED(PASSWORD_FEATURE)
  #include "feature/password/password.h"
#endif

#if DGUS_LCD_UI_MKS
  #include "lcd/extui/dgus/DGUSScreenHandler.h"
#endif

#if HAS_DRIVER_SAFE_POWER_PROTECT
  #include "feature/stepper_driver_safety.h"
#endif

#if ENABLED(PSU_CONTROL)
  #include "feature/power.h"
#endif

#if ENABLED(EASYTHREED_UI)
  #include "feature/easythreed_ui.h"
#endif

#if ENABLED(MARLIN_TEST_BUILD)
  #include "tests/marlin_tests.h"
#endif

#if HAS_RS485_SERIAL
  #include "feature/rs485.h"
#endif

PGMSTR(M112_KILL_STR, "M112 Shutdown");

MarlinState marlin_state = MarlinState::MF_INITIALIZING;

// For M109 and M190, this flag may be cleared (by M108) to exit the wait loop
bool wait_for_heatup = false;

// For M0/M1, this flag may be cleared (by M108) to exit the wait-for-user loop
#if HAS_RESUME_CONTINUE
  bool wait_for_user; // = false

  void wait_for_user_response(millis_t ms/*=0*/, const bool no_sleep/*=false*/) {
    UNUSED(no_sleep);
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    wait_for_user = true;
    if (ms) ms += millis(); // expire time
    while (wait_for_user && !(ms && ELAPSED(millis(), ms)))
      idle(TERN_(ADVANCED_PAUSE_FEATURE, no_sleep));
    wait_for_user = false;
    while (ui.button_pressed()) safe_delay(50);
  }

#endif

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

/**
 * Stepper Reset (RigidBoard, et.al.)
 */
#if HAS_STEPPER_RESET
  void disableStepperDrivers() { OUT_WRITE(STEPPER_RESET_PIN, LOW); } // Drive down to keep motor driver chips in reset
  void enableStepperDrivers()  { SET_INPUT(STEPPER_RESET_PIN); }      // Set to input, allowing pullups to pull the pin high
#endif

/**
 * Sensitive pin test for M42, M226
 */

#include "pins/sensitive_pins.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
#pragma GCC diagnostic ignored "-Wsign-compare"

bool pin_is_protected(const pin_t pin) {
  #define pgm_read_pin(P) (sizeof(pin_t) == 2 ? (pin_t)pgm_read_word(P) : (pin_t)pgm_read_byte(P))
  for (uint8_t i = 0; i < COUNT(sensitive_dio); ++i)
    if (pin == pgm_read_pin(&sensitive_dio[i])) return true;
  for (uint8_t i = 0; i < COUNT(sensitive_aio); ++i)
    if (pin == analogInputToDigitalPin(pgm_read_pin(&sensitive_aio[i]))) return true;
  return false;
}

#pragma GCC diagnostic pop

bool printer_busy() {
  return planner.has_blocks_queued() || printingIsActive();
}

/**
 * A Print Job exists when the timer is running or SD is printing
 */
bool printJobOngoing() { return print_job_timer.isRunning() || IS_SD_PRINTING(); }

/**
 * Printing is active when a job is underway but not paused
 */
bool printingIsActive() { return !did_pause_print && printJobOngoing(); }

/**
 * Printing is paused according to SD or host indicators
 */
bool printingIsPaused() {
  return did_pause_print || print_job_timer.isPaused() || IS_SD_PAUSED();
}

void startOrResumeJob() {
  if (!printingIsPaused()) {
    TERN_(GCODE_REPEAT_MARKERS, repeat.reset());
    TERN_(CANCEL_OBJECTS, cancelable.reset());
    TERN_(LCD_SHOW_E_TOTAL, e_move_accumulator = 0);
    TERN_(SET_REMAINING_TIME, ui.reset_remaining_time());
    TERN_(HAS_PRUSA_MMU3, MMU3::operation_statistics.reset_per_print_stats());
  }
  print_job_timer.start();
}

#if HAS_MEDIA

  inline void abortSDPrinting() {
    IF_DISABLED(NO_SD_AUTOSTART, card.autofile_cancel());
    card.abortFilePrintNow(TERN_(SD_RESORT, true));

    queue.clear();
    quickstop_stepper();

    print_job_timer.abort();

    IF_DISABLED(SD_ABORT_NO_COOLDOWN, thermalManager.disable_all_heaters());

    TERN(HAS_CUTTER, cutter.kill(), thermalManager.zero_fan_speeds()); // Full cutter shutdown including ISR control

    wait_for_heatup = false;

    TERN_(POWER_LOSS_RECOVERY, recovery.purge());

    #ifdef EVENT_GCODE_SD_ABORT
      queue.inject(F(EVENT_GCODE_SD_ABORT));
    #endif

    TERN_(PASSWORD_AFTER_SD_PRINT_ABORT, password.lock_machine());
  }

  inline void finishSDPrinting() {
    if (queue.enqueue_one(F("M1001"))) {      // Keep trying until it gets queued
      marlin_state = MarlinState::MF_RUNNING; // Signal to stop trying
      TERN_(PASSWORD_AFTER_SD_PRINT_END, password.lock_machine());
      TERN_(DGUS_LCD_UI_MKS, screen.sdPrintingFinished());
    }
  }

#endif // HAS_MEDIA

/**
 * Minimal management of Marlin's core activities:
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if CHDK_PIN needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check for CUSTOM USER button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 *  - Pulse FET_SAFETY_PIN if it exists
 */
inline void manage_inactivity(const bool no_stepper_sleep=false) {

  queue.get_available_commands();

  const millis_t ms = millis();

  // Prevent steppers timing-out
  const bool do_reset_timeout = no_stepper_sleep
                               || TERN0(PAUSE_PARK_NO_STEPPER_TIMEOUT, did_pause_print);

  // Reset both the M18/M84 activity timeout and the M85 max 'kill' timeout
  if (do_reset_timeout) gcode.reset_stepper_timeout(ms);

  if (gcode.stepper_max_timed_out(ms)) {
    SERIAL_ERROR_START();
    SERIAL_ECHOPGM(STR_KILL_PRE);
    SERIAL_ECHOLNPGM(STR_KILL_INACTIVE_TIME, parser.command_ptr);
    kill();
  }

  const bool has_blocks = planner.has_blocks_queued();  // Any moves in the planner?
  if (has_blocks) gcode.reset_stepper_timeout(ms);      // Reset timeout for M18/M84, M85 max 'kill', and laser.

  // M18 / M84 : Handle steppers inactive time timeout
  #if HAS_DISABLE_IDLE_AXES
    if (gcode.stepper_inactive_time) {

      static bool already_shutdown_steppers; // = false

      if (!has_blocks && !do_reset_timeout && gcode.stepper_inactive_timeout()) {
        if (!already_shutdown_steppers) {
          already_shutdown_steppers = true;

          // Individual axes will be disabled if configured
          TERN_(DISABLE_IDLE_X, stepper.disable_axis(X_AXIS));
          TERN_(DISABLE_IDLE_Y, stepper.disable_axis(Y_AXIS));
          TERN_(DISABLE_IDLE_Z, stepper.disable_axis(Z_AXIS));
          TERN_(DISABLE_IDLE_I, stepper.disable_axis(I_AXIS));
          TERN_(DISABLE_IDLE_J, stepper.disable_axis(J_AXIS));
          TERN_(DISABLE_IDLE_K, stepper.disable_axis(K_AXIS));
          TERN_(DISABLE_IDLE_U, stepper.disable_axis(U_AXIS));
          TERN_(DISABLE_IDLE_V, stepper.disable_axis(V_AXIS));
          TERN_(DISABLE_IDLE_W, stepper.disable_axis(W_AXIS));
          TERN_(DISABLE_IDLE_E, stepper.disable_e_steppers());

          TERN_(AUTO_BED_LEVELING_UBL, bedlevel.steppers_were_disabled());
        }
      }
      else
        already_shutdown_steppers = false;
    }
  #endif

  #if ENABLED(PHOTO_GCODE) && PIN_EXISTS(CHDK)
    // Check if CHDK should be set to LOW (after M240 set it HIGH)
    extern millis_t chdk_timeout;
    if (chdk_timeout && ELAPSED(ms, chdk_timeout)) {
      chdk_timeout = 0;
      WRITE(CHDK_PIN, LOW);
    }
  #endif

  #if HAS_KILL

    // Check if the kill button was pressed and wait to ensure the signal is not noise
    // typically caused by poor insulation and grounding on LCD cables.
    // Lower numbers here will increase response time and therefore safety rating.
    // It is recommended to set this as low as possibe without false triggers.
    // -------------------------------------------------------------------------------
    #ifndef KILL_DELAY
      #define KILL_DELAY 250
    #endif

    static int killCount = 0;   // make the inactivity button a bit less responsive
    if (kill_state())
      killCount++;
    else if (killCount > 0)
      killCount--;

    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if (killCount >= KILL_DELAY) {
      SERIAL_ERROR_START();
      SERIAL_ECHOPGM(STR_KILL_PRE);
      SERIAL_ECHOLNPGM(STR_KILL_BUTTON);
      kill();
    }
  #endif

  #if ENABLED(FREEZE_FEATURE)
    stepper.frozen = READ(FREEZE_PIN) == FREEZE_STATE;
  #endif

  #if HAS_HOME
    // Handle a standalone HOME button
    constexpr millis_t HOME_DEBOUNCE_DELAY = 1000UL;
    static millis_t next_home_key_ms; // = 0
    if (!IS_SD_PRINTING() && !READ(HOME_PIN)) { // HOME_PIN goes LOW when pressed
      if (ELAPSED(ms, next_home_key_ms)) {
        next_home_key_ms = ms + HOME_DEBOUNCE_DELAY;
        LCD_MESSAGE(MSG_AUTO_HOME);
        queue.inject_P(G28_STR);
      }
    }
  #endif

  #if ENABLED(CUSTOM_USER_BUTTONS)
    // Handle a custom user button if defined
    const bool printer_not_busy = !printingIsActive();
    #define HAS_CUSTOM_USER_BUTTON(N) (PIN_EXISTS(BUTTON##N) && defined(BUTTON##N##_HIT_STATE) && defined(BUTTON##N##_GCODE))
    #define HAS_BETTER_USER_BUTTON(N) HAS_CUSTOM_USER_BUTTON(N) && defined(BUTTON##N##_DESC)
    #define _CHECK_CUSTOM_USER_BUTTON(N, CODE) do{                     \
      constexpr millis_t CUB_DEBOUNCE_DELAY_##N = 250UL;               \
      static millis_t next_cub_ms_##N;                                 \
      if (BUTTON##N##_HIT_STATE == READ(BUTTON##N##_PIN)               \
        && (ENABLED(BUTTON##N##_WHEN_PRINTING) || printer_not_busy)) { \
        if (ELAPSED(ms, next_cub_ms_##N)) {                            \
          next_cub_ms_##N = ms + CUB_DEBOUNCE_DELAY_##N;               \
          CODE;                                                        \
          queue.inject(F(BUTTON##N##_GCODE));                          \
          TERN_(HAS_MARLINUI_MENU, ui.quick_feedback());               \
        }                                                              \
      }                                                                \
    }while(0)

    #define CHECK_CUSTOM_USER_BUTTON(N) _CHECK_CUSTOM_USER_BUTTON(N, NOOP)
    #define CHECK_BETTER_USER_BUTTON(N) _CHECK_CUSTOM_USER_BUTTON(N, if (strlen(BUTTON##N##_DESC)) LCD_MESSAGE_F(BUTTON##N##_DESC))

    #if HAS_BETTER_USER_BUTTON(1)
      CHECK_BETTER_USER_BUTTON(1);
    #elif HAS_CUSTOM_USER_BUTTON(1)
      CHECK_CUSTOM_USER_BUTTON(1);
    #endif
    #if HAS_BETTER_USER_BUTTON(2)
      CHECK_BETTER_USER_BUTTON(2);
    #elif HAS_CUSTOM_USER_BUTTON(2)
      CHECK_CUSTOM_USER_BUTTON(2);
    #endif
    #if HAS_BETTER_USER_BUTTON(3)
      CHECK_BETTER_USER_BUTTON(3);
    #elif HAS_CUSTOM_USER_BUTTON(3)
      CHECK_CUSTOM_USER_BUTTON(3);
    #endif
    #if HAS_BETTER_USER_BUTTON(4)
      CHECK_BETTER_USER_BUTTON(4);
    #elif HAS_CUSTOM_USER_BUTTON(4)
      CHECK_CUSTOM_USER_BUTTON(4);
    #endif
    #if HAS_BETTER_USER_BUTTON(5)
      CHECK_BETTER_USER_BUTTON(5);
    #elif HAS_CUSTOM_USER_BUTTON(5)
      CHECK_CUSTOM_USER_BUTTON(5);
    #endif
    #if HAS_BETTER_USER_BUTTON(6)
      CHECK_BETTER_USER_BUTTON(6);
    #elif HAS_CUSTOM_USER_BUTTON(6)
      CHECK_CUSTOM_USER_BUTTON(6);
    #endif
    #if HAS_BETTER_USER_BUTTON(7)
      CHECK_BETTER_USER_BUTTON(7);
    #elif HAS_CUSTOM_USER_BUTTON(7)
      CHECK_CUSTOM_USER_BUTTON(7);
    #endif
    #if HAS_BETTER_USER_BUTTON(8)
      CHECK_BETTER_USER_BUTTON(8);
    #elif HAS_CUSTOM_USER_BUTTON(8)
      CHECK_CUSTOM_USER_BUTTON(8);
    #endif
    #if HAS_BETTER_USER_BUTTON(9)
      CHECK_BETTER_USER_BUTTON(9);
    #elif HAS_CUSTOM_USER_BUTTON(9)
      CHECK_CUSTOM_USER_BUTTON(9);
    #endif
    #if HAS_BETTER_USER_BUTTON(10)
      CHECK_BETTER_USER_BUTTON(10);
    #elif HAS_CUSTOM_USER_BUTTON(10)
      CHECK_CUSTOM_USER_BUTTON(10);
    #endif
    #if HAS_BETTER_USER_BUTTON(11)
      CHECK_BETTER_USER_BUTTON(11);
    #elif HAS_CUSTOM_USER_BUTTON(11)
      CHECK_CUSTOM_USER_BUTTON(11);
    #endif
    #if HAS_BETTER_USER_BUTTON(12)
      CHECK_BETTER_USER_BUTTON(12);
    #elif HAS_CUSTOM_USER_BUTTON(12)
      CHECK_CUSTOM_USER_BUTTON(12);
    #endif
    #if HAS_BETTER_USER_BUTTON(13)
      CHECK_BETTER_USER_BUTTON(13);
    #elif HAS_CUSTOM_USER_BUTTON(13)
      CHECK_CUSTOM_USER_BUTTON(13);
    #endif
    #if HAS_BETTER_USER_BUTTON(14)
      CHECK_BETTER_USER_BUTTON(14);
    #elif HAS_CUSTOM_USER_BUTTON(14)
      CHECK_CUSTOM_USER_BUTTON(14);
    #endif
    #if HAS_BETTER_USER_BUTTON(15)
      CHECK_BETTER_USER_BUTTON(15);
    #elif HAS_CUSTOM_USER_BUTTON(15)
      CHECK_CUSTOM_USER_BUTTON(15);
    #endif
    #if HAS_BETTER_USER_BUTTON(16)
      CHECK_BETTER_USER_BUTTON(16);
    #elif HAS_CUSTOM_USER_BUTTON(16)
      CHECK_CUSTOM_USER_BUTTON(16);
    #endif
    #if HAS_BETTER_USER_BUTTON(17)
      CHECK_BETTER_USER_BUTTON(17);
    #elif HAS_CUSTOM_USER_BUTTON(17)
      CHECK_CUSTOM_USER_BUTTON(17);
    #endif
    #if HAS_BETTER_USER_BUTTON(18)
      CHECK_BETTER_USER_BUTTON(18);
    #elif HAS_CUSTOM_USER_BUTTON(18)
      CHECK_CUSTOM_USER_BUTTON(18);
    #endif
    #if HAS_BETTER_USER_BUTTON(19)
      CHECK_BETTER_USER_BUTTON(19);
    #elif HAS_CUSTOM_USER_BUTTON(19)
      CHECK_CUSTOM_USER_BUTTON(19);
    #endif
    #if HAS_BETTER_USER_BUTTON(20)
      CHECK_BETTER_USER_BUTTON(20);
    #elif HAS_CUSTOM_USER_BUTTON(20)
      CHECK_CUSTOM_USER_BUTTON(20);
    #endif
    #if HAS_BETTER_USER_BUTTON(21)
      CHECK_BETTER_USER_BUTTON(21);
    #elif HAS_CUSTOM_USER_BUTTON(21)
      CHECK_CUSTOM_USER_BUTTON(21);
    #endif
    #if HAS_BETTER_USER_BUTTON(22)
      CHECK_BETTER_USER_BUTTON(22);
    #elif HAS_CUSTOM_USER_BUTTON(22)
      CHECK_CUSTOM_USER_BUTTON(22);
    #endif
    #if HAS_BETTER_USER_BUTTON(23)
      CHECK_BETTER_USER_BUTTON(23);
    #elif HAS_CUSTOM_USER_BUTTON(23)
      CHECK_CUSTOM_USER_BUTTON(23);
    #endif
    #if HAS_BETTER_USER_BUTTON(24)
      CHECK_BETTER_USER_BUTTON(24);
    #elif HAS_CUSTOM_USER_BUTTON(24)
      CHECK_CUSTOM_USER_BUTTON(24);
    #endif
    #if HAS_BETTER_USER_BUTTON(25)
      CHECK_BETTER_USER_BUTTON(25);
    #elif HAS_CUSTOM_USER_BUTTON(25)
      CHECK_CUSTOM_USER_BUTTON(25);
    #endif
  #endif

  TERN_(EASYTHREED_UI, easythreed_ui.run());

  TERN_(USE_CONTROLLER_FAN, controllerFan.update()); // Check if fan should be turned on to cool stepper drivers down

  TERN_(AUTO_POWER_CONTROL, powerManager.check(!ui.on_status_screen() || printJobOngoing() || printingIsPaused()));

  TERN_(HOTEND_IDLE_TIMEOUT, hotend_idle.check());

  #if ANY(PSU_CONTROL, AUTO_POWER_CONTROL) && PIN_EXISTS(PS_ON_EDM)
    if ( ELAPSED(ms, powerManager.last_state_change_ms + PS_EDM_RESPONSE)
      && (READ(PS_ON_PIN) != READ(PS_ON_EDM_PIN) || TERN0(PSU_OFF_REDUNDANT, extDigitalRead(PS_ON1_PIN) != extDigitalRead(PS_ON1_EDM_PIN)))
    ) kill(GET_TEXT_F(MSG_POWER_EDM_FAULT));
  #endif

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    if (thermalManager.degHotend(active_extruder) > (EXTRUDER_RUNOUT_MINTEMP)
      && ELAPSED(ms, gcode.previous_move_ms + SEC_TO_MS(EXTRUDER_RUNOUT_SECONDS))
      && !planner.has_blocks_queued()
    ) {
      const int8_t e_stepper = TERN(HAS_SWITCHING_EXTRUDER, active_extruder >> 1, active_extruder);
      const bool e_off = !stepper.AXIS_IS_ENABLED(E_AXIS, e_stepper);
      if (e_off) stepper.ENABLE_EXTRUDER(e_stepper);

      const float olde = current_position.e;
      current_position.e += EXTRUDER_RUNOUT_EXTRUDE;
      line_to_current_position(MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED));
      current_position.e = olde;
      planner.set_e_position_mm(olde);
      planner.synchronize();

      if (e_off) stepper.DISABLE_EXTRUDER(e_stepper);

      gcode.reset_stepper_timeout(ms);
    }
  #endif // EXTRUDER_RUNOUT_PREVENT

  #if ENABLED(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (delayed_move_time && ELAPSED(ms, delayed_move_time) && IsRunning()) {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      destination = current_position;
      prepare_line_to_destination();
      planner.synchronize();
    }
  #endif

  TERN_(TEMP_STAT_LEDS, handle_status_leds());

  TERN_(MONITOR_DRIVER_STATUS, monitor_tmc_drivers());

  // Limit check_axes_activity frequency to 10Hz
  static millis_t next_check_axes_ms = 0;
  if (ELAPSED(ms, next_check_axes_ms)) {
    planner.check_axes_activity();
    next_check_axes_ms = ms + 100UL;
  }

  #if PIN_EXISTS(FET_SAFETY)
    static millis_t FET_next;
    if (ELAPSED(ms, FET_next)) {
      FET_next = ms + FET_SAFETY_DELAY;  // 2µs pulse every FET_SAFETY_DELAY mS
      OUT_WRITE(FET_SAFETY_PIN, !FET_SAFETY_INVERTED);
      DELAY_US(2);
      WRITE(FET_SAFETY_PIN, FET_SAFETY_INVERTED);
    }
  #endif
}

#if ALL(EP_BABYSTEPPING, EMERGENCY_PARSER)
  #include "feature/babystep.h"
#endif

/**
 * Standard idle routine keeps the machine alive:
 *  - Core Marlin activities
 *  - Manage heaters (and Watchdog)
 *  - Max7219 heartbeat, animation, etc.
 *
 *  Only after setup() is complete:
 *  - Handle filament runout sensors
 *  - Run HAL idle tasks
 *  - Handle Power-Loss Recovery
 *  - Run StallGuard endstop checks
 *  - Handle SD Card insert / remove
 *  - Handle USB Flash Drive insert / remove
 *  - Announce Host Keepalive state (if any)
 *  - Update the Print Job Timer state
 *  - Update the Beeper queue
 *  - Read Buttons and Update the LCD
 *  - Run i2c Position Encoders
 *  - Auto-report Temperatures / SD Status
 *  - Update the Průša MMU2
 *  - Handle Joystick jogging
 */
void idle(const bool no_stepper_sleep/*=false*/) {
  #ifdef MAX7219_DEBUG_PROFILE
    CodeProfiler idle_profiler;
  #endif

  #if ENABLED(MARLIN_DEV_MODE)
    static uint16_t idle_depth = 0;
    if (++idle_depth > 5) SERIAL_ECHOLNPGM("idle() call depth: ", idle_depth);
  #endif

  // Bed Distance Sensor task
  TERN_(BD_SENSOR, bdl.process());

  // Core Marlin activities
  manage_inactivity(no_stepper_sleep);

  // Manage Heaters (and Watchdog)
  thermalManager.task();

  // Max7219 heartbeat, animation, etc
  TERN_(MAX7219_DEBUG, max7219.idle_tasks());

  // Return if setup() isn't completed
  if (marlin_state == MarlinState::MF_INITIALIZING) goto IDLE_DONE;

  // TODO: Still causing errors
  TERN_(TOOL_SENSOR, (void)check_tool_sensor_stats(active_extruder, true));

  // Handle filament runout sensors
  #if HAS_FILAMENT_SENSOR
    if (TERN1(HAS_PRUSA_MMU2, !mmu2.enabled()) && TERN1(HAS_PRUSA_MMU3, !mmu3.enabled()))
      runout.run();
  #endif

  // Run HAL idle tasks
  hal.idletask();

  // Check network connection
  TERN_(HAS_ETHERNET, ethernet.check());

  // Handle Power-Loss Recovery
  #if ENABLED(POWER_LOSS_RECOVERY) && PIN_EXISTS(POWER_LOSS)
    if (IS_SD_PRINTING()) recovery.outage();
  #endif

  // Run StallGuard endstop checks
  #if ENABLED(SPI_ENDSTOPS)
    if (endstops.tmc_spi_homing.any && TERN1(IMPROVE_HOMING_RELIABILITY, ELAPSED(millis(), sg_guard_period)))
      for (uint8_t i = 0; i < 4; ++i) if (endstops.tmc_spi_homing_check()) break; // Read SGT 4 times per idle loop
  #endif

  // Handle SD Card insert / remove
  TERN_(HAS_MEDIA, card.manage_media());

  // Handle USB Flash Drive insert / remove
  TERN_(HAS_USB_FLASH_DRIVE, card.diskIODriver()->idle());

  // Announce Host Keepalive state (if any)
  TERN_(HOST_KEEPALIVE_FEATURE, gcode.host_keepalive());

  // Update the Print Job Timer state
  TERN_(PRINTCOUNTER, print_job_timer.tick());

  // Update the Beeper queue
  TERN_(HAS_BEEPER, buzzer.tick());

  // Handle UI input / draw events
  #if ENABLED(SOVOL_SV06_RTS)
    RTS_Update();
  #else
    ui.update();
  #endif

  // Run i2c Position Encoders
  #if ENABLED(I2C_POSITION_ENCODERS)
  {
    static millis_t i2cpem_next_update_ms;
    if (planner.has_blocks_queued()) {
      const millis_t ms = millis();
      if (ELAPSED(ms, i2cpem_next_update_ms)) {
        I2CPEM.update();
        i2cpem_next_update_ms = ms + I2CPE_MIN_UPD_TIME_MS;
      }
    }
  }
  #endif

  // Auto-report Temperatures / SD Status
  #if HAS_AUTO_REPORTING
    if (!gcode.autoreport_paused) {
      TERN_(AUTO_REPORT_TEMPERATURES, thermalManager.auto_reporter.tick());
      TERN_(AUTO_REPORT_FANS, fan_check.auto_reporter.tick());
      TERN_(AUTO_REPORT_SD_STATUS, card.auto_reporter.tick());
      TERN_(AUTO_REPORT_POSITION, position_auto_reporter.tick());
      TERN_(BUFFER_MONITORING, queue.auto_report_buffer_statistics());
    }
  #endif

  // Update the Průša MMU2
  #if HAS_PRUSA_MMU3
    mmu3.mmu_loop();
  #elif HAS_PRUSA_MMU2
    mmu2.mmu_loop();
  #endif

  // Handle Joystick jogging
  TERN_(POLL_JOG, joystick.inject_jog_moves());

  // Async Babystepping via the Emergency Parser
  #if ALL(EP_BABYSTEPPING, EMERGENCY_PARSER)
    babystep.do_ep_steps();
  #endif

  // Direct Stepping
  TERN_(DIRECT_STEPPING, page_manager.write_responses());

  // Update the LVGL interface
  TERN_(HAS_TFT_LVGL_UI, LV_TASK_HANDLER());

  // Manage Fixed-time Motion Control
  TERN_(FT_MOTION, ftMotion.loop());

  IDLE_DONE:
  TERN_(MARLIN_DEV_MODE, idle_depth--);

  return;
}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void kill(FSTR_P const lcd_error/*=nullptr*/, FSTR_P const lcd_component/*=nullptr*/, const bool steppers_off/*=false*/) {
  thermalManager.disable_all_heaters();

  TERN_(HAS_CUTTER, cutter.kill()); // Full cutter shutdown including ISR control

  // Echo the LCD message to serial for extra context
  if (lcd_error) { SERIAL_ECHO_START(); SERIAL_ECHOLN(lcd_error); }

  #if HAS_DISPLAY
    ui.kill_screen(lcd_error ?: GET_TEXT_F(MSG_KILLED), lcd_component ?: FPSTR(NUL_STR));
  #else
    UNUSED(lcd_error); UNUSED(lcd_component);
  #endif

  TERN_(HAS_TFT_LVGL_UI, lv_draw_error_message(lcd_error));

  // "Error:Printer halted. kill() called!"
  SERIAL_ERROR_MSG(STR_ERR_KILLED);

  #ifdef ACTION_ON_KILL
    hostui.kill();
  #endif

  minkill(steppers_off);
}

void minkill(const bool steppers_off/*=false*/) {

  // Wait a short time (allows messages to get out before shutting down.
  for (int i = 1000; i--;) DELAY_US(600);

  cli(); // Stop interrupts

  // Wait to ensure all interrupts stopped
  for (int i = 1000; i--;) DELAY_US(250);

  // Reiterate heaters off
  thermalManager.disable_all_heaters();

  TERN_(HAS_CUTTER, cutter.kill());  // Reiterate cutter shutdown

  // Power off all steppers (for M112) or just the E steppers
  steppers_off ? stepper.disable_all_steppers() : stepper.disable_e_steppers();

  TERN_(PSU_CONTROL, powerManager.power_off());

  TERN_(HAS_SUICIDE, suicide());

  #if ANY(HAS_KILL, SOFT_RESET_ON_KILL)

    // Wait for both KILL and ENC to be released
    while (TERN0(HAS_KILL, kill_state()) || TERN0(SOFT_RESET_ON_KILL, ui.button_pressed()))
      hal.watchdog_refresh();

    // Wait for either KILL or ENC to be pressed again
    while (TERN1(HAS_KILL, !kill_state()) && TERN1(SOFT_RESET_ON_KILL, !ui.button_pressed()))
      hal.watchdog_refresh();

    // Reboot the board
    hal.reboot();

  #else

    for (;;) hal.watchdog_refresh();  // Wait for RESET button or power-cycle

  #endif
}

/**
 * Turn off heaters and stop the print in progress
 * After a stop the machine may be resumed with M999
 */
void stop() {
  thermalManager.disable_all_heaters(); // 'unpause' taken care of in here

  print_job_timer.stop();

  #if ANY(PROBING_FANS_OFF, ADVANCED_PAUSE_FANS_PAUSE)
    thermalManager.set_fans_paused(false); // Un-pause fans for safety
  #endif

  if (!IsStopped()) {
    SERIAL_ERROR_MSG(STR_ERR_STOPPED);
    LCD_MESSAGE(MSG_STOPPED);
    safe_delay(350);       // allow enough time for messages to get out before stopping
    marlin_state = MarlinState::MF_STOPPED;
  }
}

inline void tmc_standby_setup() {
  #if PIN_EXISTS(X_STDBY)
    SET_INPUT_PULLDOWN(X_STDBY_PIN);
  #endif
  #if PIN_EXISTS(X2_STDBY)
    SET_INPUT_PULLDOWN(X2_STDBY_PIN);
  #endif
  #if PIN_EXISTS(Y_STDBY)
    SET_INPUT_PULLDOWN(Y_STDBY_PIN);
  #endif
  #if PIN_EXISTS(Y2_STDBY)
    SET_INPUT_PULLDOWN(Y2_STDBY_PIN);
  #endif
  #if PIN_EXISTS(Z_STDBY)
    SET_INPUT_PULLDOWN(Z_STDBY_PIN);
  #endif
  #if PIN_EXISTS(Z2_STDBY)
    SET_INPUT_PULLDOWN(Z2_STDBY_PIN);
  #endif
  #if PIN_EXISTS(Z3_STDBY)
    SET_INPUT_PULLDOWN(Z3_STDBY_PIN);
  #endif
  #if PIN_EXISTS(Z4_STDBY)
    SET_INPUT_PULLDOWN(Z4_STDBY_PIN);
  #endif
  #if PIN_EXISTS(I_STDBY)
    SET_INPUT_PULLDOWN(I_STDBY_PIN);
  #endif
  #if PIN_EXISTS(J_STDBY)
    SET_INPUT_PULLDOWN(J_STDBY_PIN);
  #endif
  #if PIN_EXISTS(K_STDBY)
    SET_INPUT_PULLDOWN(K_STDBY_PIN);
  #endif
  #if PIN_EXISTS(U_STDBY)
    SET_INPUT_PULLDOWN(U_STDBY_PIN);
  #endif
  #if PIN_EXISTS(V_STDBY)
    SET_INPUT_PULLDOWN(V_STDBY_PIN);
  #endif
  #if PIN_EXISTS(W_STDBY)
    SET_INPUT_PULLDOWN(W_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E0_STDBY)
    SET_INPUT_PULLDOWN(E0_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E1_STDBY)
    SET_INPUT_PULLDOWN(E1_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E2_STDBY)
    SET_INPUT_PULLDOWN(E2_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E3_STDBY)
    SET_INPUT_PULLDOWN(E3_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E4_STDBY)
    SET_INPUT_PULLDOWN(E4_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E5_STDBY)
    SET_INPUT_PULLDOWN(E5_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E6_STDBY)
    SET_INPUT_PULLDOWN(E6_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E7_STDBY)
    SET_INPUT_PULLDOWN(E7_STDBY_PIN);
  #endif
}

/**
 * Marlin Firmware entry-point. Abandon Hope All Ye Who Enter Here.
 * Setup before the program loop:
 *
 *  - Call any special pre-init set for the board
 *  - Put TMC drivers into Low Power Standby mode
 *  - Init the serial ports (so setup can be debugged)
 *  - Set up the kill and suicide pins
 *  - Prepare (disable) board JTAG and Debug ports
 *  - Init serial for a connected MKS TFT with WiFi
 *  - Install Marlin custom Exception Handlers, if set.
 *  - Init Marlin's HAL interfaces (for SPI, i2c, etc.)
 *  - Init some optional hardware and features:
 *    • MAX Thermocouple pins
 *    • Duet Smart Effector
 *    • Filament Runout Sensor
 *    • TMC220x Stepper Drivers (Serial)
 *    • PSU control
 *    • Power-loss Recovery
 *    • Stepper Driver Reset: DISABLE
 *    • TMC Stepper Drivers (SPI)
 *    • Run hal.init_board() for additional pins setup
 *    • ESP WiFi
 *  - Get the Reset Reason and report it
 *  - Print startup messages and diagnostics
 *  - Calibrate the HAL DELAY for precise timing
 *  - Init the buzzer, possibly a custom timer
 *  - Init more optional hardware:
 *    • Color LED illumination
 *    • NeoPixel illumination
 *    • Controller Fan
 *    • Creality DWIN LCD (show boot image)
 *    • Tare the Probe if possible
 *  - Mount the (most likely external) SD Card
 *  - Load settings from EEPROM (or use defaults)
 *  - Init the Ethernet Port
 *  - Init Touch Buttons (for emulated DOGLCD)
 *  - Adjust the (certainly wrong) current position by the home offset
 *  - Init the Planner::position (steps) based on current (native) position
 *  - Initialize more managers and peripherals:
 *    • Temperatures
 *    • Print Job Timer
 *    • Endstops and Endstop Interrupts
 *    • Stepper ISR - Kind of Important!
 *    • Servos
 *    • Servo-based Probe
 *    • Photograph Pin
 *    • Laser/Spindle tool Power / PWM
 *    • Coolant Control
 *    • Bed Probe
 *    • Stepper Driver Reset: ENABLE
 *    • Digipot I2C - Stepper driver current control
 *    • Stepper DAC - Stepper driver current control
 *    • Solenoid (probe, or for other use)
 *    • Home Pin
 *    • Custom User Buttons
 *    • Red/Blue Status LEDs
 *    • Case Light
 *    • Prusa MMU filament changer
 *    • Fan Multiplexer
 *    • Mixing Extruder
 *    • BLTouch Probe
 *    • I2C Position Encoders
 *    • Custom I2C Bus handlers
 *    • Enhanced tools or extruders:
 *      • Switching Extruder
 *      • Switching Nozzle
 *      • Parking Extruder
 *      • Magnetic Parking Extruder
 *      • Switching Toolhead
 *      • Electromagnetic Switching Toolhead
 *    • Watchdog Timer - Also Kind of Important!
 *    • Closed Loop Controller
 *  - Run Startup Commands, if defined
 *  - Tell host to close Host Prompts
 *  - Test Trinamic driver connections
 *  - Init Prusa MMU2 filament changer
 *  - Init and test BL24Cxx EEPROM
 *  - Init Creality DWIN encoder, show faux progress bar
 *  - Reset Status Message / Show Service Messages
 *  - Init MAX7219 LED Matrix
 *  - Init Direct Stepping (Klipper-style motion control)
 *  - Init TFT LVGL UI (with 3D Graphics)
 *  - Apply Password Lock - Hold for Authentication
 *  - Open Touch Screen Calibration screen, if not calibrated
 *  - Set Marlin to RUNNING State
 */
void setup() {
  #ifdef FASTIO_INIT
    FASTIO_INIT();
  #endif

  #ifdef BOARD_PREINIT
    BOARD_PREINIT(); // Low-level init (before serial init)
  #endif

  tmc_standby_setup();  // TMC Low Power Standby pins must be set early or they're not usable

  // Check startup - does nothing if bootloader sets MCUSR to 0
  const byte mcu = hal.get_reset_source();
  hal.clear_reset_source();

  #if ENABLED(MARLIN_DEV_MODE)
    auto log_current_ms = [&](PGM_P const msg) {
      SERIAL_ECHO_START();
      TSS('[', millis(), F("] ")).echo();
      SERIAL_ECHOLNPGM_P(msg);
    };
    #define SETUP_LOG(M) log_current_ms(PSTR(M))
  #else
    #define SETUP_LOG(...) NOOP
  #endif
  #define SETUP_RUN(C) do{ SETUP_LOG(STRINGIFY(C)); C; }while(0)

  MYSERIAL1.begin(BAUDRATE);
  millis_t serial_connect_timeout = millis() + 1000UL;
  while (!MYSERIAL1.connected() && PENDING(millis(), serial_connect_timeout)) { /*nada*/ }

  #if ENABLED(SOVOL_SV06_RTS)
    LCD_SERIAL.begin(BAUDRATE);
    serial_connect_timeout = millis() + 1000UL;
    while (!LCD_SERIAL.connected() && PENDING(millis(), serial_connect_timeout)) { /*nada*/ }
  #endif

  #if HAS_MULTI_SERIAL && !HAS_ETHERNET
    #ifndef BAUDRATE_2
      #define BAUDRATE_2 BAUDRATE
    #endif
    MYSERIAL2.begin(BAUDRATE_2);
    serial_connect_timeout = millis() + 1000UL;
    while (!MYSERIAL2.connected() && PENDING(millis(), serial_connect_timeout)) { /*nada*/ }
    #ifdef SERIAL_PORT_3
      #ifndef BAUDRATE_3
        #define BAUDRATE_3 BAUDRATE
      #endif
      MYSERIAL3.begin(BAUDRATE_3);
      serial_connect_timeout = millis() + 1000UL;
      while (!MYSERIAL3.connected() && PENDING(millis(), serial_connect_timeout)) { /*nada*/ }
    #endif
  #endif
  SERIAL_ECHOLNPGM("start");

  // Set up these pins early to prevent suicide
  #if HAS_KILL
    SETUP_LOG("KILL_PIN");
    #if KILL_PIN_STATE
      SET_INPUT_PULLDOWN(KILL_PIN);
    #else
      SET_INPUT_PULLUP(KILL_PIN);
    #endif
  #endif

  #if ENABLED(FREEZE_FEATURE)
    SETUP_LOG("FREEZE_PIN");
    #if FREEZE_STATE
      SET_INPUT_PULLDOWN(FREEZE_PIN);
    #else
      SET_INPUT_PULLUP(FREEZE_PIN);
    #endif
  #endif

  #if HAS_SUICIDE
    SETUP_LOG("SUICIDE_PIN");
    OUT_WRITE(SUICIDE_PIN, !SUICIDE_PIN_STATE);
  #endif

  #ifdef JTAGSWD_RESET
    SETUP_LOG("JTAGSWD_RESET");
    JTAGSWD_RESET();
  #endif

  // Disable any hardware debug to free up pins for IO
  #if ENABLED(DISABLE_DEBUG) && defined(JTAGSWD_DISABLE)
    delay(10);
    SETUP_LOG("JTAGSWD_DISABLE");
    JTAGSWD_DISABLE();
  #elif ENABLED(DISABLE_JTAG) && defined(JTAG_DISABLE)
    delay(10);
    SETUP_LOG("JTAG_DISABLE");
    JTAG_DISABLE();
  #endif

  TERN_(DYNAMIC_VECTORTABLE, hook_cpu_exceptions()); // If supported, install Marlin exception handlers at runtime

  SETUP_RUN(hal.init());

  // Init and disable SPI thermocouples; this is still needed
  #if TEMP_SENSOR_IS_MAX_TC(0) || (TEMP_SENSOR_IS_MAX_TC(REDUNDANT) && REDUNDANT_TEMP_MATCH(SOURCE, E0))
    OUT_WRITE(TEMP_0_CS_PIN, HIGH);  // Disable
  #endif
  #if TEMP_SENSOR_IS_MAX_TC(1) || (TEMP_SENSOR_IS_MAX_TC(REDUNDANT) && REDUNDANT_TEMP_MATCH(SOURCE, E1))
    OUT_WRITE(TEMP_1_CS_PIN, HIGH);
  #endif
  #if TEMP_SENSOR_IS_MAX_TC(2) || (TEMP_SENSOR_IS_MAX_TC(REDUNDANT) && REDUNDANT_TEMP_MATCH(SOURCE, E2))
    OUT_WRITE(TEMP_2_CS_PIN, HIGH);
  #endif
  #if TEMP_SENSOR_IS_MAX_TC(BED)
    OUT_WRITE(TEMP_BED_CS_PIN, HIGH);
  #endif

  #if ENABLED(DUET_SMART_EFFECTOR) && PIN_EXISTS(SMART_EFFECTOR_MOD)
    OUT_WRITE(SMART_EFFECTOR_MOD_PIN, LOW);   // Put Smart Effector into NORMAL mode
  #endif

  #if HAS_FILAMENT_SENSOR
    SETUP_RUN(runout.setup());
  #endif

  #if HAS_TMC_UART
    SETUP_RUN(tmc_serial_begin());
  #endif

  #if HAS_TMC_SPI
    #if DISABLED(TMC_USE_SW_SPI)
      SETUP_RUN(SPI.begin());
    #endif
    SETUP_RUN(tmc_init_cs_pins());
  #endif

  #if ENABLED(PSU_CONTROL)
    SETUP_LOG("PSU_CONTROL");
    powerManager.init();
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    SETUP_RUN(recovery.setup());
  #endif

  #if HAS_STEPPER_RESET
    SETUP_RUN(disableStepperDrivers());
  #endif

  SETUP_RUN(hal.init_board());

  #if ENABLED(WIFISUPPORT)
    SETUP_RUN(esp_wifi_init());
  #endif

  // Report Reset Reason
  if (mcu & RST_POWER_ON)  SERIAL_ECHOLNPGM(STR_POWERUP);
  if (mcu & RST_EXTERNAL)  SERIAL_ECHOLNPGM(STR_EXTERNAL_RESET);
  if (mcu & RST_BROWN_OUT) SERIAL_ECHOLNPGM(STR_BROWNOUT_RESET);
  if (mcu & RST_WATCHDOG)  SERIAL_ECHOLNPGM(STR_WATCHDOG_RESET);
  if (mcu & RST_SOFTWARE)  SERIAL_ECHOLNPGM(STR_SOFTWARE_RESET);

  // Identify myself as Marlin x.x.x
  SERIAL_ECHOLNPGM("Marlin " SHORT_BUILD_VERSION);
  #ifdef STRING_DISTRIBUTION_DATE
    SERIAL_ECHO_MSG(
      " Last Updated: " STRING_DISTRIBUTION_DATE
      " | Author: " STRING_CONFIG_H_AUTHOR
    );
  #endif
  SERIAL_ECHO_MSG(" Compiled: " __DATE__);
  SERIAL_ECHO_MSG(STR_FREE_MEMORY, hal.freeMemory(), STR_PLANNER_BUFFER_BYTES, sizeof(block_t) * (BLOCK_BUFFER_SIZE));

  // Some HAL need precise delay adjustment
  calibrate_delay_loop();

  // Init buzzer pin(s)
  #if HAS_BEEPER
    SETUP_RUN(buzzer.init());
  #endif

  // Set up LEDs early
  #if HAS_COLOR_LEDS
    SETUP_RUN(leds.setup());
  #endif

  #if ENABLED(NEOPIXEL2_SEPARATE)
    SETUP_RUN(leds2.setup());
  #endif

  #if ENABLED(USE_CONTROLLER_FAN)     // Set up fan controller to initialize also the default configurations.
    SETUP_RUN(controllerFan.setup());
  #endif

  TERN_(HAS_FANCHECK, fan_check.init());

  // UI must be initialized before EEPROM
  // (because EEPROM code calls the UI).
  #if ENABLED(SOVOL_SV06_RTS)
    SETUP_RUN(RTS_Update());
  #else
    SETUP_RUN(ui.init());
  #endif

  #if PIN_EXISTS(SAFE_POWER)
    #if HAS_DRIVER_SAFE_POWER_PROTECT
      SETUP_RUN(stepper_driver_backward_check());
    #else
      SETUP_LOG("SAFE_POWER");
      OUT_WRITE(SAFE_POWER_PIN, HIGH);
    #endif
  #endif

  #if HAS_MEDIA && ANY(SDCARD_EEPROM_EMULATION, POWER_LOSS_RECOVERY)
    SETUP_RUN(card.mount());          // Mount media with settings before first_load
  #endif

  // Prepare some LCDs to display early
  #if HAS_EARLY_LCD_SETTINGS
    SETUP_RUN(settings.load_lcd_state());
  #endif

  #if ALL(HAS_WIRED_LCD, SHOW_BOOTSCREEN)
    SETUP_RUN(ui.show_bootscreen());
    const millis_t bootscreen_ms = millis();
  #endif

  SETUP_RUN(settings.first_load());   // Load data from EEPROM if available (or use defaults)
                                      // This also updates variables in the planner, elsewhere

  #if ENABLED(PROBE_TARE)
    SETUP_RUN(probe.tare_init());
  #endif

  #if HAS_ETHERNET
    SETUP_RUN(ethernet.init());
  #endif

  #if HAS_TOUCH_BUTTONS
    SETUP_RUN(touchBt.init());
  #endif

  TERN_(HAS_HOME_OFFSET, current_position += home_offset); // Init current position based on home_offset

  sync_plan_position();               // Vital to init stepper/planner equivalent for current_position

  SETUP_RUN(thermalManager.init());   // Initialize temperature loop

  SETUP_RUN(print_job_timer.init());  // Initial setup of print job timer

  SETUP_RUN(endstops.init());         // Init endstops and pullups

  #if ENABLED(DELTA) && !HAS_SOFTWARE_ENDSTOPS
    SETUP_RUN(refresh_delta_clip_start_height()); // Init safe delta height without soft endstops
  #endif

  SETUP_RUN(stepper.init());          // Init stepper. This enables interrupts!

  #if HAS_SERVOS
    SETUP_RUN(servo_init());
  #endif

  #if HAS_Z_SERVO_PROBE
    SETUP_RUN(probe.servo_probe_init());
  #endif

  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif

  #if HAS_CUTTER
    SETUP_RUN(cutter.init());
  #endif

  #if ENABLED(COOLANT_MIST)
    OUT_WRITE(COOLANT_MIST_PIN, COOLANT_MIST_INVERT);   // Init Mist Coolant OFF
  #endif
  #if ENABLED(COOLANT_FLOOD)
    OUT_WRITE(COOLANT_FLOOD_PIN, COOLANT_FLOOD_INVERT); // Init Flood Coolant OFF
  #endif

  #if HAS_BED_PROBE
    #if PIN_EXISTS(PROBE_ENABLE)
      OUT_WRITE(PROBE_ENABLE_PIN, LOW); // Disable
    #endif
    SETUP_RUN(endstops.enable_z_probe(false));
  #endif

  #if HAS_STEPPER_RESET
    SETUP_RUN(enableStepperDrivers());
  #endif

  #if HAS_MOTOR_CURRENT_I2C
    SETUP_RUN(digipot_i2c.init());
  #endif

  #if HAS_MOTOR_CURRENT_DAC
    SETUP_RUN(stepper_dac.init());
  #endif

  #if ANY(Z_PROBE_SLED, SOLENOID_PROBE) && HAS_SOLENOID_1
    OUT_WRITE(SOL1_PIN, LOW); // OFF
  #endif

  #if HAS_HOME
    SET_INPUT_PULLUP(HOME_PIN);
  #endif

  #if ENABLED(CUSTOM_USER_BUTTONS)
    #define INIT_CUSTOM_USER_BUTTON_PIN(N) do{ SET_INPUT(BUTTON##N##_PIN); WRITE(BUTTON##N##_PIN, !BUTTON##N##_HIT_STATE); }while(0)

    #if HAS_CUSTOM_USER_BUTTON(1)
      INIT_CUSTOM_USER_BUTTON_PIN(1);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(2)
      INIT_CUSTOM_USER_BUTTON_PIN(2);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(3)
      INIT_CUSTOM_USER_BUTTON_PIN(3);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(4)
      INIT_CUSTOM_USER_BUTTON_PIN(4);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(5)
      INIT_CUSTOM_USER_BUTTON_PIN(5);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(6)
      INIT_CUSTOM_USER_BUTTON_PIN(6);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(7)
      INIT_CUSTOM_USER_BUTTON_PIN(7);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(8)
      INIT_CUSTOM_USER_BUTTON_PIN(8);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(9)
      INIT_CUSTOM_USER_BUTTON_PIN(9);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(10)
      INIT_CUSTOM_USER_BUTTON_PIN(10);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(11)
      INIT_CUSTOM_USER_BUTTON_PIN(11);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(12)
      INIT_CUSTOM_USER_BUTTON_PIN(12);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(13)
      INIT_CUSTOM_USER_BUTTON_PIN(13);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(14)
      INIT_CUSTOM_USER_BUTTON_PIN(14);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(15)
      INIT_CUSTOM_USER_BUTTON_PIN(15);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(16)
      INIT_CUSTOM_USER_BUTTON_PIN(16);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(17)
      INIT_CUSTOM_USER_BUTTON_PIN(17);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(18)
      INIT_CUSTOM_USER_BUTTON_PIN(18);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(19)
      INIT_CUSTOM_USER_BUTTON_PIN(19);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(20)
      INIT_CUSTOM_USER_BUTTON_PIN(20);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(21)
      INIT_CUSTOM_USER_BUTTON_PIN(21);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(22)
      INIT_CUSTOM_USER_BUTTON_PIN(22);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(23)
      INIT_CUSTOM_USER_BUTTON_PIN(23);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(24)
      INIT_CUSTOM_USER_BUTTON_PIN(24);
    #endif
    #if HAS_CUSTOM_USER_BUTTON(25)
      INIT_CUSTOM_USER_BUTTON_PIN(25);
    #endif
  #endif

  #if PIN_EXISTS(STAT_LED_RED)
    OUT_WRITE(STAT_LED_RED_PIN, LOW); // OFF
  #endif
  #if PIN_EXISTS(STAT_LED_BLUE)
    OUT_WRITE(STAT_LED_BLUE_PIN, LOW); // OFF
  #endif

  #if ENABLED(CASE_LIGHT_ENABLE)
    SETUP_RUN(caselight.init());
  #endif

  #if HAS_PRUSA_MMU1
    SETUP_RUN(mmu_init());
  #endif

  #if HAS_FANMUX
    SETUP_RUN(fanmux_init());
  #endif

  #if ENABLED(MIXING_EXTRUDER)
    SETUP_RUN(mixer.init());
  #endif

  #if ENABLED(BLTOUCH)
    SETUP_RUN(bltouch.init(/*set_voltage=*/true));
  #endif

  #if ENABLED(MAGLEV4)
    OUT_WRITE(MAGLEV_TRIGGER_PIN, LOW);
  #endif

  #if ENABLED(I2C_POSITION_ENCODERS)
    SETUP_RUN(I2CPEM.init());
  #endif

  #if ENABLED(EXPERIMENTAL_I2CBUS) && I2C_SLAVE_ADDRESS > 0
    SETUP_LOG("i2c...");
    i2c.onReceive(i2c_on_receive);
    i2c.onRequest(i2c_on_request);
  #endif

  #if DO_SWITCH_EXTRUDER
    SETUP_RUN(move_extruder_servo(0));  // Initialize extruder servo
  #endif

  #if ENABLED(SWITCHING_NOZZLE)
    SETUP_LOG("SWITCHING_NOZZLE");
    // Initialize nozzle servo(s)
    #if SWITCHING_NOZZLE_TWO_SERVOS
      lower_nozzle(0);
      raise_nozzle(1);
    #else
      move_nozzle_servo(0);
    #endif
  #endif

  #if ENABLED(PARKING_EXTRUDER)
    SETUP_RUN(pe_solenoid_init());
  #elif ENABLED(MAGNETIC_PARKING_EXTRUDER)
    SETUP_RUN(mpe_settings_init());
  #elif ENABLED(SWITCHING_TOOLHEAD)
    SETUP_RUN(swt_init());
  #elif ENABLED(ELECTROMAGNETIC_SWITCHING_TOOLHEAD)
    SETUP_RUN(est_init());
  #endif

  #if ENABLED(USE_WATCHDOG)
    SETUP_RUN(hal.watchdog_init());   // Reinit watchdog after hal.get_reset_source call
  #endif

  #if ENABLED(EXTERNAL_CLOSED_LOOP_CONTROLLER)
    SETUP_RUN(closedloop.init());
  #endif

  #ifdef STARTUP_COMMANDS
    SETUP_LOG("STARTUP_COMMANDS");
    queue.inject(F(STARTUP_COMMANDS));
  #endif

  #if ENABLED(HOST_PROMPT_SUPPORT)
    SETUP_RUN(hostui.prompt_end());
  #endif

  #if HAS_DRIVER_SAFE_POWER_PROTECT
    SETUP_RUN(stepper_driver_backward_report());
  #endif

  #if HAS_PRUSA_MMU3
    if (mmu3.mmu_hw_enabled) SETUP_RUN(mmu3.start());
    SETUP_RUN(mmu3.status());
    SETUP_RUN(spooljoin.initStatus());
  #elif HAS_PRUSA_MMU2
    SETUP_RUN(mmu2.init());
  #endif

  #if ENABLED(IIC_BL24CXX_EEPROM)
    BL24CXX::init();
    const uint8_t err = BL24CXX::check();
    SERIAL_ECHO_TERNARY(err, "BL24CXX Check ", "failed", "succeeded", "!\n");
  #endif

  #if ENABLED(DWIN_CREALITY_LCD)
    SETUP_RUN(dwinInitScreen());
  #elif ENABLED(SOVOL_SV06_RTS)
    SETUP_RUN(rts.init());
  #endif

  #if HAS_SERVICE_INTERVALS && DISABLED(DWIN_CREALITY_LCD)
    SETUP_RUN(ui.reset_status(true));  // Show service messages or keep current status
  #endif

  #if ENABLED(MAX7219_DEBUG)
    SETUP_RUN(max7219.init());
  #endif

  #if ENABLED(DIRECT_STEPPING)
    SETUP_RUN(page_manager.init());
  #endif

  #if HAS_TFT_LVGL_UI
    #if HAS_MEDIA
      if (!card.isMounted()) SETUP_RUN(card.mount()); // Mount SD to load graphics and fonts
    #endif
    SETUP_RUN(tft_lvgl_init());
  #endif

  #if ALL(HAS_WIRED_LCD, SHOW_BOOTSCREEN)
    const millis_t elapsed = millis() - bootscreen_ms;
    #if ENABLED(MARLIN_DEV_MODE)
      SERIAL_ECHOLNPGM("elapsed=", elapsed);
    #endif
    SETUP_RUN(ui.bootscreen_completion(elapsed));
  #endif

  #if ENABLED(PASSWORD_ON_STARTUP)
    SETUP_RUN(password.lock_machine());      // Will not proceed until correct password provided
  #endif

  #if ALL(HAS_MARLINUI_MENU, TOUCH_SCREEN_CALIBRATION) && ANY(TFT_CLASSIC_UI, TFT_COLOR_UI)
    SETUP_RUN(ui.check_touch_calibration());
  #endif

  #if ENABLED(EASYTHREED_UI)
    SETUP_RUN(easythreed_ui.init());
  #endif

  #if HAS_TRINAMIC_CONFIG && DISABLED(PSU_DEFAULT_OFF)
    SETUP_RUN(test_tmc_connection());
  #endif

  #if ENABLED(BD_SENSOR)
    SETUP_RUN(bdl.init(I2C_BD_SDA_PIN, I2C_BD_SCL_PIN, I2C_BD_DELAY));
  #endif

  #if HAS_RS485_SERIAL
    SETUP_RUN(rs485_init());
  #endif

  #if ENABLED(FT_MOTION)
    SETUP_RUN(ftMotion.init());
  #endif

  marlin_state = MarlinState::MF_RUNNING;

  #ifdef STARTUP_TUNE
    // Play a short startup tune before continuing.
    constexpr uint16_t tune[] = STARTUP_TUNE;
    for (uint8_t i = 0; i < COUNT(tune) - 1; i += 2) BUZZ(tune[i + 1], tune[i]);
  #endif

  SETUP_LOG("setup() completed.");

  TERN_(MARLIN_TEST_BUILD, runStartupTests());
}

/**
 * The main Marlin program loop
 *
 *  - Call idle() to handle all tasks between G-code commands
 *      Note that no G-codes from the queue can be executed during idle()
 *      but many G-codes can be called directly anytime like macros.
 *  - Check whether SD card auto-start is needed now.
 *  - Check whether SD print finishing is needed now.
 *  - Run one G-code command from the immediate or main command queue
 *    and open up one space. Commands in the main queue may come from sd
 *    card, host, or by direct injection. The queue will continue to fill
 *    as long as idle() or manage_inactivity() are being called.
 */
void loop() {
  do {
    idle();

    #if HAS_MEDIA
      if (card.flag.abort_sd_printing) abortSDPrinting();
      if (marlin_state == MarlinState::MF_SD_COMPLETE) finishSDPrinting();
    #endif

    queue.advance();

    #if ANY(POWER_OFF_TIMER, POWER_OFF_WAIT_FOR_COOLDOWN)
      powerManager.checkAutoPowerOff();
    #endif

    endstops.event_handler();

    TERN_(HAS_TFT_LVGL_UI, printer_state_polling());

    TERN_(MARLIN_TEST_BUILD, runPeriodicTests());

  } while (ENABLED(__AVR__)); // Loop forever on slower (AVR) boards
}
