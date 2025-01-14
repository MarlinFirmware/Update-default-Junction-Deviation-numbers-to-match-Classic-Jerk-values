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

#include "../../inc/MarlinConfig.h"

#if HAS_PRUSA_MMU2

/**
 * mmu2.cpp - Support for Průša MMU2 and MMU2S
 */

#include "mmu2.h"
#include "../../lcd/menu/menu_mmu2.h"

MMU2 mmu2;

#include "../../gcode/gcode.h"
#include "../../lcd/marlinui.h"
#include "../../libs/buzzer.h"
#include "../../libs/nozzle.h"
#include "../../module/temperature.h"
#include "../../module/planner.h"
#include "../../module/stepper.h"
#include "../../MarlinCore.h"

#if ENABLED(HOST_PROMPT_SUPPORT)
  #include "../host_actions.h"
#endif

#if ENABLED(EXTENSIBLE_UI)
  #include "../../lcd/extui/ui_api.h"
#endif

#define DEBUG_OUT ENABLED(MMU_DEBUG)
#include "../../core/debug_out.h"

#define MMU_TODELAY 100
#define MMU_TIMEOUT 10
#define MMU_CMD_TIMEOUT 45000UL // 45s timeout for mmu commands (except P0)
#define MMU_P0_TIMEOUT 3000UL   // Timeout for P0 command: 3seconds

#define MMU2_SEND(S) tx_str(F(S "\n"))
#define MMU2_RECV(S) rx_str(F(S "\n"))

#if ENABLED(MMU2_EXTRUDER_SENSOR)
  uint8_t mmu_idl_sens = 0;
  static bool mmu_loading_flag = false;
#endif

#define MMU_CMD_NONE 0
#define MMU_CMD_T0   0x10  // up to supported filaments
#define MMU_CMD_L0   0x20  // up to supported filaments
#define MMU_CMD_C0   0x30
#define MMU_CMD_U0   0x40
#define MMU_CMD_E0   0x50  // up to supported filaments
#define MMU_CMD_R0   0x60
#define MMU_CMD_F0   0x70  // up to supported filaments

#define MMU_REQUIRED_FW_BUILDNR TERN(MMU2_MODE_12V, 132, 126)

#define MMU2_NO_TOOL 99
#define MMU_BAUD    115200

bool MMU2::_enabled, MMU2::ready;
#if HAS_PRUSA_MMU2S
  bool MMU2::mmu2s_triggered;
#endif
uint8_t MMU2::cmd, MMU2::cmd_arg, MMU2::last_cmd, MMU2::extruder;
int8_t MMU2::state = 0;
volatile int8_t MMU2::finda = 1;
volatile bool MMU2::finda_runout_valid;
millis_t MMU2::prev_request, MMU2::prev_P0_request;
char MMU2::rx_buffer[MMU_RX_SIZE], MMU2::tx_buffer[MMU_TX_SIZE];

struct E_Step {
  float extrude;        //!< extrude distance in mm
  feedRate_t feedRate;  //!< feed rate in mm/s
};

inline void unscaled_mmu2_e_move(const float &dist, const feedRate_t fr_mm_s, const bool sync=true) {
  current_position.e += dist / planner.e_factor[active_extruder];
  line_to_current_position(fr_mm_s);
  if (sync) planner.synchronize();
}

MMU2::MMU2() {
  rx_buffer[0] = '\0';
}

void MMU2::init() {

  set_runout_valid(false);

  #if PIN_EXISTS(MMU_RST)
    WRITE(MMU_RST_PIN, HIGH);
    SET_OUTPUT(MMU_RST_PIN);
  #endif

  MMU_SERIAL.begin(MMU_BAUD);
  extruder = MMU2_NO_TOOL;

  safe_delay(10);
  reset();
  rx_buffer[0] = '\0';
  state = -1;
}

void MMU2::reset() {
  DEBUG_ECHOLNPGM("MMU <= reset");

  #if PIN_EXISTS(MMU_RST)
    WRITE(MMU_RST_PIN, LOW);
    safe_delay(20);
    WRITE(MMU_RST_PIN, HIGH);
  #else
    MMU2_SEND("X0");  // Send soft reset
  #endif
}

int8_t MMU2::get_current_tool() { return extruder == MMU2_NO_TOOL ? -1 : extruder; }

#if ANY(HAS_PRUSA_MMU2S, MMU2_EXTRUDER_SENSOR)
  #define FILAMENT_PRESENT() (READ(FIL_RUNOUT1_PIN) != FIL_RUNOUT1_STATE)
#else
  #define FILAMENT_PRESENT() true
#endif

void mmu2_attn_buzz(const bool two=false) {
  BUZZ(200, 404);
  if (two) { BUZZ(10, 0); BUZZ(200, 404); }
}

// Avoiding sscanf significantly reduces build size
void MMU2::mmu_loop() {

  switch (state) {

    case 0: break;

    case -1:
      if (rx_start()) {
        prev_P0_request = millis();   // Initialize finda sensor timeout
        DEBUG_ECHOLNPGM("MMU => 'start'");
        DEBUG_ECHOLNPGM("MMU <= 'S1'");
        MMU2_SEND("S1");    // Read Version
        state = -2;
      }
      else if (ELAPSED(millis(), prev_request + 30000)) { // 30sec after reset disable MMU
        SERIAL_ECHOLNPGM("MMU not responding - DISABLED");
        state = 0;
      }
      break;

    case -2:
      if (rx_ok()) {
        const uint16_t version = uint16_t(strtoul(rx_buffer, nullptr, 10));
        DEBUG_ECHOLNPGM("MMU => ", version, "\nMMU <= 'S2'");
        MMU2_SEND("S2");    // Read Build Number
        state = -3;
      }
      break;

    case -3:
      if (rx_ok()) {
        const uint16_t buildnr = uint16_t(strtoul(rx_buffer, nullptr, 10));
        DEBUG_ECHOLNPGM("MMU => ", buildnr);

        check_version(buildnr);

        #if ENABLED(MMU2_MODE_12V)
          DEBUG_ECHOLNPGM("MMU <= 'M1'");
          MMU2_SEND("M1");    // Stealth Mode
          state = -5;
        #else
          DEBUG_ECHOLNPGM("MMU <= 'P0'");
          MMU2_SEND("P0");    // Read FINDA
          state = -4;
        #endif
      }
      break;

    #if ENABLED(MMU2_MODE_12V)
      case -5:
        // response to M1
        if (rx_ok()) {
          DEBUG_ECHOLNPGM("MMU => ok");
          DEBUG_ECHOLNPGM("MMU <= 'P0'");
          MMU2_SEND("P0");    // Read FINDA
          state = -4;
        }
        break;
    #endif

    case -4:
      if (rx_ok()) {
        const uint8_t findex = uint8_t(rx_buffer[0] - '0');
        if (findex <= 1) finda = findex;

        DEBUG_ECHOLNPGM("MMU => ", finda, "\nMMU - ENABLED");

        _enabled = true;
        state = 1;
        TERN_(HAS_PRUSA_MMU2S, mmu2s_triggered = false);
      }
      break;

    case 1:
      if (cmd) {
        if (WITHIN(cmd, MMU_CMD_T0, MMU_CMD_T0 + EXTRUDERS - 1)) {
          // tool change
          const int filament = cmd - MMU_CMD_T0;
          DEBUG_ECHOLNPGM("MMU <= T", filament);
          tx_printf(F("T%d\n"), filament);
          TERN_(MMU2_EXTRUDER_SENSOR, mmu_idl_sens = 1); // enable idler sensor, if any
          state = 3; // wait for response
        }
        else if (WITHIN(cmd, MMU_CMD_L0, MMU_CMD_L0 + EXTRUDERS - 1)) {
          // load
          const int filament = cmd - MMU_CMD_L0;
          DEBUG_ECHOLNPGM("MMU <= L", filament);
          tx_printf(F("L%d\n"), filament);
          state = 3; // wait for response
        }
        else if (cmd == MMU_CMD_C0) {
          // continue loading
          DEBUG_ECHOLNPGM("MMU <= 'C0'");
          MMU2_SEND("C0");
          state = 3; // wait for response
        }
        else if (cmd == MMU_CMD_U0) {
          // unload current
          DEBUG_ECHOLNPGM("MMU <= 'U0'");
          MMU2_SEND("U0");
          state = 3; // wait for response
        }
        else if (WITHIN(cmd, MMU_CMD_E0, MMU_CMD_E0 + EXTRUDERS - 1)) {
          // eject filament
          const int filament = cmd - MMU_CMD_E0;
          DEBUG_ECHOLNPGM("MMU <= E", filament);
          tx_printf(F("E%d\n"), filament);
          state = 3; // wait for response
        }
        else if (cmd == MMU_CMD_R0) {
          // recover after eject
          DEBUG_ECHOLNPGM("MMU <= 'R0'");
          MMU2_SEND("R0");
          state = 3; // wait for response
        }
        else if (WITHIN(cmd, MMU_CMD_F0, MMU_CMD_F0 + EXTRUDERS - 1)) {
          // filament type
          const int filament = cmd - MMU_CMD_F0;
          DEBUG_ECHOLNPGM("MMU <= F", filament, " ", cmd_arg);
          tx_printf(F("F%d %d\n"), filament, cmd_arg);
          state = 3; // wait for response
        }

        last_cmd = cmd;
        cmd = MMU_CMD_NONE;
      }
      else if (ELAPSED(millis(), prev_P0_request + 300)) {
        MMU2_SEND("P0");  // Read FINDA
        state = 2; // wait for response
      }

      TERN_(HAS_PRUSA_MMU2S, check_filament());
      break;

    case 2:   // response to command P0
      if (rx_ok()) {
        const uint8_t findex = uint8_t(rx_buffer[0] - '0');
        if (findex <= 1) finda = findex;

        // This is super annoying. Only activate if necessary
        //if (finda_runout_valid) DEBUG_ECHOLNPGM("MMU <= 'P0'\nMMU => ", p_float_t(finda, 6));

        if (!finda && finda_runout_valid) filament_runout();
        if (cmd == MMU_CMD_NONE) ready = true;
        state = 1;
      }
      else if (ELAPSED(millis(), prev_request + MMU_P0_TIMEOUT)) // Resend request after timeout (3s)
        state = 1;

      TERN_(HAS_PRUSA_MMU2S, check_filament());
      break;

    case 3:   // response to mmu commands
      #if ENABLED(MMU2_EXTRUDER_SENSOR)
        if (mmu_idl_sens) {
          if (FILAMENT_PRESENT() && mmu_loading_flag) {
            DEBUG_ECHOLNPGM("MMU <= 'A'");
            MMU2_SEND("A");   // send 'abort' request
            mmu_idl_sens = 0;
            DEBUG_ECHOLNPGM("MMU IDLER_SENSOR = 0 - ABORT");
          }
        }
      #endif

      if (rx_ok()) {
        #if HAS_PRUSA_MMU2S
          // Respond to C0 MMU command in MMU2S model
          const bool keep_trying = !mmu2s_triggered && last_cmd == MMU_CMD_C0;
          if (keep_trying) {
            // MMU ok received but filament sensor not triggered, retrying...
            DEBUG_ECHOLNPGM("MMU => 'ok' (no filament in gears)");
            DEBUG_ECHOLNPGM("MMU <= 'C0' (keep trying)");
            MMU2_SEND("C0");
          }
        #else
          constexpr bool keep_trying = false;
        #endif

        if (!keep_trying) {
          DEBUG_ECHOLNPGM("MMU => 'ok'");
          ready = true;
          state = 1;
          last_cmd = MMU_CMD_NONE;
        }
      }
      else if (ELAPSED(millis(), prev_request + MMU_CMD_TIMEOUT)) {
        // resend request after timeout
        if (last_cmd) {
          DEBUG_ECHOLNPGM("MMU retry");
          cmd = last_cmd;
          last_cmd = MMU_CMD_NONE;
        }
        state = 1;
      }
      TERN_(HAS_PRUSA_MMU2S, check_filament());
      break;
  }
}

/**
 * Check if MMU was started
 */
bool MMU2::rx_start() {
  // check for start message
  return MMU2_RECV("start");
}

/**
 * Check if the data received ends with the given string.
 */
bool MMU2::rx_str(FSTR_P fstr) {
  PGM_P pstr = FTOP(fstr);

  uint8_t i = strlen(rx_buffer);

  while (MMU_SERIAL.available()) {
    rx_buffer[i++] = MMU_SERIAL.read();

    if (i == sizeof(rx_buffer) - 1) {
      DEBUG_ECHOLNPGM("rx buffer overrun");
      break;
    }
  }
  rx_buffer[i] = '\0';

  uint8_t len = strlen_P(pstr);

  if (i < len) return false;

  pstr += len;

  while (len--) {
    char c0 = pgm_read_byte(pstr--), c1 = rx_buffer[i--];
    if (c0 == c1) continue;
    if (c0 == '\r' && c1 == '\n') continue;  // match cr as lf
    if (c0 == '\n' && c1 == '\r') continue;  // match lf as cr
    return false;
  }
  return true;
}

/**
 * Transfer data to MMU, no argument
 */
void MMU2::tx_str(FSTR_P fstr) {
  clear_rx_buffer();
  PGM_P pstr = FTOP(fstr);
  while (const char c = pgm_read_byte(pstr)) { MMU_SERIAL.write(c); pstr++; }
  prev_request = millis();
}

/**
 * Transfer data to MMU, single argument
 */
void MMU2::tx_printf(FSTR_P format, int argument = -1) {
  clear_rx_buffer();
  const uint8_t len = sprintf_P(tx_buffer, FTOP(format), argument);
  for (uint8_t i = 0; i < len; ++i) MMU_SERIAL.write(tx_buffer[i]);
  prev_request = millis();
}

/**
 * Transfer data to MMU, two arguments
 */
void MMU2::tx_printf(FSTR_P format, int argument1, int argument2) {
  clear_rx_buffer();
  const uint8_t len = sprintf_P(tx_buffer, FTOP(format), argument1, argument2);
  for (uint8_t i = 0; i < len; ++i) MMU_SERIAL.write(tx_buffer[i]);
  prev_request = millis();
}

/**
 * Empty the rx buffer
 */
void MMU2::clear_rx_buffer() {
  while (MMU_SERIAL.available()) MMU_SERIAL.read();
  rx_buffer[0] = '\0';
}

/**
 * Check if we received 'ok' from MMU
 */
bool MMU2::rx_ok() {
  if (MMU2_RECV("ok")) {
    prev_P0_request = millis();
    return true;
  }
  return false;
}

/**
 * Check if MMU has compatible firmware
 */
void MMU2::check_version(const uint16_t buildnr) {
  if (buildnr < MMU_REQUIRED_FW_BUILDNR) {
    SERIAL_ERROR_MSG("Invalid MMU2 firmware. Version >= " STRINGIFY(MMU_REQUIRED_FW_BUILDNR) " required.");
    kill(GET_TEXT_F(MSG_KILL_MMU2_FIRMWARE));
  }
}

static void mmu2_not_responding() {
  LCD_MESSAGE(MSG_MMU2_NOT_RESPONDING);
  BUZZ(100, 659);
  BUZZ(200, 698);
  BUZZ(100, 659);
  BUZZ(300, 440);
  BUZZ(100, 659);
}

inline void beep_bad_cmd() { BUZZ(400, 40); }

#if HAS_PRUSA_MMU2S

  /**
   * Load filament until the sensor at the gears is triggered
   * and give up after a number of attempts set with MMU2_C0_RETRY.
   * Each try has a timeout before returning a fail state.
   */
  bool MMU2::load_to_gears() {
    command(MMU_CMD_C0);
    manage_response(true, true);
    for (uint8_t i = 0; i < MMU2_C0_RETRY; ++i) {  // Keep loading until filament reaches gears
      if (mmu2s_triggered) break;
      command(MMU_CMD_C0);
      manage_response(true, true);
      check_filament();
    }
    const bool success = mmu2s_triggered && can_load();
    if (!success) mmu2_not_responding();
    return success;
  }

  /**
   * Handle tool change
   */
  void MMU2::tool_change(const uint8_t index) {

    if (!_enabled) return;

    set_runout_valid(false);

    if (index != extruder) {
      if (ENABLED(MMU_IR_UNLOAD_MOVE) && FILAMENT_PRESENT()) {
        DEBUG_ECHOLNPGM("Unloading\n");
        while (FILAMENT_PRESENT())                      // Filament present? Keep unloading.
          unscaled_mmu2_e_move(-0.25, MMM_TO_MMS(120)); // 0.25mm is a guessed value. Adjust to preference.
      }

      stepper.disable_extruder();
      ui.status_printf(0, GET_TEXT_F(MSG_MMU2_LOADING_FILAMENT), int(index + 1));

      command(MMU_CMD_T0 + index);
      manage_response(true, true);

      if (load_to_gears()) {
        extruder = index; // filament change is finished
        active_extruder = 0;
        stepper.enable_extruder();
        SERIAL_ECHO_MSG(STR_ACTIVE_EXTRUDER, extruder);
      }
      ui.reset_status();
    }

    set_runout_valid(true);
  }

  /**
   * Handle special T?/Tx/Tc commands
   *
   * T? Gcode to extrude shouldn't have to follow, load to extruder wheels is done automatically
   * Tx Same as T?, except nozzle doesn't have to be preheated. Tc must be placed after extruder nozzle is preheated to finish filament load.
   * Tc Load to nozzle after filament was prepared by Tx and extruder nozzle is already heated.
   */
  void MMU2::tool_change(const char *special) {
      if (!_enabled) return;

      set_runout_valid(false);

      switch (*special) {
        case '?': {
          #if ENABLED(MMU_MENUS)
            const uint8_t index = mmu2_choose_filament();
            while (!thermalManager.wait_for_hotend(active_extruder, false)) safe_delay(100);
            load_to_nozzle(index);
          #else
            beep_bad_cmd();
          #endif
        } break;

        case 'x': {
          #if ENABLED(MMU_MENUS)
            planner.synchronize();
            const uint8_t index = mmu2_choose_filament();
            stepper.disable_extruder();
            command(MMU_CMD_T0 + index);
            manage_response(true, true);

            if (load_to_gears()) {
              mmu_loop();
              stepper.enable_extruder();
              extruder = index;
              active_extruder = 0;
            }
          #else
            beep_bad_cmd();
          #endif
        } break;

        case 'c': {
          while (!thermalManager.wait_for_hotend(active_extruder, false)) safe_delay(100);
          load_to_nozzle_sequence();
        } break;
      }

      set_runout_valid(true);
  }

#elif ENABLED(MMU2_EXTRUDER_SENSOR)

  /**
   * Handle tool change
   */
  void MMU2::tool_change(const uint8_t index) {
    if (!_enabled) return;

    set_runout_valid(false);

    if (index != extruder) {
      stepper.disable_extruder();
      if (FILAMENT_PRESENT()) {
        DEBUG_ECHOLNPGM("Unloading\n");
        mmu_loading_flag = false;
        command(MMU_CMD_U0);
        manage_response(true, true);
      }
      ui.status_printf(0, GET_TEXT_F(MSG_MMU2_LOADING_FILAMENT), int(index + 1));
      mmu_loading_flag = true;
      command(MMU_CMD_T0 + index);
      manage_response(true, true);
      mmu_continue_loading();
      //command(MMU_CMD_C0);
      extruder = index;
      active_extruder = 0;

      stepper.enable_extruder();
      SERIAL_ECHO_MSG(STR_ACTIVE_EXTRUDER, extruder);

      ui.reset_status();
    }

    set_runout_valid(true);
  }

  /**
   * Handle special T?/Tx/Tc commands
   *
   * T? Gcode to extrude shouldn't have to follow, load to extruder wheels is done automatically
   * Tx Same as T?, except nozzle doesn't have to be preheated. Tc must be placed after extruder nozzle is preheated to finish filament load.
   * Tc Load to nozzle after filament was prepared by Tx and extruder nozzle is already heated.
   */
  void MMU2::tool_change(const char *special) {
    if (!_enabled) return;

    set_runout_valid(false);

    switch (*special) {
      case '?': {
        DEBUG_ECHOLNPGM("case ?\n");
        #if ENABLED(MMU_MENUS)
          uint8_t index = mmu2_choose_filament();
          while (!thermalManager.wait_for_hotend(active_extruder, false)) safe_delay(100);
          load_to_nozzle(index);
        #else
          beep_bad_cmd();
        #endif
      } break;

      case 'x': {
        DEBUG_ECHOLNPGM("case x\n");
        #if ENABLED(MMU_MENUS)
          planner.synchronize();
          uint8_t index = mmu2_choose_filament();
          stepper.disable_extruder();
          command(MMU_CMD_T0 + index);
          manage_response(true, true);
          mmu_continue_loading();
          command(MMU_CMD_C0);
          mmu_loop();

          stepper.enable_extruder();
          extruder = index;
          active_extruder = 0;
        #else
          beep_bad_cmd();
        #endif
      } break;

      case 'c': {
        DEBUG_ECHOLNPGM("case c\n");
        while (!thermalManager.wait_for_hotend(active_extruder, false)) safe_delay(100);
        load_to_nozzle_sequence();
      } break;
    }

    set_runout_valid(true);
  }

  void MMU2::mmu_continue_loading() {
    // Try to load the filament a limited number of times
    bool fil_present = 0;
    for (uint8_t i = 0; i < MMU2_LOADING_ATTEMPTS_NR; i++) {
      DEBUG_ECHOLNPGM("Load attempt #", i + 1);

      // Done as soon as filament is present
      fil_present = FILAMENT_PRESENT();
      if (fil_present) break;

      // Attempt to load the filament, 1mm at a time, for 3s
      command(MMU_CMD_C0);
      stepper.enable_extruder();
      const millis_t expire_ms = millis() + 3000;
      do {
        current_position.e += 1;
        line_to_current_position(MMU_LOAD_FEEDRATE);
        planner.synchronize();
        // When (T0 rx->ok) load is ready, but in fact it did not load
        // successfully or an overload created pressure in the extruder.
        // Send (C0) to load more and move E_AXIS a little to release pressure.
        if ((fil_present = FILAMENT_PRESENT())) MMU2_SEND("A");
      } while (!fil_present && PENDING(millis(), expire_ms));
      stepper.disable_extruder();
      manage_response(true, true);
    }

    // Was the filament still missing in the last check?
    if (!fil_present) {
      DEBUG_ECHOLNPGM("Filament never reached sensor, runout");
      filament_runout();
    }
    mmu_idl_sens = 0;
  }

#else // !HAS_PRUSA_MMU2S && !MMU2_EXTRUDER_SENSOR

  /**
   * Handle tool change
   */
  void MMU2::tool_change(const uint8_t index) {
    if (!_enabled) return;

    set_runout_valid(false);

    if (index != extruder) {
      stepper.disable_extruder();
      ui.status_printf(0, GET_TEXT_F(MSG_MMU2_LOADING_FILAMENT), int(index + 1));
      command(MMU_CMD_T0 + index);
      manage_response(true, true);
      command(MMU_CMD_C0);
      extruder = index; // Filament change is finished
      active_extruder = 0;
      stepper.enable_extruder();
      SERIAL_ECHO_MSG(STR_ACTIVE_EXTRUDER, extruder);
      ui.reset_status();
    }

    set_runout_valid(true);
  }

  /**
   * Handle special T?/Tx/Tc commands
   *
   * T? Gcode to extrude shouldn't have to follow, load to extruder wheels is done automatically
   * Tx Same as T?, except nozzle doesn't have to be preheated. Tc must be placed after extruder nozzle is preheated to finish filament load.
   * Tc Load to nozzle after filament was prepared by Tx and extruder nozzle is already heated.
   */
  void MMU2::tool_change(const char *special) {
    if (!_enabled) return;

    set_runout_valid(false);

    switch (*special) {
      case '?': {
        DEBUG_ECHOLNPGM("case ?\n");
        #if ENABLED(MMU_MENUS)
          uint8_t index = mmu2_choose_filament();
          while (!thermalManager.wait_for_hotend(active_extruder, false)) safe_delay(100);
          load_to_nozzle(index);
        #else
          beep_bad_cmd();
        #endif
      } break;

      case 'x': {
        DEBUG_ECHOLNPGM("case x\n");
        #if ENABLED(MMU_MENUS)
          planner.synchronize();
          uint8_t index = mmu2_choose_filament();
          stepper.disable_extruder();
          command(MMU_CMD_T0 + index);
          manage_response(true, true);
          command(MMU_CMD_C0);
          mmu_loop();

          stepper.enable_extruder();
          extruder = index;
          active_extruder = 0;
        #else
          beep_bad_cmd();
        #endif
      } break;

      case 'c': {
        DEBUG_ECHOLNPGM("case c\n");
        while (!thermalManager.wait_for_hotend(active_extruder, false)) safe_delay(100);
        load_to_nozzle_sequence();
      } break;
    }

    set_runout_valid(true);
  }

#endif // HAS_PRUSA_MMU2S

/**
 * Set next command
 */
void MMU2::command(const uint8_t mmu_cmd) {
  if (!_enabled) return;
  cmd = mmu_cmd;
  ready = false;
}

/**
 * Wait for response from MMU
 */
bool MMU2::get_response() {
  while (cmd != MMU_CMD_NONE) idle();

  while (!ready) {
    idle();
    if (state != 3) break;
  }

  const bool ret = ready;
  ready = false;

  return ret;
}

/**
 * Wait for response and deal with timeout if necessary
 */
void MMU2::manage_response(const bool move_axes, const bool turn_off_nozzle) {

  constexpr xyz_pos_t park_point = NOZZLE_PARK_POINT;
  bool response = false, mmu_print_saved = false;
  xyz_pos_t resume_position;
  celsius_t resume_hotend_temp = thermalManager.degTargetHotend(active_extruder);

  KEEPALIVE_STATE(PAUSED_FOR_USER);

  while (!response) {

    response = get_response(); // wait for "ok" from mmu

    if (!response) {          // No "ok" was received in reserved time frame, user will fix the issue on mmu unit
      if (!mmu_print_saved) { // First occurrence. Save current position, park print head, disable nozzle heater.

        planner.synchronize();

        mmu_print_saved = true;

        SERIAL_ECHOLNPGM("MMU not responding");

        resume_hotend_temp = thermalManager.degTargetHotend(active_extruder);
        resume_position = current_position;

        if (move_axes && all_axes_homed()) nozzle.park(0, park_point);

        if (turn_off_nozzle) thermalManager.setTargetHotend(0, active_extruder);

        mmu2_not_responding();
      }
    }
    else if (mmu_print_saved) {
      SERIAL_ECHOLNPGM("\nMMU starts responding");

      if (turn_off_nozzle && resume_hotend_temp) {
        thermalManager.setTargetHotend(resume_hotend_temp, active_extruder);
        LCD_MESSAGE(MSG_HEATING);
        ERR_BUZZ();
        while (!thermalManager.wait_for_hotend(active_extruder, false)) safe_delay(1000);
      }

      LCD_MESSAGE(MSG_MMU2_RESUMING);
      mmu2_attn_buzz(true);

      #pragma GCC diagnostic push
      #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

      if (move_axes && all_axes_homed()) {
        // Move XY to starting position, then Z
        do_blocking_move_to_xy(resume_position, feedRate_t(NOZZLE_PARK_XY_FEEDRATE));
        // Move Z_AXIS to saved position
        do_blocking_move_to_z(resume_position.z, feedRate_t(NOZZLE_PARK_Z_FEEDRATE));
      }

      #pragma GCC diagnostic pop
    }
  }
}

void MMU2::set_filament_type(const uint8_t index, const uint8_t filamentType) {
  if (!_enabled) return;

  cmd_arg = filamentType;
  command(MMU_CMD_F0 + index);

  manage_response(true, true);
}

void MMU2::filament_runout() {
  queue.inject(F(MMU2_FILAMENT_RUNOUT_SCRIPT));
  planner.synchronize();
}

#if HAS_PRUSA_MMU2S

  void MMU2::check_filament() {
    const bool present = FILAMENT_PRESENT();
    if (cmd == MMU_CMD_NONE && last_cmd == MMU_CMD_C0) {
      if (present && !mmu2s_triggered) {
        DEBUG_ECHOLNPGM("MMU <= 'A'");
        MMU2_SEND("A");
      }
      // Slowly spin the extruder during C0
      else {
        while (planner.movesplanned() < 3)
          unscaled_mmu2_e_move(0.25, MMM_TO_MMS(120), false);
      }
    }
    mmu2s_triggered = present;
  }

  bool MMU2::can_load() {
    static constexpr E_Step can_load_sequence[] PROGMEM = { MMU2_CAN_LOAD_SEQUENCE },
                  can_load_increment_sequence[] PROGMEM = { MMU2_CAN_LOAD_INCREMENT_SEQUENCE };

    execute_extruder_sequence(can_load_sequence, COUNT(can_load_sequence));

    int filament_detected_count = 0;
    const int steps = (MMU2_CAN_LOAD_RETRACT) / (MMU2_CAN_LOAD_INCREMENT);
    DEBUG_ECHOLNPGM("MMU can_load:");
    for (uint8_t i = 0; i < steps; ++i) {
      execute_extruder_sequence(can_load_increment_sequence, COUNT(can_load_increment_sequence));
      check_filament(); // Don't trust the idle function
      DEBUG_CHAR(mmu2s_triggered ? 'O' : 'o');
      if (mmu2s_triggered) ++filament_detected_count;
    }

    if (filament_detected_count <= steps - (MMU2_CAN_LOAD_DEVIATION) / (MMU2_CAN_LOAD_INCREMENT)) {
      DEBUG_ECHOLNPGM(" failed.");
      return false;
    }

    DEBUG_ECHOLNPGM(" succeeded.");
    return true;
  }

#endif

// Load filament into MMU2
void MMU2::load_to_feeder(const uint8_t index) {
  if (!_enabled) return;

  command(MMU_CMD_L0 + index);
  manage_response(false, false);
  mmu2_attn_buzz();
}

/**
 * Switch material and load to nozzle
 */
bool MMU2::load_to_nozzle(const uint8_t index) {
  if (!_enabled) return false;

  if (thermalManager.tooColdToExtrude(active_extruder)) {
    mmu2_attn_buzz();
    LCD_ALERTMESSAGE(MSG_HOTEND_TOO_COLD);
    return false;
  }

  if (TERN0(MMU_IR_UNLOAD_MOVE, index != extruder) && FILAMENT_PRESENT()) {
    DEBUG_ECHOLNPGM("Unloading\n");
    ramming_sequence();                             // Unloading instructions from printer side when operating LCD
    while (FILAMENT_PRESENT())                      // Filament present? Keep unloading.
      unscaled_mmu2_e_move(-0.25, MMM_TO_MMS(120)); // 0.25mm is a guessed value. Adjust to preference.
  }

  stepper.disable_extruder();
  command(MMU_CMD_T0 + index);
  manage_response(true, true);

  const bool success = load_to_gears();
  if (success) {
    mmu_loop();
    extruder = index;
    active_extruder = 0;
    load_to_nozzle_sequence();
    mmu2_attn_buzz();
  }
  return success;
}

bool MMU2::eject_filament(const uint8_t index, const bool recover) {

  if (!_enabled) return false;

  if (thermalManager.tooColdToExtrude(active_extruder)) {
    mmu2_attn_buzz();
    LCD_ALERTMESSAGE(MSG_HOTEND_TOO_COLD);
    return false;
  }

  LCD_MESSAGE(MSG_MMU2_EJECTING_FILAMENT);

  unscaled_mmu2_e_move(-(MMU2_FILAMENTCHANGE_EJECT_FEED), MMM_TO_MMS(2500));
  command(MMU_CMD_E0 + index);
  manage_response(false, false);

  if (recover)  {
    LCD_MESSAGE(MSG_MMU2_REMOVE_AND_CLICK);
    mmu2_attn_buzz();
    TERN_(HOST_PROMPT_SUPPORT, hostui.continue_prompt(GET_TEXT_F(MSG_MMU2_EJECT_RECOVER)));
    TERN_(EXTENSIBLE_UI, ExtUI::onUserConfirmRequired(GET_TEXT_F(MSG_MMU2_EJECT_RECOVER)));
    TERN_(HAS_RESUME_CONTINUE, wait_for_user_response());
    mmu2_attn_buzz();

    command(MMU_CMD_R0);
    manage_response(false, false);
  }

  ui.reset_status();

  // no active tool
  extruder = MMU2_NO_TOOL;

  set_runout_valid(false);

  mmu2_attn_buzz();

  stepper.disable_extruder();

  return true;
}

/**
 * Unload from hotend and retract to MMU
 */
bool MMU2::unload() {

  if (!_enabled) return false;

  if (thermalManager.tooColdToExtrude(active_extruder)) {
    mmu2_attn_buzz();
    LCD_ALERTMESSAGE(MSG_HOTEND_TOO_COLD);
    return false;
  }

  // Unload sequence to optimize shape of the tip of the unloaded filament
  ramming_sequence();

  command(MMU_CMD_U0);
  manage_response(false, true);

  mmu2_attn_buzz();

  // no active tool
  extruder = MMU2_NO_TOOL;

  set_runout_valid(false);

  return true;
}

void MMU2::ramming_sequence() {
  static const E_Step sequence[] PROGMEM = { MMU2_RAMMING_SEQUENCE };
  execute_extruder_sequence(sequence, COUNT(sequence));
}

void MMU2::load_to_nozzle_sequence() {
  static const E_Step sequence[] PROGMEM = { MMU2_LOAD_TO_NOZZLE_SEQUENCE };
  execute_extruder_sequence(sequence, COUNT(sequence));
}

void MMU2::execute_extruder_sequence(const E_Step * const sequence, const uint8_t steps) {

  planner.synchronize();

  const E_Step *step = sequence;

  for (uint8_t i = 0; i < steps; ++i) {
    const float es = pgm_read_float(&(step->extrude));
    const feedRate_t fr_mm_m = pgm_read_float(&(step->feedRate));
    DEBUG_ECHO_MSG("E step ", es, "/", fr_mm_m);
    unscaled_mmu2_e_move(es, MMM_TO_MMS(fr_mm_m));
    step++;
  }

  stepper.disable_extruder();
}

#endif // HAS_PRUSA_MMU2
