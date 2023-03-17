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
 * lcd/extui/malyan/malyan.cpp
 *
 * LCD implementation for Malyan's LCD, a separate ESP8266 MCU running
 * on Serial1 for the M200 board. This module outputs a pseudo-G-code
 * wrapped in curly braces which the LCD implementation translates into
 * actual G-code commands.
 *
 * Added to Marlin for Mini/Malyan M200
 * Unknown commands as of Jan 2018: {H:}
 * Not currently implemented:
 * {E:} when sent by LCD. Meaning unknown.
 *
 * Notes for connecting to boards that are not Malyan:
 * The LCD is 3.3v, so if powering from a RAMPS 1.4 board or
 * other 5v/12v board, use a buck converter to power the LCD and
 * the 3.3v side of a logic level shifter. Aux1 on the RAMPS board
 * has Serial1 and 12v, making it perfect for this.
 * Copyright (c) 2017 Jason Nelson (xC0000005)
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(MALYAN_LCD)

//#define DEBUG_MALYAN_LCD

#include "malyan.h"
#include "../ui_api.h"
#include "../../marlinui.h"

#include "../../../sd/cardreader.h"
#include "../../../module/temperature.h"
#include "../../../module/stepper.h"
#include "../../../module/motion.h"
#include "../../../libs/duration_t.h"
#include "../../../module/printcounter.h"
#include "../../../gcode/queue.h"

#define DEBUG_OUT ENABLED(DEBUG_MALYAN_LCD)
#include "../../../core/debug_out.h"

// This is based on longest sys command + a filename, plus some buffer
// in case we encounter some data we don't recognize
// There is no evidence a line will ever be this long, but better safe than sorry
#define MAX_CURLY_COMMAND (32 + LONG_FILENAME_LENGTH) * 2

// Track incoming command bytes from the LCD
uint16_t inbound_count;

// For sending print completion messages
bool last_printing_status = false;

// Everything written needs the high bit set.
void write_to_lcd(FSTR_P const fmsg) {
  PGM_P pmsg = FTOP(fmsg);
  char encoded_message[MAX_CURLY_COMMAND];
  uint8_t message_length = _MIN(strlen_P(pmsg), sizeof(encoded_message));

  for (uint_fast8_t i = 0; i < message_length; ++i)
    encoded_message[i] = pgm_read_byte(&pmsg[i]) | 0x80;

  LCD_SERIAL.Print::write(encoded_message, message_length);
}

void write_to_lcd(const char * const cmsg) {
  char encoded_message[MAX_CURLY_COMMAND];
  const uint8_t message_length = _MIN(strlen(cmsg), sizeof(encoded_message));

  for (uint_fast8_t i = 0; i < message_length; ++i)
    encoded_message[i] = cmsg[i] | 0x80;

  LCD_SERIAL.Print::write(encoded_message, message_length);
}

// {E:<msg>} is for error states.
void set_lcd_error(FSTR_P const error, FSTR_P const component/*=nullptr*/) {
  write_to_lcd(F("{E:"));
  write_to_lcd(error);
  if (component) {
    write_to_lcd(F(" "));
    write_to_lcd(component);
  }
  write_to_lcd(F("}"));
}

/**
 * Process an LCD 'C' command.
 * These are currently all temperature commands
 * {C:T0190}
 * Set temp for hotend to 190
 * {C:P050}
 * Set temp for bed to 50
 *
 * {C:S09} set feedrate to 90 %.
 * {C:S12} set feedrate to 120 %.
 *
 * the command portion begins after the :
 */
void process_lcd_c_command(const char *command) {
  const int target_val = command[1] ? atoi(command + 1) : -1;
  if (target_val < 0) {
    DEBUG_ECHOLNPGM("UNKNOWN C COMMAND ", command);
    return;
  }
  switch (command[0]) {
    case 'C': // Cope with both V1 early rev and later LCDs.
    case 'S':
      feedrate_percentage = target_val * 10;
      LIMIT(feedrate_percentage, SPEED_EDIT_MIN, SPEED_EDIT_MAX);
      break;

    case 'T':
      // Sometimes the LCD will send commands to turn off both extruder and bed, though
      // this should not happen since the printing screen is up. Better safe than sorry.
      if (!print_job_timer.isRunning() || target_val > 0)
        ExtUI::setTargetTemp_celsius(target_val, ExtUI::extruder_t::E0);
      break;

    #if HAS_HEATED_BED
      case 'P': ExtUI::setTargetTemp_celsius(target_val, ExtUI::heater_t::BED); break;
    #endif

    default: DEBUG_ECHOLNPGM("UNKNOWN C COMMAND ", command);
  }
}

/**
 * Process an LCD 'B' command.
 * {B:0} results in: {T0:008/195}{T1:000/000}{TP:000/000}{TQ:000C}{TT:000000}
 * T0/T1 are hot end temperatures, TP is bed, TQ is percent, and TT is probably
 * time remaining (HH:MM:SS). The UI can't handle displaying a second hotend,
 * but the stock firmware always sends it, and it's always zero.
 */
void process_lcd_eb_command(const char *command) {
  char elapsed_buffer[10];
  static uint8_t iteration = 0;
  duration_t elapsed;
  switch (command[0]) {
    case '0': {
      elapsed = print_job_timer.duration();
      sprintf_P(elapsed_buffer, PSTR("%02u%02u%02u"), uint16_t(elapsed.hour()), uint16_t(elapsed.minute()) % 60, uint16_t(elapsed.second()) % 60);

      char message_buffer[MAX_CURLY_COMMAND];
      uint8_t done_pct = print_job_timer.isRunning() ? (iteration * 10) : 100;
      iteration = (iteration + 1) % 10; // Provide progress animation
      #if HAS_MEDIA
        if (ExtUI::isPrintingFromMedia() || ExtUI::isPrintingFromMediaPaused())
          done_pct = card.percentDone();
      #endif

      sprintf_P(message_buffer,
        PSTR("{T0:%03i/%03i}{T1:000/000}{TP:%03i/%03i}{TQ:%03i}{TT:%s}"),
        thermalManager.wholeDegHotend(0), thermalManager.degTargetHotend(0),
        #if HAS_HEATED_BED
          thermalManager.wholeDegBed(), thermalManager.degTargetBed(),
        #else
          0, 0,
        #endif
        TERN(HAS_MEDIA, done_pct, 0),
        elapsed_buffer
      );
      write_to_lcd(message_buffer);
    } break;

    default: DEBUG_ECHOLNPGM("UNKNOWN E/B COMMAND ", command);
  }
}

/**
 * Process an LCD 'J' command.
 * These are currently all movement commands.
 * The command portion begins after the :
 * Move X Axis
 *
 * {J:E}{J:X-200}{J:E}
 * {J:E}{J:X+200}{J:E}
 * X, Y, Z, A (extruder)
 */
template<typename T>
void j_move_axis(const char *command, const T axis) {
  const float dist = atof(command + 1) / 10.0;
  ExtUI::setAxisPosition_mm(ExtUI::getAxisPosition_mm(axis) + dist, axis);
};

void process_lcd_j_command(const char *command) {
  switch (command[0]) {
    case 'E': break;
    case 'A': j_move_axis<ExtUI::extruder_t>(command, ExtUI::extruder_t::E0); break;
    case 'Y': j_move_axis<ExtUI::axis_t>(command, ExtUI::axis_t::Y); break;
    case 'Z': j_move_axis<ExtUI::axis_t>(command, ExtUI::axis_t::Z); break;
    case 'X': j_move_axis<ExtUI::axis_t>(command, ExtUI::axis_t::X); break;
    default: DEBUG_ECHOLNPGM("UNKNOWN J COMMAND ", command);
  }
}

/**
 * Process an LCD 'P' command, related to homing and printing.
 * Cancel:
 * {P:X}
 *
 * Home all axes:
 * {P:H}
 *
 * Print a file:
 * {P:000}
 * The File number is specified as a three digit value.
 * Printer responds with:
 * {PRINTFILE:Mini_SNES_Bottom.gcode}
 * {SYS:BUILD}echo:Now fresh file: Mini_SNES_Bottom.gcode
 * File opened: Mini_SNES_Bottom.gcode Size: 5805813
 * File selected
 * {SYS:BUILD}
 * T:-2526.8 E:0
 * T:-2533.0 E:0
 * T:-2537.4 E:0
 * Note only the curly brace stuff matters.
 */
void process_lcd_p_command(const char *command) {

  switch (command[0]) {
    case 'P':
        ExtUI::pausePrint();
        write_to_lcd(F("{SYS:PAUSED}"));
        break;
    case 'R':
        ExtUI::resumePrint();
        write_to_lcd(F("{SYS:RESUMED}"));
        break;
    case 'X':
        write_to_lcd(F("{SYS:CANCELING}"));
        ExtUI::stopPrint();
        write_to_lcd(F("{SYS:STARTED}"));
        break;
    case 'H': queue.enqueue_now_P(G28_STR); break; // Home all axes
    default: {
      #if HAS_MEDIA
        // Print file 000 - a three digit number indicating which
        // file to print in the SD card. If it's a directory,
        // then switch to the directory.

        // Find the name of the file to print.
        // It's needed to echo the PRINTFILE option.
        // The {S:L} command should've ensured the SD card was mounted.
        card.selectFileByIndex(atoi(command));

        // There may be a difference in how V1 and V2 LCDs handle subdirectory
        // prints. Investigate more. This matches the V1 motion controller actions
        // but the V2 LCD switches to "print" mode on {SYS:DIR} response.
        if (card.flag.filenameIsDir) {
          card.cd(card.filename);
          write_to_lcd(F("{SYS:DIR}"));
        }
        else {
          char message_buffer[MAX_CURLY_COMMAND];
          sprintf_P(message_buffer, PSTR("{PRINTFILE:%s}"), card.longest_filename());
          write_to_lcd(message_buffer);
          write_to_lcd(F("{SYS:BUILD}"));
          card.openAndPrintFile(card.filename);
        }
      #endif
    } break; // default
  } // switch
}

/**
 * Handle an lcd 'S' command
 * {S:I} - Temperature request
 * {T0:999/000}{T1:000/000}{TP:004/000}
 *
 * {S:L} - File Listing request
 * Printer Response:
 * {FILE:buttons.gcode}
 * {FILE:update.bin}
 * {FILE:nupdate.bin}
 * {FILE:fcupdate.flg}
 * {SYS:OK}
 */
void process_lcd_s_command(const char *command) {
  switch (command[0]) {
    case 'I': {
      // temperature information
      char message_buffer[MAX_CURLY_COMMAND];
      sprintf_P(message_buffer, PSTR("{T0:%03i/%03i}{T1:000/000}{TP:%03i/%03i}"),
        thermalManager.wholeDegHotend(0), thermalManager.degTargetHotend(0),
        #if HAS_HEATED_BED
          thermalManager.wholeDegBed(), thermalManager.degTargetBed()
        #else
          0, 0
        #endif
      );
      write_to_lcd(message_buffer);
    } break;

    case 'L': {
      #if HAS_MEDIA
        if (!card.isMounted()) card.mount();

        // A more efficient way to do this would be to
        // implement a callback in the ls_SerialPrint code, but
        // that requires changes to the core cardreader class that
        // would not benefit the majority of users. Since one can't
        // select a file for printing during a print, there's
        // little reason not to do it this way.
        char message_buffer[MAX_CURLY_COMMAND];
        int16_t file_count = card.get_num_items();
        for (int16_t i = 0; i < file_count; i++) {
          card.selectFileByIndex(i);
          sprintf_P(message_buffer, card.flag.filenameIsDir ? PSTR("{DIR:%s}") : PSTR("{FILE:%s}"), card.longest_filename());
          write_to_lcd(message_buffer);
        }

        write_to_lcd(F("{SYS:OK}"));
      #endif
    } break;

    default: DEBUG_ECHOLNPGM("UNKNOWN S COMMAND ", command);
  }
}

/**
 * Receive a curly brace command and translate to G-code.
 * Currently {E:0} is not handled. Its function is unknown,
 * but it occurs during the temp window after a sys build.
 */
void process_lcd_command(const char *command) {
  const char *current = command;

  byte command_code = *current++;
  if (*current == ':') {

    current++; // skip the :

    switch (command_code) {
      case 'S': process_lcd_s_command(current); break;
      case 'J': process_lcd_j_command(current); break;
      case 'P': process_lcd_p_command(current); break;
      case 'C': process_lcd_c_command(current); break;
      case 'B':
      case 'E': process_lcd_eb_command(current); break;
      default: DEBUG_ECHOLNPGM("UNKNOWN COMMAND ", command);
    }
  }
  else
    DEBUG_ECHOLNPGM("UNKNOWN COMMAND FORMAT ", command);
}

//
// Parse LCD commands mixed with G-Code
//
void parse_lcd_byte(const byte b) {
  static char inbound_buffer[MAX_CURLY_COMMAND];

  static uint8_t parsing = 0;                   // Parsing state
  static bool prevcr = false;                   // Was the last c a CR?

  const char c = b & 0x7F;

  if (parsing) {
    const bool is_lcd = parsing == 1;           // 1 for LCD
    if ( ( is_lcd && c == '}')                  // Closing brace on LCD command
      || (!is_lcd && c == '\n')                 // LF on a G-code command
    ) {
      inbound_buffer[inbound_count] = '\0';     // Reset before processing
      inbound_count = 0;                        // Reset buffer index
      if (parsing == 1)
        process_lcd_command(inbound_buffer);    // Handle the LCD command
      else
        queue.enqueue_one_now(inbound_buffer);  // Handle the G-code command
      parsing = 0;                              // Unflag and...
    }
    else if (inbound_count < MAX_CURLY_COMMAND - 2)
      inbound_buffer[inbound_count++] = is_lcd ? c : b; // Buffer while space remains
  }
  else {
         if (c == '{')            parsing = 1;  // Brace opens an LCD command
    else if (prevcr && c == '\n') parsing = 2;  // CRLF indicates G-code
    prevcr = (c == '\r');                       // Remember if it was a CR
  }
}

/**
 * UC means connected.
 * UD means disconnected
 * The stock firmware considers USB initialized as "connected."
 */
void update_usb_status(const bool forceUpdate) {
  static bool last_usb_connected_status = false;
  // This is mildly different than stock, which
  // appears to use the usb discovery status.
  // This is more logical.
  if (last_usb_connected_status != MYSERIAL1.connected() || forceUpdate) {
    last_usb_connected_status = MYSERIAL1.connected();
    write_to_lcd(last_usb_connected_status ? F("{R:UC}\r\n") : F("{R:UD}\r\n"));
  }
}

#endif // MALYAN_LCD
