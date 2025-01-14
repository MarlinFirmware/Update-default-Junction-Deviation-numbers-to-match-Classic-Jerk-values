/**
 * anycubic_i3mega_lcd.cpp  --- Support for Anycubic i3 Mega TFT
 * Created by Christian Hopp on 09.12.17.
 * Improved by David Ramiro
 * Converted to ExtUI by John BouAntoun 21 June 2020
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(ANYCUBIC_LCD_I3MEGA)

//#define ANYCUBIC_LCD_DEBUG
#define DEBUG_OUT ENABLED(ANYCUBIC_LCD_DEBUG)
#include "../../../core/debug_out.h"

#include "anycubic_i3mega_lcd.h"
#include "../ui_api.h"

#include "../../../libs/numtostr.h"
#include "../../../module/stepper.h" // for disable_all_steppers
#include "../../../module/motion.h"  // for quickstop_stepper, A20 read printing speed, feedrate_percentage
#include "../../../MarlinCore.h"     // for disable_steppers
#include "../../../inc/MarlinConfig.h"

// command sending macro's with debugging capability
#define SEND_PGM(x)       send_P(PSTR(x))
#define SENDLINE_PGM(x)   sendLine_P(PSTR(x))
#define SEND_PGM_VAL(x,y) (send_P(PSTR(x)), sendLine(i16tostr3rj(y)))
#define SEND(x)           send(x)
#define SENDLINE(x)       sendLine(x)
#define SENDLINE_DBG_PGM(x,y)       do{ sendLine_P(PSTR(x)); DEBUG_ECHOLNPGM(y); }while(0)
#define SENDLINE_DBG_PGM_VAL(x,y,z) do{ sendLine_P(PSTR(x)); DEBUG_ECHOLNPGM(y, z); }while(0)

// Append ".gcode" to filename, if requested. Used for some DGUS-clone displays with built-in filter.
// Filenames are limited to 26 characters, so the actual name for the FILENAME can be 20 characters at most.
// If a longer string is desired without "extension, use the ALTNAME macro to provide a (longer) alternative.
#define SPECIAL_MENU_FILENAME(A) A TERN_(ANYCUBIC_LCD_GCODE_EXT, ".gcode")
#define SPECIAL_MENU_ALTNAME(A, B) TERN(ANYCUBIC_LCD_GCODE_EXT, A ".gcode", B)

AnycubicTFT anycubicTFT;

char AnycubicTFT::tftCommands[TFTBUFSIZE][TFT_MAX_CMD_SIZE];
int AnycubicTFT::tftBufLen = 0,
    AnycubicTFT::tftBufIndR = 0,
    AnycubicTFT::tftBufIndW = 0;
char* AnycubicTFT::tftStrchrPtr;
uint8_t AnycubicTFT::specialMenu = false;
AnycubicMediaPrintState AnycubicTFT::mediaPrintingState = AMPRINTSTATE_NOT_PRINTING;
AnycubicMediaPauseState AnycubicTFT::mediaPauseState = AMPAUSESTATE_NOT_PAUSED;

char AnycubicTFT::selectedDirectory[30];
char AnycubicTFT::selectedFile[FILENAME_LENGTH];

// Serial helpers
static void sendNewLine() { LCD_SERIAL.write('\r'); LCD_SERIAL.write('\n'); }
static void send(const char *str) { LCD_SERIAL.print(str); }
static void send_P(PGM_P str) {
  while (const char c = pgm_read_byte(str++))
    LCD_SERIAL.write(c);
}
static void sendLine(const char *str) { send(str); sendNewLine(); }
static void sendLine_P(PGM_P str) { send_P(str); sendNewLine(); }

using namespace ExtUI;

AnycubicTFT::AnycubicTFT() {}

void AnycubicTFT::onSetup() {
  #ifndef LCD_BAUDRATE
    #define LCD_BAUDRATE 115200
  #endif
  LCD_SERIAL.begin(LCD_BAUDRATE);

  SENDLINE_DBG_PGM("J17", "TFT Serial Debug: Main board reset... J17"); // J17 Main board reset
  delay_ms(10);

  // Init the state of the key pins running on the TFT
  #if HAS_FILAMENT_SENSOR
    SET_INPUT_PULLUP(FIL_RUNOUT1_PIN);
  #endif

  mediaPrintingState = AMPRINTSTATE_NOT_PRINTING;
  mediaPauseState = AMPAUSESTATE_NOT_PAUSED;

  // doSDCardStateCheck();
  SENDLINE_DBG_PGM("J12", "TFT Serial Debug: Ready... J12"); // J12 Ready
  delay_ms(10);

  doFilamentRunoutCheck();
  selectedFile[0] = 0;

  #if ENABLED(STARTUP_CHIME)
    injectCommands(F("M300 P250 S554\nM300 P250 S554\nM300 P250 S740\nM300 P250 S554\nM300 P250 S740\nM300 P250 S554\nM300 P500 S831"));
  #endif
  DEBUG_ECHOLNPGM("TFT Serial Debug: Finished startup");
}

void AnycubicTFT::onCommandScan() {
  static millis_t nextStopCheck = 0; // used to slow the stopped print check down to reasonable times
  const millis_t ms = millis();
  if (ELAPSED(ms, nextStopCheck)) {
    nextStopCheck = ms + 1000UL;
    if (mediaPrintingState == AMPRINTSTATE_STOP_REQUESTED && isNozzleHomed()) {
      DEBUG_ECHOLNPGM("TFT Serial Debug: Finished stopping print, releasing motors ...");
      mediaPrintingState = AMPRINTSTATE_NOT_PRINTING;
      mediaPauseState = AMPAUSESTATE_NOT_PAUSED;
      injectCommands(F("M84\nM27")); // disable stepper motors and force report of SD status
      delay_ms(200);
      // tell printer to release resources of print to indicate it is done
      SENDLINE_DBG_PGM("J14", "TFT Serial Debug: SD Print Stopped... J14");
    }
  }

  if (tftBufLen < (TFTBUFSIZE) - 1)
    getCommandFromTFT();

  if (tftBufLen) {
    --tftBufLen;
    tftBufIndR = (tftBufIndR + 1) % (TFTBUFSIZE);
  }
}

void AnycubicTFT::onKillTFT() {
  SENDLINE_DBG_PGM("J11", "TFT Serial Debug: Kill command... J11");
}

void AnycubicTFT::onSDCardStateChange(bool isMounted) {
  DEBUG_ECHOLNPGM("TFT Serial Debug: onSDCardStateChange event triggered...", isMounted);
  doSDCardStateCheck();
}

void AnycubicTFT::onSDCardError() {
  DEBUG_ECHOLNPGM("TFT Serial Debug: onSDCardError event triggered...");
  SENDLINE_DBG_PGM("J21", "TFT Serial Debug: On SD Card Error ... J21");
}

void AnycubicTFT::onFilamentRunout() {
  DEBUG_ECHOLNPGM("TFT Serial Debug: onFilamentRunout triggered...");
  doFilamentRunoutCheck();
}

void AnycubicTFT::onUserConfirmRequired(const char * const msg) {
  DEBUG_ECHOLNPGM("TFT Serial Debug: onUserConfirmRequired triggered... ", msg);

  #if HAS_MEDIA
    /**
     * Need to handle the process of following states
     * "Nozzle Parked"
     * "Load Filament"
     * "Filament Purging..."
     * "HeaterTimeout"
     * "Reheat finished."
     *
     * NOTE:  The only way to handle these states is strcmp_P with the msg unfortunately (very expensive)
     */
    if (strcmp_P(msg, PSTR("Nozzle Parked")) == 0) {
      mediaPrintingState = AMPRINTSTATE_PAUSED;
      mediaPauseState    = AMPAUSESTATE_PARKED;
      // enable continue button
      SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm SD print paused done... J18");
    }
    else if (strcmp_P(msg, PSTR("Load Filament")) == 0) {
      mediaPrintingState = AMPRINTSTATE_PAUSED;
      mediaPauseState    = AMPAUSESTATE_FILAMENT_OUT;
      // enable continue button
      SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm Filament is out... J18");
      SENDLINE_DBG_PGM("J23", "TFT Serial Debug: UserConfirm Blocking filament prompt... J23");
    }
    else if (strcmp_P(msg, PSTR("Filament Purging...")) == 0) {
      mediaPrintingState = AMPRINTSTATE_PAUSED;
      mediaPauseState    = AMPAUSESTATE_PARKING;
      // TODO: JBA I don't think J05 just disables the continue button, i think it injects a rogue M25. So taking this out
      // disable continue button
      // SENDLINE_DBG_PGM("J05", "TFT Serial Debug: UserConfirm SD Filament Purging... J05"); // J05 printing pause

      // enable continue button
      SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm Filament is purging... J18");
    }
    else if (strcmp_P(msg, PSTR("HeaterTimeout")) == 0) {
      mediaPrintingState = AMPRINTSTATE_PAUSED;
      mediaPauseState    = AMPAUSESTATE_HEATER_TIMEOUT;
      // enable continue button
      SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm SD Heater timeout... J18");
    }
    else if (strcmp_P(msg, PSTR("Reheat finished.")) == 0) {
      mediaPrintingState = AMPRINTSTATE_PAUSED;
      mediaPauseState    = AMPAUSESTATE_REHEAT_FINISHED;
      // enable continue button
      SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm SD Reheat done... J18");
    }
  #endif
}

float AnycubicTFT::codeValue() {
  return (strtod(&tftCommands[tftBufIndR][tftStrchrPtr - tftCommands[tftBufIndR] + 1], nullptr));
}

bool AnycubicTFT::codeSeen(char code) {
  tftStrchrPtr = strchr(tftCommands[tftBufIndR], code);
  return !!tftStrchrPtr; // Return True if a character was found
}

bool AnycubicTFT::isNozzleHomed() {
  const float xPosition = getAxisPosition_mm((axis_t) X);
  const float yPosition = getAxisPosition_mm((axis_t) Y);
  return WITHIN(xPosition, X_MIN_POS - 0.1, X_MIN_POS + 0.1) &&
         WITHIN(yPosition, Y_MIN_POS - 0.1, Y_MIN_POS + 0.1);
}

void AnycubicTFT::handleSpecialMenu() {
  /**
   * NOTE: that the file selection command actual lowercases the entire selected file/foldername, so charracter comparisons need to be lowercase.
   */
  if (selectedDirectory[0] != '<') {
    DEBUG_ECHOLNPGM("TFT Serial Debug: Attempted to handleSpecialMenu on non-special menu... ", selectedDirectory);
    return;
  }

  switch (selectedDirectory[1]) {
    default: break;

    case 'e': specialMenu = false; return; // "<exit>"

    #if ENABLED(PROBE_MANUALLY)

      case '0':
        switch (selectedDirectory[2]) {
          default: break;

          case '1': // "<01ZUp0.1>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Z Up 0.1"));
            injectCommands(F("G91\nG1 Z+0.1\nG90"));
            break;

          case '2': // "<02ZUp0.02>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Z Up 0.02"));
            injectCommands(F("G91\nG1 Z+0.02\nG90"));
            break;

          case '3': // "<03ZDn0.02>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Z Down 0.02"));
            injectCommands(F("G91\nG1 Z-0.02\nG90"));
            break;

          case '4': // "<04ZDn0.1>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Z Down 0.1"));
            injectCommands(F("G91\nG1 Z-0.1\nG90"));
            break;

          case '5': // "<05PrehtBed>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Preheat Bed"));
            injectCommands(F("M140 S65"));
            break;

          case '6': // "<06SMeshLvl>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Start Mesh Leveling"));
            injectCommands(F("G29S1"));
            break;

          case '7': // "<07MeshNPnt>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Next Mesh Point"));
            injectCommands(F("G29S2"));
            break;

          case '8': // "<08HtEndPID>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Auto Tune Hotend PID"));
            // need to dwell for half a second to give the fan a chance to start before the pid tuning starts
            injectCommands(F("M106 S204\nG4 P500\nM303 E0 S215 C15 U1"));
            break;

          case '9': // "<09HtBedPID>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Auto Tune Hotbed PID"));
            injectCommands(F("M303 E-1 S65 C6 U1"));
            break;
        }
        break;

      case '1':
        switch (selectedDirectory[2]) {
          default: break;

          case '0': // "<10FWDeflts>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Load FW Defaults"));
            injectCommands(F("M502\nM300 P105 S1661\nM300 P210 S1108"));
            break;

          case '1': // "<11SvEEPROM>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Save EEPROM"));
            injectCommands(F("M500\nM300 P105 S1108\nM300 P210 S1661"));
            break;
        }
        break;

    #else // !PROBE_MANUALLY

      case '0':
        switch (selectedDirectory[2]) {
          default: break;

          case '1': // "<01PrehtBed>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Preheat Bed"));
            injectCommands(F("M140 S65"));
            break;

          case '2': // "<02ABL>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Auto Bed Leveling"));
            injectCommands(F("G29N"));
            break;

          case '3': // "<03HtendPID>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Auto Tune Hotend PID"));
            // need to dwell for half a second to give the fan a chance to start before the pid tuning starts
            injectCommands(F("M106 S204\nG4 P500\nM303 E0 S215 C15 U1"));
            break;

          case '4': // "<04HtbedPID>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Auto Tune Hotbed PID"));
            injectCommands(F("M303 E-1 S65 C6 U1"));
            break;

          case '5': // "<05FWDeflts>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Load FW Defaults"));
            injectCommands(F("M502\nM300 P105 S1661\nM300 P210 S1108"));
            break;

          case '6': // "<06SvEEPROM>"
            SERIAL_ECHOLNPGM("Special Menu: ", F("Save EEPROM"));
            injectCommands(F("M500\nM300 P105 S1108\nM300 P210 S1661"));
            break;

          case '7': // <07SendM108>
            SERIAL_ECHOLNPGM("Special Menu: ", F("Send User Confirmation"));
            injectCommands(F("M108"));
            break;
        }
        break;

    #endif // !PROBE_MANUALLY
  }

}

void AnycubicTFT::renderCurrentFileList() {
  #if HAS_MEDIA
    uint16_t selectedNumber = 0;
    selectedDirectory[0] = 0;
    selectedFile[0] = 0;
    FileList currentFileList;

    SENDLINE_PGM("FN "); // Filelist start

    if (!isMediaMounted() && !specialMenu) {
      SENDLINE_DBG_PGM("J02", "TFT Serial Debug: No SD Card mounted to render Current File List... J02");

      SENDLINE_PGM("<SPECI~1.GCO");
      SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Special Menu>"));
    }
    else {
      if (codeSeen('S'))
        selectedNumber = codeValue();

      if (specialMenu)
        renderSpecialMenu(selectedNumber);
      else if (selectedNumber <= currentFileList.count())
        renderCurrentFolder(selectedNumber);
    }
    SENDLINE_PGM("END"); // Filelist stop
  #endif // HAS_MEDIA
}

void AnycubicTFT::renderSpecialMenu(uint16_t selectedNumber) {

  switch (selectedNumber) {
    default: break;

    #if ENABLED(PROBE_MANUALLY)
      case 0: // First Page
        SENDLINE_PGM("<01ZUP~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Z Up 0.1>"));
        SENDLINE_PGM("<02ZUP~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Z Up 0.02>"));
        SENDLINE_PGM("<03ZDO~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Z Down 0.02>"));
        SENDLINE_PGM("<04ZDO~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Z Down 0.1>"));
        break;

      case 4: // Second Page
        SENDLINE_PGM("<05PRE~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Preheat Bed>"));
        SENDLINE_PGM("<06MES~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_ALTNAME("<Mesh Leveling>", "<Start Mesh Leveling>"));
        SENDLINE_PGM("<07NEX~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Next Mesh Point>"));
        SENDLINE_PGM("<08PID~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<PID Tune Hotend>"));
        break;

      case 8: // Third Page
        SENDLINE_PGM("<09PID~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<PID Tune Hotbed>"));
        SENDLINE_PGM("<10FWD~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Load FW Defaults>"));
        SENDLINE_PGM("<11SAV~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Save EEPROM>"));
        SENDLINE_PGM("<EXIT_~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Exit>"));
        break;

    #else // !PROBE_MANUALLY

      case 0: // First Page
        SENDLINE_PGM("<01PRE~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Preheat Bed>"));
        SENDLINE_PGM("<02ABL~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Auto Bed Leveling>"));
        SENDLINE_PGM("<03PID~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_ALTNAME("<PID Tune Hotend>", "<Auto Tune Hotend PID>"));
        SENDLINE_PGM("<04PID~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_ALTNAME("<PID Tune Hotbed>", "<Auto Tune Hotbed PID>"));
        break;

      case 4: // Second Page
        SENDLINE_PGM("<05FWD~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Load FW Defaults>"));
        SENDLINE_PGM("<06SAV~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Save EEPROM>"));
        SENDLINE_PGM("<06SEN~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_ALTNAME("<User Confirmation>", "<Send User Confirmation>"));
        SENDLINE_PGM("<EXIT_~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Exit>"));
        break;

    #endif // !PROBE_MANUALLY
  }
}

void AnycubicTFT::renderCurrentFolder(uint16_t selectedNumber) {
  FileList currentFileList;
  const uint16_t dir_files = currentFileList.count(),
                 max_files = (dir_files - selectedNumber) < 4 ? dir_files : selectedNumber + 3;

  for (uint16_t cnt = selectedNumber; cnt <= max_files; cnt++) {
    if (cnt == 0) { // Special Entry
      if (currentFileList.isAtRootDir()) {
        SENDLINE_PGM("<SPECI~1.GCO");
        SENDLINE_PGM(SPECIAL_MENU_FILENAME("<Special Menu>"));
      }
      else {
        SENDLINE_PGM("/..");
        SENDLINE_PGM("/..");
      }
    }
    else {
      currentFileList.seek(cnt - 1, false);

      DEBUG_ECHOLN(currentFileList.filename());
      if (currentFileList.isDir()) {
        SEND_PGM("/");
        SENDLINE(currentFileList.shortFilename());
        SEND_PGM("/");
        SENDLINE(currentFileList.filename());

      }
      else {
        SENDLINE(currentFileList.shortFilename());
        SENDLINE(currentFileList.filename());
      }
    }
  }
}

void AnycubicTFT::onPrintTimerStarted() {
  #if HAS_MEDIA
    if (mediaPrintingState == AMPRINTSTATE_PRINTING)
      SENDLINE_DBG_PGM("J04", "TFT Serial Debug: Starting SD Print... J04"); // J04 Starting Print
  #endif
}

void AnycubicTFT::onPrintTimerPaused() {
  #if HAS_MEDIA
    if (isPrintingFromMedia()) {
      mediaPrintingState = AMPRINTSTATE_PAUSED;
      mediaPauseState    = AMPAUSESTATE_PARKING;
    }
  #endif
}

void AnycubicTFT::onPrintTimerStopped() {
  #if HAS_MEDIA
    if (mediaPrintingState == AMPRINTSTATE_PRINTING) {
      mediaPrintingState = AMPRINTSTATE_NOT_PRINTING;
      mediaPauseState    = AMPAUSESTATE_NOT_PAUSED;
      SENDLINE_DBG_PGM("J14", "TFT Serial Debug: SD Print Completed... J14");
    }
    // otherwise it was stopped by the printer so don't send print completed signal to TFT
  #endif
}

#define ROUND(val) int((val)+0.5f)

void AnycubicTFT::getCommandFromTFT() {
  static int serial_count = 0;

  char *starpos = nullptr;
  while (LCD_SERIAL.available() > 0 && tftBufLen < (TFTBUFSIZE)) {
    char c = LCD_SERIAL.read();
    if (c != '\n' && c != '\r' && c != ':' && serial_count < (TFT_MAX_CMD_SIZE - 1)) {
      tftCommands[tftBufIndW][serial_count++] = c;
      continue;
    }

    if (!serial_count) return; // if empty line

    tftCommands[tftBufIndW][serial_count] = 0; // terminate string

    if ((strchr(tftCommands[tftBufIndW], 'A') != nullptr)) {
      int16_t a_command;
      tftStrchrPtr = strchr(tftCommands[tftBufIndW], 'A');
      a_command = ((int)((strtod(&tftCommands[tftBufIndW][tftStrchrPtr - tftCommands[tftBufIndW] + 1], nullptr))));

      if (a_command > 7 && a_command != 20) { // No debugging of status polls, please!
        DEBUG_ECHOLNPGM("TFT Serial Command: ", tftCommands[tftBufIndW]);
      }

      switch (a_command) {
        case 0: { // A0 GET HOTEND TEMP
          const celsius_float_t hotendActualTemp = getActualTemp_celsius(E0);
          SEND_PGM_VAL("A0V ", ROUND(hotendActualTemp));
        }
        break;

        case 1: { // A1  GET HOTEND TARGET TEMP
          const celsius_float_t hotendTargetTemp = getTargetTemp_celsius(E0);
          SEND_PGM_VAL("A1V ", ROUND(hotendTargetTemp));
        }
        break;

        case 2: { // A2 GET HOTBED TEMP
          const celsius_float_t heatedBedActualTemp = getActualTemp_celsius(BED);
          SEND_PGM_VAL("A2V ", ROUND(heatedBedActualTemp));
        }
        break;

        case 3: { // A3 GET HOTBED TARGET TEMP
          const celsius_float_t heatedBedTargetTemp = getTargetTemp_celsius(BED);
          SEND_PGM_VAL("A3V ", ROUND(heatedBedTargetTemp));
        } break;

        case 4: { // A4 GET FAN SPEED
          SEND_PGM_VAL("A4V ", int(getActualFan_percent(FAN0)));
        } break;

        case 5: { // A5 GET CURRENT COORDINATE
          const float xPosition = getAxisPosition_mm(X),
                      yPosition = getAxisPosition_mm(Y),
                      zPosition = getAxisPosition_mm(Z);
          SEND_PGM("A5V X: "); LCD_SERIAL.print(xPosition);
          SEND_PGM(   " Y: "); LCD_SERIAL.print(yPosition);
          SEND_PGM(   " Z: "); LCD_SERIAL.print(zPosition);
          SENDLINE_PGM("");
        } break;

        case 6: // A6 GET SD CARD PRINTING STATUS
          #if HAS_MEDIA
            if (isPrintingFromMedia()) {
              SEND_PGM("A6V ");
              if (isMediaMounted())
                SENDLINE(ui8tostr3rj(getProgress_percent()));
              else
                SENDLINE_DBG_PGM("J02", "TFT Serial Debug: No SD Card mounted to return printing status... J02");
            }
            else
              SENDLINE_PGM("A6V ---");
          #endif
          break;

        case 7: { // A7 GET PRINTING TIME
          const uint32_t elapsedSeconds = getProgress_seconds_elapsed();
          SEND_PGM("A7V ");
          if (elapsedSeconds != 0) {  // print time
            const uint32_t elapsedMinutes = elapsedSeconds / 60;
            SEND(ui8tostr2(elapsedMinutes / 60));
            SEND_PGM(" H ");
            SEND(ui8tostr2(elapsedMinutes % 60));
            SENDLINE_PGM(" M");
          }
          else
            SENDLINE_PGM(" 999:999");
        }
        break;

        case 8: // A8 GET  SD LIST
          #if HAS_MEDIA
            selectedFile[0] = 0;
            renderCurrentFileList();
          #endif
          break;

        case 9: // A9 pause sd print
          TERN_(HAS_MEDIA, if (isPrintingFromMedia()) pausePrint());
          break;

        case 10: // A10 resume sd print
          #if HAS_MEDIA
            if (isPrintingFromMediaPaused())
              resumePrint();
          #endif
          break;

        case 11: // A11 STOP SD PRINT
          TERN_(HAS_MEDIA, stopPrint());
          break;

        case 12: // A12 kill
          kill(F(STR_ERR_KILLED));
          break;

        case 13: // A13 SELECTION FILE
          #if HAS_MEDIA
            if (isMediaMounted()) {
              starpos = (strchr(tftStrchrPtr + 4, '*'));
              if (tftStrchrPtr[4] == '/') {
                strcpy(selectedDirectory, tftStrchrPtr + 5);
                selectedFile[0] = 0;
                SENDLINE_DBG_PGM("J21", "TFT Serial Debug: Clear file selection... J21 "); // J21 Not File Selected
                SENDLINE_PGM("");
              }
              else if (tftStrchrPtr[4] == '<') {
                strcpy(selectedDirectory, tftStrchrPtr + 4);
                specialMenu = true;
                selectedFile[0] = 0;
                SENDLINE_DBG_PGM("J21", "TFT Serial Debug: Clear file selection... J21 "); // J21 Not File Selected
                SENDLINE_PGM("");
              }
              else {
                selectedDirectory[0] = 0;

                if (starpos) *(starpos - 1) = '\0';

                strcpy(selectedFile, tftStrchrPtr + 4);
                SENDLINE_DBG_PGM_VAL("J20", "TFT Serial Debug: File Selected... J20 ", selectedFile); // J20 File Selected
              }
            }
          #endif
          break;

        case 14: // A14 START PRINTING
          #if HAS_MEDIA
            if (!isPrinting() && strlen(selectedFile) > 0)
              startPrint();
          #endif
          break;

        case 15: // A15 RESUMING FROM OUTAGE
          // TODO: JBA implement resume form outage
          break;

        case 16: { // A16 set hotend temp
          uint16_t tempvalue;
          if (codeSeen('S')) {
            tempvalue = constrain(codeValue(), 0, 275);
            setTargetTemp_celsius(tempvalue, (extruder_t)E0);
          }
          else if (codeSeen('C') && !isPrinting()) {
            if (getAxisPosition_mm(Z) < 10)
              injectCommands(F("G1 Z10")); // RASE Z AXIS
            tempvalue = constrain(codeValue(), 0, 275);
            setTargetTemp_celsius(tempvalue, (extruder_t)E0);
          }
        }
        break;

        case 17: { // A17 set heated bed temp
          uint16_t tempbed;
          if (codeSeen('S')) {
            tempbed = constrain(codeValue(), 0, 100);
            setTargetTemp_celsius(tempbed, (heater_t)BED);
          }
        }
        break;

        case 18: { // A18 set fan speed
          float fanPercent;
          if (codeSeen('S')) {
            fanPercent = constrain(codeValue(), 0, 100);
            setTargetFan_percent(fanPercent, FAN0);
          }
          else
            fanPercent = 100;

          setTargetFan_percent(fanPercent, FAN0);
          SENDLINE_PGM("");
        }
        break;

        case 19: // A19 stop stepper drivers - sent on stop extrude command and on turn motors off command
          if (!isPrinting()) {
            quickstop_stepper();
            stepper.disable_all_steppers();
          }

          SENDLINE_PGM("");
          break;

        case 20: // A20 read printing speed
          if (codeSeen('S'))
            feedrate_percentage = constrain(codeValue(), 40, 999);
          else
            SEND_PGM_VAL("A20V ", feedrate_percentage);
          break;

        case 21: // A21 all home
          if (!isPrinting() && !isPrintingFromMediaPaused()) {
            if (codeSeen('X') || codeSeen('Y') || codeSeen('Z')) {
              if (codeSeen('X'))
                injectCommands(F("G28X"));
              if (codeSeen('Y'))
                injectCommands(F("G28Y"));
              if (codeSeen('Z'))
                injectCommands(F("G28Z"));
            }
            else if (codeSeen('C')) {
              injectCommands_P(G28_STR);
            }
          }
          break;

        case 22: // A22 move X/Y/Z or extrude
          if (!isPrinting()) {
            float coorvalue;
            uint16_t movespeed = 0;
            char commandStr[30];
            char fullCommandStr[38];

            commandStr[0] = 0; // empty string
            if (codeSeen('F'))  // Set feedrate
              movespeed = codeValue();

            if (codeSeen('X')) { // Move in X direction
              coorvalue = codeValue();
              if ((coorvalue <= 0.2) && coorvalue > 0)
                sprintf_P(commandStr, PSTR("G1 X0.1F%i"), movespeed);
              else if ((coorvalue <= -0.1) && coorvalue > -1)
                sprintf_P(commandStr, PSTR("G1 X-0.1F%i"), movespeed);
              else
                sprintf_P(commandStr, PSTR("G1 X%iF%i"), int(coorvalue), movespeed);
            }
            else if (codeSeen('Y')) {  // Move in Y direction
              coorvalue = codeValue();
              if ((coorvalue <= 0.2) && coorvalue > 0)
                sprintf_P(commandStr, PSTR("G1 Y0.1F%i"), movespeed);
              else if ((coorvalue <= -0.1) && coorvalue > -1)
                sprintf_P(commandStr, PSTR("G1 Y-0.1F%i"), movespeed);
              else
                sprintf_P(commandStr, PSTR("G1 Y%iF%i"), int(coorvalue), movespeed);
            }
            else if (codeSeen('Z')) {  // Move in Z direction
              coorvalue = codeValue();
              if ((coorvalue <= 0.2) && coorvalue > 0)
                sprintf_P(commandStr, PSTR("G1 Z0.1F%i"), movespeed);
              else if ((coorvalue <= -0.1) && coorvalue > -1)
                sprintf_P(commandStr, PSTR("G1 Z-0.1F%i"), movespeed);
              else
                sprintf_P(commandStr, PSTR("G1 Z%iF%i"), int(coorvalue), movespeed);
            }
            else if (codeSeen('E')) { // Extrude
              coorvalue = codeValue();
              if ((coorvalue <= 0.2) && coorvalue > 0)
                sprintf_P(commandStr, PSTR("G1 E0.1F%i"), movespeed);
              else if ((coorvalue <= -0.1) && coorvalue > -1)
                sprintf_P(commandStr, PSTR("G1 E-0.1F%i"), movespeed);
              else
                sprintf_P(commandStr, PSTR("G1 E%iF500"), int(coorvalue));
            }

            if (strlen(commandStr) > 0) {
              sprintf_P(fullCommandStr, PSTR("G91\n%s\nG90"), commandStr);
              DEBUG_ECHOPGM("TFT Serial Debug: A22 Move final request with gcode... ");
              DEBUG_ECHOLN(fullCommandStr);
              injectCommands(fullCommandStr);
            }
          }
          SENDLINE_PGM("");
          break;

        case 23: // A23 preheat pla
          if (!isPrinting()) {
            if (getAxisPosition_mm(Z) < 10)
              injectCommands(F("G1 Z10")); // RASE Z AXIS

            setTargetTemp_celsius(PREHEAT_1_TEMP_BED, (heater_t)BED);
            setTargetTemp_celsius(PREHEAT_1_TEMP_HOTEND, (extruder_t)E0);
            SENDLINE_PGM("OK");
          }
          break;

        case 24:// A24 preheat abs
          if (!isPrinting()) {
            if (getAxisPosition_mm(Z) < 10)
              injectCommands(F("G1 Z10")); // RASE Z AXIS

            setTargetTemp_celsius(PREHEAT_2_TEMP_BED, (heater_t)BED);
            setTargetTemp_celsius(PREHEAT_2_TEMP_HOTEND, (extruder_t)E0);
            SENDLINE_PGM("OK");
          }
          break;

        case 25: // A25 cool down
          if (!isPrinting()) {
            setTargetTemp_celsius(0, (heater_t)BED);
            setTargetTemp_celsius(0, (extruder_t)E0);

            SENDLINE_DBG_PGM("J12", "TFT Serial Debug: Cooling down... J12"); // J12 cool down
          }
          break;

        case 26: // A26 refresh SD
          #if HAS_MEDIA
            if (isMediaMounted()) {
              if (strlen(selectedDirectory) > 0) {
                FileList currentFileList;
                if ((selectedDirectory[0] == '.') && (selectedDirectory[1] == '.')) {
                  currentFileList.upDir();
                }
                else {
                  if (selectedDirectory[0] == '<')
                    handleSpecialMenu();
                  else
                    currentFileList.changeDir(selectedDirectory);
                }
              }
            }
            else {
              SENDLINE_DBG_PGM("J02", "TFT Serial Debug: No SD Card mounted to refresh SD A26... J02");
            }

            selectedDirectory[0] = 0;
          #endif
          break;

          #if ENABLED(SERVO_ENDSTOPS)
            case 27: break; // A27 servos angles adjust
          #endif

        case 28: // A28 filament test
          if (codeSeen('O'))
            NOOP;
          else if (codeSeen('C'))
            NOOP;
          SENDLINE_PGM("");
          break;

        case 33: // A33 get version info
          SEND_PGM("J33 ");
          SENDLINE_PGM(DETAILED_BUILD_VERSION);
          break;

        default:
          break;
      }
    }

    tftBufIndW = (tftBufIndW + 1) % (TFTBUFSIZE);
    tftBufLen++;
    serial_count = 0; // clear buffer

  } // while
}

void AnycubicTFT::doSDCardStateCheck() {
  #if HAS_MEDIA
    const bool isMounted = isMediaMounted();
    if (isMounted)
      SENDLINE_DBG_PGM("J00", "TFT Serial Debug: SD card state changed... isMounted");
    else
      SENDLINE_DBG_PGM("J01", "TFT Serial Debug: SD card state changed... !isMounted");

  #endif
}

void AnycubicTFT::doFilamentRunoutCheck() {
  #if HAS_FILAMENT_SENSOR
    // NOTE: getFilamentRunoutState() only returns the runout state if the job is printing
    // we want to actually check the status of the pin here, regardless of printstate
    if (READ(FIL_RUNOUT1_PIN) == FIL_RUNOUT1_STATE) {
      if (mediaPrintingState == AMPRINTSTATE_PRINTING || mediaPrintingState == AMPRINTSTATE_PAUSED || mediaPrintingState == AMPRINTSTATE_PAUSE_REQUESTED) {
        // play tone to indicate filament is out
        injectCommands(F("\nM300 P200 S1567\nM300 P200 S1174\nM300 P200 S1567\nM300 P200 S1174\nM300 P2000 S1567"));

        // tell the user that the filament has run out and wait
        SENDLINE_DBG_PGM("J23", "TFT Serial Debug: Blocking filament prompt... J23");
      }
      else {
        SENDLINE_DBG_PGM("J15", "TFT Serial Debug: Non blocking filament runout... J15");
      }
    }
  #endif // HAS_FILAMENT_SENSOR
}

void AnycubicTFT::startPrint() {
  #if HAS_MEDIA
    if (!isPrinting() && strlen(selectedFile) > 0) {
      DEBUG_ECHOLNPGM("TFT Serial Debug: About to print file ... ", isPrinting(), " ", selectedFile);
      mediaPrintingState = AMPRINTSTATE_PRINTING;
      mediaPauseState    = AMPAUSESTATE_NOT_PAUSED;
      printFile(selectedFile);
    }
  #endif // SDUPPORT
}

void AnycubicTFT::pausePrint() {
  #if HAS_MEDIA
    if (isPrintingFromMedia() && mediaPrintingState != AMPRINTSTATE_STOP_REQUESTED && mediaPauseState == AMPAUSESTATE_NOT_PAUSED) {
      mediaPrintingState = AMPRINTSTATE_PAUSE_REQUESTED;
      mediaPauseState    = AMPAUSESTATE_NOT_PAUSED; // need the userconfirm method to update pause state
      SENDLINE_DBG_PGM("J05", "TFT Serial Debug: SD print pause started... J05"); // J05 printing pause

      // for some reason pausing the print doesn't retract the extruder so force a manual one here
      injectCommands(F("G91\nG1 E-2 F1800\nG90"));
      pausePrint();
    }
  #endif
}

void AnycubicTFT::resumePrint() {
  #if HAS_MEDIA
    #if HAS_FILAMENT_SENSOR
      if (READ(FIL_RUNOUT1_PIN) == FIL_RUNOUT1_STATE) {
        DEBUG_ECHOLNPGM("TFT Serial Debug: Resume Print with filament sensor still tripped... ");

        // trigger the user message box
        doFilamentRunoutCheck();

        // re-enable the continue button
        SENDLINE_DBG_PGM("J18", "TFT Serial Debug: Resume Print with filament sensor still tripped... J18");
        return;
      }
    #endif

    if (mediaPauseState == AMPAUSESTATE_HEATER_TIMEOUT) {
      mediaPauseState = AMPAUSESTATE_REHEATING;
      // TODO: JBA I don't think J05 just disables the continue button, i think it injects a rogue M25. So taking this out
      // // disable the continue button
      // SENDLINE_DBG_PGM("J05", "TFT Serial Debug: Resume called with heater timeout... J05"); // J05 printing pause

      // reheat the nozzle
      setUserConfirmed();
    }
    else {
      mediaPrintingState = AMPRINTSTATE_PRINTING;
      mediaPauseState    = AMPAUSESTATE_NOT_PAUSED;

      SENDLINE_DBG_PGM("J04", "TFT Serial Debug: SD print resumed... J04"); // J04 printing form sd card now
      resumePrint();
    }
  #endif
}

void AnycubicTFT::stopPrint() {
  #if HAS_MEDIA
    mediaPrintingState = AMPRINTSTATE_STOP_REQUESTED;
    mediaPauseState    = AMPAUSESTATE_NOT_PAUSED;
    SENDLINE_DBG_PGM("J16", "TFT Serial Debug: SD print stop called... J16");

    // for some reason stopping the print doesn't retract the extruder so force a manual one here
    injectCommands(F("G91\nG1 E-2 F1800\nG90"));
    stopPrint();
  #endif
}

#endif // ANYCUBIC_LCD_I3MEGA
