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

#include "../inc/MarlinConfigPre.h"

#if ENABLED(EASYTHREED_UI)

#include "easythreed_ui.h"
#include "pause.h"
#include "../module/temperature.h"
#include "../module/printcounter.h"
#include "../sd/cardreader.h"
#include "../gcode/queue.h"
#include "../module/motion.h"
#include "../module/planner.h"
#include "../MarlinCore.h"

EasythreedUI easythreed_ui;

#define BTN_DEBOUNCE_MS 20

void EasythreedUI::init() {
  SET_INPUT_PULLUP(BTN_HOME);     SET_OUTPUT(BTN_HOME_GND);
  SET_INPUT_PULLUP(BTN_FEED);     SET_OUTPUT(BTN_FEED_GND);
  SET_INPUT_PULLUP(BTN_RETRACT);  SET_OUTPUT(BTN_RETRACT_GND);
  SET_INPUT_PULLUP(BTN_PRINT);
  SET_OUTPUT(EASYTHREED_LED_PIN);
}

void EasythreedUI::run() {
  blinkLED();
  loadButton();
  printButton();
}

enum LEDInterval : uint16_t {
  LED_OFF     =    0,
  LED_ON      = 4000,
  LED_BLINK_0 = 2500,
  LED_BLINK_1 = 1500,
  LED_BLINK_2 = 1000,
  LED_BLINK_3 =  800,
  LED_BLINK_4 =  500,
  LED_BLINK_5 =  300,
  LED_BLINK_6 =  150,
  LED_BLINK_7 =   50
};

uint16_t blink_interval_ms = LED_ON;   // Status LED on Start button

void EasythreedUI::blinkLED() {
  static millis_t prev_blink_interval_ms = 0, blink_start_ms = 0;

  if (blink_interval_ms == LED_OFF) { WRITE(EASYTHREED_LED_PIN, HIGH); return; } // OFF
  if (blink_interval_ms >= LED_ON)  { WRITE(EASYTHREED_LED_PIN,  LOW); return; } // ON

  const millis_t ms = millis();
  if (prev_blink_interval_ms != blink_interval_ms) {
    prev_blink_interval_ms = blink_interval_ms;
    blink_start_ms = ms;
  }
  if (PENDING(ms, blink_start_ms + blink_interval_ms))
    WRITE(EASYTHREED_LED_PIN, LOW);
  else if (PENDING(ms, blink_start_ms + 2 * blink_interval_ms))
    WRITE(EASYTHREED_LED_PIN, HIGH);
  else
    blink_start_ms = ms;
}

//
// Filament Load/Unload Button
// Load/Unload buttons are a 3 position switch with a common center ground.
//
void EasythreedUI::loadButton() {
  if (printingIsActive()) return;

  enum FilamentStatus : uint8_t { FS_IDLE, FS_PRESS, FS_CHECK, FS_PROCEED };
  static uint8_t filament_status = FS_IDLE;
  static millis_t filament_time = 0;

  switch (filament_status) {

    case FS_IDLE:
      if (!READ(BTN_RETRACT) || !READ(BTN_FEED)) {                  // If feed/retract switch is toggled...
        filament_status++;                                          // ...proceed to next test.
        filament_time = millis();
      }
      break;

    case FS_PRESS:
      if (ELAPSED(millis(), filament_time + BTN_DEBOUNCE_MS)) {     // After a short debounce delay...
        if (!READ(BTN_RETRACT) || !READ(BTN_FEED)) {                // ...if switch still toggled...
          thermalManager.setTargetHotend(EXTRUDE_MINTEMP + 10, 0);  // Start heating up
          blink_interval_ms = LED_BLINK_7;                          // Set the LED to blink fast
          filament_status++;
        }
        else
          filament_status = FS_IDLE;                                // Switch not toggled long enough
      }
      break;

    case FS_CHECK:
      if (READ(BTN_RETRACT) && READ(BTN_FEED)) {                    // Switch in center position (stop)
        blink_interval_ms = LED_ON;                                 // LED on steady
        filament_status = FS_IDLE;
        thermalManager.disable_all_heaters();
      }
      else if (thermalManager.hotEnoughToExtrude(0)) {              // Is the hotend hot enough to move material?
        filament_status++;                                          // Proceed to feed / retract.
        blink_interval_ms = LED_BLINK_5;                            // Blink ~3 times per second
      }
      break;

    case FS_PROCEED: {
      // Feed or Retract just once. Hard abort all moves and return to idle on swicth release.
      static bool flag = false;
      if (READ(BTN_RETRACT) && READ(BTN_FEED)) {                    // Switch in center position (stop)
        flag = false;                                               // Restore flag to false
        filament_status = FS_IDLE;                                  // Go back to idle state
        quickstop_stepper();                                        // Hard-stop all the steppers ... now!
        thermalManager.disable_all_heaters();                       // And disable all the heaters
        blink_interval_ms = LED_ON;
      }
      else if (!flag) {
        flag = true;
        queue.inject(!READ(BTN_RETRACT) ? F("G91\nG0 E10 F180\nG0 E-120 F180\nM104 S0") : F("G91\nG0 E100 F120\nM104 S0"));
      }
    } break;
  }

}

#if HAS_STEPPER_RESET
  void disableStepperDrivers();
#endif

//
// Print Start/Pause/Resume Button
//
void EasythreedUI::printButton() {
  enum KeyStatus : uint8_t { KS_IDLE, KS_PRESS, KS_PROCEED };
  static uint8_t key_status = KS_IDLE;
  static millis_t key_time = 0;

  enum PrintFlag : uint8_t { PF_START, PF_PAUSE, PF_RESUME };
  static PrintFlag print_key_flag = PF_START;

  const millis_t ms = millis();

  switch (key_status) {
    case KS_IDLE:
      if (!READ(BTN_PRINT)) {                                       // Print/Pause/Resume button pressed?
        key_time = ms;                                              // Save start time
        key_status++;                                               // Go to debounce test
      }
      break;

    case KS_PRESS:
      if (ELAPSED(ms, key_time + BTN_DEBOUNCE_MS))                  // Wait for debounce interval to expire
        key_status = READ(BTN_PRINT) ? KS_IDLE : KS_PROCEED;        // Proceed if still pressed
      break;

    case KS_PROCEED:
      if (!READ(BTN_PRINT)) break;                                  // Wait for the button to be released
      key_status = KS_IDLE;                                         // Ready for the next press
      if (PENDING(ms, key_time + 1200 - BTN_DEBOUNCE_MS)) {         // Register a press < 1.2 seconds
        switch (print_key_flag) {
          case PF_START: {                                          // The "Print" button starts an SD card print
            if (printingIsActive()) break;                          // Already printing? (find another line that checks for 'is planner doing anything else right now?')
            blink_interval_ms = LED_BLINK_2;                        // Blink the indicator LED at 1 second intervals
            print_key_flag = PF_PAUSE;                              // The "Print" button now pauses the print
            card.mount();                                           // Force SD card to mount - now!
            if (!card.isMounted()) {                                // Failed to mount?
              blink_interval_ms = LED_OFF;                          // Turn off LED
              print_key_flag = PF_START;
              return;                                               // Bail out
            }
            card.ls();                                              // List all files to serial output
            const int16_t filecnt = card.get_num_items();           // Count printable files in cwd
            if (filecnt == 0) return;                               // None are printable?
            card.selectFileByIndex(filecnt);                        // Select the last file according to current sort options
            card.openAndPrintFile(card.filename);                   // Start printing it
          } break;
          case PF_PAUSE: {                                          // Pause printing (not currently firing)
            if (!printingIsActive()) break;
            blink_interval_ms = LED_ON;                             // Set indicator to steady ON
            queue.inject(F("M25"));                                 // Queue Pause
            print_key_flag = PF_RESUME;                             // The "Print" button now resumes the print
          } break;
          case PF_RESUME: {                                         // Resume printing
            if (printingIsActive()) break;
            blink_interval_ms = LED_BLINK_2;                        // Blink the indicator LED at 1 second intervals
            queue.inject(F("M24"));                                 // Queue resume
            print_key_flag = PF_PAUSE;                              // The "Print" button now pauses the print
          } break;
        }
      }
      else {                                                        // Register a longer press
        if (print_key_flag == PF_START && !printingIsActive())  {   // While not printing, this moves Z up 10mm
          blink_interval_ms = LED_ON;
          queue.inject(F("G91\nG0 Z10 F600\nG90"));                 // Raise Z soon after returning to main loop
        }
        else {                                                      // While printing, cancel print
          card.abortFilePrintSoon();                                // There is a delay while the current steps play out
          blink_interval_ms = LED_OFF;                              // Turn off LED
        }
        planner.synchronize();                                      // Wait for commands already in the planner to finish
        TERN_(HAS_STEPPER_RESET, disableStepperDrivers());          // Disable all steppers - now!
        print_key_flag = PF_START;                                  // The "Print" button now starts a new print
      }
      break;
  }
}
#endif // EASYTHREED_UI
