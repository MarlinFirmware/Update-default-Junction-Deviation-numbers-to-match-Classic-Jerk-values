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

/**
 * lcd/extui/anycubic_vyper/dgus_tft.cpp
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(ANYCUBIC_LCD_VYPER)

#include "dgus_tft.h"
#include "../anycubic/Tunes.h"
#include "FileNavigator.h"

#include "../../../gcode/queue.h"
#include "../../../sd/cardreader.h"
#include "../../../libs/numtostr.h"
#include "../../../MarlinCore.h"
#include "../../../core/serial.h"
#include "../../../module/stepper.h"
#include "../../../module/probe.h"
#include "../../../module/temperature.h"

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "../../../feature/runout.h"
#endif

#define DEBUG_OUT ACDEBUGLEVEL
#include "../../../core/debug_out.h"

#include <string>

namespace Anycubic {

  DgusTFT::p_fun fun_array[] = {
    DgusTFT::page1,  DgusTFT::page2,  DgusTFT::page3,  DgusTFT::page4,  DgusTFT::page5,  DgusTFT::page6,
    DgusTFT::page7,  DgusTFT::page8,  DgusTFT::page9,  DgusTFT::page10, DgusTFT::page11, DgusTFT::page12,
    DgusTFT::page13, DgusTFT::page14, DgusTFT::page15, DgusTFT::page16, DgusTFT::page17, DgusTFT::page18,
    DgusTFT::page19, DgusTFT::page20, DgusTFT::page21, DgusTFT::page22, DgusTFT::page23, DgusTFT::page24,
    DgusTFT::page25, DgusTFT::page26, DgusTFT::page27, DgusTFT::page28, DgusTFT::page29, DgusTFT::page30,
    DgusTFT::page31, DgusTFT::page32
    #if HAS_LEVELING
      , DgusTFT::page33 , DgusTFT::page34
    #endif
  };

  printer_state_t DgusTFT::printer_state;
  paused_state_t DgusTFT::pause_state;
  #if HAS_HOTEND
    heater_state_t DgusTFT::hotend_state;
  #endif
  #if HAS_HEATED_BED
    heater_state_t DgusTFT::hotbed_state;
  #endif
  char DgusTFT::selectedfile[MAX_PATH_LEN];
  char DgusTFT::panel_command[MAX_CMND_LEN];
  uint8_t DgusTFT::command_len;
  file_menu_t DgusTFT::file_menu;

  bool DgusTFT::data_received;
  uint8_t DgusTFT::data_buf[DATA_BUF_SIZE];
  uint8_t DgusTFT::data_index;
  uint16_t DgusTFT::page_index_now, DgusTFT::page_index_last, DgusTFT::page_index_last_2;
  uint8_t DgusTFT::pop_up_index;
  uint32_t DgusTFT::key_value;
  uint8_t DgusTFT::lcd_txtbox_index;
  uint8_t DgusTFT::lcd_txtbox_page;
  uint16_t DgusTFT::feedrate_back;
  lcd_info_t DgusTFT::lcd_info, DgusTFT::lcd_info_back;
  language_t DgusTFT::ui_language;

  #if ACDEBUG(AC_MARLIN)
    uint16_t page_index_saved;          // flags to keep from bombing the host display
    uint32_t key_value_saved;
  #endif

  void DEBUG_PRINT_PAUSED_STATE(const paused_state_t state, FSTR_P const msg=nullptr);
  void DEBUG_PRINT_PRINTER_STATE(const printer_state_t state, FSTR_P const msg=nullptr);
  void DEBUG_PRINT_TIMER_EVENT(const timer_event_t event, FSTR_P const msg=nullptr);
  void DEBUG_PRINT_MEDIA_EVENT(const media_event_t event, FSTR_P const msg=nullptr);

  void set_brightness();

  DgusTFT dgus;

  DgusTFT::DgusTFT() {
    data_buf[0] = '\0';
    pop_up_index = 100;
    page_index_now = page_index_last = page_index_last_2 = 1;
    lcd_txtbox_index = 0;
    feedrate_back = -1;
  }

  void DgusTFT::startup() {
    #if ACDEBUG(AC_MARLIN)
      DEBUG_ECHOLNPGM("DgusTFT::startup()");
    #endif
    selectedfile[0] = '\0';
    panel_command[0] = '\0';
    command_len = 0;
    printer_state = AC_printer_idle;
    pause_state = AC_paused_idle;
    TERN_(HAS_HOTEND, hotend_state = AC_heater_off);
    TERN_(HAS_HEATED_BED, hotbed_state = AC_heater_off);
    file_menu = AC_menu_file;
    set_language(ui_language); // use language stored in EEPROM

    // Filament runout is handled by Marlin settings in Configuration.h
    // opt_set    FIL_RUNOUT_STATE HIGH  // Pin state indicating that filament is NOT present.
    // opt_enable FIL_RUNOUT_PULLUP

    TFTSer.begin(115200);

    // Signal Board has reset
    tftSendLn(AC_msg_main_board_has_reset);

    // Enable levelling and Disable end stops during print
    // as Z home places nozzle above the bed so we need to allow it past the end stops
    injectCommands(AC_cmnd_enable_leveling);

    #if ACDEBUGLEVEL
      DEBUG_ECHOLNPGM("startup   AC Debug Level ", ACDEBUGLEVEL);
    #endif
    tftSendLn(AC_msg_ready);
  }

  void DgusTFT::paramInit() {

    #if ACDEBUG(AC_MARLIN)
      DEBUG_ECHOLNPGM("DgusTFT::paramInit()");
    #endif

    if (lcd_info.language == CHS)
      page_index_now = 1;
    else if (lcd_info.language == ENG)
      page_index_now = 121;

    lcdAudioSet(lcd_info.audio_on);

    #if ACDEBUG(AC_MARLIN)
      if (lcd_info.language == CHS)
        DEBUG_ECHOLNPGM("paramInit   lcd language: CHS");
      else if (lcd_info.language == ENG)
        DEBUG_ECHOLNPGM("paramInit   lcd language: ENG");

      if (lcd_info.audio_on)
        DEBUG_ECHOLNPGM("paramInit   lcd audio: ON");
      else
        DEBUG_ECHOLNPGM("paramInit   lcd audio: OFF");
    #endif

    requestValueFromTFT(0x14);  // get page ID
  }

  void DgusTFT::idleLoop() {
    if (readTFTCommand()) {
      processPanelRequest();
      command_len = 0;
    }

    #if ACDEBUG(AC_MARLIN)
      if (key_value) {
        DEBUG_ECHOLNPGM("idleLoop   page: ", page_index_now);
        DEBUG_ECHOLNPGM("key: ", key_value);
      }
    #endif

    // Periodically update main page
    if ((page_index_now == 121 || page_index_now == 1) && ((millis() % 500) == 0)) {
      TERN_(HAS_HOTEND, send_temperature_hotend(TXT_MAIN_HOTEND));
      TERN_(HAS_HEATED_BED, send_temperature_bed(TXT_MAIN_BED));
      set_brightness();
      delay(1);  // wait for millis() to advance so this clause isn't repeated
    }

    switch (page_index_now) {
      case 115: page115(); break;
      case 117: page117(); break;
      //case 124: page124(); break;
      //case 125: page125(); break;
      case 170: page170(); break;

      #if ENABLED(POWER_LOSS_RECOVERY)
        case 171: page171(); break;
        case 173: page173(); break;
      #endif

      #if HAS_LEVELING
        case 175: page175(); break;
        case 176: page176(); break;
      #endif

      case 177 ... 198: {
        #if 0 // ACDEBUG(AC_MARLIN)
          DEBUG_ECHOLNPGM("line: ", __LINE__);
          DEBUG_ECHOLNPGM("func: ", page_index_now);
        #endif
        //page177_to_198();
      } break;

      case 199 ... 200: {
        #if 0 // ACDEBUG(AC_MARLIN)
          DEBUG_ECHOLNPGM("line: ", __LINE__);
          DEBUG_ECHOLNPGM("func: ", page_index_now);
        #endif
        page199_to_200();
      } break;

      case 201: case 204: page201(); break;
      case 202: case 205: page202(); break;
      case 203: case 206: page203(); break;

      default:
        if (lcd_info.language == CHS) {
          if (WITHIN(page_index_now, 1, 1 + COUNT(fun_array))) {
            fun_array[page_index_now - 1]();
          }
          else {
            #if ACDEBUG(AC_MARLIN)
              DEBUG_ECHOLNPGM("line: ", __LINE__);
              DEBUG_ECHOLNPGM("fun doesn't exist: ", page_index_now);
            #endif
          }
        }
        else if (lcd_info.language == ENG) {
          if (WITHIN(page_index_now, 121, 121 + COUNT(fun_array))) {
            fun_array[page_index_now - 121]();  // ENG page_index is 120 more than CHS
          }
          else {
            SERIAL_ECHOLNPGM("lcd function doesn't exist");
            SERIAL_ECHOLNPGM("page_index_last: ", page_index_last);
            SERIAL_ECHOLNPGM("page_index_last_2: ", page_index_last_2);
          }
        }
        break;
    }

    pop_up_manager();
    key_value = 0;

    checkHeaters();
  }

  uint8_t FSHlength(FSTR_P FSHinput) {
    PGM_P FSHinputPointer = reinterpret_cast<PGM_P>(FSHinput);
    uint8_t stringLength  = 0;
    while (pgm_read_byte(FSHinputPointer++)) stringLength++;
    return stringLength;
  }

  #undef GET_TEXT
  #define GET_TEXT(MSG) Language_en::MSG

  void DgusTFT::printerKilled(FSTR_P error_p, FSTR_P component_p) {

    // copy string in FLASH to RAM for strcmp_P

    uint8_t textLength = FSHlength(error_p);
    char error[FSHlength(error_p) + 1];
    memcpy_P(error, error_p, textLength + 1);  // +1 for the null terminator

    textLength = FSHlength(component_p);
    char component[FSHlength(component_p) + 1];
    memcpy_P(component, component_p, textLength + 1);  // +1 for the null terminator

    tftSendLn(AC_msg_kill_lcd);
    #if ACDEBUG(AC_MARLIN)
      DEBUG_ECHOLNPGM("printerKilled()\nerror: ", error, "\ncomponent: ", component);
    #endif

    if (strcmp_P(error, GET_TEXT(MSG_ERR_HEATING_FAILED)) == 0) {

      if (strcmp_P(component, GET_TEXT(MSG_BED)) == 0) {
        changePageOfTFT(PAGE_CHS_ABNORMAL_BED_HEATER);
        SERIAL_ECHOLNPGM("Check Bed heater");
      }
      else if (strcmp_P(component, PSTR("E1")) == 0) {
        changePageOfTFT(PAGE_CHS_ABNORMAL_HOTEND_HEATER);
        SERIAL_ECHOLNPGM("Check E1 heater");
      }

    }
    else if (strcmp_P(error, GET_TEXT(MSG_ERR_MINTEMP)) == 0) {

      if (strcmp_P(component, GET_TEXT(MSG_BED)) == 0) {
        changePageOfTFT(PAGE_CHS_ABNORMAL_BED_NTC);
        SERIAL_ECHOLNPGM("Check Bed thermistor");
      }
      else if (strcmp_P(component, PSTR("E1")) == 0) {
        changePageOfTFT(PAGE_CHS_ABNORMAL_HOTEND_NTC);
        SERIAL_ECHOLNPGM("Check E1 thermistor");
      }

    }
    else if (strcmp_P(error, GET_TEXT(MSG_ERR_MAXTEMP)) == 0) {

      if (strcmp_P(component, GET_TEXT(MSG_BED)) == 0) {
        changePageOfTFT(PAGE_CHS_ABNORMAL_BED_NTC);
        SERIAL_ECHOLNPGM("Check Bed thermistor");
      }
      else if (strcmp_P(component, PSTR("E1")) == 0) {
        changePageOfTFT(PAGE_CHS_ABNORMAL_HOTEND_NTC);
        SERIAL_ECHOLNPGM("Check E1 thermistor");
      }

    }
    else if (strcmp_P(error, GET_TEXT(MSG_ERR_THERMAL_RUNAWAY)) == 0) {

      if (strcmp_P(component, GET_TEXT(MSG_BED)) == 0) {
        changePageOfTFT(PAGE_CHS_ABNORMAL_BED_HEATER);
        SERIAL_ECHOLNPGM("Check Bed thermal runaway");
      }
      else if (strcmp_P(component, PSTR("E1")) == 0) {
        changePageOfTFT(PAGE_CHS_ABNORMAL_HOTEND_HEATER);
        SERIAL_ECHOLNPGM("Check E1 thermal runaway");
      }

    }
    else if (strcmp_P(error, GET_TEXT(MSG_KILL_HOMING_FAILED)) == 0) {

      if (strcmp_P(component, PSTR("X")) == 0) {
        changePageOfTFT(PAGE_CHS_ABNORMAL_X_ENDSTOP);
        SERIAL_ECHOLNPGM("Check X endstop");
      }
      else if (strcmp_P(component, PSTR("Y")) == 0) {
        changePageOfTFT(PAGE_CHS_ABNORMAL_Y_ENDSTOP);
        SERIAL_ECHOLNPGM("Check Y endstop");
      }
      else if (strcmp_P(component, PSTR("Z")) == 0) {
        changePageOfTFT(PAGE_CHS_ABNORMAL_Z_ENDSTOP);
        SERIAL_ECHOLNPGM("Check Z endstop");
      }

    }

  }

  void DgusTFT::set_descript_color(const uint16_t color, const uint8_t index/*=lcd_txtbox_index*/) {
    sendColorToTFT(color, TXT_DESCRIPT_0 + 0x30 * (index - 1));
  }

  void DgusTFT::mediaEvent(media_event_t event) {
    #if ACDEBUG(AC_MARLIN)
      DEBUG_PRINT_MEDIA_EVENT(event);
    #endif
    switch (event) {
      case AC_media_inserted:

        filenavigator.reset();

        lcd_txtbox_page = 0;
        if (lcd_txtbox_index) {
          set_descript_color(COLOR_BLUE);
          lcd_txtbox_index = 0;
        }

        sendFileList(lcd_txtbox_index);

        break;

      case AC_media_removed:
        tftSendLn(AC_msg_sd_card_removed);

        filenavigator.reset();

        lcd_txtbox_page = 0;
        if (lcd_txtbox_index) {
          set_descript_color(COLOR_BLUE);
          lcd_txtbox_index = 0;
        }

        sendFileList(lcd_txtbox_index);
        break;

      case AC_media_error:
        tftSendLn(AC_msg_no_sd_card);
        break;
    }
  }

  void DgusTFT::timerEvent(timer_event_t event) {

    #if ACDEBUG(AC_MARLIN)
      DEBUG_PRINT_TIMER_EVENT(event);
      DEBUG_PRINT_PRINTER_STATE(printer_state);
    #endif

    switch (event) {
      case AC_timer_started:
        setSoftEndstopState(false);  // disable endstops to print
        printer_state = AC_printer_printing;
        tftSendLn(AC_msg_print_from_sd_card);
      break;

      case AC_timer_paused:
        //printer_state = AC_printer_paused;
        //pause_state = AC_paused_idle;
        tftSendLn(AC_msg_paused);
        break;

      case AC_timer_stopped:
        if (printer_state != AC_printer_idle) {
          if (printer_state == AC_printer_stopping_from_media_remove) {
            changePageOfTFT(PAGE_NO_SD);
          }
          else {
            printer_state = AC_printer_stopping;

            // Report Printing Time in minutes
            sendTimeToTFT(getProgress_seconds_elapsed() / 60, TXT_FINISH_TIME);
            changePageOfTFT(PAGE_PRINT_FINISH);
            tftSendLn(AC_msg_print_complete);
            pop_up_index = 100;
          }
        }
        setSoftEndstopState(true); // enable endstops
        break;
    }
  }

  #if HAS_FILAMENT_SENSOR

    void DgusTFT::filamentRunout() {
      #if ACDEBUG(AC_MARLIN)
        DEBUG_PRINT_PRINTER_STATE(printer_state, F("filamentRunout() "));

        // 1 Signal filament out
        tftSendLn(isPrintingFromMedia() ? AC_msg_filament_out_alert : AC_msg_filament_out_block);
        //printer_state = AC_printer_filament_out;

        DEBUG_ECHOLNPGM("getFilamentRunoutState: ", getFilamentRunoutState());
      #endif

      pop_up_index = 15;  // show filament lack.

      if (FILAMENT_IS_OUT()) {
        playTune(FilamentOut);

        feedrate_back = getFeedrate_percent();

        if (isPrintingFromMedia()) {
          pausePrint();
          printer_state = AC_printer_pausing;
          pause_state   = AC_paused_filament_lack;
        }
      }
    }

  #endif // HAS_FILAMENT_SENSOR

  void DgusTFT::confirmationRequest(const char * const msg) {
    // M108 continue
    #if ACDEBUG(AC_MARLIN)
      DEBUG_ECHOLNPGM("HomingComplete, line: ", __LINE__);
      DEBUG_ECHOLNPGM("confirmationRequest() ", msg);
      DEBUG_PRINT_PRINTER_STATE(printer_state);
      DEBUG_PRINT_PAUSED_STATE(pause_state);
    #endif

    switch (printer_state) {
      case AC_printer_pausing: {
        if (strcmp_P(msg, MARLIN_msg_print_paused) == 0 || strcmp_P(msg, MARLIN_msg_nozzle_parked) == 0) {
          if (pause_state != AC_paused_filament_lack)
            changePageOfTFT(PAGE_STATUS1);    // enable continue button
          printer_state = AC_printer_paused;
        }
      }
      break;

      #if ENABLED(POWER_LOSS_RECOVERY)
        case AC_printer_resuming_from_power_outage:
      #endif
      case AC_printer_printing:
      case AC_printer_paused:
        // Heater timout, send acknowledgement
        if (strcmp_P(msg, MARLIN_msg_heater_timeout) == 0) {
          pause_state = AC_paused_heater_timed_out;
          tftSendLn(AC_msg_paused); // enable continue button
          playTune(HeaterTimeout);
        }
        // Reheat finished, send acknowledgement
        else if (strcmp_P(msg, MARLIN_msg_reheat_done) == 0) {
          #if ACDEBUG(AC_MARLIN)
            DEBUG_ECHOLNPGM("send M108 ", __LINE__);
          #endif
          injectCommands(F("M108"));

          if (pause_state != AC_paused_filament_lack)
            pause_state = AC_paused_idle;

          tftSendLn(AC_msg_paused); // enable continue button
        }
        // Filament Purging, send acknowledgement enter run mode
        else if (strcmp_P(msg, MARLIN_msg_filament_purging) == 0) {
          pause_state = AC_paused_purging_filament;
          tftSendLn(AC_msg_paused); // enable continue button
        }
        else if (strcmp_P(msg, MARLIN_msg_nozzle_parked) == 0) {
          #if ACDEBUG(AC_MARLIN)
            DEBUG_ECHOLNPGM("send M108 ", __LINE__);
          #endif
          injectCommands(F("M108"));

          if (pause_state != AC_paused_filament_lack)
            pause_state = AC_paused_idle;
        }

        break;

      default: break;
    }
  }

  void DgusTFT::statusChange(const char * const msg) {
    #if ACDEBUG(AC_MARLIN)
      DEBUG_ECHOLNPGM("statusChange() ", msg);
      DEBUG_PRINT_PRINTER_STATE(printer_state);
      DEBUG_PRINT_PAUSED_STATE(pause_state);
    #endif
    bool msg_matched = false;

    #if HAS_LEVELING
      static grid_count_t probe_cnt = 0;
    #endif

    // Visible in main page
    sendTxtToTFT_P(msg, TXT_MAIN_MESSAGE);

    // The only way to get printer status is to parse messages
    // Use the state to minimise the work we do here.
    switch (printer_state) {
      #if HAS_LEVELING
        case AC_printer_probing:

          if (strncmp_P(msg, MARLIN_msg_probing_point, strlen_P(MARLIN_msg_probing_point)) == 0)
            probe_cnt++;

          // If probing completes ok save the mesh and park
          // Ignore the custom machine name
          if (strcmp_P(msg + strlen(MACHINE_NAME), MARLIN_msg_ready) == 0) {
            if (probe_cnt == GRID_MAX_POINTS) {
              probe_cnt = 0;
              injectCommands(F("M500"));    // G27 park nozzle
              //changePageOfTFT(PAGE_PreLEVEL);
              fakeChangePageOfTFT(PAGE_PreLEVEL); // Prevent UI refreshing too quickly when probing is done
              printer_state = AC_printer_idle;
              msg_matched   = true;
            }
          }

          // If probing fails don't save the mesh raise the probe above the bad point
          if (strcmp_P(msg, MARLIN_msg_probing_failed) == 0) {
            playTune(BeepBeepBeeep);
            injectCommands(F("G1 Z50 F500"));
            changePageOfTFT(PAGE_CHS_ABNORMAL_LEVELING_SENSOR);
            tftSendLn(AC_msg_probing_complete);
            printer_state = AC_printer_idle;
            msg_matched   = true;
          }

          if (strcmp_P(msg, MARLIN_msg_probe_preheat_start) == 0)
            changePageOfTFT(PAGE_CHS_PROBE_PREHEATING);

          if (strcmp_P(msg, MARLIN_msg_probe_preheat_stop) == 0)
            changePageOfTFT(PAGE_LEVELING);

          break;
      #endif

      case AC_printer_printing:
        if (strcmp_P(msg, MARLIN_msg_reheating) == 0) {
          tftSendLn(AC_msg_paused); // enable continue button
          changePageOfTFT(PAGE_STATUS2);
          msg_matched = true;
        }
        else if (strcmp_P(msg, MARLIN_msg_media_removed) == 0) {
          msg_matched   = true;
          printer_state = AC_printer_stopping_from_media_remove;
        }
        else {
          #if HAS_FILAMENT_SENSOR
            #if ACDEBUG(AC_MARLIN)
              DEBUG_ECHOLNPGM("setFilamentRunoutState: ", __LINE__);
            #endif
            setFilamentRunoutState(false);
          #endif
        }
        break;

      case AC_printer_pausing:
        if (strcmp_P(msg, MARLIN_msg_print_paused) == 0) {
          if (pause_state != AC_paused_filament_lack) {
            changePageOfTFT(PAGE_STATUS1);        // show resume
            pause_state = AC_paused_idle;
          }

          printer_state = AC_printer_paused;
          msg_matched = true;
        }
        break;

      case AC_printer_paused:
        if (strcmp_P(msg, MARLIN_msg_print_paused) == 0) {
          if (pause_state != AC_paused_filament_lack) {
            changePageOfTFT(PAGE_STATUS1);        // show resume
            pause_state = AC_paused_idle;
          }

          printer_state = AC_printer_paused;
          msg_matched = true;
        }
        break;

      case AC_printer_stopping:
        if (strcmp_P(msg, MARLIN_msg_print_aborted) == 0) {
          changePageOfTFT(PAGE_MAIN);
          printer_state = AC_printer_idle;
          msg_matched   = true;
        }
        break;

      default: break;
    }

    // If not matched earlier see if this was a heater message
    if (!msg_matched) {
      #if HAS_HOTEND
        if (strcmp_P(msg, MARLIN_msg_extruder_heating) == 0) {
          tftSendLn(AC_msg_nozzle_heating);
          hotend_state = AC_heater_temp_set;
          return;
        }
      #endif
      #if HAS_HEATED_BED
        if (strcmp_P(msg, MARLIN_msg_bed_heating) == 0) {
          tftSendLn(AC_msg_bed_heating);
          hotbed_state = AC_heater_temp_set;
        }
      #endif
    }
  }

  #if ENABLED(POWER_LOSS_RECOVERY)

    void DgusTFT::powerLossRecovery() {
      printer_state = AC_printer_resuming_from_power_outage; // Play tune to notify user we can recover.
    }

  #endif // POWER_LOSS_RECOVERY

  void DgusTFT::homingStart() {
    if (!isPrintingFromMedia())
      changePageOfTFT(PAGE_CHS_HOMING);
  }

  void DgusTFT::homingComplete() {
    if (lcd_info.language == ENG && page_index_last > 120)
      page_index_last -= 120;

    #if ACDEBUG(AC_MARLIN)
      DEBUG_ECHOLNPGM("HomingComplete, line: ", __LINE__);
      DEBUG_ECHOLNPGM("page_index_last: ", page_index_last);
    #endif

    if (!isPrintingFromMedia())
      changePageOfTFT(page_index_last);
  }

  void DgusTFT::tftSend(FSTR_P const fstr/*=nullptr*/) {  // A helper to print PROGMEM string to the panel
    #if ACDEBUG(AC_SOME)
      DEBUG_ECHO(fstr);
    #endif
    PGM_P str = FTOP(fstr);
    while (const char c = pgm_read_byte(str++)) TFTSer.write(c);
  }

  void DgusTFT::tftSendLn(FSTR_P const fstr/*=nullptr*/) {
    if (fstr) {
      #if ACDEBUG(AC_SOME)
        DEBUG_ECHOPGM("> ");
      #endif
      tftSend(fstr);
      #if ACDEBUG(AC_SOME)
        SERIAL_EOL();
      #endif
    }
    TFTSer.println();
  }

  void DgusTFT::sendValueToTFT(const uint16_t value, const uint16_t address) {
    uint8_t data[] = { 0x5A, 0xA5, 0x05, 0x82, uint8_t(address >> 8), uint8_t(address & 0xFF), uint8_t(value >> 8), uint8_t(value & 0xFF) };
    for (uint8_t i = 0; i < COUNT(data); ++i) TFTSer.write(data[i]);
  }

  void DgusTFT::requestValueFromTFT(const uint16_t address) {
    uint8_t data[] = { 0x5A, 0xA5, 0x04, 0x83, uint8_t(address >> 8), uint8_t(address & 0xFF), 0x01 };
    for (uint8_t i = 0; i < COUNT(data); ++i) TFTSer.write(data[i]);
  }

  void DgusTFT::sendTxtToTFT(const char *pdata, const uint16_t address) {
    uint8_t data_len = strlen(pdata);
    uint8_t data[] = { 0x5A, 0xA5, uint8_t(data_len + 5), 0x82, uint8_t(address >> 8), uint8_t(address & 0xFF) };
    for (uint8_t i = 0; i < COUNT(data); ++i) TFTSer.write(data[i]);
    for (uint8_t i = 0; i < data_len; ++i) TFTSer.write(pdata[i]);
    TFTSer.write(0xFF); TFTSer.write(0xFF);
  }

  void DgusTFT::sendColorToTFT(const uint16_t color, const uint16_t address) {
    uint16_t color_address = address + 3;
    uint8_t data[] = { 0x5A, 0xA5, 0x05, 0x82, uint8_t(color_address >> 8), uint8_t(color_address & 0xFF), uint8_t(color >> 8), uint8_t(color & 0xFF) };
    for (uint8_t i = 0; i < COUNT(data); ++i) TFTSer.write(data[i]);
  }

  void DgusTFT::sendTimeToTFT(const uint32_t minutes, const uint16_t address) {
    sendTxtToTFT(MString<20>().setf(PSTR("%3d H %3d M"), minutes / 60, minutes % 60), TXT_PRINT_TIME);
  }

  void DgusTFT::sendReadNumOfTxtToTFT(const uint8_t number, const uint16_t address) {
    uint8_t data[] = { 0x5A, 0xA5, 0x04, 0x83, uint8_t(address >> 8), uint8_t(address & 0xFF), number };
    for (uint8_t i = 0; i < COUNT(data); ++i) TFTSer.write(data[i]);
  }

  void DgusTFT::changePageOfTFT(const uint16_t page_index, const bool no_send/*=false*/) {
    #if ACDEBUG(AC_MARLIN)
      DEBUG_ECHOLNPGM("changePageOfTFT: ", page_index);
    #endif

    uint32_t data_temp = 0;

    if (lcd_info.language == CHS) {
      data_temp = page_index;
    }
    else if (lcd_info.language == ENG) {
      switch (page_index) {
        case PAGE_OUTAGE_RECOVERY:
          data_temp = PAGE_ENG_OUTAGE_RECOVERY; break;
        case PAGE_CHS_PROBE_PREHEATING:
          data_temp = PAGE_ENG_PROBE_PREHEATING; break;
        case PAGE_CHS_HOMING ... PAGE_ENG_HOMING:
          data_temp = page_index + 12; break;
        case PAGE_CHS_PROBE_PRECHECK ... PAGE_CHS_PROBE_PRECHECK_FAILED:
          data_temp = page_index + 3; break;
        default:
          data_temp = page_index + 120; break;
      }
    }

    if (!no_send) {
      uint8_t data[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, uint8_t(data_temp >> 8), uint8_t(data_temp & 0xFF) };
      for (uint8_t i = 0; i < COUNT(data); ++i) TFTSer.write(data[i]);
    }

    page_index_last_2 = page_index_last;
    page_index_last   = page_index_now;
    page_index_now    = data_temp;

    #if ACDEBUG(AC_MARLIN)
      DEBUG_ECHOLNPGM("page_index_last_2: ", page_index_last_2);
      DEBUG_ECHOLNPGM("page_index_last: ", page_index_last);
      DEBUG_ECHOLNPGM("page_index_now: ", page_index_now);
    #endif
  }

  void DgusTFT::fakeChangePageOfTFT(const uint16_t page_index) {
    #if ACDEBUG(AC_MARLIN)
      if (page_index_saved != page_index_now)
        DEBUG_ECHOLNPGM("fakeChangePageOfTFT: ", page_index);
    #endif
    changePageOfTFT(page_index, true);
  }

  void DgusTFT::debugPage(int page/*=0*/) {
    #if ACDEBUG(AC_ALL)
      if (page == 0) page = page_index_now;
      if (page_index_saved != page_index_now || key_value_saved != key_value) {
        DEBUG_ECHOLNPGM("page", page, "  page_index_last_2: ", page_index_last_2,  "  page_index_last: ", page_index_last, "  page_index_now: ", page_index_now, "  key: ", key_value);
        page_index_saved = page_index_now;
        key_value_saved = key_value;
      }
    #endif
    UNUSED(page);
  }

  void DgusTFT::showAboutPage() {
    sendTxtToTFT_P(getFirmwareName_str(), TXT_VERSION);
    changePageOfTFT(PAGE_ABOUT);
  }

  void DgusTFT::lcdAudioSet(const bool audio_on) {
    // On:  5A A5 07 82 00 80 5A 00 00 1A
    // Off: 5A A5 07 82 00 80 5A 00 00 12
    uint8_t data[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x80, 0x5A, 0x00, 0x00, uint8_t(audio_on ? 0x1A : 0x12) };
    for (uint8_t i = 0; i < 10; ++i) TFTSer.write(data[i]);
  }

  bool DgusTFT::readTFTCommand() {
    static uint8_t length = 0, cnt = 0, tft_receive_steps = 0;
    uint8_t data;

    if (0 == TFTSer.available() || data_received) return false;

    data = TFTSer.read();

    if (tft_receive_steps == 0) {
      if (data != 0x5A) {
        cnt = 0;
        length = 0;
        data_index = 0;
        data_received = false;
        return false;
      }

      while (!TFTSer.available()) TERN_(USE_WATCHDOG, hal.watchdog_refresh());

      data = TFTSer.read();
      // MYSERIAL.write(data );
      if (data == 0xA5) tft_receive_steps = 2;
    }
    else if (tft_receive_steps == 2) {
      length = data;
      tft_receive_steps = 3;
      data_index = 0;
      cnt = 0;
    }
    else if (tft_receive_steps == 3) {
      if (data_index >= (DATA_BUF_SIZE -1)) {
        #if ACDEBUG(AC_MARLIN)
          DEBUG_ECHOLNPGM("lcd uart buff overflow: ", data_index);
        #endif
        data_index = 0;
        data_received = false;
        return false;
      }
      data_buf[data_index++] = data;
      cnt++;
      if (cnt >= length) {   // Receive complete
        tft_receive_steps = 0;
        cnt = 0;
        data_index = 0;
        data_received = true;
        return true;
      }
    }

    return false;
  }

  #if 0
    {
      //SERIAL_ECHOLNPGM("readTFTCommand: ", millis());
      //return -1;

      bool command_ready = false;
      uint8_t data       = 0;

      while (TFTSer.available() > 0 && command_len < MAX_CMND_LEN) {
        data = TFTSer.read();
        if (0 == command_len) {
          // if
        }

        panel_command[command_len] =
          if (panel_command[command_len] == '\n') {
          command_ready = true;
          break;
        }
        command_len++;
      }

      if (command_ready) {
        panel_command[command_len] = 0x00;
        #if ACDEBUG(AC_ALL)
          DEBUG_ECHOLNPGM("< panel_command ", panel_command);
        #endif
        #if ACDEBUG(AC_SOME)
          // Ignore status request commands
          uint8_t req = atoi(&panel_command[1]);
          if (req > 7 && req != 20) {
            DEBUG_ECHOLNPGM("> ", panel_command);
            DEBUG_PRINT_PRINTER_STATE(printer_state);
          }
        #endif
      }
      return command_ready;
    }
  #endif

  int8_t DgusTFT::findCmdPos(const char * buff, const char q) {
    for (int8_t pos = 0; pos < MAX_CMND_LEN; ++pos)
      if (buff[pos] == q) return pos;
    return -1;
  }

  void DgusTFT::checkHeaters() {
    static uint32_t time_next = 0;
    const millis_t ms = millis();
    if (PENDING(ms, time_next)) return;
    time_next = ms + 500;

    float temp = 0;

    #if HAS_HOTEND
      // If the hotend temp is abnormal, confirm state before signalling panel
      static uint8_t faultE0Duration = 0;
      temp = getActualTemp_celsius(E0);
      if (!WITHIN(temp, HEATER_0_MINTEMP, HEATER_0_MAXTEMP)) {
        faultE0Duration++;
        if (faultE0Duration >= AC_HEATER_FAULT_VALIDATION_TIME) {
          tftSendLn(AC_msg_nozzle_temp_abnormal);
          #if ACDEBUG(AC_MARLIN)
            DEBUG_ECHOLNPGM("Extruder temp abnormal! : ", temp);
          #endif
          faultE0Duration = 0;
        }
      }
    #endif

    #if HAS_HEATED_BED
      static uint8_t faultBedDuration = 0;
      temp = getActualTemp_celsius(BED);
      if (!WITHIN(temp, BED_MINTEMP, BED_MAXTEMP)) {
        faultBedDuration++;
        if (faultBedDuration >= AC_HEATER_FAULT_VALIDATION_TIME) {
          tftSendLn(AC_msg_bed_temp_abnormal);
          #if ACDEBUG(AC_MARLIN)
            DEBUG_ECHOLNPGM("Bed temp abnormal! : ", temp);
          #endif
          faultBedDuration = 0;
        }
      }
    #endif

    #if 0
      // Update panel with hotend heater status
      if (hotend_state != AC_heater_temp_reached) {
        if (WITHIN(getActualTemp_celsius(E0) - getTargetTemp_celsius(E0), -1, 1)) {
          tftSendLn(AC_msg_nozzle_heating_done);
          hotend_state = AC_heater_temp_reached;
        }
      }

      // Update panel with bed heater status
      if (hotbed_state != AC_heater_temp_reached) {
        if (WITHIN(getActualTemp_celsius(BED) - getTargetTemp_celsius(BED), -0.5, 0.5)) {
          tftSendLn(AC_msg_bed_heating_done);
          hotbed_state = AC_heater_temp_reached;
        }
      }
    #endif
  }

  void DgusTFT::sendFileList(int8_t startindex) {
    // Respond to panel request for 4 files starting at index
    #if ACDEBUG(AC_INFO)
      DEBUG_ECHOLNPGM("## sendFileList ## ", startindex);
    #endif
    filenavigator.getFiles(startindex);
  }

  void DgusTFT::selectFile() {
    strlcpy(selectedfile, panel_command + 4, command_len - 3);
    #if ACDEBUG(AC_FILE)
      DEBUG_ECHOLNPGM(" Selected File: ", selectedfile);
    #endif
    switch (selectedfile[0]) {
      case '/':   // Valid file selected
        tftSendLn(AC_msg_sd_file_open_success);
        break;
      case '<':   // .. (go up folder level)
        filenavigator.upDIR();
        tftSendLn(AC_msg_sd_file_open_failed);
        sendFileList(0);
        break;
      default:   // enter sub folder
        filenavigator.changeDIR(selectedfile);
        tftSendLn(AC_msg_sd_file_open_failed);
        sendFileList(0);
        break;
    }
  }

  void DgusTFT::processPanelRequest() {
    uint16_t control_index = 0;
    uint32_t control_value;
    uint16_t temp;

    if (data_received) {
      data_received = false;

      if (0x83 == data_buf[0]) {
        control_index = uint16_t(data_buf[1] << 8) | uint16_t(data_buf[2]);
        if ((control_index & 0xF000) == KEY_ADDRESS) { // is KEY
          //key_index = control_index;
          key_value = (uint16_t(data_buf[4]) << 8) | uint16_t(data_buf[5]);
        }

        #if HAS_HOTEND
          else if (control_index == TXT_HOTEND_TARGET || control_index == TXT_ADJUST_HOTEND) { // hotend target temp
            control_value = (uint16_t(data_buf[4]) << 8) | uint16_t(data_buf[5]);
            temp = constrain(uint16_t(control_value), 0, thermalManager.hotend_max_target(0));
            setTargetTemp_celsius(temp, E0);
          }
        #endif

        #if HAS_HEATED_BED
          else if (control_index == TXT_BED_TARGET || control_index == TXT_ADJUST_BED) {// bed target temp
            control_value = (uint16_t(data_buf[4]) << 8) | uint16_t(data_buf[5]);
            temp = constrain(uint16_t(control_value), 0, BED_MAX_TARGET);
            setTargetTemp_celsius(temp, BED);
          }
        #endif

        #if HAS_FAN
          else if (control_index == TXT_FAN_SPEED_TARGET) { // fan speed
            control_value = (uint16_t(data_buf[4]) << 8) | uint16_t(data_buf[5]);
            temp = constrain(uint16_t(control_value), 0, 100);
            sendValueToTFT(temp, TXT_FAN_SPEED_NOW);
            sendValueToTFT(temp, TXT_FAN_SPEED_TARGET);
            setTargetFan_percent(temp, FAN0);
          }
        #endif

        else if (control_index == TXT_PRINT_SPEED_TARGET || control_index == TXT_ADJUST_SPEED) { // print speed
          control_value = (uint16_t(data_buf[4]) << 8) | uint16_t(data_buf[5]);
          const uint16_t feedrate = constrain(uint16_t(control_value), 40, 999);
          //feedrate_percentage=constrain(control_value,40,999);
          sendTxtToTFT(MString<6>(feedrate), TXT_PRINT_SPEED);
          sendValueToTFT(feedrate, TXT_PRINT_SPEED_NOW);
          sendValueToTFT(feedrate, TXT_PRINT_SPEED_TARGET);
          setFeedrate_percent(feedrate);
        }

        else if (control_index == REG_LCD_READY) {
          control_value = (uint32_t(data_buf[3]) << 16) | (uint32_t(data_buf[4]) << 8) | uint32_t(data_buf[5]);
          if (control_value == 0x010072) { // startup last gif
            lcdAudioSet(lcd_info.audio_on);

            sendValueToTFT(2, ADDRESS_MOVE_DISTANCE);

            #if ENABLED(CASE_LIGHT_ENABLE)
              sendValueToTFT(getCaseLightState(), ADDRESS_SYSTEM_LED_STATUS);
              sendValueToTFT(getCaseLightState(), ADDRESS_PRINT_SETTING_LED_STATUS);
            #endif

            #if ENABLED(POWER_LOSS_RECOVERY)
              const bool is_outage = AC_printer_resuming_from_power_outage == printer_state;
              if (is_outage) {
                changePageOfTFT(PAGE_OUTAGE_RECOVERY);
                #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
                  char filename[64] = { '\0' };
                  card.getLongPath(filename, recovery.info.sd_filename);
                  sendTxtToTFT(filename, TXT_OUTAGE_RECOVERY_FILE);
                #else
                  sendTxtToTFT(recovery.info.sd_filename, TXT_OUTAGE_RECOVERY_FILE);
                #endif
                playTune(SOS);
              }
            #else
              constexpr bool is_outage = false;
            #endif

            if (!is_outage) changePageOfTFT(PAGE_MAIN);

          }
          else if (control_value == 0x010000) {         // startup first gif
            // startup tunes are defined in Tunes.h
            playTune(Anycubic_PowerOn);                 // takes 3500 ms
          }
        }

        /*
        else if ((control_index & 0xF000) == 0x2000) {  // is TXT ADDRESS
          tft_txt_index = control_index;
          j = 0;
          for (i = 4; ;i++) {
            tft_txt_buf[j] = data_buf[i];
            if (tft_txt_buf[j] == 0xFF) {
              tft_txt_buf[j] = 0;
              break;
            }
            j++;
          }
        }
        */
      }
      else if (0x82 == data_buf[0]) {
        // send_cmd_to_pc(cmd ,start );
      }
    }
  }

  void set_brightness() {
    uint8_t data[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x82, 0x64, 0x32, 0x03, 0xE8 };
    for (uint8_t i = 0; i < COUNT(data); ++i) TFTSer.write(data[i]);
  }

  void DgusTFT::set_language(language_t language) {
    lcd_info.language = ui_language = lcd_info_back.language = language;
  }

  void DgusTFT::toggle_language() {
    lcd_info.language = ui_language = (lcd_info.language == CHS ? ENG : CHS);
  }

  void DgusTFT::goto_system_page() {
    changePageOfTFT(lcd_info.language == CHS
      ? (lcd_info.audio_on ? PAGE_SYSTEM_CHS_AUDIO_ON : PAGE_SYSTEM_CHS_AUDIO_OFF)
      : (lcd_info.audio_on ? 11 : 50) // PAGE_SYSTEM_ENG_AUDIO_ON/OFF - 120
    );
  }

  void DgusTFT::toggle_audio() {
    FLIP(lcd_info.audio_on);
    goto_system_page();
    lcdAudioSet(lcd_info.audio_on);
  }

  void DgusTFT::store_changes() {
    if (lcd_info_back.language != lcd_info.language || lcd_info_back.audio_on != lcd_info.audio_on) {
      lcd_info_back = lcd_info;
      injectCommands(F("M500"));
    }
  }

  #if HAS_HOTEND
    void DgusTFT::send_temperature_hotend(const uint16_t address) {
      sendTxtToTFT(MString<16>(uint16_t(getActualTemp_celsius(E0)), '/', uint16_t(getTargetTemp_celsius(E0))), address);
    }
  #endif

  #if HAS_HEATED_BED
    void DgusTFT::send_temperature_bed(const uint16_t address) {
      sendTxtToTFT(MString<16>(uint16_t(getActualTemp_celsius(BED)), '/', uint16_t(getTargetTemp_celsius(BED))), address);
    }
  #endif

  void DgusTFT::page1() {
    debugPage(1);

    switch (key_value) {
      case 0: break;

      case 1: { // main page, print
        lcd_txtbox_page = 0;
        if (lcd_txtbox_index) {
          set_descript_color(COLOR_BLUE);
          lcd_txtbox_index = 0;
        }
        changePageOfTFT(PAGE_FILE);
        sendFileList(0);
      } break;

      case 2: { // tool
        changePageOfTFT(PAGE_TOOL);
        #if ENABLED(CASE_LIGHT_ENABLE)
          sendValueToTFT(getCaseLightState(), ADDRESS_SYSTEM_LED_STATUS);
        #endif
      } break;

      case 3: // prepare
        changePageOfTFT(PAGE_PREPARE);
        break;

      case 4: // system
        goto_system_page();
        break;
    }

    #if HAS_HOTEND || HAS_HEATED_BED
      static millis_t flash_time = 0;
      const millis_t ms = millis();
      if (PENDING(ms, flash_time)) return;
      flash_time = ms + 1500;

      TERN_(HAS_HOTEND, send_temperature_hotend(TXT_PREHEAT_HOTEND));
      TERN_(HAS_HEATED_BED, send_temperature_bed(TXT_PREHEAT_BED));
    #endif
  }

  void DgusTFT::page2() {
    debugPage(2);
    char file_index = 0;

    switch (key_value) {
      case 0: break;

      case 1: // return
        changePageOfTFT(PAGE_MAIN);
        set_descript_color(COLOR_BLUE);
        break;

      case 2: // page up
        if (lcd_txtbox_page > 0) {
          lcd_txtbox_page--;

          set_descript_color(COLOR_BLUE);
          lcd_txtbox_index = 0;

          sendFileList(lcd_txtbox_page * 5);
        }
        break;

      case 3: // page down
        if ((lcd_txtbox_page + 1) * 5 < filenavigator.getFileNum()) {
          lcd_txtbox_page++;

          set_descript_color(COLOR_BLUE);
          lcd_txtbox_index = 0;

          sendFileList(lcd_txtbox_page * 5);
        }
        break;

      case 4:   // page refresh
        if (!isMediaMounted()) safe_delay(500);

        filenavigator.reset();

        lcd_txtbox_page = 0;
        if (lcd_txtbox_index) {
          set_descript_color(COLOR_BLUE);
          lcd_txtbox_index = 0;
        }
        sendFileList(lcd_txtbox_index);
        break;

      case 5: // resume of outage(last power off)
        #if ACDEBUG(AC_MARLIN)
          DEBUG_PRINT_PRINTER_STATE(printer_state);
        #endif
        if (lcd_txtbox_index > 0 && lcd_txtbox_index  < 6) {   // 1~5

          if (filenavigator.filelist.seek(lcd_txtbox_page * 5 + (lcd_txtbox_index - 1))) {

            set_descript_color(COLOR_BLUE);

            TERN_(CASE_LIGHT_ENABLE, setCaseLightState(true));

            sendTxtToTFT(MString<17>(filenavigator.filelist.longFilename()), TXT_PRINT_NAME);

            #if ENABLED(POWER_LOSS_RECOVERY)
              if (printer_state == AC_printer_resuming_from_power_outage) {
                // Need to home here to restore the Z position
                //injectCommands(AC_cmnd_power_loss_recovery);
                //SERIAL_ECHOLNPGM("start resuming from power outage: ", AC_cmnd_power_loss_recovery);
                changePageOfTFT(PAGE_STATUS2);    // show pause
                injectCommands(F("M1000"));       // home and start recovery
              }
            #endif
          }
        }
        break;

      case 6: // start print
        if (lcd_txtbox_index > 0 && lcd_txtbox_index  < 6) {    // 1~5

          if (filenavigator.filelist.seek(lcd_txtbox_page * 5 + lcd_txtbox_index - 1)) {
            #if 0
              SERIAL_ECHOLNPGM("start print: ", lcd_txtbox_page * 5 + (lcd_txtbox_index - 1));
              SERIAL_ECHOLNPGM("start print: ", filenavigator.filelist.shortFilename());
              SERIAL_ECHOLNPGM("start print: ", filenavigator.filelist.longFilename());
            #endif

            set_descript_color(COLOR_BLUE);

            // Allows printer to restart the job if we don't want to recover
            if (printer_state == AC_printer_resuming_from_power_outage) {
              injectCommands(F("M1000 C"));   // Cancel recovery
              printer_state = AC_printer_idle;
            }

            TERN_(CASE_LIGHT_ENABLE, setCaseLightState(true));
            printFile(filenavigator.filelist.shortFilename());

            sendTxtToTFT(MString<17>(filenavigator.filelist.longFilename()), TXT_PRINT_NAME);
            sendTxtToTFT(MString<6>(uint16_t(getFeedrate_percent())), TXT_PRINT_SPEED);
            sendTxtToTFT(MString<6>(uint16_t(getProgress_percent())), TXT_PRINT_PROGRESS);
            sendTimeToTFT(0, TXT_PRINT_TIME);

            changePageOfTFT(PAGE_STATUS2);
          }
        }
        break;

      case 7: // txtbox 1 click
      case 8: // txtbox 2 click
      case 9: // txtbox 3 click
      case 10: // txtbox 4 click

      case 11: { // txtbox 5 click
        static uint8_t lcd_txtbox_index_last = 0;

        lcd_txtbox_index = key_value - 6;

        // lcd_txtbox_page 0~...
        // lcd_txtbox_index 1~5
        file_index = lcd_txtbox_page * 5 + (lcd_txtbox_index - 1);
        if (file_index < filenavigator.getFileNum()) {

          set_descript_color(COLOR_RED);

          if (lcd_txtbox_index_last && lcd_txtbox_index_last != lcd_txtbox_index)    // 1~5
            set_descript_color(COLOR_BLUE, lcd_txtbox_index_last);
          lcd_txtbox_index_last = lcd_txtbox_index;
        }
      } break;
    }
  }

  void DgusTFT::page3() {
    debugPage(3);

    static millis_t flash_time = 0;
    const millis_t ms = millis();

    switch (key_value) {
      case 0: break;

      case 1:    // return
        if (!isPrintingFromMedia()) // only idle status can return
          changePageOfTFT(PAGE_FILE);
        break;

      case 2:     // resume print
        #if ACDEBUG(AC_MARLIN)
          DEBUG_PRINT_PRINTER_STATE(printer_state);
          DEBUG_PRINT_PAUSED_STATE(pause_state);
        #endif
        if ( pause_state == AC_paused_idle
          || pause_state == AC_paused_filament_lack
          || printer_state == AC_printer_resuming_from_power_outage
        ) {
          printer_state = AC_printer_idle;
          pause_state = AC_paused_idle;
          resumePrint();
          changePageOfTFT(PAGE_STATUS2);        // show pause print
          flash_time = ms + 1500;
        }
        else
          setUserConfirmed();
        break;

      case 3:     // print stop
        if (isPrintingFromMedia())
          changePageOfTFT(PAGE_STOP_CONF);
        break;

      case 4:     // print change param
        changePageOfTFT(PAGE_ADJUST);
        TERN_(CASE_LIGHT_ENABLE, sendValueToTFT(getCaseLightState(), ADDRESS_PRINT_SETTING_LED_STATUS));
        TERN_(HAS_HOTEND, sendValueToTFT(uint16_t(getTargetTemp_celsius(E0)), TXT_ADJUST_HOTEND));
        TERN_(HAS_HEATED_BED, sendValueToTFT(uint16_t(getTargetTemp_celsius(BED)), TXT_ADJUST_BED));
        feedrate_back = getFeedrate_percent();
        sendValueToTFT(uint16_t(feedrate_back), TXT_ADJUST_SPEED);
        flash_time = ms + 1500;
        break;
    }

    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1500;

    const uint16_t ifeedrate = uint16_t(getFeedrate_percent());
    if (feedrate_back != ifeedrate) {
      if (ifeedrate != 0)
        sendTxtToTFT(MString<6>(ifeedrate), TXT_PRINT_SPEED);
      else
        sendTxtToTFT(MString<6>(feedrate_back), TXT_PRINT_SPEED);

      #if ACDEBUG(AC_MARLIN)
        DEBUG_ECHOLNPGM("print speed: ", ifeedrate, " feedrate_back: ", feedrate_back);
      #endif
      feedrate_back = ifeedrate;
    }

    static uint8_t progress_last = 0;
    if (progress_last != getProgress_percent()) {
      progress_last = getProgress_percent();
      sendTxtToTFT(MString<6>(progress_last), TXT_PRINT_PROGRESS);
    }

    // Report Printing Time in minutes
    sendTimeToTFT(getProgress_seconds_elapsed() / 60, TXT_PRINT_TIME);

    TERN_(HAS_HOTEND, send_temperature_hotend(TXT_PRINT_HOTEND));
    TERN_(HAS_HEATED_BED, send_temperature_bed(TXT_PRINT_BED));
  }

  void DgusTFT::page4() {
    debugPage(4);


    switch (key_value) {
      case 0: break;

      case 1:   // return
        if (!isPrintingFromMedia()) // only is idle status can return
          changePageOfTFT(PAGE_FILE);
        break;

      case 2:    // print pause
        if (isPrintingFromMedia()) {
          pausePrint();
          printer_state = AC_printer_pausing;
          pause_state = AC_paused_idle;
          changePageOfTFT(PAGE_WAIT_PAUSE);
          //injectCommands(F("M108"));     // stop waiting temperature M109
        }
        break;

      case 3:   // print stop
        if (isPrintingFromMedia())
          changePageOfTFT(PAGE_STOP_CONF);
        break;

      case 4:   // print settings
        changePageOfTFT(PAGE_ADJUST);
        TERN_(CASE_LIGHT_ENABLE, sendValueToTFT(getCaseLightState(), ADDRESS_PRINT_SETTING_LED_STATUS));
        TERN_(HAS_HOTEND, sendValueToTFT(uint16_t(getTargetTemp_celsius(E0)), TXT_ADJUST_HOTEND));
        TERN_(HAS_HEATED_BED, sendValueToTFT(uint16_t(getTargetTemp_celsius(BED)), TXT_ADJUST_BED));
        feedrate_back = getFeedrate_percent();
        sendValueToTFT((uint16_t)feedrate_back, TXT_ADJUST_SPEED);
        TERN_(HAS_FAN, sendValueToTFT(uint16_t(getActualFan_percent(FAN0)), TXT_FAN_SPEED_TARGET));
        sendTxtToTFT(ftostr52sprj(getZOffset_mm()) + 3, TXT_LEVEL_OFFSET);
        requestValueFromTFT(TXT_ADJUST_SPEED);  // attempt to make feedrate visible on visit to this page
        break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1500;

    const uint16_t ifeedrate = uint16_t(getFeedrate_percent());
    if (feedrate_back != ifeedrate) {
      if (ifeedrate != 0)
        sendTxtToTFT(MString<6>(ifeedrate), TXT_PRINT_SPEED);
      else
        sendTxtToTFT(MString<6>(feedrate_back), TXT_PRINT_SPEED);
      feedrate_back = ifeedrate;
    }

    static uint8_t progress_last = 0;
    if (progress_last != getProgress_percent()) {
      progress_last = getProgress_percent();
      sendTxtToTFT(MString<6>(progress_last), TXT_PRINT_PROGRESS);
    }

    // Report Printing Time in minutes
    sendTimeToTFT(getProgress_seconds_elapsed() / 60, TXT_PRINT_TIME);

    TERN_(HAS_HOTEND, send_temperature_hotend(TXT_PRINT_HOTEND));
    TERN_(HAS_HEATED_BED, send_temperature_bed(TXT_PRINT_BED));
  }

  void DgusTFT::page5() {          // print settings
    debugPage(5);
    static bool z_change = false;

    switch (key_value) {
      case 0: break;

      case 1: // return
        if (AC_printer_printing == printer_state)
          changePageOfTFT(PAGE_STATUS2);  // show pause
        else if (AC_printer_paused == printer_state)
          changePageOfTFT(PAGE_STATUS1);  // show print
        break;

      #if ENABLED(MESH_EDIT_MENU)

        case 2: { // -
          float z_off = getZOffset_mm();
          //SERIAL_ECHOLNPGM("z_off: ", z_off);
          //setSoftEndstopState(false);
          if (z_off <= -5) return;
          z_off -= 0.05f;
          setZOffset_mm(z_off);

          sendTxtToTFT(ftostr52sprj(getZOffset_mm()) + 2, TXT_LEVEL_OFFSET);

          //if (isAxisPositionKnown(Z)) {  // Move Z axis
          //  SERIAL_ECHOLNPGM("Z now:", getAxisPosition_mm(Z));
          //  const float currZpos = getAxisPosition_mm(Z);
          //  setAxisPosition_mm(currZpos-0.05, Z);
          //  SERIAL_ECHOLNPGM("Z now:", getAxisPosition_mm(Z));
          //}

          #if ENABLED(BABYSTEPPING)
            int16_t steps = mmToWholeSteps(-0.05, Z);
            babystepAxis_steps(steps, Z);
          #endif

          GRID_LOOP(x, y) {
            const xy_uint8_t pos { x, y };
            const float currval = getMeshPoint(pos);
            #if ACDEBUG(AC_MARLIN)
              DEBUG_ECHOLNPGM("x: ", x, " y: ", y, " z: ", currval);
            #endif
            setMeshPoint(pos, constrain(currval - 0.05f, AC_LOWEST_MESHPOINT_VAL, 5));
          }

          z_change = true;

          //setSoftEndstopState(true);
        } break;

        case 3: { // +
          float z_off = getZOffset_mm();
          //SERIAL_ECHOLNPGM("z_off: ", z_off);
          //setSoftEndstopState(false);

          if (z_off >= 5) return;
          z_off += 0.05f;
          setZOffset_mm(z_off);

          sendTxtToTFT(ftostr52sprj(getZOffset_mm()) + 2, TXT_LEVEL_OFFSET);

          //int16_t steps = mmToWholeSteps(constrain(Zshift,-0.05,0.05), Z);

          /*
          if (isAxisPositionKnown(Z)) {  // Move Z axis
            SERIAL_ECHOLNPGM("Z now:", getAxisPosition_mm(Z));
            const float currZpos = getAxisPosition_mm(Z);
            setAxisPosition_mm(currZpos-0.05, Z);
            SERIAL_ECHOLNPGM("Z now:", getAxisPosition_mm(Z));
          }
          */

          #if ENABLED(BABYSTEPPING)
            int16_t steps = mmToWholeSteps(0.05, Z);
            babystepAxis_steps(steps, Z);
          #endif

          GRID_LOOP(x, y) {
            const xy_uint8_t pos { x, y };
            const float currval = getMeshPoint(pos);
            //SERIAL_ECHOLNPGM("x: ", x, " y: ", y, " z: ", currval);
            setMeshPoint(pos, constrain(currval + 0.05f, AC_LOWEST_MESHPOINT_VAL, 5));
          }

          z_change = true;

          //setSoftEndstopState(true);
        } break;

      #endif // MESH_EDIT_MENU

      #if ENABLED(CASE_LIGHT_ENABLE)
        case 4: {   // light control
          const bool cls = !getCaseLightState();
          sendValueToTFT(cls, ADDRESS_PRINT_SETTING_LED_STATUS);
          setCaseLightState(cls);
        } break;
      #endif

      case 5:
        changePageOfTFT(PAGE_DONE);
        break;

      case 6: break;

      case 7:
        TERN_(HAS_HEATED_BED, requestValueFromTFT(TXT_ADJUST_BED));
        requestValueFromTFT(TXT_ADJUST_SPEED);
        TERN_(HAS_HOTEND, requestValueFromTFT(TXT_ADJUST_HOTEND));
        TERN_(HAS_FAN, requestValueFromTFT(TXT_FAN_SPEED_TARGET));

        if (z_change == true) {
          injectCommands(F("M500"));
          z_change = false;
        }

        if (AC_printer_printing == printer_state)
          changePageOfTFT(PAGE_STATUS2);    // show pause
        else if (AC_printer_paused == printer_state)
          changePageOfTFT(PAGE_STATUS1);    // show print

        break;
    }
  }

  void DgusTFT::page6() {
    debugPage(6);
    switch (key_value) {
      case 0: break;
      case 1: break;
    }
  }

  void DgusTFT::page7() { // tools
    debugPage(7);
    switch (key_value) {
      case 0: break;

      case 1:       // return
        changePageOfTFT(PAGE_MAIN);
        break;

      case 2:
        changePageOfTFT(PAGE_MOVE);
        break;

      case 3:       // set temperature
        changePageOfTFT(PAGE_TEMP);
        #if HAS_HOTEND
          sendValueToTFT(uint16_t(getActualTemp_celsius(E0)), TXT_HOTEND_NOW);
          sendValueToTFT(uint16_t(getTargetTemp_celsius(E0)), TXT_HOTEND_TARGET);
        #endif
        #if HAS_HEATED_BED
          sendValueToTFT(uint16_t(getActualTemp_celsius(BED)), TXT_BED_NOW);
          sendValueToTFT(uint16_t(getTargetTemp_celsius(BED)), TXT_BED_TARGET);
        #endif
        break;

      case 4: {
        changePageOfTFT(PAGE_SPEED);
        #if HAS_FAN
          sendValueToTFT(uint16_t(getActualFan_percent(FAN0)), TXT_FAN_SPEED_NOW);
          sendValueToTFT(uint16_t(getTargetFan_percent(FAN0)), TXT_FAN_SPEED_TARGET);
        #endif
        const uint16_t ifeedrate = uint16_t(getFeedrate_percent());
        sendValueToTFT(ifeedrate, TXT_PRINT_SPEED_NOW);
        sendValueToTFT(ifeedrate, TXT_PRINT_SPEED_TARGET);
      } break;

      case 5:       // turn off the xyz motor
        if (!isMoving())
          stepper.disable_all_steppers();
        break;

      #if ENABLED(CASE_LIGHT_ENABLE)
        case 6: {   // light control
          const bool cls = !getCaseLightState();
          sendValueToTFT(cls, ADDRESS_SYSTEM_LED_STATUS);
          setCaseLightState(cls);
        } break;
      #endif
    }
  }

  void DgusTFT::page8() {
    debugPage(8);
    //static uint16_t movespeed = 50;
    static float move_dis = 1.0f;

    if (key_value == 2 || key_value == 4
        || key_value == 6 || key_value == 8
        || key_value == 10 || (key_value == 12 && !isMoving())
    ) {
      if (getAxisPosition_mm(Z) < 0) setAxisPosition_mm(0, Z, 8);
    }

    //if (!planner.has_blocks_queued()) return;

    switch (key_value) {
      case 0:
        break;

      case 1:    // return
        changePageOfTFT(PAGE_TOOL);
        break;

      case 5:
        if (!isMoving())
          injectCommands(F("G28 X"));
        break;

      case 9:
        if (!isMoving())
          injectCommands(F("G28 Y"));
        break;

      case 13:
        if (!isMoving())
          injectCommands(F("G28 Z"));
        break;

      case 17:
        if (!isMoving())
          injectCommands(F("G28"));
        break;

      case 2:       // X-
        if (!isMoving())
          setAxisPosition_mm(getAxisPosition_mm(X) - move_dis, X, 50);
        break;

      case 4:       // X+
        if (!isMoving())
          setAxisPosition_mm(getAxisPosition_mm(X) + move_dis, X, 50);
        break;

      case 6:       // Y+
        if (!isMoving())
          setAxisPosition_mm(getAxisPosition_mm(Y) - move_dis, Y, 50);
        break;

      case 8:       // Y-
        if (!isMoving())
          setAxisPosition_mm(getAxisPosition_mm(Y) + move_dis, Y, 50);
        break;

      case 10:      // Z-
        if (!isMoving())
          setAxisPosition_mm(getAxisPosition_mm(Z) - move_dis, Z, 8);
        break;

      case 12:      // Z+
        if (!isMoving())
          setAxisPosition_mm(getAxisPosition_mm(Z) + move_dis, Z, 8);
        break;

      case 3:
        move_dis = 0.1f;
        sendValueToTFT(1, ADDRESS_MOVE_DISTANCE);
        break;

      case 7:
        move_dis = 1.0f;
        sendValueToTFT(2, ADDRESS_MOVE_DISTANCE);
        break;

      case 11:
        move_dis = 10.0f;
        sendValueToTFT(3, ADDRESS_MOVE_DISTANCE);
        break;

      //case 14:
      //  movespeed = 3000; //SERIAL_ECHOLN(movespeed);
      //  break;
      //
      //case 15:
      //  movespeed = 2000; //SERIAL_ECHOLN(movespeed);
      //  break;
      //
      //case 16:
      //  movespeed = 1000; //SERIAL_ECHOLN(movespeed);
      //  break;
    }
  }

  void DgusTFT::page9() {
    debugPage(9);

    switch (key_value) {
      case 0: break;

      case 1:    // return
        changePageOfTFT(PAGE_TOOL);
        break;

      case 2: break;
      case 3: break;
      case 4: break;
      case 5: break;

      case 6:     // cooling
        setTargetTemp_celsius(0, E0);
        setTargetTemp_celsius(0, BED);
        changePageOfTFT(PAGE_TOOL);
        break;

      case 7:     // send target temp
        requestValueFromTFT(TXT_HOTEND_TARGET);
        requestValueFromTFT(TXT_BED_TARGET);
        changePageOfTFT(PAGE_TOOL);
        break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1500;

    sendValueToTFT(uint16_t(getActualTemp_celsius(E0)), TXT_HOTEND_NOW);
    sendValueToTFT(uint16_t(getActualTemp_celsius(BED)), TXT_BED_NOW);
  }

  void DgusTFT::page10() {
    debugPage(10);

    switch (key_value) {
      case 0: break;
      case 1:       // return
        changePageOfTFT(PAGE_TOOL);
        break;

      case 2: break;
      case 3: break;
      case 4: break;
      case 5: break;

      case 6:       // ok
        requestValueFromTFT(TXT_FAN_SPEED_TARGET);
        requestValueFromTFT(TXT_PRINT_SPEED_TARGET);
        changePageOfTFT(PAGE_TOOL);
        break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1500;

    sendValueToTFT(uint16_t(getActualFan_percent(FAN0)), TXT_FAN_SPEED_NOW);
    sendValueToTFT(uint16_t(getFeedrate_percent()), TXT_PRINT_SPEED_NOW);
  }

  void DgusTFT::page11() {
    debugPage(11);
    switch (key_value) {
      case 0: break;

      case 1:       // return
        changePageOfTFT(PAGE_MAIN);
        store_changes();
        break;

      case 2:       // language
        toggle_language();
        goto_system_page();
        break;

      case 3: break;

      case 4:       // audio
        toggle_audio();
        break;

      case 5:       // about
        showAboutPage();
        break;

      case 6:
        changePageOfTFT(PAGE_RECORD);
        break;
    }
  }

  void DgusTFT::page12() {
    debugPage(12);
    switch (key_value) {
      case 0: break;
      case 1:        // return
        changePageOfTFT(PAGE_SYSTEM_CHS_AUDIO_ON);
        break;
    }
  }

  void DgusTFT::page13() {
    debugPage(13);
    switch (key_value) {
      case 0: break;

      case 1:    // return
        goto_system_page();
        break;

      case 2: break;
    }
  }

  void DgusTFT::page14() {
    debugPage(14);
    switch (key_value) {
      case 0: break;
      case 1: break; // return
      case 2: break;
      case 3: break;
      case 4: break;
    }
  }

  void DgusTFT::page15() {
    debugPage(15);

    switch (key_value) {
      case 0: break;

      case 1:        // return
        changePageOfTFT(PAGE_MAIN);
        break;

      case 2:
        changePageOfTFT(PAGE_PreLEVEL);
        break;

      #if HAS_HOTEND || HAS_HEATED_BED
        case 3: {
          changePageOfTFT(PAGE_PREHEAT);
          TERN_(HAS_HOTEND, send_temperature_hotend(TXT_PREHEAT_HOTEND));
          TERN_(HAS_HEATED_BED, send_temperature_bed(TXT_PREHEAT_BED));
        } break;
      #endif

      #if HAS_EXTRUDERS
        case 4: {
          send_temperature_hotend(TXT_FILAMENT_TEMP);
          changePageOfTFT(PAGE_FILAMENT);
        } break;
      #endif
    }
  }

  void DgusTFT::page16() {    // AUTO LEVELING
    debugPage(16);
    switch (key_value) {
      case 0: break;
      case 1:            // return
        changePageOfTFT(PAGE_PREPARE);
        break;

      case 2:
        if (!isPrinting()) {
          //changePageOfTFT(PAGE_LEVEL_ENSURE);
          changePageOfTFT(PAGE_CHS_PROBE_PRECHECK);
        }
        break;

      case 3: {
        sendTxtToTFT(ftostr52sprj(getZOffset_mm()) + 2, TXT_LEVEL_OFFSET);
        changePageOfTFT(PAGE_LEVEL_ADVANCE);
      } break;

      case 4:
        changePageOfTFT(PAGE_AUTO_OFFSET);
        break;
    }
  }

  void DgusTFT::page17() {
    debugPage(17);
    float z_off;
    switch (key_value) {
      case 0: break;

      case 1:        // return
        changePageOfTFT(PAGE_PreLEVEL);
        break;

      case 2: { // Babystep Z Offset -
        if (getZOffset_mm() <= -5) return;

        setSoftEndstopState(false);

        z_off = getZOffset_mm() - 0.01f;
        setZOffset_mm(z_off);

        sendTxtToTFT(ftostr52sprj(getZOffset_mm()) + 2, TXT_LEVEL_OFFSET);

        if (isAxisPositionKnown(Z)) {
          const float currZpos = getAxisPosition_mm(Z);
          setAxisPosition_mm(currZpos - 0.01f, Z);
        }

        setSoftEndstopState(true);
      } break;

      case 3: { // Babystep Z Offset +
        if (getZOffset_mm() >= 5) return;

        setSoftEndstopState(false);

        z_off = getZOffset_mm() + 0.01f;
        setZOffset_mm(z_off);

        sendTxtToTFT(ftostr52sprj(getZOffset_mm()) + 2, TXT_LEVEL_OFFSET);

        if (isAxisPositionKnown(Z)) {          // Move Z axis
          const float currZpos = getAxisPosition_mm(Z);
          setAxisPosition_mm(currZpos + 0.01f, Z);
        }

        setSoftEndstopState(true);
      } break;

      case 4:
        #if ACDEBUG(AC_MARLIN)
          DEBUG_ECHOLNPGM("z off: ", ftostr52sprj(getZOffset_mm()));
        #endif
        #if HAS_LEVELING
          GRID_LOOP(x, y) {
            const xy_uint8_t pos { x, y };
            const float currval = getMeshPoint(pos);
            setMeshPoint(pos, constrain(currval + getZOffset_mm(), AC_LOWEST_MESHPOINT_VAL, 5));
          }
          injectCommands(F("M500"));
        #endif
        changePageOfTFT(PAGE_PREPARE);
        break;
    }
  }

  #if HAS_HOTEND || HAS_HEATED_BED

    void DgusTFT::page18() {     // preheat
    debugPage(18);

      switch (key_value) {
        case 0: break;

        case 1:         // return
          changePageOfTFT(PAGE_PREPARE);
          break;

        case 2:         // PLA
          TERN_(HAS_HOTEND, setTargetTemp_celsius(190, E0));
          TERN_(HAS_HEATED_BED, setTargetTemp_celsius(60, BED));
          changePageOfTFT(PAGE_PREHEAT);
          break;

        case 3:         // ABS
          TERN_(HAS_HOTEND, setTargetTemp_celsius(240, E0));
          TERN_(HAS_HEATED_BED, setTargetTemp_celsius(100, BED));
          changePageOfTFT(PAGE_PREHEAT);
          break;
      }

      static millis_t flash_time = 0;
      const millis_t ms = millis();
      if (PENDING(ms, flash_time)) return;
      flash_time = ms + 1500;

      TERN_(HAS_HOTEND, send_temperature_hotend(TXT_PREHEAT_HOTEND));
      TERN_(HAS_HEATED_BED, send_temperature_bed(TXT_PREHEAT_BED));
    }

  #endif // HAS_HOTEND || HAS_HEATED_BED

  #if HAS_EXTRUDERS

    void DgusTFT::page19() {       // Filament
      debugPage(19);
      static char filament_status = 0;
      static millis_t flash_time  = 0;
      switch (key_value) {
        case 0: break;

        case 1:           // return
          filament_status = 0;
          injectCommands(F("G90"));
          changePageOfTFT(PAGE_PREPARE);
          break;

        case 2:           // Filament in
          if (getActualTemp_celsius(E0) < 220) {
            filament_status = 0;
            changePageOfTFT(PAGE_FILAMENT_HEAT);
          }
          else {
            filament_status = 1;
            injectCommands(F("G91"));
          }
          break;

        case 3:           // filament out
          if (getActualTemp_celsius(E0) < 220) {
            filament_status = 0;
            changePageOfTFT(PAGE_FILAMENT_HEAT);
          }
          else {
            filament_status = 2;
            injectCommands(F("G91"));
          }
          break;

        case 4:           // stop
          filament_status = 0;
          break;

      }

      const millis_t ms = millis();
      if (PENDING(ms, flash_time)) return;
      flash_time = ms + 1500;

      send_temperature_hotend(TXT_FILAMENT_TEMP);

      if (!isPrinting()) {
        if (filament_status == 1) {
          if (canMove(E0) && !commandsInQueue())
            injectCommands(AC_cmnd_manual_load_filament);
        }
        else if (filament_status == 2) {
          if (canMove(E0) && !commandsInQueue())
            injectCommands(AC_cmnd_manual_unload_filament);
        }
      }
    }

  #endif // HAS_EXTRUDERS

  void DgusTFT::page20() {       // confirm
    debugPage(20);

    switch (key_value) {
      case 0: break;
      case 1:        // return
        changePageOfTFT(page_index_last);
        break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  void DgusTFT::page21() {
    debugPage(21);

    switch (key_value) {
      case 0: break;

      case 1:        // return
        changePageOfTFT(page_index_last);
        break;

      case 2: break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  void DgusTFT::page22() {       // print finish
    debugPage(22);

    switch (key_value) {
      case 0: break;

      case 1:          // OK to finish
        TERN_(CASE_LIGHT_ENABLE, setCaseLightState(false));
        changePageOfTFT(PAGE_MAIN);
        break;

      case 2: break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  void DgusTFT::page23() {
    debugPage(23);

    switch (key_value) {
      case 0: break;
      case 1: changePageOfTFT(page_index_last); break; // return
      case 2: changePageOfTFT(page_index_last); break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  void DgusTFT::page24() {
    debugPage(24);

    switch (key_value) {
      case 0: break;
      case 1: changePageOfTFT(page_index_last); break; // return
      case 2: changePageOfTFT(page_index_last); break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  void DgusTFT::page25() {           // lack filament
    debugPage(25);

    switch (key_value) {
      case 0: break;

      case 1:             // return
        #if ACDEBUG(AC_MARLIN)
          DEBUG_PRINT_PRINTER_STATE(printer_state);
          DEBUG_PRINT_PAUSED_STATE(pause_state);
        #endif
        if (AC_printer_printing == printer_state)
          changePageOfTFT(PAGE_STATUS2);              // show pause
        else if (AC_printer_paused == printer_state) {
          //injectCommands(F("M108"));
          changePageOfTFT(PAGE_STATUS1);              // show resume
        }
        break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  void DgusTFT::page26() {
    debugPage(26);

    switch (key_value) {
      case 0: break;
      case 1: changePageOfTFT(page_index_last); break; // return
      case 2: break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  void DgusTFT::page27() {
    debugPage(27);

    switch (key_value) {
      case 0: break;

      case 1:           // print stop confirmed
        if (isPrintingFromMedia()) {
          printer_state = AC_printer_stopping;
          stopPrint();
          changePageOfTFT(PAGE_MAIN);
        }
        else {
          if (printer_state == AC_printer_resuming_from_power_outage)
            injectCommands(F("M1000 C"));         // Cancel recovery
          printer_state = AC_printer_idle;
        }
        break;

      case 2:           // return
        if (AC_printer_printing == printer_state)
          changePageOfTFT(PAGE_STATUS2);          // show pause
        else if (AC_printer_paused == printer_state)
          changePageOfTFT(PAGE_STATUS1);          // show print
        break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  void DgusTFT::page28() {
    debugPage(28);

    switch (key_value) {
      case 0: break;
      case 1: changePageOfTFT(page_index_last); break; // return
      case 2: break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  void DgusTFT::page29() {
    debugPage(29);

    switch (key_value) {
      case 0: break;

      case 1:        // return
        TERN_(CASE_LIGHT_ENABLE, setCaseLightState(false));
        changePageOfTFT(PAGE_MAIN);
        break;

      case 2: break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  void DgusTFT::page30() {       // Auto heat filament
    debugPage(30);

    switch (key_value) {
      case 0: break;

      case 1:           // return
        setTargetTemp_celsius(230, E0);
        changePageOfTFT(PAGE_FILAMENT);
        break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  void DgusTFT::page31() {
    debugPage(31);

    switch (key_value) {
      case 0: break;
      case 1: break; // return
      case 2: break;
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  void DgusTFT::page32() {
    debugPage(32);

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  #if HAS_LEVELING

    void DgusTFT::page33() {
      debugPage(33);

      switch (key_value) {
        case 0: break;

        case 1:         // auto leveling start
          injectCommands(F("G28\nG29"));
          printer_state = AC_printer_probing;

          // this will cause leveling->preheating->leveling
          #if 0
            #if ENABLED(PREHEAT_BEFORE_LEVELING)
              if (getTargetTemp_celsius(E0) < LEVELING_NOZZLE_TEMP
                 || getTargetTemp_celsius(BED) < LEVELING_BED_TEMP
              ) {
                setTargetTemp_celsius(LEVELING_NOZZLE_TEMP, E0);
                setTargetTemp_celsius(LEVELING_BED_TEMP, BED);
                changePageOfTFT(PAGE_CHS_PROBE_PREHEATING);
              }
              else
                changePageOfTFT(PAGE_LEVELING);

            #else
              changePageOfTFT(PAGE_LEVELING);
            #endif
          #endif

          changePageOfTFT(PAGE_LEVELING);
          break;

        case 2:
          changePageOfTFT(PAGE_PreLEVEL);
          break;
      }

      static millis_t flash_time = 0;
      const millis_t ms = millis();
      if (PENDING(ms, flash_time)) return;
      flash_time = ms + 1500;
    }

    void DgusTFT::page34() {
      debugPage(34);

      #if HAS_HOTEND || HAS_HEATED_BED
        static millis_t flash_time = 0;
        const millis_t ms = millis();
        if (PENDING(ms, flash_time)) return;
        flash_time = ms + 1500;

        TERN_(HAS_HOTEND, send_temperature_hotend(TXT_MAIN_HOTEND));
        TERN_(HAS_HEATED_BED, send_temperature_bed(TXT_MAIN_BED));
      #endif

      if (pop_up_index == 25) {
        pop_up_index = 100;
        changePageOfTFT(PAGE_PreLEVEL);
      }
    }

  #endif // HAS_LEVELING

  void DgusTFT::page115() {
    debugPage(115);

    switch (key_value) {

      case 0: break;
      case 1: changePageOfTFT(PAGE_PreLEVEL); break;

      case 2: {
        injectCommands(F("M1024 S3"));   // -1
        //char value[20]
        //sprintf_P(value, PSTR("G1 Z%iF%i")); enqueue_and_echo_command_now(value); }
      } break;

      case 3: injectCommands(F("M1024 S4")); break; // 1
      case 4: injectCommands(F("M1024 S1")); break; // -0.1
      case 5: injectCommands(F("M1024 S2")); break; // 0.1
      case 6: injectCommands(F("M1024 S0")); break; // prepare, move x y to center
      case 7: injectCommands(F("M1024 S5")); break; // 0.1
    }

    static millis_t flash_time = 0;
    const millis_t ms = millis();
    if (PENDING(ms, flash_time)) return;
    flash_time = ms + 1000;
  }

  void DgusTFT::page117() {  // Page CHS Mute handler
    debugPage(117);
    switch (key_value) {
      case 0: break;

      case 1:
        changePageOfTFT(PAGE_MAIN);
        store_changes();
        break;

      case 2:       // language
        toggle_language();
        goto_system_page();
        break;

      case 3: break;

      case 4:       // audio
        toggle_audio();
        break;

      case 5:       // about
        showAboutPage();
        break;

      case 6:
        changePageOfTFT(PAGE_RECORD);
        break;
    }
  }

  void DgusTFT::page124() {  // first time into page 124 the feedrate percent is not set
    debugPage(124);
    sendValueToTFT(uint16_t(getFeedrate_percent()), TXT_PRINT_SPEED_NOW);
  }

  void DgusTFT::page125() {  // first time into page 125 the feedrate percent is not set
    debugPage(125);
    sendValueToTFT(uint16_t(getFeedrate_percent()), TXT_PRINT_SPEED_NOW);
  }

  void DgusTFT::page170() {  // ENG Mute handler
    debugPage(170);
    switch (key_value) {
      case 0: break;

      case 1:
        changePageOfTFT(PAGE_MAIN);
        store_changes();
        break;

      case 2:       // language
        toggle_language();
        goto_system_page();
        break;

      case 3: break;

      case 4:       // audio
        toggle_audio();
        break;

      case 5:       // about
        showAboutPage();
        break;

      case 6:
        changePageOfTFT(PAGE_RECORD);
        break;
    }
  }

  #if ENABLED(POWER_LOSS_RECOVERY)

    void DgusTFT::page171() {  // CHS power outage resume handler
      debugPage(171);
      #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
        char filename[64] = { '\0' };
      #endif

      switch (key_value) {
        case 0: break;

        case 1: {     // resume
          changePageOfTFT(PAGE_OUTAGE_RECOVERY);
          #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
            card.getLongPath(filename, recovery.info.sd_filename);
            filename[17] = '\0';
            sendTxtToTFT(filename, TXT_OUTAGE_RECOVERY_FILE);
          #else
            sendTxtToTFT(recovery.info.sd_filename, TXT_OUTAGE_RECOVERY_FILE);
          #endif

          sendTxtToTFT(MString<6>(uint16_t(getFeedrate_percent())), TXT_PRINT_SPEED);
          sendTxtToTFT(MString<6>(uint16_t(getProgress_percent())), TXT_PRINT_PROGRESS);

          changePageOfTFT(PAGE_STATUS2);              // show pause
          injectCommands(F("M355 S1\nM1000"));        // case light on, home and start recovery
        } break;

        case 2:       // cancel
          printer_state = AC_printer_idle;
          changePageOfTFT(PAGE_MAIN);
          injectCommands(F("M355 S0\nM1000 C"));      // cancel recovery
          break;
      }
    }

    void DgusTFT::page173() {  // ENG power outage resume handler
      debugPage(173);
      #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
        char filename[64] = { '\0' };
      #endif

      switch (key_value) {
        case 0: break;

        case 1: {     // resume
          changePageOfTFT(PAGE_OUTAGE_RECOVERY);
          #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
            card.getLongPath(filename, recovery.info.sd_filename);
            filename[17] = '\0';
            sendTxtToTFT(filename, TXT_OUTAGE_RECOVERY_FILE);
          #else
            sendTxtToTFT(recovery.info.sd_filename, TXT_OUTAGE_RECOVERY_FILE);
          #endif

          sendTxtToTFT(MString<6>(uint16_t(getFeedrate_percent())), TXT_PRINT_SPEED);
          sendTxtToTFT(MString<6>(uint16_t(getProgress_percent())), TXT_PRINT_PROGRESS);

          changePageOfTFT(PAGE_STATUS2);          // show pause
          injectCommands(F("M355 S1\nM1000"));    // case light on, home and start recovery
        } break;

        case 2:       // cancel
          printer_state = AC_printer_idle;
          changePageOfTFT(PAGE_MAIN);
          injectCommands(F("M355 S0\nM1000 C"));  // cancel recovery
          break;
      }
    }

  #endif // POWER_LOSS_RECOVERY

  #if HAS_LEVELING

    void DgusTFT::page175() {     // CHS probe preheating handler
      debugPage(175);

      #if HAS_HOTEND || HAS_HEATED_BED
        static millis_t flash_time = 0;
        const millis_t ms = millis();
        if (PENDING(ms, flash_time)) return;
        flash_time = ms + 1500;

        TERN_(HAS_HOTEND, send_temperature_hotend(TXT_MAIN_HOTEND));
        TERN_(HAS_HEATED_BED, send_temperature_bed(TXT_MAIN_BED));
      #endif
    }

    void DgusTFT::page176() {     // ENG probe preheating handler
      debugPage(176);

      #if HAS_HOTEND || HAS_HEATED_BED
        static millis_t flash_time = 0;
        const millis_t ms = millis();
        if (PENDING(ms, flash_time)) return;
        flash_time = ms + 1500;

        TERN_(HAS_HOTEND, send_temperature_hotend(TXT_MAIN_HOTEND));
        TERN_(HAS_HEATED_BED, send_temperature_bed(TXT_MAIN_BED));
      #endif
    }

  #endif // HAS_LEVELING

  void DgusTFT::page177_to_198() {
    debugPage();
    switch (key_value) {
      case 1:       // return
        #if ACDEBUG(AC_MARLIN)
          //DEBUG_ECHOLNPGM("page_index_now: ", page_index_now);
          //DEBUG_ECHOLNPGM("page_index_last: ", page_index_last);
          //DEBUG_ECHOLNPGM("page_index_last_2: ", page_index_last_2);
        #endif

        if ((WITHIN(page_index_now, PAGE_CHS_ABNORMAL_X_ENDSTOP, PAGE_CHS_ABNORMAL_Z_ENDSTOP))
         || (WITHIN(page_index_now, PAGE_ENG_ABNORMAL_X_ENDSTOP, PAGE_ENG_ABNORMAL_Z_ENDSTOP))
        ) {
          if (lcd_info.language == ENG) {
            if (page_index_last_2 > 120) page_index_last_2 -= 120;
            if (page_index_last > 120) page_index_last -= 120;
          }

          if (PAGE_STATUS1 == page_index_last_2 || PAGE_STATUS2 == page_index_last_2 || PAGE_PRINT_FINISH == page_index_last)
            changePageOfTFT(PAGE_MAIN);
          else
            changePageOfTFT(page_index_last_2);
        }
        else {
          if (lcd_info.language == ENG && page_index_last > 120)
            page_index_last -= 120;
          changePageOfTFT(page_index_last);
        }

        onSurviveInKilled();
        stepper.disable_all_steppers();
        break;

      default: break;
    }
  }

  #if 0
    void DgusTFT::page178_to_181_190_to_193() {  // temperature abnormal
      debugPage();
      #if ACDEBUG(AC_ALL)
      if ((page_index_saved != page_index_now) || (key_value_saved != key_value)) {
        DEBUG_ECHOLNPGM("page178_to_181_190_to_193  page_index_last_2: ", page_index_last_2,  "  page_index_last: ", page_index_last, "  page_index_now: ", page_index_now, "  key: ", key_value);
        page_index_saved = page_index_now;
        key_value_saved = key_value;
      }
      #endif
      switch (key_value) {
        case 1:     // return
          SERIAL_ECHOLNPGM("page_index_now: ", page_index_now);
          SERIAL_ECHOLNPGM("page_index_last: ", page_index_last);
          SERIAL_ECHOLNPGM("page_index_last_2: ", page_index_last_2);

          if (isPrinting() || isPrintingPaused() || isPrintingFromMedia()) {
            printer_state = AC_printer_stopping;
            stopPrint();
            changePageOfTFT(PAGE_MAIN);
          }
          else
            changePageOfTFT(page_index_last);

          onSurviveInKilled();
          break;

        default: break;
      }
    }
  #endif

  void DgusTFT::page199_to_200() {
    debugPage();
    switch (key_value) {
      case 1:       // return
        #if ACDEBUG(AC_MARLIN)
          //DEBUG_ECHOLNPGM("page_index_now: ", page_index_now);
          //DEBUG_ECHOLNPGM("page_index_last: ", page_index_last);
          //DEBUG_ECHOLNPGM("page_index_last_2: ", page_index_last_2);
        #endif
        onSurviveInKilled();
        changePageOfTFT(PAGE_PreLEVEL);
        break;

      default: break;
    }
  }

  inline void ProbeTare() {
    #if PIN_EXISTS(AUTO_LEVEL_TX)
      OUT_WRITE(AUTO_LEVEL_TX_PIN, LOW);
      delay(300);
      OUT_WRITE(AUTO_LEVEL_TX_PIN, HIGH);
      delay(100);
    #endif
  }

  inline bool getProbeState() { return PROBE_TRIGGERED(); }

  void DgusTFT::page201() {  // probe precheck
    debugPage(201);
    static millis_t probe_check_time   = 0;
    static millis_t temperature_time   = 0;
    static uint8_t probe_check_counter = 0;
    static uint8_t probe_state_last    = 0;
    static bool probe_tare_flag        = 0;

    if (!probe_tare_flag) {
      ProbeTare();

      delay(100);

      if (getProbeState()) {        // triggered too early
        probe_check_counter = 0;
        probe_tare_flag = 0;
        changePageOfTFT(PAGE_CHS_PROBE_PRECHECK_FAILED);
      }
      probe_tare_flag = 1;
    }

    switch (key_value) {
      case 1:     // cancel
        probe_check_counter = 0;
        probe_tare_flag = 0;
        changePageOfTFT(PAGE_PreLEVEL);
        break;

      default: break;
    }

    if (ELAPSED(millis(), probe_check_time)) {
      probe_check_time = millis() + 300;

      if (!probe_state_last && getProbeState()) {
        probe_check_counter = 0;
        probe_tare_flag = 0;
        changePageOfTFT(PAGE_CHS_PROBE_PRECHECK_OK);
      }

      probe_state_last = getProbeState();

      if (probe_check_counter++ >= 200) {         // waiting for 1 min
        probe_check_counter = 0;
        probe_tare_flag = 0;
        changePageOfTFT(PAGE_CHS_PROBE_PRECHECK_FAILED);
      }
    }

    if (ELAPSED(millis(), temperature_time)) {
      temperature_time = millis() + 1500;
      TERN_(HAS_HOTEND, send_temperature_hotend(TXT_MAIN_HOTEND));
      TERN_(HAS_HEATED_BED, send_temperature_bed(TXT_MAIN_BED));
    }
  }

  void DgusTFT::page202() {  // probe precheck ok
    debugPage(202);

    //static millis_t flash_time = 0;
    //static millis_t probe_check_counter = 0;
    //static uint8_t probe_state_last = 0;

    delay(3000);

    injectCommands(F("G28\nG29"));
    printer_state = AC_printer_probing;
    changePageOfTFT(PAGE_LEVELING);
  }

  void DgusTFT::page203() {    // probe precheck failed
    debugPage(203);
    //static millis_t probe_check_counter = 0;
    //static uint8_t probe_state_last = 0;

    #if HAS_HOTEND || HAS_HEATED_BED
      static millis_t flash_time = 0;
      const millis_t ms = millis();
      if (PENDING(ms, flash_time)) return;
      flash_time = ms + 1500;

      TERN_(HAS_HOTEND, send_temperature_hotend(TXT_MAIN_HOTEND));
      TERN_(HAS_HEATED_BED, send_temperature_bed(TXT_MAIN_BED));
    #endif
  }

  void DgusTFT::pop_up_manager() {
    #if ACDEBUG(AC_ALL)
      static uint8_t pop_up_index_saved; // = 0
      if (pop_up_index_saved != pop_up_index) {
        DEBUG_ECHOLNPGM("pop_up_manager  pop_up_index: ", pop_up_index);
        pop_up_index_saved = pop_up_index;
      }
    #endif

    switch (pop_up_index) {
      case 10:      // T0 error
        if (page_index_now != PAGE_ABNORMAL)
          changePageOfTFT(PAGE_ABNORMAL);
        pop_up_index = 100;
        break;

      case 15:      // filament lack
      case 23:
        if (page_index_now != PAGE_FILAMENT_LACK)
          changePageOfTFT(PAGE_FILAMENT_LACK);
        pop_up_index = 100;
        break;

      case 16:      // stop wait
        changePageOfTFT(PAGE_WAIT_STOP);
        pop_up_index = 100;
        break;

      case 18:
        changePageOfTFT(PAGE_STATUS1);
        pop_up_index = 100;
        break;

      case 24: {
        // Report Printing Time in minutes
        sendTimeToTFT(getProgress_seconds_elapsed() / 60, TXT_FINISH_TIME);
        changePageOfTFT(PAGE_PRINT_FINISH);
        //tftSendLn(AC_msg_print_complete);   // no idea why this causes a compile error
        pop_up_index = 100;
      } break;

      case 25:  // LEVEL DONE
        changePageOfTFT(PAGE_PreLEVEL);
        pop_up_index = 100;
        break;
    }
  }

  void DEBUG_PRINT_PAUSED_STATE(const paused_state_t state, FSTR_P const msg/*=nullptr*/) {
    if (msg) DEBUG_ECHO(msg);
    DEBUG_ECHOPGM("Paused state: ", state, "  ");
    switch (state) {
      case AC_paused_heater_timed_out: DEBUG_ECHOPGM("AC_paused_heater_timed_out"); break;
      case AC_paused_filament_lack: DEBUG_ECHOPGM("AC_paused_filament_lack"); break;
      case AC_paused_purging_filament: DEBUG_ECHOPGM("AC_paused_purging_filament"); break;
      case AC_paused_idle: DEBUG_ECHOPGM("AC_paused_idle"); break;
    }
    DEBUG_EOL();
  }

  // Human-readable debugging

  void DEBUG_PRINT_PRINTER_STATE(const printer_state_t state, FSTR_P const msg/*=nullptr*/) {
    if (msg) DEBUG_ECHO(msg);
    DEBUG_ECHOPGM("Printer State: ", state, "  ");
    switch (state) {
      case AC_printer_idle: DEBUG_ECHOPGM("AC_printer_idle"); break;
      case AC_printer_probing: DEBUG_ECHOPGM("AC_printer_probing"); break;
      case AC_printer_printing: DEBUG_ECHOPGM("AC_printer_printing"); break;
      case AC_printer_pausing: DEBUG_ECHOPGM("AC_printer_pausing"); break;
      case AC_printer_paused: DEBUG_ECHOPGM("AC_printer_paused"); break;
      case AC_printer_stopping: DEBUG_ECHOPGM("AC_printer_stopping"); break;
      case AC_printer_stopping_from_media_remove: DEBUG_ECHOPGM("AC_printer_stopping_from_media_remove"); break;
      case AC_printer_resuming_from_power_outage: DEBUG_ECHOPGM("AC_printer_resuming_from_power_outage"); break;
    }
    DEBUG_EOL();
  }

  void DEBUG_PRINT_TIMER_EVENT(const timer_event_t event, FSTR_P const msg/*=nullptr*/) {
    if (msg) DEBUG_ECHOPGM(msg, event);
    DEBUG_ECHOPGM("timerEvent() ", event, "  ");
    switch (event) {
      case AC_timer_started: DEBUG_ECHOPGM("AC_timer_started"); break;
      case AC_timer_paused: DEBUG_ECHOPGM("AC_timer_paused"); break;
      case AC_timer_stopped: DEBUG_ECHOPGM("AC_timer_stopped"); break;
    }
    DEBUG_EOL();
  }

  void DEBUG_PRINT_MEDIA_EVENT(const media_event_t event, FSTR_P const msg/*=nullptr*/) {
    if (msg) DEBUG_ECHOPGM(msg, event);
    DEBUG_ECHOPGM("ProcessMediaStatus() ", event, "  ");
    switch (event) {
      case AC_media_inserted: DEBUG_ECHOPGM("AC_media_inserted"); break;
      case AC_media_removed: DEBUG_ECHOPGM("AC_media_removed"); break;
      case AC_media_error: DEBUG_ECHOPGM("AC_media_error"); break;
    }
    DEBUG_EOL();
  }

} // namespace

#endif // ANYCUBIC_LCD_VYPER
