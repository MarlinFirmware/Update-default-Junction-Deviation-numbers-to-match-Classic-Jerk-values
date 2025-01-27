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

//
// Main Menu
//

#include "../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_MENU

#include "menu_item.h"
#include "../../module/temperature.h"
#include "../../gcode/queue.h"
#include "../../module/printcounter.h"
#include "../../module/stepper.h"
#include "../../sd/cardreader.h"

#if ENABLED(PSU_CONTROL)
  #include "../../feature/power.h"
#endif

#if HAS_GAMES && DISABLED(LCD_INFO_MENU)
  #include "game/game.h"
#endif

#if ANY(HAS_MEDIA, HOST_PROMPT_SUPPORT) || defined(ACTION_ON_CANCEL)
  #define MACHINE_CAN_STOP 1
#endif
#if ANY(HAS_MEDIA, HOST_PROMPT_SUPPORT, PARK_HEAD_ON_PAUSE) || defined(ACTION_ON_PAUSE)
  #define MACHINE_CAN_PAUSE 1
#endif

#if ENABLED(MMU_MENUS)
  #include "menu_mmu2.h"
#endif

#if ENABLED(PASSWORD_FEATURE)
  #include "../../feature/password/password.h"
#endif

#if (ENABLED(HOST_START_MENU_ITEM) && defined(ACTION_ON_START)) || (ENABLED(HOST_SHUTDOWN_MENU_ITEM) && defined(SHUTDOWN_ACTION))
  #include "../../feature/host_actions.h"
#endif

#if ENABLED(GCODE_REPEAT_MARKERS)
  #include "../../feature/repeat.h"
#endif

void menu_tune();
void menu_cancelobject();
void menu_motion();
void menu_temperature();
void menu_configuration();

#if ANY(HAS_LEVELING, HAS_BED_PROBE, ASSISTED_TRAMMING_WIZARD, LCD_BED_TRAMMING)
  void menu_probe_level();
#endif

#if HAS_POWER_MONITOR
  void menu_power_monitor();
#endif

#if ENABLED(MIXING_EXTRUDER)
  void menu_mixer();
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  void menu_change_filament();
#endif

#if ENABLED(LCD_INFO_MENU)
  void menu_info();
#endif

#if ENABLED(LED_CONTROL_MENU)
  void menu_led();
#elif ALL(CASE_LIGHT_MENU, CASELIGHT_USES_BRIGHTNESS)
  void menu_case_light();
#elif ENABLED(CASE_LIGHT_MENU)
  #include "../../feature/caselight.h"
#endif

#if HAS_CUTTER
  void menu_spindle_laser();
#endif

#if ENABLED(PREHEAT_SHORTCUT_MENU_ITEM)
  void menu_preheat_only();
#endif

#if ENABLED(HOTEND_IDLE_TIMEOUT)
  void menu_hotend_idle();
#endif

#if HAS_MULTI_LANGUAGE
  void menu_language();
#endif

#if ENABLED(CUSTOM_MENU_MAIN)

  void _lcd_custom_menu_main_gcode(FSTR_P const fstr) {
    queue.inject(fstr);
    TERN_(CUSTOM_MENU_MAIN_SCRIPT_AUDIBLE_FEEDBACK, ui.completion_feedback());
    TERN_(CUSTOM_MENU_MAIN_SCRIPT_RETURN, ui.return_to_status());
  }

  void custom_menus_main() {
    START_MENU();
    BACK_ITEM(MSG_MAIN_MENU);

    #define HAS_CUSTOM_ITEM_MAIN(N) (defined(MAIN_MENU_ITEM_##N##_DESC) && defined(MAIN_MENU_ITEM_##N##_GCODE))

    #ifdef CUSTOM_MENU_MAIN_SCRIPT_DONE
      #define _DONE_SCRIPT "\n" CUSTOM_MENU_MAIN_SCRIPT_DONE
    #else
      #define _DONE_SCRIPT ""
    #endif
    #define GCODE_LAMBDA_MAIN(N) []{ _lcd_custom_menu_main_gcode(F(MAIN_MENU_ITEM_##N##_GCODE _DONE_SCRIPT)); }
    #define _CUSTOM_ITEM_MAIN(N) ACTION_ITEM_F(F(MAIN_MENU_ITEM_##N##_DESC), GCODE_LAMBDA_MAIN(N));
    #define _CUSTOM_ITEM_MAIN_CONFIRM(N)          \
      SUBMENU_F(F(MAIN_MENU_ITEM_##N##_DESC), []{ \
          MenuItem_confirm::confirm_screen(       \
            GCODE_LAMBDA_MAIN(N), nullptr,        \
            F(MAIN_MENU_ITEM_##N##_DESC "?")      \
          );                                      \
        })

    #define CUSTOM_ITEM_MAIN(N) do{ \
      constexpr char c = MAIN_MENU_ITEM_##N##_GCODE[strlen(MAIN_MENU_ITEM_##N##_GCODE) - 1]; \
      static_assert(c != '\n' && c != '\r', "MAIN_MENU_ITEM_" STRINGIFY(N) "_GCODE cannot have a newline at the end. Please remove it."); \
      if (ENABLED(MAIN_MENU_ITEM_##N##_CONFIRM)) \
        _CUSTOM_ITEM_MAIN_CONFIRM(N); \
      else \
        _CUSTOM_ITEM_MAIN(N); \
    }while(0)

    #if HAS_CUSTOM_ITEM_MAIN(1)
      CUSTOM_ITEM_MAIN(1);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(2)
      CUSTOM_ITEM_MAIN(2);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(3)
      CUSTOM_ITEM_MAIN(3);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(4)
      CUSTOM_ITEM_MAIN(4);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(5)
      CUSTOM_ITEM_MAIN(5);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(6)
      CUSTOM_ITEM_MAIN(6);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(7)
      CUSTOM_ITEM_MAIN(7);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(8)
      CUSTOM_ITEM_MAIN(8);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(9)
      CUSTOM_ITEM_MAIN(9);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(10)
      CUSTOM_ITEM_MAIN(10);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(11)
      CUSTOM_ITEM_MAIN(11);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(12)
      CUSTOM_ITEM_MAIN(12);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(13)
      CUSTOM_ITEM_MAIN(13);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(14)
      CUSTOM_ITEM_MAIN(14);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(15)
      CUSTOM_ITEM_MAIN(15);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(16)
      CUSTOM_ITEM_MAIN(16);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(17)
      CUSTOM_ITEM_MAIN(17);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(18)
      CUSTOM_ITEM_MAIN(18);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(19)
      CUSTOM_ITEM_MAIN(19);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(20)
      CUSTOM_ITEM_MAIN(20);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(21)
      CUSTOM_ITEM_MAIN(21);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(22)
      CUSTOM_ITEM_MAIN(22);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(23)
      CUSTOM_ITEM_MAIN(23);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(24)
      CUSTOM_ITEM_MAIN(24);
    #endif
    #if HAS_CUSTOM_ITEM_MAIN(25)
      CUSTOM_ITEM_MAIN(25);
    #endif
    END_MENU();
  }

#endif // CUSTOM_MENU_MAIN

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  // This menu item is last with an encoder. Otherwise, somewhere in the middle.
  #if E_STEPPERS == 1 && DISABLED(FILAMENT_LOAD_UNLOAD_GCODES)
    #define FILAMENT_CHANGE_ITEM() YESNO_ITEM(MSG_FILAMENTCHANGE, menu_change_filament, nullptr, \
                                    GET_TEXT_F(MSG_FILAMENTCHANGE), (const char *)nullptr, F("?"))
  #else
    #define FILAMENT_CHANGE_ITEM() SUBMENU(MSG_FILAMENTCHANGE, menu_change_filament)
  #endif
#endif

void menu_main() {
  const bool busy = printingIsActive();
  #if HAS_MEDIA
    const bool card_detected = card.isMounted(),
               card_open = card_detected && card.isFileOpen();
  #endif

  START_MENU();
  BACK_ITEM(MSG_INFO_SCREEN);

  #if HAS_MEDIA && !defined(MEDIA_MENU_AT_TOP) && !HAS_MARLINUI_ENCODER
    #define MEDIA_MENU_AT_TOP
  #endif

  if (busy) {
    #if MACHINE_CAN_PAUSE
      ACTION_ITEM(MSG_PAUSE_PRINT, ui.pause_print);
    #endif
    #if MACHINE_CAN_STOP
      SUBMENU(MSG_STOP_PRINT, []{
        MenuItem_confirm::select_screen(
          GET_TEXT_F(MSG_BUTTON_STOP), GET_TEXT_F(MSG_BACK),
          ui.abort_print, nullptr,
          GET_TEXT_F(MSG_STOP_PRINT), (const char *)nullptr, F("?")
        );
      });
    #endif

    #if ENABLED(GCODE_REPEAT_MARKERS)
      if (repeat.is_active())
        ACTION_ITEM(MSG_END_LOOPS, repeat.cancel);
    #endif

    SUBMENU(MSG_TUNE, menu_tune);

    #if ENABLED(CANCEL_OBJECTS) && DISABLED(SLIM_LCD_MENUS)
      SUBMENU(MSG_CANCEL_OBJECT, []{ editable.int8 = -1; ui.goto_screen(menu_cancelobject); });
    #endif
  }
  else {
    #if ALL(HAS_MEDIA, MEDIA_MENU_AT_TOP)
      // BEGIN MEDIA MENU
      if (card_detected) {
        if (!card_open) {
          #if ENABLED(MENU_ADDAUTOSTART)
            ACTION_ITEM(MSG_RUN_AUTO_FILES, card.autofile_begin); // Run Auto Files
          #endif

          #if HAS_SD_DETECT
            GCODES_ITEM(MSG_CHANGE_MEDIA, F("M21" TERN_(HAS_MULTI_VOLUME, "S"))); // M21 Change Media
            #if HAS_MULTI_VOLUME
              GCODES_ITEM(MSG_ATTACH_USB_MEDIA, F("M21U")); // M21 Attach USB Media
            #endif
          #else                                             // - or -
            ACTION_ITEM(MSG_RELEASE_MEDIA, []{              // M22 Release Media
              queue.inject(F("M22"));
              #if ENABLED(TFT_COLOR_UI)
                // Menu display issue on item removal with multi language selection menu
                if (encoderTopLine > 0) encoderTopLine--;
                ui.refresh();
              #endif
            });
          #endif
          SUBMENU(MSG_MEDIA_MENU, MEDIA_MENU_GATEWAY);      // Media Menu (or Password First)
        }
      }
      else {
        #if HAS_SD_DETECT
          ACTION_ITEM(MSG_NO_MEDIA, nullptr);               // "No Media"
        #else
          #if HAS_MULTI_VOLUME
            GCODES_ITEM(MSG_ATTACH_SD_MEDIA, F("M21S"));    // M21S Attach SD Card
            GCODES_ITEM(MSG_ATTACH_USB_MEDIA, F("M21U"));   // M21U Attach USB Media
          #else
            GCODES_ITEM(MSG_ATTACH_MEDIA, F("M21"));        // M21 Attach Media
          #endif
        #endif
      }
      // END MEDIA MENU
    #endif

    if (TERN0(MACHINE_CAN_PAUSE, printingIsPaused()))
      ACTION_ITEM(MSG_RESUME_PRINT, ui.resume_print);

    #if ENABLED(HOST_START_MENU_ITEM) && defined(ACTION_ON_START)
      ACTION_ITEM(MSG_HOST_START_PRINT, hostui.start);
    #endif

    #if ENABLED(PREHEAT_SHORTCUT_MENU_ITEM)
      SUBMENU(MSG_PREHEAT_CUSTOM, menu_preheat_only);
    #endif

    SUBMENU(MSG_MOTION, menu_motion);

    #if ANY(HAS_LEVELING, HAS_BED_PROBE, ASSISTED_TRAMMING_WIZARD, LCD_BED_TRAMMING)
      SUBMENU(MSG_PROBE_AND_LEVEL, menu_probe_level);
    #endif
  }

  #if HAS_CUTTER
    SUBMENU(MSG_CUTTER(MENU), STICKY_SCREEN(menu_spindle_laser));
  #endif

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    FILAMENT_CHANGE_ITEM();
  #endif

  #if HAS_TEMPERATURE
    SUBMENU(MSG_TEMPERATURE, menu_temperature);
  #endif

  #if HAS_POWER_MONITOR
    SUBMENU(MSG_POWER_MONITOR, menu_power_monitor);
  #endif

  #if ENABLED(MIXING_EXTRUDER)
    SUBMENU(MSG_MIXER, menu_mixer);
  #endif

  #if ENABLED(MMU_MENUS)
    // MMU3 can show print stats which can be useful during
    // the print, so MMU menus are required for MMU3.
    if (TERN1(HAS_PRUSA_MMU2, !busy)) SUBMENU(MSG_MMU2_MENU, menu_mmu2);
  #endif

  SUBMENU(MSG_CONFIGURATION, menu_configuration);

  #if ENABLED(CUSTOM_MENU_MAIN)
    if (TERN1(CUSTOM_MENU_MAIN_ONLY_IDLE, !busy)) {
      #ifdef CUSTOM_MENU_MAIN_TITLE
        SUBMENU_F(F(CUSTOM_MENU_MAIN_TITLE), custom_menus_main);
      #else
        SUBMENU(MSG_CUSTOM_COMMANDS, custom_menus_main);
      #endif
    }
  #endif

  #if ENABLED(LCD_INFO_MENU)
    SUBMENU(MSG_INFO_MENU, menu_info);
  #endif

  #if ENABLED(LED_CONTROL_MENU)
    SUBMENU(MSG_LIGHTS, menu_led);
  #elif ALL(CASE_LIGHT_MENU, CASELIGHT_USES_BRIGHTNESS)
    SUBMENU(MSG_CASE_LIGHT, menu_case_light);
  #elif ENABLED(CASE_LIGHT_MENU)
    EDIT_ITEM(bool, MSG_CASE_LIGHT, &caselight.on, caselight.update_enabled);
  #endif

  //
  // Switch power on/off
  //
  #if ENABLED(PSU_CONTROL)
    if (powerManager.psu_on)
      #if ENABLED(PS_OFF_CONFIRM)
        CONFIRM_ITEM(MSG_SWITCH_PS_OFF,
          MSG_YES, MSG_NO,
          ui.poweroff, nullptr,
          GET_TEXT_F(MSG_SWITCH_PS_OFF), (const char *)nullptr, F("?")
        );
      #else
        ACTION_ITEM(MSG_SWITCH_PS_OFF, ui.poweroff);
      #endif
    else
      GCODES_ITEM(MSG_SWITCH_PS_ON, F("M80"));
  #endif

  #if HAS_MEDIA && DISABLED(MEDIA_MENU_AT_TOP)
    // BEGIN MEDIA MENU
    if (card_detected) {
      if (!card_open) {
        #if ENABLED(MENU_ADDAUTOSTART)
          ACTION_ITEM(MSG_RUN_AUTO_FILES, card.autofile_begin); // Run Auto Files
        #endif

        #if HAS_SD_DETECT
          GCODES_ITEM(MSG_CHANGE_MEDIA, F("M21" TERN_(HAS_MULTI_VOLUME, "S"))); // M21 Change Media
          #if HAS_MULTI_VOLUME
            GCODES_ITEM(MSG_ATTACH_USB_MEDIA, F("M21U")); // M21 Attach USB Media
          #endif
        #else                                             // - or -
          ACTION_ITEM(MSG_RELEASE_MEDIA, []{              // M22 Release Media
            queue.inject(F("M22"));
            #if ENABLED(TFT_COLOR_UI)
              // Menu display issue on item removal with multi language selection menu
              if (encoderTopLine > 0) encoderTopLine--;
              ui.refresh();
            #endif
          });
        #endif
        SUBMENU(MSG_MEDIA_MENU, MEDIA_MENU_GATEWAY);      // Media Menu (or Password First)
      }
    }
    else {
      #if HAS_SD_DETECT
        ACTION_ITEM(MSG_NO_MEDIA, nullptr);               // "No Media"
      #else
        #if HAS_MULTI_VOLUME
          GCODES_ITEM(MSG_ATTACH_SD_MEDIA, F("M21S"));    // M21S Attach SD Card
          GCODES_ITEM(MSG_ATTACH_USB_MEDIA, F("M21U"));   // M21U Attach USB Media
        #else
          GCODES_ITEM(MSG_ATTACH_MEDIA, F("M21"));        // M21 Attach Media
        #endif
      #endif
    }
    // END MEDIA MENU
  #endif

  #if HAS_SERVICE_INTERVALS
    static auto _service_reset = [](const int index) {
      print_job_timer.resetServiceInterval(index);
      ui.completion_feedback();
      ui.reset_status();
      ui.return_to_status();
    };
    #if SERVICE_INTERVAL_1 > 0
      CONFIRM_ITEM_F(F(SERVICE_NAME_1),
        MSG_BUTTON_RESET, MSG_BUTTON_CANCEL,
        []{ _service_reset(1); }, nullptr,
        GET_TEXT_F(MSG_SERVICE_RESET), F(SERVICE_NAME_1), F("?")
      );
    #endif
    #if SERVICE_INTERVAL_2 > 0
      CONFIRM_ITEM_F(F(SERVICE_NAME_2),
        MSG_BUTTON_RESET, MSG_BUTTON_CANCEL,
        []{ _service_reset(2); }, nullptr,
        GET_TEXT_F(MSG_SERVICE_RESET), F(SERVICE_NAME_2), F("?")
      );
    #endif
    #if SERVICE_INTERVAL_3 > 0
      CONFIRM_ITEM_F(F(SERVICE_NAME_3),
        MSG_BUTTON_RESET, MSG_BUTTON_CANCEL,
        []{ _service_reset(3); }, nullptr,
        GET_TEXT_F(MSG_SERVICE_RESET), F(SERVICE_NAME_3), F("?")
      );
    #endif
  #endif

  #if HAS_GAMES && DISABLED(LCD_INFO_MENU)
    #if ENABLED(GAMES_EASTER_EGG)
      SKIP_ITEM();
      SKIP_ITEM();
      SKIP_ITEM();
    #endif
    // Game sub-menu or the individual game
    {
      SUBMENU(
        #if HAS_GAME_MENU
          MSG_GAMES, menu_game
        #elif ENABLED(MARLIN_BRICKOUT)
          MSG_BRICKOUT, brickout.enter_game
        #elif ENABLED(MARLIN_INVADERS)
          MSG_INVADERS, invaders.enter_game
        #elif ENABLED(MARLIN_SNAKE)
          MSG_SNAKE, snake.enter_game
        #elif ENABLED(MARLIN_MAZE)
          MSG_MAZE, maze.enter_game
        #endif
      );
    }
  #endif

  #if HAS_MULTI_LANGUAGE
    SUBMENU(LANGUAGE, menu_language);
  #endif

  #if ENABLED(HOST_SHUTDOWN_MENU_ITEM) && defined(SHUTDOWN_ACTION)
    SUBMENU(MSG_HOST_SHUTDOWN, []{
      MenuItem_confirm::select_screen(
        GET_TEXT_F(MSG_BUTTON_PROCEED), GET_TEXT_F(MSG_BUTTON_CANCEL),
        []{ ui.return_to_status(); hostui.shutdown(); }, nullptr,
        GET_TEXT_F(MSG_HOST_SHUTDOWN), (const char *)nullptr, F("?")
      );
    });
  #endif

  END_MENU();
}

#endif // HAS_MARLINUI_MENU
