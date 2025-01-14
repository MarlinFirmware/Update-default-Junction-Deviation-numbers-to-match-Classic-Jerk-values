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

#if ENABLED(MMU_MENUS)

#include "../../MarlinCore.h"

#if HAS_PRUSA_MMU3
  #include "../../feature/mmu3/mmu3.h"
  #include "../../feature/mmu3/mmu3_reporting.h"
  #include "../../feature/mmu3/SpoolJoin.h"
#else
  #include "../../feature/mmu/mmu2.h"
#endif

#include "menu_mmu2.h"
#include "menu_item.h"

//
// Load Filament
//

inline void action_mmu2_load_to_nozzle(const uint8_t tool) {
  ui.reset_status();
  ui.return_to_status();
  ui.status_printf(0, GET_TEXT_F(MSG_MMU2_LOADING_FILAMENT), int(tool + 1));
  TERN(HAS_PRUSA_MMU3, mmu3.load_to_nozzle(tool), mmu2.load_to_nozzle(tool));
  ui.reset_status();
}

void _mmu2_load_to_feeder(const uint8_t tool) {
  ui.reset_status();
  ui.return_to_status();
  ui.status_printf(0, GET_TEXT_F(MSG_MMU2_LOADING_FILAMENT), int(tool + 1));
  TERN(HAS_PRUSA_MMU3, mmu3.load_to_feeder(tool), mmu2.load_to_feeder(tool));
  ui.reset_status();
}

void action_mmu2_load_all() {
  EXTRUDER_LOOP() _mmu2_load_to_feeder(e);
}

void menu_mmu2_load_filament() {
  START_MENU();
  BACK_ITEM(MSG_MMU2_MENU);
  ACTION_ITEM(MSG_MMU2_ALL, action_mmu2_load_all);
  EXTRUDER_LOOP() ACTION_ITEM_N(e, MSG_MMU2_FILAMENT_N, []{ _mmu2_load_to_feeder(MenuItemBase::itemIndex); });
  END_MENU();
}

void menu_mmu2_load_to_nozzle() {
  START_MENU();
  BACK_ITEM(MSG_MMU2_MENU);
  EXTRUDER_LOOP() ACTION_ITEM_N(e, MSG_MMU2_FILAMENT_N, []{ action_mmu2_load_to_nozzle(MenuItemBase::itemIndex); });
  END_MENU();
}

//
// Eject Filament
//

void _mmu2_eject_filament(uint8_t index) {
  ui.reset_status();
  ui.return_to_status();
  ui.status_printf(0, GET_TEXT_F(MSG_MMU2_EJECTING_FILAMENT), int(index + 1));
  if (mmu3.eject_filament(index, true)) ui.reset_status();
}

void _mmu2_cut_filament(uint8_t index) {
  ui.reset_status();
  ui.return_to_status();
  ui.status_printf(0, GET_TEXT_F(MSG_MMU2_CUTTING_FILAMENT), int(index + 1));
  if (TERN0(HAS_PRUSA_MMU3, mmu3.cut_filament(index, true)))
    ui.reset_status();
}

void action_mmu2_unload_filament() {
  ui.reset_status();
  ui.return_to_status();
  LCD_MESSAGE(MSG_MMU2_UNLOADING_FILAMENT);
  while (!TERN(HAS_PRUSA_MMU3, mmu3.unload(), mmu2.unload())) {
    safe_delay(50);
    TERN(HAS_PRUSA_MMU3, MMU3::marlin_idle(true), idle());
  }
  ui.reset_status();
}

void menu_mmu2_eject_filament() {
  START_MENU();
  BACK_ITEM(MSG_MMU2_MENU);
  EXTRUDER_LOOP() ACTION_ITEM_N(e, MSG_MMU2_FILAMENT_N, []{ _mmu2_eject_filament(MenuItemBase::itemIndex); });
  END_MENU();
}

// Cutter

#if HAS_PRUSA_MMU3

void menu_mmu3_cutter_set_mode(uint8_t mode) { mmu3.cutter_mode = mode; }
void menu_mmu3_cutter_disable() { menu_mmu3_cutter_set_mode(0); }
void menu_mmu3_cutter_enable() { menu_mmu3_cutter_set_mode(1); }
void menu_mmu3_cutter_always() { menu_mmu3_cutter_set_mode(2); }

void menu_mmu3_cutter() {
  START_MENU();
  BACK_ITEM(MSG_MMU2_MENU);
  ACTION_ITEM(MSG_MMU_CUTTER_MODE_DISABLE, menu_mmu3_cutter_disable);
  ACTION_ITEM(MSG_MMU_CUTTER_MODE_ENABLE, menu_mmu3_cutter_enable);
  ACTION_ITEM(MSG_MMU_CUTTER_MODE_ALWAYS, menu_mmu3_cutter_always);
  END_MENU();
}

void menu_mmu3_cut_filament() {
  START_MENU();
  BACK_ITEM(MSG_MMU2_MENU);
  EXTRUDER_LOOP() ACTION_ITEM_N(e, MSG_MMU2_FILAMENT_N, []{ _mmu2_cut_filament(MenuItemBase::itemIndex); });
  END_MENU();
}

// SpoolJoin
void spool_join_status() { spooljoin.initStatus(); }

// Fail Stats Menu
void menu_mmu3_fail_stats_last_print() {
  if (ui.use_click()) return ui.go_back();
  char buffer1[LCD_WIDTH], buffer2[LCD_WIDTH];

  // had to cast the uint8_t values to uint16_t before formatting them.
  const uint16_t fail_num = MMU3::operation_statistics.fail_num;
  const uint16_t load_fail_num = MMU3::operation_statistics.load_fail_num;

  sprintf_P(buffer1, PSTR("%hu"), fail_num);
  sprintf_P(buffer2, PSTR("%hu"), load_fail_num);

  START_SCREEN();
  STATIC_ITEM(
    TERN(printJobOngoing(), MSG_MMU_CURRENT_PRINT_FAILURES, MSG_MMU_LAST_PRINT_FAILURES),
    SS_INVERT
  );
  #ifndef __AVR__
    // TODO: I couldn't make this work on AVR
    PSTRING_ITEM(MSG_MMU_FAILS, buffer1, SS_FULL);
    PSTRING_ITEM(MSG_MMU_LOAD_FAILS, buffer2, SS_FULL);
  #endif
  END_SCREEN();
}

void menu_mmu3_fail_stas_total() {
  if (ui.use_click()) return ui.go_back();
  char buffer1[LCD_WIDTH], buffer2[LCD_WIDTH], buffer3[LCD_WIDTH];

  sprintf_P(buffer1, PSTR("%hu"), MMU3::operation_statistics.fail_total_num);
  sprintf_P(buffer2, PSTR("%hu"), MMU3::operation_statistics.load_fail_total_num);
  sprintf_P(buffer3, PSTR("%hu"), mmu3.tmcFailures());

  START_SCREEN();
  STATIC_ITEM(MSG_MMU_TOTAL_FAILURES, SS_INVERT);
  #ifndef __AVR__
    // TODO: I couldn't make this work on AVR
    PSTRING_ITEM(MSG_MMU_FAILS, buffer1, SS_FULL);
    PSTRING_ITEM(MSG_MMU_LOAD_FAILS, buffer2, SS_FULL);
    PSTRING_ITEM(MSG_MMU_POWER_FAILS, buffer3, SS_FULL);
  #endif
  END_SCREEN();
}

#if ENABLED(MARLIN_DEV_MODE)
  void menu_mmu3_dev_increment_fail_stat() {
    MMU3::operation_statistics.increment_mmu_fails();
  }

  void menu_mmu3_dev_increment_load_fail_stat() {
    MMU3::operation_statistics.increment_load_fails();
  }
#endif

static void mmu3_reset_fail_stats() {
  bool result = MMU3::operation_statistics.reset_fail_stats();
  ui.go_back();
  MarlinUI::completion_feedback(result);
}

static void mmu3_reset_stats() {
  bool result = MMU3::operation_statistics.reset_stats();
  ui.go_back();
  MarlinUI::completion_feedback(result);
}

void menu_mmu3_toolchange_stat_total() {
  if (ui.use_click()) return ui.go_back();
  char buffer1[LCD_WIDTH];
  sprintf_P(buffer1, PSTR("%u"), MMU3::operation_statistics.tool_change_counter);

  char buffer2[LCD_WIDTH];
  sprintf_P(buffer2, PSTR("%lu"), MMU3::operation_statistics.tool_change_total_counter);

  START_SCREEN();
  STATIC_ITEM(MSG_MMU_MATERIAL_CHANGES, SS_INVERT);
  #ifndef __AVR__
    // TODO: I couldn't make this work on AVR
    if (printJobOngoing())
      PSTRING_ITEM(MSG_MMU_CURRENT_PRINT, buffer1, SS_FULL);
    else
      PSTRING_ITEM(MSG_MMU_LAST_PRINT, buffer1, SS_FULL);
    PSTRING_ITEM(MSG_MMU_TOTAL, buffer2, SS_FULL);
  #endif
  END_SCREEN();
}

void menu_mmu3_statistics() {
  START_MENU();
  BACK_ITEM(MSG_MMU2_MENU);
  #if ENABLED(MARLIN_DEV_MODE)
    ACTION_ITEM(MSG_MMU_DEV_INCREMENT_FAILS, menu_mmu3_dev_increment_fail_stat);
    ACTION_ITEM(MSG_MMU_DEV_INCREMENT_LOAD_FAILS, menu_mmu3_dev_increment_load_fail_stat);
  #endif

  SUBMENU(
    TERN(printJobOngoing(), MSG_MMU_CURRENT_PRINT_FAILURES, MSG_MMU_LAST_PRINT_FAILURES),
    menu_mmu3_fail_stats_last_print
  );
  SUBMENU(MSG_MMU_TOTAL_FAILURES, menu_mmu3_fail_stas_total);
  SUBMENU(MSG_MMU_MATERIAL_CHANGES, menu_mmu3_toolchange_stat_total);
  CONFIRM_ITEM(MSG_MMU_RESET_FAIL_STATS,
    MSG_BUTTON_RESET, MSG_BUTTON_CANCEL,
    mmu3_reset_fail_stats, nullptr,
    GET_TEXT_F(MSG_MMU_RESET_FAIL_STATS), (const char *)nullptr, F("?")
  );
  CONFIRM_ITEM(MSG_MMU_RESET_STATS,
    MSG_BUTTON_RESET, MSG_BUTTON_CANCEL,
    mmu3_reset_stats, nullptr,
    GET_TEXT_F(MSG_MMU_RESET_STATS), (const char *)nullptr, F("?")
  );
  END_MENU();
}

#endif // HAS_PRUSA_MMU3

//
// MMU2 Menu
//

void action_mmu2_reset() {
  #if HAS_PRUSA_MMU3
    #if PIN_EXISTS(MMU_RST)
      mmu3.reset(MMU3::MMU3::ResetForm::ResetPin);
    #else
      mmu3.reset(MMU3::MMU3::ResetForm::Software);
    #endif
  #else
    mmu2.init();
  #endif
  ui.reset_status();
}

void menu_mmu2() {
  const bool busy = printJobOngoing(); // printingIsActive();

  START_MENU();
  BACK_ITEM(MSG_MAIN_MENU);

  // MMU2/MMU3 Commands
  if (!busy && TERN1(HAS_PRUSA_MMU3, mmu3.mmu_hw_enabled)) {
    SUBMENU(MSG_MMU2_LOAD_FILAMENT, menu_mmu2_load_filament);
    SUBMENU(MSG_MMU2_LOAD_TO_NOZZLE, menu_mmu2_load_to_nozzle);
    SUBMENU(MSG_MMU2_EJECT_FILAMENT, menu_mmu2_eject_filament);
    ACTION_ITEM(MSG_MMU2_UNLOAD_FILAMENT, action_mmu2_unload_filament);
  }

  #if HAS_PRUSA_MMU3
    // MMU3 Enable/Disable
    #ifndef __AVR__
      editable.state = mmu3.mmu_hw_enabled;
      EDIT_ITEM_F(bool, F("MMU"), &mmu3.mmu_hw_enabled, []{
        if (editable.state)
          mmu3.stop();
        else
          mmu3.start();
      });
    #endif

    // SpoolJoin Enable/Disable
    EDIT_ITEM(bool, MSG_MMU_SPOOL_JOIN, &spooljoin.enabled, spool_join_status);

    // Cutter Enable/Disable
    bool cutter_enabled = mmu3.cutter_mode != 0;
    editable.state = cutter_enabled;
    EDIT_ITEM(bool, MSG_MMU_CUTTER, &cutter_enabled, []{
      menu_mmu3_cutter_set_mode((uint8_t)!editable.state);
    });
    if (!busy && MMU3::cutter_enabled() && mmu3.mmu_hw_enabled) {
      SUBMENU(MSG_MMU2_CUT_FILAMENT, menu_mmu3_cut_filament);
    }

    // Statistics
    SUBMENU(MSG_MMU_STATISTICS, menu_mmu3_statistics);
  #endif

  if (TERN1(HAS_PRUSA_MMU3, mmu3.mmu_hw_enabled)) {
    ACTION_ITEM(MSG_MMU2_RESET, action_mmu2_reset);
  }

  END_MENU();
}

//
// T* Choose Filament
//

int8_t feeder_index;
bool wait_for_mmu_menu;

inline void action_mmu2_chosen(const uint8_t index) {
  feeder_index = index;
  wait_for_mmu_menu = false;
}

void menu_mmu2_choose_filament() {
  START_MENU();
  #if LCD_HEIGHT > 2
    STATIC_ITEM(MSG_MMU2_CHOOSE_FILAMENT_HEADER, SS_DEFAULT|SS_INVERT);
  #endif
  EXTRUDER_LOOP() ACTION_ITEM_N(e, MSG_MMU2_FILAMENT_N, []{ action_mmu2_chosen(MenuItemBase::itemIndex); });
  END_MENU();
}

//
// MMU2 Filament Runout
//

void menu_mmu2_pause() {
  feeder_index = mmu3.get_current_tool();
  START_MENU();
  #if LCD_HEIGHT > 2
    STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER, SS_DEFAULT|SS_INVERT);
  #endif
  ACTION_ITEM(MSG_MMU2_RESUME,          []{ wait_for_mmu_menu = false; });
  #if HAS_PRUSA_MMU3
    ACTION_ITEM(MSG_MMU2_UNLOAD_FILAMENT, []{ mmu3.unload(); });
    ACTION_ITEM(MSG_MMU2_LOAD_FILAMENT,   []{ mmu3.load_to_feeder(feeder_index); });
    ACTION_ITEM(MSG_MMU2_LOAD_TO_NOZZLE,  []{ mmu3.load_to_nozzle(feeder_index); });
  #else
    ACTION_ITEM(MSG_MMU2_UNLOAD_FILAMENT, []{ mmu2.unload(); });
    ACTION_ITEM(MSG_MMU2_LOAD_FILAMENT,   []{ mmu2.load_to_feeder(feeder_index); });
    ACTION_ITEM(MSG_MMU2_LOAD_TO_NOZZLE,  []{ mmu2.load_to_nozzle(feeder_index); });
  #endif
  END_MENU();
}

void mmu2_M600(const bool automatic/*=false*/) {
  // Disable automatic switching if MMU3 is not enabled or spool join is disabled
  #if HAS_PRUSA_MMU3
    if (automatic && spooljoin.enabled) {
      uint8_t slot;
      slot = spooljoin.nextSlot();
      mmu3.load_to_nozzle(slot);
      return;
    }
  #else
    UNUSED(automatic);
  #endif
  ui.defer_status_screen();
  ui.goto_screen(menu_mmu2_pause);
  wait_for_mmu_menu = true;
  while (wait_for_mmu_menu) idle();
}

uint8_t mmu2_choose_filament() {
  ui.defer_status_screen();
  ui.goto_screen(menu_mmu2_choose_filament);
  wait_for_mmu_menu = true;
  while (wait_for_mmu_menu) idle();
  ui.return_to_status();
  return feeder_index;
}

#endif // MMU_MENUS
