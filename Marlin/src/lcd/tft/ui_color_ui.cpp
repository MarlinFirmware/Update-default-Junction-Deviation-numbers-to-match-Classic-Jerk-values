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

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(TFT_COLOR_UI)

#include "ui_common.h"

#include "../marlinui.h"
#include "../menu/menu.h"
#include "../../libs/numtostr.h"

#include "../../sd/cardreader.h"
#include "../../module/temperature.h"
#include "../../module/printcounter.h"
#include "../../module/planner.h"
#include "../../module/motion.h"

#if DISABLED(LCD_PROGRESS_BAR) && ALL(FILAMENT_LCD_DISPLAY, HAS_MEDIA)
  #include "../../feature/filwidth.h"
  #include "../../gcode/parser.h"
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL)
  #include "../../feature/bedlevel/bedlevel.h"
#endif

void MarlinUI::tft_idle() {
  #if ENABLED(TOUCH_SCREEN)
    if (TERN0(HAS_DISPLAY_SLEEP, lcd_sleep_task())) return;
    if (draw_menu_navigation) {
      add_control(NAVIGATION_PAGE_UP_X, NAVIGATION_PAGE_UP_Y, PAGE_UP, imgPageUp, encoderTopLine > 0);
      add_control(NAVIGATION_PAGE_DOWN_X, NAVIGATION_PAGE_DOWN_Y, PAGE_DOWN, imgPageDown, encoderTopLine + LCD_HEIGHT < screen_items);
      add_control(NAVIGATION_BACK_X, NAVIGATION_BACK_Y, BACK, imgBack);
      draw_menu_navigation = false;
    }
  #endif

  tft.queue.async();

  TERN_(TOUCH_SCREEN, if (tft.queue.is_empty()) touch.idle()); // Touch driver is not DMA-aware, so only check for touch controls after screen drawing is completed
}

#if ENABLED(SHOW_BOOTSCREEN)

  void MarlinUI::show_bootscreen() {
    tft.queue.reset();

    tft.canvas(0, 0, TFT_WIDTH, TFT_HEIGHT);
    tft.set_background(COLOR_BACKGROUND);
    tft.add_image(BOOTSCREEN_LOGO_X, BOOTSCREEN_LOGO_Y, imgBootScreen);
    #ifdef WEBSITE_URL
      tft_string.set(WEBSITE_URL);
      tft.add_text(tft_string.center(TFT_WIDTH), BOOTSCREEN_SITE_URL_Y, COLOR_WEBSITE_URL, tft_string);
    #endif

    tft.queue.sync();
  }

  void MarlinUI::bootscreen_completion(const millis_t sofar) {
    if ((BOOTSCREEN_TIMEOUT) > sofar) safe_delay((BOOTSCREEN_TIMEOUT) - sofar);
    clear_for_drawing();
  }
#endif

void MarlinUI::draw_kill_screen() {
  tft.queue.reset();
  tft.fill(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_KILL_SCREEN_BG);

  text_line(KILL_SCREEN_STATUS_Y, COLOR_KILL_SCREEN_BG);
  tft_string.set(status_message);
  tft_string.trim();
  tft.add_text(tft_string.center(TFT_WIDTH), 0, COLOR_MENU_TEXT, tft_string);

  text_line(KILL_SCREEN_HALTED_Y, COLOR_KILL_SCREEN_BG);
  tft_string.set(GET_TEXT(MSG_HALTED));
  tft_string.trim();
  tft.add_text(tft_string.center(TFT_WIDTH), 0, COLOR_MENU_TEXT, tft_string);

  text_line(KILL_SCREEN_RESET_Y, COLOR_KILL_SCREEN_BG);
  tft_string.set(GET_TEXT(MSG_PLEASE_RESET));
  tft_string.trim();
  tft.add_text(tft_string.center(TFT_WIDTH), 0, COLOR_MENU_TEXT, tft_string);

  tft.queue.sync();
}

void draw_heater_status(uint16_t x, uint16_t y, const int8_t heater) {
  MarlinImage image = imgHotEnd;
  celsius_t currentTemperature, targetTemperature;

  if (heater >= 0) { // HotEnd
    #if HAS_EXTRUDERS
      currentTemperature = thermalManager.wholeDegHotend(heater);
      targetTemperature = thermalManager.degTargetHotend(heater);
    #else
      return;
    #endif
  }
  #if HAS_HEATED_BED
    else if (heater == H_BED) {
      currentTemperature = thermalManager.wholeDegBed();
      targetTemperature = thermalManager.degTargetBed();
    }
  #endif
  #if HAS_TEMP_CHAMBER
    else if (heater == H_CHAMBER) {
      currentTemperature = thermalManager.wholeDegChamber();
      #if HAS_HEATED_CHAMBER
        targetTemperature = thermalManager.degTargetChamber();
      #else
        targetTemperature = ABSOLUTE_ZERO;
      #endif
    }
  #endif
  #if HAS_TEMP_COOLER
    else if (heater == H_COOLER) {
      currentTemperature = thermalManager.wholeDegCooler();
      targetTemperature = TERN(HAS_COOLER, thermalManager.degTargetCooler(), ABSOLUTE_ZERO);
    }
  #endif
  else return;

  TERN_(TOUCH_SCREEN, if (targetTemperature >= 0) touch.add_control(HEATER, x, y, TEMP_FAN_CONTROL_W, TEMP_FAN_CONTROL_H, heater));
  tft.canvas(x, y, TEMP_FAN_CONTROL_W, TEMP_FAN_CONTROL_H);
  tft.set_background(COLOR_BACKGROUND);

  uint16_t color = currentTemperature < 0 ? COLOR_INACTIVE : COLOR_COLD;

  if (heater >= 0) { // HotEnd
    if (currentTemperature >= 50) color = COLOR_HOTEND;
  }
  #if HAS_HEATED_BED
    else if (heater == H_BED) {
      if (currentTemperature >= 50) color = COLOR_HEATED_BED;
      image = targetTemperature > 0 ? imgBedHeated : imgBed;
    }
  #endif
  #if HAS_TEMP_CHAMBER
    else if (heater == H_CHAMBER) {
      if (currentTemperature >= 50) color = COLOR_CHAMBER;
      image = targetTemperature > 0 ? imgChamberHeated : imgChamber;
    }
  #endif
  #if HAS_TEMP_COOLER
    else if (heater == H_COOLER) {
      if (currentTemperature <= 26) color = COLOR_COLD;
      if (currentTemperature > 26) color = COLOR_RED;
      image = targetTemperature > 26 ? imgCoolerHot : imgCooler;
    }
  #endif

  tft.add_image(TEMP_ICON_X, TEMP_ICON_Y, image, color);

  tft_string.set(i16tostr3rj(currentTemperature));
  tft_string.add(LCD_STR_DEGREE);
  tft_string.trim();
  tft.add_text(TEMP_CURRENT_TEXT_X, TEMP_CURRENT_TEXT_Y, color, tft_string);

  if (targetTemperature >= 0) {
    tft_string.set(i16tostr3rj(targetTemperature));
    tft_string.add(LCD_STR_DEGREE);
    tft_string.trim();
    tft.add_text(TEMP_TARGET_TEXT_X, TEMP_TARGET_TEXT_Y, color, tft_string);
  }
}

void draw_fan_status(uint16_t x, uint16_t y, const bool blink) {
  TERN_(TOUCH_SCREEN, touch.add_control(FAN, x, y, TEMP_FAN_CONTROL_W, TEMP_FAN_CONTROL_H));
  tft.canvas(x, y, TEMP_FAN_CONTROL_W, TEMP_FAN_CONTROL_H);
  tft.set_background(COLOR_BACKGROUND);

  uint8_t fanSpeed = thermalManager.fan_speed[0];
  MarlinImage image;

  if (fanSpeed >= 127)
    image = blink ? imgFanFast1 : imgFanFast0;
  else if (fanSpeed > 0)
    image = blink ? imgFanSlow1 : imgFanSlow0;
  else
    image = imgFanIdle;

  tft.add_image(FAN_ICON_X, FAN_ICON_Y, image, COLOR_FAN);

  tft_string.set(ui8tostr4pctrj(thermalManager.fan_speed[0]));
  tft_string.trim();
  tft.add_text(FAN_TEXT_X, FAN_TEXT_Y, COLOR_FAN, tft_string);
}

void MarlinUI::draw_status_screen() {
  const bool blink = get_blink();
  TERN_(TOUCH_SCREEN, touch.clear());

  // Statuses of heaters and fans
  for (uint16_t i = 0; i < _MIN(ITEMS_COUNT, MAX_ITEMS); i++) {
    switch (i) {
      #if HAS_EXTRUDERS
        case ITEM_E0: draw_heater_status(ITEM_X(i), ITEM_Y, H_E0); break;
      #endif
      #if HAS_MULTI_HOTEND
        case ITEM_E1: draw_heater_status(ITEM_X(i), ITEM_Y, H_E1); break;
      #endif
      #if HOTENDS > 2
        case ITEM_E2: draw_heater_status(ITEM_X(i), ITEM_Y, H_E2); break;
      #endif
      #if HAS_HEATED_BED
        case ITEM_BED: draw_heater_status(ITEM_X(i), ITEM_Y, H_BED); break;
      #endif
      #if HAS_TEMP_CHAMBER
        case ITEM_CHAMBER: draw_heater_status(ITEM_X(i), ITEM_Y, H_CHAMBER); break;
      #endif
      #if HAS_TEMP_COOLER
        case ITEM_COOLER: draw_heater_status(ITEM_X(i), ITEM_Y, H_COOLER); break;
      #endif
      #if HAS_FAN
        case ITEM_FAN: draw_fan_status(ITEM_X(i), ITEM_Y, blink); break;
      #endif
    }
  }

  // Coordinates
  #if ENABLED(MOVE_AXIS_SCREEN)
    TERN_(TOUCH_SCREEN, touch.add_control(MENU_SCREEN, COORDINATES_X, COORDINATES_Y, COORDINATES_W, COORDINATES_H, (intptr_t) ui.move_axis_screen));
  #endif

  tft.canvas(COORDINATES_X, COORDINATES_Y, COORDINATES_W, COORDINATES_H);
  tft.set_background(COLOR_BACKGROUND);
  tft.add_rectangle(0, 0, COORDINATES_W, COORDINATES_H, COLOR_AXIS_HOMED);

  #if HAS_X_AXIS && defined(X_MARK_X) && defined(X_MARK_Y) && defined(X_VALUE_X) && defined(X_VALUE_Y)
    tft.add_text(X_MARK_X, X_MARK_Y, COLOR_AXIS_HOMED , "X");
    const bool nhx = axis_should_home(X_AXIS);
    tft_string.set(blink && nhx ? "?" : ftostr4sign(LOGICAL_X_POSITION(current_position.x)));
    tft.add_text(X_VALUE_X, X_VALUE_Y, nhx ? COLOR_AXIS_NOT_HOMED : COLOR_AXIS_HOMED, tft_string);
  #endif

  #if HAS_Y_AXIS && defined(Y_MARK_X) && defined(Y_MARK_Y) && defined(Y_VALUE_X) && defined(Y_VALUE_Y)
    tft.add_text(Y_MARK_X, Y_MARK_Y, COLOR_AXIS_HOMED , "Y");
    const bool nhy = axis_should_home(Y_AXIS);
    tft_string.set(blink && nhy ? "?" : ftostr4sign(LOGICAL_Y_POSITION(current_position.y)));
    tft.add_text(Y_VALUE_X, Y_VALUE_Y, nhy ? COLOR_AXIS_NOT_HOMED : COLOR_AXIS_HOMED, tft_string);
  #endif

  #if HAS_Z_AXIS && defined(Z_MARK_X) && defined(Z_MARK_Y) && defined(Z_VALUE_X) && defined(Z_VALUE_Y) && defined(Z_VALUE_OFFSET)
    tft.add_text(Z_MARK_X, Z_MARK_Y, COLOR_AXIS_HOMED , "Z");
    uint16_t offset = Z_VALUE_OFFSET;
    const bool nhz = axis_should_home(Z_AXIS);
    if (blink && nhz)
      tft_string.set('?');
    else {
      const float z = LOGICAL_Z_POSITION(current_position.z);
      tft_string.set(ftostr52sp((int16_t)z));
      tft_string.rtrim();
      offset += tft_string.width();

      tft_string.set(ftostr52sp(z));
      offset -= tft_string.width();
    }
    tft.add_text(Z_VALUE_X - offset, Z_VALUE_Y, nhz ? COLOR_AXIS_NOT_HOMED : COLOR_AXIS_HOMED, tft_string);
  #endif

  #if ENABLED(LCD_SHOW_E_TOTAL) && defined(E_MARK_X) && defined(E_MARK_Y) && defined(E_VALUE_X) && defined(E_VALUE_Y)
    tft.add_text(E_MARK_X, E_MARK_Y, COLOR_AXIS_HOMED, "E");
    if (printingIsActive()) {
      const uint8_t escale = e_move_accumulator >= 10000.0f ? 10 : 1; // After 10m switch to cm to fit into 4 digits output of ftostr4sign()
      tft_string.set(ftostr4sign(e_move_accumulator / escale));
      const uint16_t e_value_x = E_VALUE_X;
      tft_string.add(escale == 10 ? " cm" : " mm");
      tft.add_text(e_value_x, E_VALUE_Y, COLOR_AXIS_HOMED, tft_string);
    }
    else {
      tft_string.set("--");
      tft.add_text(E_VALUE_X, E_VALUE_Y, COLOR_AXIS_HOMED, tft_string);
    }
  #endif

  // Feed rate
  tft.canvas(FEEDRATE_X, FEEDRATE_Y, FEEDRATE_W, FEEDRATE_H);
  tft.set_background(COLOR_BACKGROUND);
  uint16_t color = feedrate_percentage == 100 ? COLOR_RATE_100 : COLOR_RATE_ALTERED;
  tft.add_image(0, 0, imgFeedRate, color);
  tft_string.set(i16tostr3rj(feedrate_percentage));
  tft_string.add('%');
  tft.add_text(36, tft_string.vcenter(30), color, tft_string);
  TERN_(TOUCH_SCREEN, touch.add_control(FEEDRATE, FEEDRATE_X, FEEDRATE_Y, FEEDRATE_W, FEEDRATE_H));

  #if HAS_EXTRUDERS
    // Flow rate
    tft.canvas(FLOWRATE_X, FLOWRATE_Y, FLOWRATE_W, FLOWRATE_H);
    tft.set_background(COLOR_BACKGROUND);
    color = planner.flow_percentage[0] == 100 ? COLOR_RATE_100 : COLOR_RATE_ALTERED;
    tft.add_image(FLOWRATE_ICON_X, FLOWRATE_ICON_X, imgFlowRate, color);
    tft_string.set(i16tostr3rj(planner.flow_percentage[active_extruder]));
    tft_string.add('%');
    tft.add_text(FLOWRATE_TEXT_X, FLOWRATE_TEXT_Y, color, tft_string);
    TERN_(TOUCH_SCREEN, touch.add_control(FLOWRATE, FLOWRATE_X, FLOWRATE_Y, FLOWRATE_W, FLOWRATE_H, active_extruder));
  #endif

  #if ENABLED(TOUCH_SCREEN)
    add_control(MENU_ICON_X, MENU_ICON_Y, menu_main, imgSettings);
    #if HAS_MEDIA
      const bool cm = card.isMounted(), pa = printingIsActive();
      if (cm && pa)
        add_control(SDCARD_ICON_X, SDCARD_ICON_Y, STOP, imgCancel, true, COLOR_CONTROL_CANCEL);
      else
        add_control(SDCARD_ICON_X, SDCARD_ICON_Y, menu_media, imgSD, cm && !pa, COLOR_CONTROL_ENABLED, COLOR_CONTROL_DISABLED);
    #endif
  #endif

  #if ANY(SHOW_ELAPSED_TIME, SHOW_REMAINING_TIME)
    char buffer[22];
    duration_t elapsed = print_job_timer.duration();
  #endif

  const progress_t progress = TERN(HAS_PRINT_PROGRESS_PERMYRIAD, get_progress_permyriad, get_progress_percent)();

  #if ENABLED(SHOW_ELAPSED_TIME)
    elapsed.toDigital(buffer);
    tft.canvas(ELAPSED_TIME_X, ELAPSED_TIME_Y, ELAPSED_TIME_W, ELAPSED_TIME_H);
    tft.set_background(COLOR_BACKGROUND);
    tft_string.set(buffer);
    #if defined(ELAPSED_TIME_IMAGE_X) && defined(ELAPSED_TIME_IMAGE_Y)
      tft.add_image(ELAPSED_TIME_IMAGE_X, ELAPSED_TIME_IMAGE_Y, imgTimeElapsed, COLOR_PRINT_TIME);
    #endif
    tft.add_text(ELAPSED_TIME_TEXT_X, ELAPSED_TIME_TEXT_Y, COLOR_PRINT_TIME, tft_string);
  #endif

  #if ENABLED(SHOW_REMAINING_TIME)
    // Get the estimate, first from M73
    uint32_t estimate_remaining = (0
      #if ALL(SET_PROGRESS_MANUALLY, SET_REMAINING_TIME)
        + get_remaining_time()
      #endif
    );
    // If no M73 estimate is available but we have progress data, calculate time remaining assuming time elapsed is linear with progress
    if (!estimate_remaining && progress > 0)
      estimate_remaining = elapsed.value * (100 * (PROGRESS_SCALE) - progress) / progress;

    // Generate estimate string
    if (!estimate_remaining)
      tft_string.set("-");
    else {
      duration_t estimation = estimate_remaining;
      estimation.toString(buffer);
      tft_string.set(buffer);
    }

    tft.canvas(REMAINING_TIME_X, REMAINING_TIME_Y, REMAINING_TIME_W, REMAINING_TIME_H);
    tft.set_background(COLOR_BACKGROUND);
    tft_string.set(buffer);
    color = printingIsActive() ? COLOR_PRINT_TIME : COLOR_INACTIVE;
    #if defined(REMAINING_TIME_IMAGE_X) && defined(REMAINING_TIME_IMAGE_Y)
      tft.add_image(REMAINING_TIME_IMAGE_X, REMAINING_TIME_IMAGE_Y, imgTimeRemaining, color);
    #endif
    tft.add_text(REMAINING_TIME_TEXT_X, REMAINING_TIME_TEXT_Y, color, tft_string);
  #endif

  // Progress bar
  // TODO: print percentage text for SHOW_PROGRESS_PERCENT
  tft.canvas(PROGRESS_BAR_X, PROGRESS_BAR_Y, PROGRESS_BAR_W, PROGRESS_BAR_H);
  tft.set_background(COLOR_PROGRESS_BG);
  tft.add_rectangle(0, 0, PROGRESS_BAR_W, PROGRESS_BAR_H, COLOR_PROGRESS_FRAME);
  if (progress)
    tft.add_bar(1, 1, ((PROGRESS_BAR_W - 2) * progress / (PROGRESS_SCALE)) / 100, 7, COLOR_PROGRESS_BAR);

  // Status message
  tft.canvas(STATUS_MESSAGE_X, STATUS_MESSAGE_Y, STATUS_MESSAGE_W, STATUS_MESSAGE_H);
  tft.set_background(COLOR_BACKGROUND);
  tft_string.set(status_message);
  tft_string.trim();
  tft.add_text(STATUS_MESSAGE_TEXT_X, STATUS_MESSAGE_TEXT_Y, COLOR_STATUS_MESSAGE, tft_string);
}

// Low-level draw_edit_screen can be used to draw an edit screen from anyplace
void MenuEditItemBase::draw_edit_screen(FSTR_P const ftpl, const char * const value/*=nullptr*/) {
  ui.encoder_direction_normal();
  TERN_(TOUCH_SCREEN, touch.clear());

  uint16_t line = 1;

  menu_line(line++);
  tft_string.set(ftpl, itemIndex, itemStringC, itemStringF);
  tft_string.trim();
  tft.add_text(tft_string.center(TFT_WIDTH), MENU_TEXT_Y, COLOR_MENU_TEXT, tft_string);

  TERN_(AUTO_BED_LEVELING_UBL, if (ui.external_control) line++);  // ftostr52() will overwrite *value so *value has to be displayed first

  menu_line(line);
  tft_string.set(value);
  tft_string.trim();
  tft.add_text(tft_string.center(TFT_WIDTH), MENU_TEXT_Y, COLOR_MENU_VALUE, tft_string);

  #if ENABLED(AUTO_BED_LEVELING_UBL)
    if (ui.external_control) {
      menu_line(line - 1);

      tft_string.set(X_LBL);
      tft.add_text(UBL_X_LABEL_X, MENU_TEXT_Y, COLOR_MENU_TEXT, tft_string);
      tft_string.set(ftostr52(LOGICAL_X_POSITION(current_position.x)));
      tft_string.trim();
      tft.add_text(UBL_X_TEXT_X, MENU_TEXT_Y, COLOR_MENU_VALUE, tft_string);

      tft_string.set(Y_LBL);
      tft.add_text(UBL_Y_LABEL_X, MENU_TEXT_Y, COLOR_MENU_TEXT, tft_string);
      tft_string.set(ftostr52(LOGICAL_X_POSITION(current_position.y)));
      tft_string.trim();
      tft.add_text(UBL_Y_TEXT_X, MENU_TEXT_Y, COLOR_MENU_VALUE, tft_string);
    }
  #endif

  if (ui.can_show_slider() && maxEditValue > 0) {
    tft.canvas((TFT_WIDTH - SLIDER_W) / 2, SLIDER_Y, SLIDER_W, 16);
    tft.set_background(COLOR_BACKGROUND);

    int16_t position = (SLIDER_W - 2) * ui.encoderPosition / maxEditValue;
    tft.add_bar(0, 7, 1, 2, ui.encoderPosition == 0 ? COLOR_SLIDER_INACTIVE : COLOR_SLIDER);
    tft.add_bar(1, 6, position, 4, COLOR_SLIDER);
    tft.add_bar(position + 1, 6, SLIDER_W - 2 - position, 4, COLOR_SLIDER_INACTIVE);
    tft.add_bar(SLIDER_W - 1, 7, 1, 2, int32_t(ui.encoderPosition) == maxEditValue ? COLOR_SLIDER : COLOR_SLIDER_INACTIVE);

    #if ENABLED(TOUCH_SCREEN)
      tft.add_image((SLIDER_W - 8) * ui.encoderPosition / maxEditValue, 0, imgSlider, COLOR_SLIDER);
      touch.add_control(SLIDER, (TFT_WIDTH - SLIDER_W) / 2, SLIDER_Y - 8, SLIDER_W, 32, maxEditValue);
    #endif
  }

  tft.draw_edit_screen_buttons();
}

void TFT::draw_edit_screen_buttons() {
  #if ENABLED(TOUCH_SCREEN)
    add_control(BUTTON_DECREASE_X, BUTTON_DECREASE_Y, DECREASE, imgDecrease);
    add_control(BUTTON_INCREASE_X, BUTTON_INCREASE_Y, INCREASE, imgIncrease);
    add_control(BUTTON_CLICK_X, BUTTON_CLICK_Y, CLICK, imgConfirm);
  #endif
}

// The Select Screen presents a prompt and two "buttons"
void MenuItem_confirm::draw_select_screen(FSTR_P const yes, FSTR_P const no, const bool yesno, FSTR_P const fpre, const char * const string/*=nullptr*/, FSTR_P const fsuf/*=nullptr*/) {
  uint16_t line = 1;

  if (!string) line++;

  menu_line(line++);
  tft_string.set(fpre);
  tft_string.trim();
  tft.add_text(tft_string.center(TFT_WIDTH), MENU_TEXT_Y, COLOR_MENU_TEXT, tft_string);

  if (string) {
    menu_line(line++);
    tft_string.set(string);
    tft_string.trim();
    tft.add_text(tft_string.center(TFT_WIDTH), MENU_TEXT_Y, COLOR_MENU_TEXT, tft_string);
  }

  if (fsuf) {
    menu_line(line);
    tft_string.set(fsuf);
    tft_string.trim();
    tft.add_text(tft_string.center(TFT_WIDTH), MENU_TEXT_Y, COLOR_MENU_TEXT, tft_string);
  }
  #if ENABLED(TOUCH_SCREEN)
    if (no)  add_control(BUTTON_CANCEL_X, BUTTON_CANCEL_Y, CANCEL,  imgCancel,  true, yesno ? HALF(COLOR_CONTROL_CANCEL) : COLOR_CONTROL_CANCEL);
    if (yes) add_control(BUTTON_CONFIRM_X, BUTTON_CONFIRM_Y, CONFIRM, imgConfirm, true, yesno ? COLOR_CONTROL_CONFIRM : HALF(COLOR_CONTROL_CONFIRM));
  #else
    // Even without touch screen "no" and "yes" buttons are still need to be displayed
    if (no)  add_control(BUTTON_CANCEL_X, BUTTON_CANCEL_Y, NONE,  imgCancel,  true, yesno ? HALF(COLOR_CONTROL_CANCEL) : COLOR_CONTROL_CANCEL);
    if (yes) add_control(BUTTON_CONFIRM_X, BUTTON_CONFIRM_Y, NONE, imgConfirm, true, yesno ? COLOR_CONTROL_CONFIRM : HALF(COLOR_CONTROL_CONFIRM));
  #endif
}

#if ENABLED(ADVANCED_PAUSE_FEATURE)

  void MarlinUI::draw_hotend_status(const uint8_t row, const uint8_t extruder) {
    #if ENABLED(TOUCH_SCREEN)
      touch.clear();
      draw_menu_navigation = false;
      touch.add_control(RESUME_CONTINUE, 0, 0, TFT_WIDTH, TFT_HEIGHT);
    #endif

    menu_line(row);
    tft_string.set(GET_TEXT(MSG_FILAMENT_CHANGE_NOZZLE));
    tft_string.add('E');
    tft_string.add((char)('1' + extruder));
    tft_string.add(' ');
    tft_string.add(i16tostr3rj(thermalManager.wholeDegHotend(extruder)));
    tft_string.add(LCD_STR_DEGREE);
    tft_string.add(F(" / "));
    tft_string.add(i16tostr3rj(thermalManager.degTargetHotend(extruder)));
    tft_string.add(LCD_STR_DEGREE);
    tft_string.trim();
    tft.add_text(tft_string.center(TFT_WIDTH), MENU_TEXT_Y, COLOR_MENU_TEXT, tft_string);
  }

#endif // ADVANCED_PAUSE_FEATURE

#if ENABLED(AUTO_BED_LEVELING_UBL)

  void MarlinUI::ubl_plot(const uint8_t x_plot, const uint8_t y_plot) {

    tft.canvas(UBL_GRID_X, UBL_GRID_Y, UBL_GRID_W, UBL_GRID_H);
    tft.set_background(COLOR_BACKGROUND);
    tft.add_rectangle(0, 0, UBL_GRID_W, UBL_GRID_H, COLOR_WHITE);

    for (uint16_t x = 0; x < (GRID_MAX_POINTS_X); x++)
      for (uint16_t y = 0; y < (GRID_MAX_POINTS_Y); y++)
        if (position_is_reachable({ bedlevel.get_mesh_x(x), bedlevel.get_mesh_y(y) }))
          tft.add_bar(1 + (x * 2 + 1) * (UBL_GRID_W - 4) / (GRID_MAX_POINTS_X) / 2, UBL_GRID_H - 3 - ((y * 2 + 1) * (UBL_GRID_H - 4) / (GRID_MAX_POINTS_Y) / 2), 2, 2, COLOR_UBL);

    tft.add_rectangle((x_plot * 2 + 1) * (UBL_GRID_W - 4) / (GRID_MAX_POINTS_X) / 2 - 1, UBL_GRID_H - 5 - ((y_plot * 2 + 1) * (UBL_GRID_H - 4) / (GRID_MAX_POINTS_Y) / 2), 6, 6, COLOR_UBL);

    const xy_pos_t pos = { bedlevel.get_mesh_x(x_plot), bedlevel.get_mesh_y(y_plot) },
                   lpos = pos.asLogical();

    tft.canvas(UBL_COORDINATES_X, UBL_X_COORDINATE_Y, UBL_COORDINATES_W, MENU_ITEM_HEIGHT);
    tft.set_background(COLOR_BACKGROUND);
    tft_string.set(X_LBL);
    tft.add_text(0, MENU_TEXT_Y, COLOR_MENU_TEXT, tft_string);
    tft_string.set(ftostr52(lpos.x));
    tft_string.trim();
    tft.add_text(UBL_COORDINATES_W - tft_string.width(), MENU_TEXT_Y, COLOR_MENU_VALUE, tft_string);

    tft.canvas(UBL_COORDINATES_X, UBL_Y_COORDINATE_Y, UBL_COORDINATES_W, MENU_ITEM_HEIGHT);
    tft.set_background(COLOR_BACKGROUND);
    tft_string.set(Y_LBL);
    tft.add_text(0, MENU_TEXT_Y, COLOR_MENU_TEXT, tft_string);
    tft_string.set(ftostr52(lpos.y));
    tft_string.trim();
    tft.add_text(UBL_COORDINATES_W - tft_string.width(), MENU_TEXT_Y, COLOR_MENU_VALUE, tft_string);

    tft.canvas(UBL_COORDINATES_X, UBL_Z_COORDINATE_Y, UBL_COORDINATES_W, MENU_ITEM_HEIGHT);
    tft.set_background(COLOR_BACKGROUND);
    tft_string.set(Z_LBL);
    tft.add_text(0, MENU_TEXT_Y, COLOR_MENU_TEXT, tft_string);
    tft_string.set(isnan(bedlevel.z_values[x_plot][y_plot]) ? "-----" : ftostr43sign(bedlevel.z_values[x_plot][y_plot]));
    tft_string.trim();
    tft.add_text(UBL_COORDINATES_W - tft_string.width(), MENU_TEXT_Y, COLOR_MENU_VALUE, tft_string);

    #define UBL_TEXT_AREA_W (2 * UBL_CONTROL_OFFSET + 32)

    tft.canvas(UBL_GRID_X + (UBL_GRID_W - UBL_TEXT_AREA_W) / 2, UBL_GRID_Y + UBL_GRID_H + UBL_CONTROL_OFFSET + 16 - MENU_ITEM_HEIGHT / 2, UBL_TEXT_AREA_W, MENU_ITEM_HEIGHT);
    tft.set_background(COLOR_BACKGROUND);
    tft_string.set(ui8tostr3rj(x_plot));
    tft_string.trim();
    tft.add_text(tft_string.center(UBL_TEXT_AREA_W), MENU_TEXT_Y, COLOR_MENU_VALUE, tft_string);

    tft.canvas(UBL_GRID_X + UBL_GRID_W + UBL_CONTROL_OFFSET + 16 - UBL_TEXT_AREA_W / 2, UBL_GRID_Y + (UBL_GRID_H - MENU_ITEM_HEIGHT) / 2, UBL_TEXT_AREA_W, MENU_ITEM_HEIGHT);
    tft.set_background(COLOR_BACKGROUND);
    tft_string.set(ui8tostr3rj(y_plot));
    tft_string.trim();
    tft.add_text(tft_string.center(UBL_TEXT_AREA_W), MENU_TEXT_Y, COLOR_MENU_VALUE, tft_string);

    #if ENABLED(TOUCH_SCREEN)
      touch.clear();
      draw_menu_navigation = false;
      add_control(UBL_GRID_X + UBL_GRID_W + UBL_CONTROL_OFFSET,      UBL_GRID_Y + UBL_CONTROL_OFFSET,                   UBL,  (ENCODER_STEPS_PER_MENU_ITEM) * (GRID_MAX_POINTS_X), imgUp);
      add_control(UBL_GRID_X + UBL_GRID_W + UBL_CONTROL_OFFSET,      UBL_GRID_Y + UBL_GRID_H - UBL_CONTROL_OFFSET - 32, UBL, -(ENCODER_STEPS_PER_MENU_ITEM) * (GRID_MAX_POINTS_X), imgDown);
      add_control(UBL_GRID_X + UBL_CONTROL_OFFSET,                   UBL_GRID_Y + UBL_GRID_H + UBL_CONTROL_OFFSET,      UBL, -(ENCODER_STEPS_PER_MENU_ITEM), imgLeft);
      add_control(UBL_GRID_X + UBL_GRID_W - UBL_CONTROL_OFFSET - 32, UBL_GRID_Y + UBL_GRID_H + UBL_CONTROL_OFFSET,      UBL,   ENCODER_STEPS_PER_MENU_ITEM, imgRight);

      add_control(UBL_ACTION_X, UBL_ACTION_Y, CLICK, imgLeveling);
      add_control(NAVIGATION_BACK_X, NAVIGATION_BACK_Y, BACK, imgBack);
    #endif
  }
#endif // AUTO_BED_LEVELING_UBL

#endif // TFT_COLOR_UI
