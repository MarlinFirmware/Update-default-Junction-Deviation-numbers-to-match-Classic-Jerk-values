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

#include "../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_MENU

#include "menu.h"
#include "../../module/planner.h"
#include "../../module/motion.h"
#include "../../module/printcounter.h"
#include "../../module/temperature.h"
#include "../../gcode/queue.h"

#if HAS_SOUND
  #include "../../libs/buzzer.h"
#endif

#if ENABLED(BABYSTEP_ZPROBE_OFFSET)
  #include "../../module/probe.h"
#endif

#if HAS_LEVELING
  #include "../../feature/bedlevel/bedlevel.h"
#endif

////////////////////////////////////////////
///////////// Global Variables /////////////
////////////////////////////////////////////

#if HAS_LEVELING && ANY(LCD_BED_TRAMMING, PROBE_OFFSET_WIZARD, X_AXIS_TWIST_COMPENSATION)
  bool menu_leveling_was_active; // = false
#endif
#if ANY(PROBE_MANUALLY, MESH_BED_LEVELING, X_AXIS_TWIST_COMPENSATION)
  uint8_t manual_probe_index; // = 0
#endif

// Menu Navigation
int8_t encoderTopLine, encoderLine, screen_items;

typedef struct {
  screenFunc_t menu_function;     // The screen's function
  uint32_t encoder_position;      // The position of the encoder
  int8_t top_line, items;         // The amount of scroll, and the number of items
  #if HAS_SCREEN_TIMEOUT
    bool sticky;                  // The screen is sticky
  #endif
} menuPosition;
menuPosition screen_history[6];
uint8_t screen_history_depth = 0;

int8_t MenuItemBase::itemIndex;         // Index number for draw and action
FSTR_P MenuItemBase::itemStringF;       // A string for substitution
const char *MenuItemBase::itemStringC;
chimera_t editable;                     // Value Editing

// Menu Edit Items
FSTR_P       MenuEditItemBase::editLabel;
void*        MenuEditItemBase::editValue;
int32_t      MenuEditItemBase::minEditValue,
             MenuEditItemBase::maxEditValue;
screenFunc_t MenuEditItemBase::callbackFunc;
bool         MenuEditItemBase::liveEdit;

////////////////////////////////////////////
//////// Menu Navigation & History /////////
////////////////////////////////////////////

void MarlinUI::return_to_status() { goto_screen(status_screen); }

void MarlinUI::push_current_screen() {
  if (screen_history_depth < COUNT(screen_history))
    screen_history[screen_history_depth++] = { currentScreen, encoderPosition, encoderTopLine, screen_items OPTARG(HAS_SCREEN_TIMEOUT, screen_is_sticky()) };
}

void MarlinUI::_goto_previous_screen(TERN_(TURBO_BACK_MENU_ITEM, const bool is_back/*=false*/)) {
  IF_DISABLED(TURBO_BACK_MENU_ITEM, constexpr bool is_back = false);
  TERN_(HAS_TOUCH_BUTTONS, on_edit_screen = false);
  if (screen_history_depth > 0) {
    menuPosition &sh = screen_history[--screen_history_depth];
    goto_screen(sh.menu_function,
      is_back ? 0 : sh.encoder_position,
      is_back ? 0 : sh.top_line,
      sh.items
    );
    defer_status_screen(TERN_(HAS_SCREEN_TIMEOUT, sh.sticky));
  }
  else
    return_to_status();
}

////////////////////////////////////////////
/////////// Menu Editing Actions ///////////
////////////////////////////////////////////

// All Edit Screens run the same way, but `draw_edit_screen` is implementation-specific
void MenuEditItemBase::edit_screen(strfunc_t strfunc, loadfunc_t loadfunc) {
  // Reset repeat_delay for Touch Buttons
  TERN_(HAS_TOUCH_BUTTONS, ui.repeat_delay = BUTTON_DELAY_EDIT);
  // Constrain ui.encoderPosition to 0 ... maxEditValue (calculated in encoder steps)
  ui.encoderPosition = constrain(int32_t(ui.encoderPosition), 0, maxEditValue);
  // If drawing is flagged then redraw the (whole) edit screen
  if (ui.should_draw())
    draw_edit_screen(strfunc(ui.encoderPosition + minEditValue));
  // If there was a click or "live editing" and encoder moved...
  if (ui.lcd_clicked || (liveEdit && ui.should_draw())) {
    // Pass the editValue pointer to the loadfunc along with the encoder plus min
    if (editValue) loadfunc(editValue, ui.encoderPosition + minEditValue);
    // If a callbackFunc was set, call it for click or always for "live editing"
    if (callbackFunc && (liveEdit || ui.lcd_clicked)) (*callbackFunc)();
    // Use up the click to finish editing and go to the previous screen
    if (ui.use_click()) ui.goto_previous_screen();
  }
}

// Going to an edit screen sets up some persistent values first
void MenuEditItemBase::goto_edit_screen(
  FSTR_P const el,        // Edit label
  void * const ev,        // Edit value pointer
  const int32_t minv,     // Encoder minimum
  const int32_t maxv,     // Encoder maximum
  const uint32_t ep,      // Initial encoder value
  const screenFunc_t cs,  // MenuItem_type::draw_edit_screen => MenuEditItemBase::edit()
  const screenFunc_t cb,  // Callback after edit
  const bool le           // Flag to call cb() during editing
) {
  TERN_(HAS_TOUCH_BUTTONS, ui.on_edit_screen = true);
  ui.screen_changed = true;
  ui.push_current_screen();
  ui.refresh();
  editLabel = el;
  editValue = ev;
  minEditValue = minv;
  maxEditValue = maxv;
  ui.encoderPosition = ep;
  ui.currentScreen = cs;
  callbackFunc = cb;
  liveEdit = le;
}

////////////////////////////////////////////
///////////////// Menu Tree ////////////////
////////////////////////////////////////////

#include "../../MarlinCore.h"

/**
 * General function to go directly to a screen
 */
void MarlinUI::goto_screen(screenFunc_t screen, const uint16_t encoder/*=0*/, const uint8_t top/*=0*/, const uint8_t items/*=0*/) {
  if (currentScreen == screen) return;

  wake_display();

  thermalManager.set_menu_cold_override(false);

  TERN_(IS_DWIN_MARLINUI, did_first_redraw = false);

  TERN_(HAS_TOUCH_BUTTONS, repeat_delay = BUTTON_DELAY_MENU);

  TERN_(SET_PROGRESS_PERCENT, progress_reset());

  /**
   * Double-click on the status screen is a shortcut for one of these:
   *   - Babystep the Probe Z Offset
   *   - Babystep the Z axis
   *   - Move the Z axis
   */
  #if ALL(DOUBLECLICK_FOR_Z_BABYSTEPPING, BABYSTEPPING)
    static millis_t doubleclick_expire_ms = 0;
    if (screen == menu_main) {
      if (on_status_screen())
        doubleclick_expire_ms = millis() + DOUBLECLICK_MAX_INTERVAL;
    }
    else if (screen == status_screen && currentScreen == menu_main && PENDING(millis(), doubleclick_expire_ms)) {
      if (BABYSTEP_ALLOWED())
        screen = TERN(BABYSTEP_ZPROBE_OFFSET, lcd_babystep_zoffset, lcd_babystep_z);
      else {
        #if ENABLED(MOVE_Z_WHEN_IDLE)
          ui.manual_move.menu_scale = MOVE_Z_IDLE_MULTIPLICATOR;
          screen = []{ lcd_move_axis(Z_AXIS); };
        #endif
      }
    }
  #endif

  //
  // Go to the new screen
  //

  currentScreen = screen;
  encoderPosition = encoder;
  encoderTopLine = top;
  screen_items = items;
  if (on_status_screen()) {
    defer_status_screen(false);
    clear_menu_history();
    TERN_(AUTO_BED_LEVELING_UBL, bedlevel.lcd_map_control = false);
  }

  clear_for_drawing();

  // Re-initialize custom characters that may be re-used
  #if HAS_MARLINUI_HD44780
    if (TERN1(AUTO_BED_LEVELING_UBL, !bedlevel.lcd_map_control))
      set_custom_characters(on_status_screen() ? CHARSET_INFO : CHARSET_MENU);
  #endif

  refresh(LCDVIEW_CALL_REDRAW_NEXT);
  screen_changed = true;
  TERN_(HAS_MARLINUI_U8GLIB, drawing_screen = false);

  TERN_(HAS_MARLINUI_MENU, encoder_direction_normal());
  enable_encoder_multiplier(false);

  set_selection(false);
}

////////////////////////////////////////////
///////////// Manual Movement //////////////
////////////////////////////////////////////

//
// Display a "synchronize" screen with a custom message until
// all moves are finished. Go back to calling screen when done.
//
void MarlinUI::synchronize(FSTR_P const fmsg/*=nullptr*/) {
  push_current_screen();

  // Hijack 'editable' for the string pointer
  editable.fstr = fmsg ?: GET_TEXT_F(MSG_MOVING);
  goto_screen([]{
    if (should_draw()) MenuItem_static::draw(LCD_HEIGHT >= 4, editable.fstr);
  });

  defer_status_screen();
  planner.synchronize(); // idle() is called until moves complete
  goto_previous_screen_no_defer();
}

/**
 * Scrolling for menus and other line-based screens
 *
 *   encoderLine is the position based on the encoder
 *   encoderTopLine is the top menu line to display
 *   screen_items is the total number of items in the menu (after one call)
 */
void scroll_screen(const uint8_t limit, const bool is_menu) {
  ui.encoder_direction_menus();
  if (int32_t(ui.encoderPosition) < 0) ui.encoderPosition = 0;
  if (ui.first_page) {
    encoderLine = ui.encoderPosition / (ENCODER_STEPS_PER_MENU_ITEM);
    ui.screen_changed = false;
  }
  if (screen_items > 0 && encoderLine >= screen_items - limit) {
    encoderLine = _MAX(0, screen_items - limit);
    ui.encoderPosition = encoderLine * (ENCODER_STEPS_PER_MENU_ITEM);
  }
  if (is_menu) {
    NOMORE(encoderTopLine, encoderLine);
    if (encoderLine >= encoderTopLine + LCD_HEIGHT)
      encoderTopLine = encoderLine - LCD_HEIGHT + 1;
  }
  else
    encoderTopLine = encoderLine;
}

#if HAS_LINE_TO_Z

  void line_to_z(const_float_t z) {
    current_position.z = z;
    line_to_current_position(manual_feedrate_mm_s.z);
  }

#endif

#if ENABLED(BABYSTEP_ZPROBE_OFFSET)

  #include "../../feature/babystep.h"

  void lcd_babystep_zoffset() {
    if (ui.use_click()) return ui.goto_previous_screen_no_defer();
    ui.defer_status_screen();
    const bool do_probe = DISABLED(BABYSTEP_HOTEND_Z_OFFSET) || active_extruder == 0;
    if (ui.encoderPosition) {
      const int16_t babystep_increment = int16_t(ui.encoderPosition) * (BABYSTEP_SIZE_Z);
      ui.encoderPosition = 0;

      const float diff = planner.mm_per_step[Z_AXIS] * babystep_increment,
                  new_probe_offset = probe.offset.z + diff,
                  new_offs = TERN(BABYSTEP_HOTEND_Z_OFFSET
                    , do_probe ? new_probe_offset : hotend_offset[active_extruder].z - diff
                    , new_probe_offset
                  );
      if (WITHIN(new_offs, PROBE_OFFSET_ZMIN, PROBE_OFFSET_ZMAX)) {

        babystep.add_steps(Z_AXIS, babystep_increment);

        if (do_probe)
          probe.offset.z = new_offs;
        else
          TERN(BABYSTEP_HOTEND_Z_OFFSET, hotend_offset[active_extruder].z = new_offs, NOOP);

        ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
      }
    }
    if (ui.should_draw()) {
      if (do_probe) {
        MenuEditItemBase::draw_edit_screen(GET_TEXT_F(MSG_BABYSTEP_PROBE_Z), BABYSTEP_TO_STR(probe.offset.z));
        TERN_(BABYSTEP_GFX_OVERLAY, ui.zoffset_overlay(probe.offset.z));
      }
      else {
        #if ENABLED(BABYSTEP_HOTEND_Z_OFFSET)
          MenuEditItemBase::draw_edit_screen(GET_TEXT_F(MSG_HOTEND_OFFSET_Z), ftostr54sign(hotend_offset[active_extruder].z));
        #endif
      }
    }
  }

#endif // BABYSTEP_ZPROBE_OFFSET

void _lcd_draw_homing() {
  if (ui.should_draw()) {
    constexpr uint8_t line = (LCD_HEIGHT - 1) / 2;
    MenuItem_static::draw(line, GET_TEXT_F(MSG_LEVEL_BED_HOMING));
  }
}

#if HAS_LEVELING
  void _lcd_toggle_bed_leveling() { set_bed_leveling_enabled(!planner.leveling_active); }
#endif

//
// Selection screen presents a prompt and two options
//
bool MarlinUI::selection; // = false
bool MarlinUI::update_selection() {
  encoder_direction_select();
  if (encoderPosition) {
    selection = int16_t(encoderPosition) > 0;
    encoderPosition = 0;
  }
  return selection;
}

void MenuItem_confirm::select_screen(
  FSTR_P const yes, FSTR_P const no,
  selectFunc_t yesFunc, selectFunc_t noFunc,
  FSTR_P const fpre, const char * const string/*=nullptr*/, FSTR_P const fsuf/*=nullptr*/
) {
  ui.defer_status_screen();
  const bool ui_selection = !yes ? false : !no || ui.update_selection(),
             got_click = ui.use_click();
  if (got_click || ui.should_draw()) {
    draw_select_screen(yes, no, ui_selection, fpre, string, fsuf);
    if (got_click) {
      selectFunc_t callFunc = ui_selection ? yesFunc : noFunc;
      if (callFunc) callFunc(); else ui.goto_previous_screen();
    }
  }
}

#endif // HAS_MARLINUI_MENU
