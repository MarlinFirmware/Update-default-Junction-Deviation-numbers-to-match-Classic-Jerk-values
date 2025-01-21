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
#pragma once

#include "../inc/MarlinConfig.h"
#include "../sd/cardreader.h"
#include "../module/motion.h"
#include "../libs/buzzer.h"
#include "buttons.h"

#if ENABLED(EEPROM_SETTINGS)
  #include "../module/settings.h"
#endif

#if ENABLED(TOUCH_SCREEN_CALIBRATION)
  #include "tft_io/touch_calibration.h"
#endif

#if E_MANUAL > 1
  #define MULTI_E_MANUAL 1
#endif

#if HAS_PRINT_PROGRESS
  #include "../module/printcounter.h"
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "../feature/pause.h"
#endif

#if ENABLED(DWIN_CREALITY_LCD)
  #include "e3v2/creality/dwin.h"
#elif ENABLED(DWIN_LCD_PROUI)
  #include "e3v2/proui/dwin.h"
#endif

#if ALL(HAS_STATUS_MESSAGE, IS_DWIN_MARLINUI)
  #include "e3v2/marlinui/marlinui_dwin.h" // for LCD_WIDTH
#endif

typedef bool (*statusResetFunc_t)();

#if HAS_WIRED_LCD

  enum LCDViewAction : uint8_t {
    LCDVIEW_NONE,
    LCDVIEW_REDRAW_NOW,
    LCDVIEW_CALL_REDRAW_NEXT,
    LCDVIEW_CLEAR_CALL_REDRAW,
    LCDVIEW_CALL_NO_REDRAW
  };

  #if HAS_ADC_BUTTONS
    uint8_t get_ADC_keyValue();
  #endif

  #if HAS_MARLINUI_MENU

    #include "lcdprint.h"

    #if !HAS_GRAPHICAL_TFT
      void _wrap_string(uint8_t &col, uint8_t &row, const char * const string, read_byte_cb_t cb_read_byte, const bool wordwrap=false);
      inline void wrap_string_P(uint8_t &col, uint8_t &row, PGM_P const pstr, const bool wordwrap=false) { _wrap_string(col, row, pstr, read_byte_rom, wordwrap); }
      inline void wrap_string(uint8_t &col, uint8_t &row, const char * const string, const bool wordwrap=false) { _wrap_string(col, row, string, read_byte_ram, wordwrap); }
    #endif

    typedef void (*screenFunc_t)();
    typedef void (*menuAction_t)();

  #endif // HAS_MARLINUI_MENU

#endif // HAS_WIRED_LCD

#if ANY(HAS_WIRED_LCD, DWIN_CREALITY_LCD_JYERSUI)
  #define LCD_WITH_BLINK 1
  #define LCD_UPDATE_INTERVAL DIV_TERN(DOUBLE_LCD_FRAMERATE, TERN(HAS_TOUCH_BUTTONS, 50, 100), 2)
#endif

#if HAS_MARLINUI_U8GLIB
  enum MarlinFont : uint8_t {
    FONT_STATUSMENU = 1,
    FONT_EDIT,
    FONT_MENU
  };
#else
  enum HD44780CharSet : uint8_t {
    CHARSET_MENU,
    CHARSET_INFO,
    CHARSET_BOOT,
    CHARSET_BOOT_CUSTOM
  };
#endif

#if HAS_PREHEAT
  typedef struct {
    #if HAS_HOTEND
      celsius_t hotend_temp;
    #endif
    #if HAS_HEATED_BED
      celsius_t bed_temp;
    #endif
    #if HAS_FAN
      uint16_t fan_speed;
    #endif
  } preheat_t;
#endif

#if HAS_MARLINUI_MENU

  // Manual Movement class
  class ManualMove {
  private:
    static AxisEnum axis;
    #if MULTI_E_MANUAL
      static int8_t e_index;
    #else
      static int8_t constexpr e_index = 0;
    #endif
    static millis_t start_time;
    #if IS_KINEMATIC
      static xyze_pos_t all_axes_destination;
    #endif
  public:
    static screenFunc_t screen_ptr;
    static float menu_scale;
    #if IS_KINEMATIC
      static float offset;
    #endif
    #if ENABLED(MANUAL_E_MOVES_RELATIVE)
      static float e_origin;
    #endif
    template <typename T>
    static void set_destination(const T& dest) {
      #if IS_KINEMATIC
        // Moves are segmented, so the entire move is not submitted at once.
        // Using a separate variable prevents corrupting the in-progress move.
        all_axes_destination = current_position;
        all_axes_destination.set(dest);
      #else
        // Moves are submitted as single line to the planner using buffer_line.
        current_position.set(dest);
      #endif
    }
    static float axis_value(const AxisEnum axis) {
      return NATIVE_TO_LOGICAL(processing ? destination[axis] : SUM_TERN(IS_KINEMATIC, current_position[axis], offset), axis);
    }
    static bool apply_diff(const AxisEnum axis, const_float_t diff, const_float_t min, const_float_t max) {
      #if IS_KINEMATIC
        float &valref = offset;
        const float rmin = min - current_position[axis], rmax = max - current_position[axis];
      #else
        float &valref = current_position[axis];
        const float rmin = min, rmax = max;
      #endif
      valref += diff;
      const float pre = valref;
      if (min != max) { if (diff < 0) NOLESS(valref, rmin); else NOMORE(valref, rmax); }
      return pre != valref;
    }
    #if IS_KINEMATIC
      static bool processing;
    #else
      static bool constexpr processing = false;
    #endif
    static void task();
    static void soon(const AxisEnum move_axis OPTARG(MULTI_E_MANUAL, const int8_t eindex=active_extruder));
  };

  void lcd_move_axis(const AxisEnum);

#endif

////////////////////////////////////////////
//////////// MarlinUI Singleton ////////////
////////////////////////////////////////////

class MarlinUI;
extern MarlinUI ui;

class MarlinUI {
public:

  MarlinUI() {
    TERN_(HAS_MARLINUI_MENU, currentScreen = status_screen);
  }

  static void init();

  static void reinit_lcd() { TERN_(REINIT_NOISY_LCD, init_lcd()); }

  #if HAS_WIRED_LCD
    static bool detected();
  #else
    static bool detected() { return true; }
  #endif

  #if HAS_MULTI_LANGUAGE
    static uint8_t language;
    static void set_language(const uint8_t lang);
  #endif

  #if HAS_MARLINUI_U8GLIB
    static void update_language_font();
  #endif

  #if ENABLED(SOUND_MENU_ITEM)
    static bool sound_on; // Initialized by settings.load
  #else
    static constexpr bool sound_on = true;
  #endif

  #if USE_MARLINUI_BUZZER
    static void buzz(const long duration, const uint16_t freq=0);
  #endif

  static void chirp() {
    TERN_(HAS_CHIRP, TERN(USE_MARLINUI_BUZZER, buzz, BUZZ)(LCD_FEEDBACK_FREQUENCY_DURATION_MS, LCD_FEEDBACK_FREQUENCY_HZ));
  }

  #if ENABLED(LCD_HAS_STATUS_INDICATORS)
    static void update_indicators();
  #endif

  #if ALL(HAS_MARLINUI_MENU, TOUCH_SCREEN_CALIBRATION)
    static void check_touch_calibration() {
      if (touch_calibration.need_calibration()) currentScreen = touch_calibration_screen;
    }
  #endif

  #if (HAS_WIRED_LCD && HAS_ENCODER_ACTION && HAS_MARLINUI_ENCODER) || HAS_DWIN_E3V2 || HAS_TFT_LVGL_UI
    #define MARLINUI_ENCODER_DELTA 1
    static int8_t get_encoder_delta(const millis_t &now=millis());
  #endif

  #if HAS_MEDIA
    #define MEDIA_MENU_GATEWAY TERN(PASSWORD_ON_SD_PRINT_MENU, password.media_gatekeeper, menu_media)
    static void media_changed(const uint8_t old_stat, const uint8_t stat);
  #endif

  #if HAS_LCD_BRIGHTNESS
    #ifndef LCD_BRIGHTNESS_MIN
      #define LCD_BRIGHTNESS_MIN   1
    #endif
    #ifndef LCD_BRIGHTNESS_MAX
      #define LCD_BRIGHTNESS_MAX 255
    #endif
    #ifndef LCD_BRIGHTNESS_DEFAULT
      #define LCD_BRIGHTNESS_DEFAULT LCD_BRIGHTNESS_MAX
    #endif
    static uint8_t brightness;
    static bool backlight;
    static void _set_brightness(); // Implementation-specific
    static void set_brightness(const uint8_t value);
    FORCE_INLINE static void refresh_brightness() { set_brightness(brightness); }
  #endif

  #if HAS_BACKLIGHT_TIMEOUT
    #if ENABLED(EDITABLE_DISPLAY_TIMEOUT)
      static uint8_t backlight_timeout_minutes;
    #else
      static constexpr uint8_t backlight_timeout_minutes = LCD_BACKLIGHT_TIMEOUT_MINS;
    #endif
    static constexpr uint8_t backlight_timeout_min = 0;
    static constexpr uint8_t backlight_timeout_max = 99;
    static millis_t backlight_off_ms;
    static void refresh_backlight_timeout();
  #elif HAS_DISPLAY_SLEEP
    #if ENABLED(EDITABLE_DISPLAY_TIMEOUT)
      static uint8_t sleep_timeout_minutes;
    #else
      static constexpr uint8_t sleep_timeout_minutes = DISPLAY_SLEEP_MINUTES;
    #endif
    static constexpr uint8_t sleep_timeout_min = 0;
    static constexpr uint8_t sleep_timeout_max = 99;
    static millis_t screen_timeout_ms;
    static void refresh_screen_timeout();
  #endif

  // Sleep or wake the display (e.g., by turning the backlight off/on).
  static bool display_is_asleep() IF_DISABLED(HAS_DISPLAY_SLEEP, { return false; });
  static void sleep_display(const bool=true) IF_DISABLED(HAS_DISPLAY_SLEEP, {});
  static void wake_display() { sleep_display(false); }

  #if HAS_PRINT_PROGRESS_PERMYRIAD
    typedef uint16_t progress_t;
    #define PROGRESS_SCALE 100U
    #define PROGRESS_MASK 0x7FFF
  #else
    typedef uint8_t progress_t;
    #define PROGRESS_SCALE 1U
    #define PROGRESS_MASK 0x7F
  #endif

  #if HAS_PRINT_PROGRESS
    #if ENABLED(SET_PROGRESS_PERCENT)
      static progress_t progress_override;
      static void set_progress(const progress_t p) { progress_override = _MIN(p, 100U * (PROGRESS_SCALE)); }
      static void set_progress_done() { progress_override = (PROGRESS_MASK + 1U) + 100U * (PROGRESS_SCALE); }
      static void progress_reset() { if (progress_override & (PROGRESS_MASK + 1U)) set_progress(0); }
    #endif
    #if ANY(SHOW_REMAINING_TIME, SET_PROGRESS_MANUALLY)
      static uint32_t _calculated_remaining_time() {
        const duration_t elapsed = print_job_timer.duration();
        const progress_t progress = _get_progress();
        return progress ? elapsed.value * (100 * (PROGRESS_SCALE) - progress) / progress : 0;
      }
      #if ENABLED(SET_REMAINING_TIME)
        static uint32_t remaining_time;
        FORCE_INLINE static void set_remaining_time(const uint32_t r) { remaining_time = r; }
        FORCE_INLINE static uint32_t get_remaining_time() { return remaining_time ?: _calculated_remaining_time(); }
        FORCE_INLINE static void reset_remaining_time() { set_remaining_time(0); }
      #else
        FORCE_INLINE static uint32_t get_remaining_time() { return _calculated_remaining_time(); }
      #endif
      #if ENABLED(SET_INTERACTION_TIME)
        static uint32_t interaction_time;
        FORCE_INLINE static void set_interaction_time(const uint32_t r) { interaction_time = r; }
        FORCE_INLINE static void reset_interaction_time() { set_interaction_time(0); }
      #endif
    #endif
    static progress_t _get_progress();
    #if HAS_PRINT_PROGRESS_PERMYRIAD
      FORCE_INLINE static uint16_t get_progress_permyriad() { return _get_progress(); }
    #endif
    static uint8_t get_progress_percent() { return uint8_t(_get_progress() / (PROGRESS_SCALE)); }
    #if LCD_WITH_BLINK && HAS_EXTRA_PROGRESS
      #if ENABLED(SHOW_PROGRESS_PERCENT)
        static void drawPercent();
      #endif
      #if ENABLED(SHOW_ELAPSED_TIME)
        static void drawElapsed();
      #endif
      #if ENABLED(SHOW_REMAINING_TIME)
        static void drawRemain();
      #endif
      #if ENABLED(SHOW_INTERACTION_TIME)
        static void drawInter();
      #endif
      static void rotate_progress();
    #else
      static void rotate_progress() {}
    #endif
  #else
    static constexpr uint8_t get_progress_percent() { return 0; }
  #endif

  static void host_notify_P(PGM_P const pstr);
  static void host_notify(FSTR_P const fstr) { host_notify_P(FTOP(fstr)); }
  static void host_notify(const char * const cstr);

  #if HAS_STATUS_MESSAGE

    #if ANY(HAS_WIRED_LCD, DWIN_LCD_PROUI)
      #if ENABLED(STATUS_MESSAGE_SCROLLING)
        #define MAX_MESSAGE_LENGTH _MAX(LONG_FILENAME_LENGTH, MAX_LANG_CHARSIZE * 2 * (LCD_WIDTH))
      #else
        #define MAX_MESSAGE_LENGTH (MAX_LANG_CHARSIZE * (LCD_WIDTH))
      #endif
    #else
      #define MAX_MESSAGE_LENGTH 63
    #endif

    static MString<MAX_MESSAGE_LENGTH> status_message;
    static uint8_t alert_level; // Higher levels block lower levels

    #if HAS_STATUS_MESSAGE_TIMEOUT
      static millis_t status_message_expire_ms; // Reset some status messages after a timeout
    #endif

    #if ENABLED(STATUS_MESSAGE_SCROLLING)
      static uint8_t status_scroll_offset;
      static void reset_status_scroll() { status_scroll_offset = 0; }
      static void advance_status_scroll();
      static char* status_and_len(uint8_t &len);
    #endif

    static bool has_status() { return !status_message.empty(); }

    /**
     * Try to set the alert level.
     * @param level Alert level. Negative to ignore and reset the level. Non-zero never expires.
     * @return      TRUE if the level could NOT be set.
     */
    static bool set_alert_level(int8_t &level);

    static void reset_status(const bool no_welcome=false);
    static void reset_alert_level() { alert_level = 0; }

    static statusResetFunc_t status_reset_callback;
    static void set_status_reset_fn(const statusResetFunc_t fn=nullptr) { status_reset_callback = fn; }

  #else

    #define MAX_MESSAGE_LENGTH 1
    static constexpr bool has_status() { return false; }

    static bool set_alert_level(int8_t) { return false; }

    static void reset_status(const bool=false) {}
    static void reset_alert_level() {}

    static void set_status_reset_fn(const statusResetFunc_t=nullptr) {}

  #endif

  /**
   * @brief Set Status with a C- or P-string and alert level.
   *
   * @param ustr  A C- or P-string, according to pgm.
   * @param level Alert level. Negative to ignore and reset the level. Non-zero never expires.
   * @param pgm   Program string flag. Only relevant on AVR.
   */
  static void _set_status_and_level(const char * const ustr, int8_t level, const bool pgm=false);

  /**
   * @brief Set Status with a C- or P-string and persistence flag.
   *
   * @param ustr    A C- or P-string, according to pgm.
   * @param persist Don't expire (Requires STATUS_EXPIRE_SECONDS) - and set alert level to 1.
   * @param pgm     Program string flag. Only relevant on AVR.
   */
  static void _set_status(const char * const ustr, const bool persist, const bool pgm=false);

  /**
   * @brief Set Alert with a C- or P-string and alert level.
   *
   * @param ustr  A C- or P-string, according to pgm.
   * @param level Alert level. Negative to ignore and reset the level. Non-zero never expires.
   * @param pgm   Program string flag. Only relevant on AVR.
   */
  static void _set_alert(const char * const ustr, int8_t level, const bool pgm=false);

  static void set_status(const char * const cstr, const bool persist=false) { _set_status(cstr, persist, false); }
  static void set_status_P(PGM_P const pstr, const bool persist=false)      { _set_status(pstr, persist, true);  }
  static void set_status(FSTR_P const fstr, const bool persist=false)       { set_status_P(FTOP(fstr), persist); }

  static void set_alert(const char * const cstr, const int8_t level=1) { _set_alert(cstr, level, false); }
  static void set_alert_P(PGM_P const pstr, const int8_t level=1)      { _set_alert(pstr, level, true);  }
  static void set_alert(FSTR_P const fstr, const int8_t level=1)       { set_alert_P(FTOP(fstr), level); }

  /**
   * @brief Set Status with a C-string and alert level.
   *
   * @param fstr  A constant F-string to set as the status.
   * @param level Alert level. Negative to ignore and reset the level. Non-zero never expires.
   */
  static void set_status_and_level(const char * const cstr, const int8_t level=0) { _set_status_and_level(cstr, level, false); }

  /**
   * @brief Set Status with a P-string and alert level.
   *
   * @param ustr  A C- or P-string, according to pgm.
   * @param level Alert level. Negative to ignore and reset the level. Non-zero never expires.
   */
  static void set_status_and_level_P(PGM_P const pstr, const int8_t level=0) { _set_status_and_level(pstr, level, true); }

  /**
   * @brief Set Status with a fixed string and alert level.
   *
   * @param fstr  A constant F-string to set as the status.
   * @param level Alert level. Negative to ignore and reset the level. Non-zero never expires.
   */
  static void set_status_and_level(FSTR_P const fstr, const int8_t level=0) { set_status_and_level_P(FTOP(fstr), level); }

  static void set_max_status(FSTR_P const fstr) { set_status_and_level(fstr, 127); }
  static void set_min_status(FSTR_P const fstr) { set_status_and_level(fstr,  -1); }

  /**
   * @brief Set a persistent status with a C-string.
   *
   * @param cstr    A C-string to set as the status.
   */
  static void set_status_no_expire_P(PGM_P const pstr)      { set_status_P(pstr, true); }
  static void set_status_no_expire(const char * const cstr) { set_status(cstr, true); }
  static void set_status_no_expire(FSTR_P const fstr)       { set_status(fstr, true); }

  /**
   * @brief Set a status with a format string and parameters.
   *
   * @param pfmt    A constant format P-string
   */
  static void status_printf_P(int8_t level, PGM_P const pfmt, ...);

  template<typename... Args>
  static void status_printf(int8_t level, FSTR_P const ffmt, Args... more) { status_printf_P(level, FTOP(ffmt), more...); }

  // Periodic or as-needed display update
  static void update() IF_DISABLED(HAS_UI_UPDATE, {});

  // Tell the screen to redraw on the next call
  FORCE_INLINE static void refresh() {
    TERN_(HAS_WIRED_LCD, refresh(LCDVIEW_CLEAR_CALL_REDRAW));
  }

  #if HAS_DISPLAY

    static void init_lcd();

    // Erase the LCD contents. Do the lowest-level thing required to clear the LCD.
    static void clear_lcd();

    // Clear the LCD before new drawing. Some LCDs do nothing because they redraw frequently.
    static void clear_for_drawing();

    static void abort_print();
    static void pause_print();
    static void resume_print();

    #if ENABLED(FLOWMETER_SAFETY)
      static void flow_fault();
    #endif

    #if ALL(HAS_MARLINUI_MENU, PSU_CONTROL)
      static void poweroff();
    #endif

    #if LCD_WITH_BLINK
      static bool get_blink();
    #endif

    #if HAS_WIRED_LCD

      static millis_t next_button_update_ms;

      static LCDViewAction lcdDrawUpdate;
      FORCE_INLINE static bool should_draw() { return bool(lcdDrawUpdate); }
      FORCE_INLINE static void refresh(const LCDViewAction type) { lcdDrawUpdate = type; }

      #if ENABLED(SHOW_CUSTOM_BOOTSCREEN)
        static void draw_custom_bootscreen(const uint8_t frame=0);
        static void show_custom_bootscreen();
      #endif

      #if ENABLED(SHOW_BOOTSCREEN)
        #ifndef BOOTSCREEN_TIMEOUT
          #define BOOTSCREEN_TIMEOUT 2500
        #endif
        static void draw_marlin_bootscreen(const bool line2=false);
        static void show_marlin_bootscreen();
        static void show_bootscreen();
        static void bootscreen_completion(const millis_t sofar);
      #endif

      #if HAS_MARLINUI_U8GLIB
        static void set_font(const MarlinFont font_nr);
      #elif IS_DWIN_MARLINUI
        static void set_font(const uint8_t font_nr);
      #endif

      #if HAS_MARLINUI_HD44780
        static void set_custom_characters(const HD44780CharSet screen_charset=CHARSET_INFO);
      #endif

      #if ENABLED(LCD_PROGRESS_BAR) && !HAS_MARLINUI_U8GLIB
        static millis_t progress_bar_ms;  // Start time for the current progress bar cycle
        static void draw_progress_bar(const uint8_t percent);
        #if PROGRESS_MSG_EXPIRE > 0
          static millis_t expire_status_ms; // = 0
          FORCE_INLINE static void reset_progress_bar_timeout() { expire_status_ms = 0; }
        #endif
      #endif

      static uint8_t lcd_status_update_delay;

      #if HAS_LCD_CONTRAST
        static uint8_t contrast;
        static void _set_contrast(); // Implementation-specific
        static void set_contrast(const uint8_t value);
        FORCE_INLINE static void refresh_contrast() { set_contrast(contrast); }
      #endif

      #if ALL(FILAMENT_LCD_DISPLAY, HAS_MEDIA)
        static millis_t next_filament_display;
        static void pause_filament_display(const millis_t ms=millis()) { next_filament_display = ms + 5000UL; }
      #endif

      static void quick_feedback(const bool clear_buttons=true);

      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        static void draw_hotend_status(const uint8_t row, const uint8_t extruder);
      #endif

      #if HAS_TOUCH_BUTTONS
        static bool on_edit_screen;
        static void screen_click(const uint8_t row, const uint8_t col, const uint8_t x, const uint8_t y);
      #endif

      static void status_screen();

    #endif // HAS_WIRED_LCD

    #if HAS_MARLINUI_U8GLIB
      static bool drawing_screen, first_page;
    #else
      static constexpr bool drawing_screen = false, first_page = true;
    #endif

    #if IS_DWIN_MARLINUI
      static bool did_first_redraw;
    #endif

    #if ANY(BABYSTEP_GFX_OVERLAY, MESH_EDIT_GFX_OVERLAY)
      static void zoffset_overlay(const int8_t dir);
      static void zoffset_overlay(const_float_t zvalue);
    #endif

    static void draw_kill_screen();
    static void kill_screen(FSTR_P const lcd_error, FSTR_P const lcd_component);
    #if DISABLED(LIGHTWEIGHT_UI)
      static void draw_status_message(const bool blink);
    #endif

  #else // No LCD

    static void init_lcd() {}
    static void clear_lcd() {}
    static void clear_for_drawing() {}
    static void kill_screen(FSTR_P const, FSTR_P const) {}

  #endif

  #if !HAS_WIRED_LCD
    static void quick_feedback(const bool=true) {}
  #endif

  static void completion_feedback(const bool good=true);

  #if HAS_MEDIA
    #if ALL(SCROLL_LONG_FILENAMES, HAS_MARLINUI_MENU)
      #define MARLINUI_SCROLL_NAME 1
    #endif
    static const char * scrolled_filename(CardReader &theCard, const uint8_t maxlen, const bool doScroll);
  #endif

  #if HAS_PREHEAT
    enum PreheatTarget : uint8_t { PT_HOTEND, PT_BED, PT_FAN, PT_CHAMBER, PT_ALL = 0xFF };
    static preheat_t material_preset[PREHEAT_COUNT];
    static FSTR_P get_preheat_label(const uint8_t m);
    static void apply_preheat(const uint8_t m, const uint8_t pmask, const uint8_t e=active_extruder);
    static void preheat_set_fan(const uint8_t m) { TERN_(HAS_FAN, apply_preheat(m, _BV(PT_FAN))); }
    static void preheat_hotend(const uint8_t m, const uint8_t e=active_extruder) { TERN_(HAS_HOTEND, apply_preheat(m, _BV(PT_HOTEND))); }
    static void preheat_hotend_and_fan(const uint8_t m, const uint8_t e=active_extruder) { preheat_hotend(m, e); preheat_set_fan(m); }
    static void preheat_bed(const uint8_t m) { TERN_(HAS_HEATED_BED, apply_preheat(m, _BV(PT_BED))); }
    static void preheat_all(const uint8_t m) { apply_preheat(m, PT_ALL); }
  #endif

  static void reset_status_timeout(const millis_t ms) {
    TERN(HAS_SCREEN_TIMEOUT, return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS, UNUSED(ms));
  }

  #if ALL(HAS_MARLINUI_MENU, ENCODER_RATE_MULTIPLIER)
    static bool encoder_multiplier_enabled;
    static void enable_encoder_multiplier(const bool onoff) { encoder_multiplier_enabled = onoff; }
  #else
    static void enable_encoder_multiplier(const bool) {}
  #endif

  #if HAS_MARLINUI_MENU

    #if HAS_TOUCH_BUTTONS
      static uint8_t touch_buttons;
      static uint16_t repeat_delay;
    #else
      static constexpr uint8_t touch_buttons = 0;
    #endif

    // Manual Movement
    static ManualMove manual_move;
    static bool can_show_slider() { return !external_control && currentScreen != manual_move.screen_ptr; }

    // Select Screen (modal NO/YES style dialog)
    static bool selection;
    static void set_selection(const bool sel) { selection = sel; }
    static bool update_selection();

    static void synchronize(FSTR_P const msg=nullptr);

    static screenFunc_t currentScreen;
    static bool screen_changed;
    static void goto_screen(const screenFunc_t screen, const uint16_t encoder=0, const uint8_t top=0, const uint8_t items=0);
    static void push_current_screen();

    // goto_previous_screen and go_back may also be used as menu item callbacks
    static void _goto_previous_screen(TERN_(TURBO_BACK_MENU_ITEM, const bool is_back));
    static void goto_previous_screen() { _goto_previous_screen(TERN_(TURBO_BACK_MENU_ITEM, false)); }
    static void go_back()              { _goto_previous_screen(TERN_(TURBO_BACK_MENU_ITEM, true)); }

    static void return_to_status();
    static bool on_status_screen() { return currentScreen == status_screen; }
    FORCE_INLINE static void run_current_screen() { (*currentScreen)(); }

    #if ENABLED(LIGHTWEIGHT_UI)
      static void lcd_in_status(const bool inStatus);
    #endif

    FORCE_INLINE static bool screen_is_sticky() {
      return TERN1(HAS_SCREEN_TIMEOUT, defer_return_to_status);
    }

    FORCE_INLINE static void defer_status_screen(const bool defer=true) {
      TERN(HAS_SCREEN_TIMEOUT, defer_return_to_status = defer, UNUSED(defer));
    }

    static void goto_previous_screen_no_defer() {
      defer_status_screen(false);
      goto_previous_screen();
    }

    #if ENABLED(SD_REPRINT_LAST_SELECTED_FILE)
      static void reselect_last_file();
    #endif

    #if ENABLED(AUTO_BED_LEVELING_UBL)
      static void ubl_plot(const uint8_t x_plot, const uint8_t y_plot);
    #endif

    #if ENABLED(AUTO_BED_LEVELING_UBL)
      static void ubl_mesh_edit_start(const_float_t initial);
      static float ubl_mesh_value();
    #endif

    static void draw_select_screen_prompt(FSTR_P const fpre, const char * const string=nullptr, FSTR_P const fsuf=nullptr);

  #else // !HAS_MARLINUI_MENU

    static void return_to_status() {}

    static constexpr bool on_status_screen() { return true; }

    #if HAS_WIRED_LCD
      FORCE_INLINE static void run_current_screen() { status_screen(); }
    #endif

  #endif // !HAS_MARLINUI_MENU

  #if ANY(HAS_MARLINUI_MENU, EXTENSIBLE_UI)
    static bool lcd_clicked;
    static bool use_click() {
      const bool click = lcd_clicked;
      lcd_clicked = false;
      return click;
    }
  #else
    static constexpr bool lcd_clicked = false;
    static bool use_click() { return false; }
  #endif

  #if ENABLED(ADVANCED_PAUSE_FEATURE) && ANY(HAS_MARLINUI_MENU, EXTENSIBLE_UI, DWIN_CREALITY_LCD_JYERSUI)
    static void pause_show_message(const PauseMessage message, const PauseMode mode=PAUSE_MODE_SAME, const uint8_t extruder=active_extruder);
  #else
    static void _pause_show_message() {}
    #define pause_show_message(...) _pause_show_message()
  #endif

  //
  // EEPROM: Reset / Init / Load / Store
  //
  #if HAS_MARLINUI_MENU
    static void reset_settings();
  #endif

  #if ENABLED(EEPROM_SETTINGS)
    #if HAS_MARLINUI_MENU
      static void init_eeprom();
      static void load_settings();
      static void store_settings();
    #endif
    static void eeprom_alert(const EEPROM_Error) TERN_(EEPROM_AUTO_INIT, {});
  #endif

  //
  // Special handling if a move is underway
  //
  #if ANY(DELTA_CALIBRATION_MENU, DELTA_AUTO_CALIBRATION, PROBE_OFFSET_WIZARD, X_AXIS_TWIST_COMPENSATION) || (ENABLED(LCD_BED_LEVELING) && ANY(PROBE_MANUALLY, MESH_BED_LEVELING))
    #define LCD_HAS_WAIT_FOR_MOVE 1
    static bool wait_for_move;
  #else
    static constexpr bool wait_for_move = false;
  #endif

  //
  // Block interaction while under external control
  //
  #if HAS_MARLINUI_MENU && ANY(AUTO_BED_LEVELING_UBL, G26_MESH_VALIDATION)
    static bool external_control;
    FORCE_INLINE static void capture() { external_control = true; }
    FORCE_INLINE static void release() { external_control = false; }
    #if ENABLED(AUTO_BED_LEVELING_UBL)
      static void external_encoder();
    #endif
  #else
    static constexpr bool external_control = false;
  #endif

  #if HAS_ENCODER_ACTION

    static volatile uint8_t buttons;
    #if IS_RRW_KEYPAD
      static volatile uint8_t keypad_buttons;
      static bool handle_keypad();
    #endif
    #if HAS_SLOW_BUTTONS
      static volatile uint8_t slow_buttons;
      static uint8_t read_slow_buttons();
    #endif

    static void update_buttons();

    #if ENABLED(ENCODER_NOISE_FILTER)
      /**
       * Some printers may have issues with EMI noise especially using a motherboard with 3.3V logic levels
       * it may cause the logical LOW to float into the undefined region and register as a logical HIGH
       * causing it to erroneously register as if someone clicked the button and in worst case make the
       * printer unusable in practice.
       */
      static bool hw_button_pressed() {
        for (uint8_t s = 0; s < ENCODER_SAMPLES; ++s) {
          if (!BUTTON_CLICK()) return false;
          safe_delay(1);
        }
        return true;
      }
    #else
      static bool hw_button_pressed() { return BUTTON_CLICK(); }
    #endif

    #if ANY(AUTO_BED_LEVELING_UBL, G26_MESH_VALIDATION)
      static void wait_for_release();
    #endif

    static uint32_t encoderPosition;

    #define ENCODERBASE (TERN(REVERSE_ENCODER_DIRECTION, -1, +1))

    #if ANY(REVERSE_MENU_DIRECTION, REVERSE_SELECT_DIRECTION)
      static int8_t encoderDirection;
    #else
      static constexpr int8_t encoderDirection = ENCODERBASE;
    #endif

    FORCE_INLINE static void encoder_direction_normal() {
      #if ANY(REVERSE_MENU_DIRECTION, REVERSE_SELECT_DIRECTION)
        encoderDirection = ENCODERBASE;
      #endif
    }

    FORCE_INLINE static void encoder_direction_menus() {
      TERN_(REVERSE_MENU_DIRECTION, encoderDirection = -(ENCODERBASE));
    }

    FORCE_INLINE static void encoder_direction_select() {
      TERN_(REVERSE_SELECT_DIRECTION, encoderDirection = -(ENCODERBASE));
    }

  #else

    static void update_buttons() {}
    static bool hw_button_pressed() { return false; }

  #endif

  static bool button_pressed() { return hw_button_pressed() || TERN0(TOUCH_SCREEN, touch_pressed()); }

  #if ENABLED(TOUCH_SCREEN_CALIBRATION)
    static void touch_calibration_screen();
  #endif

  #if HAS_GRAPHICAL_TFT
    static void move_axis_screen();
  #endif

private:

  #if HAS_SCREEN_TIMEOUT
    static millis_t return_to_status_ms;
    static bool defer_return_to_status;
  #else
    static constexpr bool defer_return_to_status = false;
  #endif

  #if HAS_STATUS_MESSAGE
    static void finish_status(const bool persist);
  #endif

  #if HAS_WIRED_LCD
    static void draw_status_screen();
    #if HAS_GRAPHICAL_TFT
      static void tft_idle();
      #if ENABLED(TOUCH_SCREEN)
        static bool touch_pressed();
      #endif
    #endif
  #endif
};

#define LCD_MESSAGE_F(S)       ui.set_status(F(S))
#define LCD_MESSAGE(M)         ui.set_status(GET_TEXT_F(M))
#define LCD_MESSAGE_MIN(M)     ui.set_min_status(GET_TEXT_F(M))
#define LCD_MESSAGE_MAX(M)     ui.set_max_status(GET_TEXT_F(M))
#define LCD_ALERTMESSAGE_F(S)  ui.set_alert(F(S))
#define LCD_ALERTMESSAGE(M)    ui.set_alert(GET_TEXT_F(M))
