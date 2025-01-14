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

#include "../inc/MarlinConfig.h"

#include "../MarlinCore.h" // for printingIsPaused
#include "../gcode/parser.h" // for axis_is_rotational, using_inch_units

#if HAS_LED_POWEROFF_TIMEOUT || ALL(HAS_WIRED_LCD, PRINTER_EVENT_LEDS) || (HAS_BACKLIGHT_TIMEOUT && defined(NEOPIXEL_BKGD_INDEX_FIRST))
  #include "../feature/leds/leds.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../feature/host_actions.h"
#endif

#if ALL(BROWSE_MEDIA_ON_INSERT, PASSWORD_ON_SD_PRINT_MENU)
  #include "../feature/password/password.h"
#endif

// All displays share the MarlinUI class
#include "marlinui.h"
MarlinUI ui;

#if HAS_DISPLAY
  #include "../gcode/queue.h"
  #include "utf8.h"
#endif

#if ENABLED(DWIN_CREALITY_LCD)
  #include "e3v2/creality/dwin.h"
#elif ENABLED(DWIN_CREALITY_LCD_JYERSUI)
  #include "e3v2/jyersui/dwin.h"
#elif ENABLED(SOVOL_SV06_RTS)
  #include "sovol_rts/sovol_rts.h"
#endif

#if ENABLED(LCD_PROGRESS_BAR) && !IS_TFTGLCD_PANEL
  #define HAS_BASIC_PROGRESS_BAR 1
#endif

#if ANY(HAS_DISPLAY, HAS_STATUS_MESSAGE, HAS_BASIC_PROGRESS_BAR)
  #include "../module/printcounter.h"
#endif

#if LCD_HAS_WAIT_FOR_MOVE
  bool MarlinUI::wait_for_move; // = false
#endif

constexpr uint8_t epps = ENCODER_PULSES_PER_STEP;

#if HAS_STATUS_MESSAGE
  #if ENABLED(STATUS_MESSAGE_SCROLLING)
    uint8_t MarlinUI::status_scroll_offset; // = 0
  #endif
  MString<MAX_MESSAGE_LENGTH> MarlinUI::status_message;
  uint8_t MarlinUI::alert_level; // = 0
  #if HAS_STATUS_MESSAGE_TIMEOUT
    millis_t MarlinUI::status_message_expire_ms; // = 0
  #endif
  statusResetFunc_t MarlinUI::status_reset_callback; // = nullptr
#endif

#if ENABLED(SET_PROGRESS_MANUALLY)
  #if ENABLED(SET_PROGRESS_PERCENT)
    MarlinUI::progress_t MarlinUI::progress_override; // = 0
  #endif
  #if ENABLED(SET_REMAINING_TIME)
    uint32_t MarlinUI::remaining_time;
  #endif
  #if ENABLED(SET_INTERACTION_TIME)
    uint32_t MarlinUI::interaction_time;
  #endif
#endif

#if HAS_MULTI_LANGUAGE
  uint8_t MarlinUI::language; // Initialized by settings.load
  void MarlinUI::set_language(const uint8_t lang) {
    if (lang < NUM_LANGUAGES) {
      language = lang;
      TERN_(HAS_MARLINUI_U8GLIB, update_language_font());
      return_to_status();
      refresh();
    }
  }
#endif

#if HAS_LCD_CONTRAST
  uint8_t MarlinUI::contrast = LCD_CONTRAST_DEFAULT; // Initialized by settings.load
  void MarlinUI::set_contrast(const uint8_t value) {
    contrast = constrain(value, LCD_CONTRAST_MIN, LCD_CONTRAST_MAX);
    _set_contrast();
  }
#endif

#if HAS_LCD_BRIGHTNESS
  uint8_t MarlinUI::brightness = LCD_BRIGHTNESS_DEFAULT;
  bool MarlinUI::backlight = true;

  void MarlinUI::set_brightness(const uint8_t value) {
    backlight = !!value;
    if (backlight) brightness = constrain(value, LCD_BRIGHTNESS_MIN, LCD_BRIGHTNESS_MAX);
    #if DISABLED(SOVOL_SV06_RTS)
      _set_brightness();
    #endif
  }
#endif

#if ENABLED(SOUND_MENU_ITEM)
  bool MarlinUI::sound_on = ENABLED(SOUND_ON_DEFAULT);
#endif

#if ENABLED(PCA9632_BUZZER)
  void MarlinUI::buzz(const long duration, const uint16_t freq/*=0*/) {
    if (sound_on) PCA9632_buzz(duration, freq);
  }
#endif

#if HAS_PREHEAT
  #include "../module/temperature.h"

  preheat_t MarlinUI::material_preset[PREHEAT_COUNT];  // Initialized by settings.load

  FSTR_P MarlinUI::get_preheat_label(const uint8_t m) {
    #define _PDEF(N) static PGMSTR(preheat_##N##_label, PREHEAT_##N##_LABEL);
    #define _PLBL(N) preheat_##N##_label,
    REPEAT_1(PREHEAT_COUNT, _PDEF);
    static PGM_P const preheat_labels[PREHEAT_COUNT] PROGMEM = { REPEAT_1(PREHEAT_COUNT, _PLBL) };
    return FPSTR((PGM_P)pgm_read_ptr(&preheat_labels[m]));
  }

  void MarlinUI::apply_preheat(const uint8_t m, const uint8_t pmask, const uint8_t e/*=active_extruder*/) {
    const preheat_t &pre = material_preset[m];
    TERN_(HAS_HOTEND,           if (TEST(pmask, PT_HOTEND))  thermalManager.setTargetHotend(pre.hotend_temp, e));
    TERN_(HAS_HEATED_BED,       if (TEST(pmask, PT_BED))     thermalManager.setTargetBed(pre.bed_temp));
    //TERN_(HAS_HEATED_CHAMBER, if (TEST(pmask, PT_CHAMBER)) thermalManager.setTargetChamber(pre.chamber_temp));
    TERN_(HAS_FAN,              if (TEST(pmask, PT_FAN))     thermalManager.set_fan_speed(0, pre.fan_speed));
  }
#endif

#if ANY(HAS_MARLINUI_MENU, EXTENSIBLE_UI)
  bool MarlinUI::lcd_clicked;
#endif

#if LCD_WITH_BLINK

  bool MarlinUI::get_blink() {
    static uint8_t blink = 0;
    static millis_t next_blink_ms = 0;
    millis_t ms = millis();
    if (ELAPSED(ms, next_blink_ms)) {
      blink ^= 0xFF;
      next_blink_ms = ms + 1000 - (LCD_UPDATE_INTERVAL) / 2;
    }
    return blink != 0;
  }

#endif

#if HAS_U8GLIB_I2C_OLED && PINS_EXIST(I2C_SCL, I2C_SDA) && DISABLED(SOFT_I2C_EEPROM)
  #include <Wire.h>
#endif

// Encoder Handling
#if HAS_ENCODER_ACTION
  uint32_t MarlinUI::encoderPosition;
  volatile int8_t encoderDiff; // Updated in update_buttons, added to encoderPosition every LCD update
#endif

#if HAS_BACKLIGHT_TIMEOUT

  #if ENABLED(EDITABLE_DISPLAY_TIMEOUT)
    uint8_t MarlinUI::backlight_timeout_minutes; // Initialized by settings.load
  #else
    constexpr uint8_t MarlinUI::backlight_timeout_minutes;
  #endif
  constexpr uint8_t MarlinUI::backlight_timeout_min, MarlinUI::backlight_timeout_max;
  millis_t MarlinUI::backlight_off_ms = 0;

  void MarlinUI::refresh_backlight_timeout() {
    backlight_off_ms = backlight_timeout_minutes ? millis() + MIN_TO_MS(backlight_timeout_minutes) : 0;
    #ifdef NEOPIXEL_BKGD_INDEX_FIRST
      neo.reset_background_color();
      neo.show();
    #elif PIN_EXISTS(LCD_BACKLIGHT)
      WRITE(LCD_BACKLIGHT_PIN, HIGH);
    #endif
  }

#elif HAS_DISPLAY_SLEEP

  #if ENABLED(EDITABLE_DISPLAY_TIMEOUT)
    uint8_t MarlinUI::sleep_timeout_minutes; // Initialized by settings.load
  #else
    constexpr uint8_t MarlinUI::sleep_timeout_minutes;
  #endif
  constexpr uint8_t MarlinUI::sleep_timeout_min, MarlinUI::sleep_timeout_max;

  millis_t MarlinUI::screen_timeout_ms = 0;
  void MarlinUI::refresh_screen_timeout() {
    screen_timeout_ms = sleep_timeout_minutes ? millis() + MIN_TO_MS(sleep_timeout_minutes) : 0;
    wake_display();
  }

#endif

void MarlinUI::init() {

  #if HAS_U8GLIB_I2C_OLED && PINS_EXIST(I2C_SCL, I2C_SDA) && DISABLED(SOFT_I2C_EEPROM)
    Wire.begin(uint8_t(I2C_SDA_PIN), uint8_t(I2C_SCL_PIN));
  #endif

  init_lcd();
  clear_lcd();

  #if BUTTON_EXISTS(EN1)
    SET_INPUT_PULLUP(BTN_EN1);
  #endif
  #if BUTTON_EXISTS(EN2)
    SET_INPUT_PULLUP(BTN_EN2);
  #endif
  #if BUTTON_EXISTS(ENC)
    SET_INPUT_PULLUP(BTN_ENC);
  #endif
  #if BUTTON_EXISTS(ENC_EN)
    SET_INPUT_PULLUP(BTN_ENC_EN);
  #endif
  #if BUTTON_EXISTS(BACK)
    SET_INPUT_PULLUP(BTN_BACK);
  #endif
  #if BUTTON_EXISTS(UP)
    SET_INPUT(BTN_UP);
  #endif
  #if BUTTON_EXISTS(DOWN)
    SET_INPUT(BTN_DOWN);
  #endif
  #if BUTTON_EXISTS(LFT)
    SET_INPUT(BTN_LEFT);
  #endif
  #if BUTTON_EXISTS(RT)
    SET_INPUT(BTN_RIGHT);
  #endif

  #if HAS_SHIFT_ENCODER

    #if ENABLED(SR_LCD_2W_NL) // Non latching 2 wire shift register

      SET_OUTPUT(SR_DATA_PIN);
      SET_OUTPUT(SR_CLK_PIN);

    #elif PIN_EXISTS(SHIFT_CLK)

      SET_OUTPUT(SHIFT_CLK_PIN);
      OUT_WRITE(SHIFT_LD_PIN, HIGH);
      #if PIN_EXISTS(SHIFT_EN)
        OUT_WRITE(SHIFT_EN_PIN, LOW);
      #endif
      SET_INPUT_PULLUP(SHIFT_OUT_PIN);

    #endif

  #endif // HAS_SHIFT_ENCODER

  #if ALL(HAS_ENCODER_ACTION, HAS_SLOW_BUTTONS)
    slow_buttons = 0;
  #endif

  update_buttons();

  TERN_(HAS_ENCODER_ACTION, encoderDiff = 0);

  reset_status(); // Set welcome message
}

#if HAS_WIRED_LCD

  #if HAS_MARLINUI_U8GLIB
    #include "dogm/marlinui_DOGM.h"
  #endif

  #include "lcdprint.h"

  #include "../module/temperature.h"
  #include "../module/planner.h"
  #include "../module/motion.h"

  #if HAS_MARLINUI_MENU
    #include "../module/settings.h"
  #endif

  #if ENABLED(AUTO_BED_LEVELING_UBL)
    #include "../feature/bedlevel/bedlevel.h"
  #endif

  #if HAS_TRINAMIC_CONFIG
    #include "../feature/tmc_util.h"
  #endif

  #if HAS_ADC_BUTTONS
    #include "../module/thermistor/thermistors.h"
  #endif

  #if HAS_POWER_MONITOR
    #include "../feature/power_monitor.h"
  #endif

  #if HAS_LED_POWEROFF_TIMEOUT
    #include "../feature/power.h"
  #endif

  #if HAS_ENCODER_ACTION
    volatile uint8_t MarlinUI::buttons;
    #if HAS_SLOW_BUTTONS
      volatile uint8_t MarlinUI::slow_buttons;
    #endif
    #if HAS_TOUCH_BUTTONS
      #include "touch/touch_buttons.h"
      bool MarlinUI::on_edit_screen = false;
    #endif
  #endif

  #if HAS_SCREEN_TIMEOUT
    bool MarlinUI::defer_return_to_status;
    millis_t MarlinUI::return_to_status_ms = 0;
  #endif

  uint8_t MarlinUI::lcd_status_update_delay = 1; // First update one loop delayed

  #if ALL(FILAMENT_LCD_DISPLAY, HAS_MEDIA)
    millis_t MarlinUI::next_filament_display; // = 0
  #endif

  millis_t MarlinUI::next_button_update_ms; // = 0

  #if HAS_MARLINUI_U8GLIB
    bool MarlinUI::drawing_screen, MarlinUI::first_page; // = false
  #endif

  #if IS_DWIN_MARLINUI
    bool MarlinUI::did_first_redraw;
  #endif

  #if HAS_MEDIA

    #if MARLINUI_SCROLL_NAME
      static uint8_t filename_scroll_pos, filename_scroll_max;
    #endif

    const char * MarlinUI::scrolled_filename(CardReader &theCard, const uint8_t maxlen, const bool doScroll) {
      const char *outstr = theCard.longest_filename();
      if (theCard.longFilename[0]) {
        #if MARLINUI_SCROLL_NAME
          if (doScroll) {
            filename_scroll_max = _MAX(0, utf8_strlen(theCard.longFilename) - maxlen);
            if (filename_scroll_max) {
              // Ensure filename_scroll_pos isn't out of bounds even though it should never happen.
              if (filename_scroll_pos > filename_scroll_max) filename_scroll_pos = 0;
              // Advance byte position corresponding to filename_scroll_pos char position
              outstr += TERN(UTF_FILENAME_SUPPORT, utf8_byte_pos_by_char_num(outstr, filename_scroll_pos), filename_scroll_pos);
            }
          }
        #else
          // Simply cut off at maxlen
          const uint8_t chop = TERN(UTF_FILENAME_SUPPORT, utf8_byte_pos_by_char_num(theCard.longFilename, maxlen), maxlen);
          theCard.longFilename[_MAX(chop, LONG_FILENAME_LENGTH - 1)] = '\0';
        #endif
      }
      return outstr;
    }

  #endif

  #if HAS_MARLINUI_MENU
    #include "menu/menu.h"

    screenFunc_t MarlinUI::currentScreen; // Initialized in CTOR
    bool MarlinUI::screen_changed;

    #if ENABLED(ENCODER_RATE_MULTIPLIER)
      bool MarlinUI::encoder_multiplier_enabled;
    #endif

    #if ANY(REVERSE_MENU_DIRECTION, REVERSE_SELECT_DIRECTION)
      int8_t MarlinUI::encoderDirection = ENCODERBASE;
    #endif

    #if HAS_TOUCH_BUTTONS
      uint8_t MarlinUI::touch_buttons;
      uint16_t MarlinUI::repeat_delay;
    #endif

    #if ANY(AUTO_BED_LEVELING_UBL, G26_MESH_VALIDATION)

      bool MarlinUI::external_control; // = false

      void MarlinUI::wait_for_release() {
        while (button_pressed()) safe_delay(50);
        safe_delay(50);
      }

    #endif

    #if !HAS_GRAPHICAL_TFT

      void _wrap_string(uint8_t &col, uint8_t &row, const char * const string, read_byte_cb_t cb_read_byte, const bool wordwrap/*=false*/) {
        SETCURSOR(col, row);
        if (!string) return;

        auto _newline = [&col, &row]{
          col = 0; row++;                 // Move col to string len (plus space)
          SETCURSOR(0, row);              // Simulate carriage return
        };

        const uint8_t *p = (uint8_t*)string;
        lchar_t wc;
        if (wordwrap) {
          const uint8_t *wrd = nullptr;
          uint8_t c = 0;
          // find the end of the part
          for (;;) {
            if (!wrd) wrd = p;            // Get word start /before/ advancing
            p = get_utf8_value_cb(p, cb_read_byte, wc);
            const bool eol = !wc;         // zero ends the string
            // End or a break between phrases?
            if (eol || wc == ' ' || wc == '-' || wc == '+' || wc == '.' || wc == '\n') {
              const bool newline_after = wc == '\n';
              if (!c && (wc == ' ' || newline_after)) { if (wrd) wrd++; continue; } // collapse extra spaces
              // Past the right and the word is not too long?
              if (col + c > LCD_WIDTH && col >= (LCD_WIDTH) / 4) _newline(); // should it wrap?
              c += !eol;                  // +1 so the space will be printed
              col += c;                   // advance col to new position
              while (c) {                 // character countdown
                --c;                      // count down to zero
                wrd = get_utf8_value_cb(wrd, cb_read_byte, wc); // get characters again
                lcd_put_lchar(wc);        // character to the LCD
              }
              if (eol) break;             // all done!
              if (newline_after) _newline();
              wrd = nullptr;              // set up for next word
            }
            else c++;                     // count word characters
          }
        }
        else {
          for (;;) {
            p = get_utf8_value_cb(p, cb_read_byte, wc);
            if (!wc) break;
            lcd_put_lchar(wc);
            col++;
            if (col >= LCD_WIDTH) _newline();
          }
        }
      }

      void MarlinUI::draw_select_screen_prompt(FSTR_P const fpre, const char * const string/*=nullptr*/, FSTR_P const fsuf/*=nullptr*/) {
        const uint8_t plen = utf8_strlen_P(FTOP(fpre)), slen = fsuf ? utf8_strlen_P(FTOP(fsuf)) : 0;
        uint8_t col = 0, row = 0;
        if (!string && plen + slen <= LCD_WIDTH) {
          col = (LCD_WIDTH - plen - slen) / 2;
          row = LCD_HEIGHT > 3 ? 1 : 0;
        }
        if (LCD_HEIGHT >= 8) row = LCD_HEIGHT / 2 - 2;
        wrap_string_P(col, row, FTOP(fpre), true);
        if (string) {
          if (col) { col = 0; row++; } // Move to the start of the next line
          wrap_string(col, row, string);
        }
        if (fsuf) wrap_string_P(col, row, FTOP(fsuf));
      }

    #endif // !HAS_GRAPHICAL_TFT

  #endif // HAS_MARLINUI_MENU

  #if IS_RRW_KEYPAD && HAS_ENCODER_ACTION

    volatile uint8_t MarlinUI::keypad_buttons;

    #if HAS_MARLINUI_MENU && !HAS_ADC_BUTTONS

      void _reprapworld_keypad_move(const AxisEnum axis, const int16_t dir) {
        ui.manual_move.menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
        ui.encoderPosition = dir;
        switch (axis) {
          TERN_(HAS_X_AXIS, case X_AXIS:)
          TERN_(HAS_Y_AXIS, case Y_AXIS:)
          TERN_(HAS_Z_AXIS, case Z_AXIS:)
            lcd_move_axis(axis);
          default: break;
        }
      }

    #endif

    bool MarlinUI::handle_keypad() {

      #if HAS_ADC_BUTTONS

        #define ADC_MIN_KEY_DELAY 100
        if (keypad_buttons) {
          #if HAS_ENCODER_ACTION
            refresh(LCDVIEW_REDRAW_NOW);
            #if HAS_MARLINUI_MENU
              if (encoderDirection == -(ENCODERBASE)) { // HAS_ADC_BUTTONS forces REVERSE_MENU_DIRECTION, so this indicates menu navigation
                     if (RRK(EN_KEYPAD_UP))     encoderPosition += ENCODER_STEPS_PER_MENU_ITEM;
                else if (RRK(EN_KEYPAD_DOWN))   encoderPosition -= ENCODER_STEPS_PER_MENU_ITEM;
                else if (RRK(EN_KEYPAD_LEFT))   { MenuItem_back::action(); quick_feedback(); }
                else if (RRK(EN_KEYPAD_RIGHT))  { return_to_status(); quick_feedback(); }
              }
              else
            #endif
            {
              #if HAS_MARLINUI_MENU
                     if (RRK(EN_KEYPAD_UP))     encoderPosition -= epps;
                else if (RRK(EN_KEYPAD_DOWN))   encoderPosition += epps;
                else if (RRK(EN_KEYPAD_LEFT))   { MenuItem_back::action(); quick_feedback(); }
                else if (RRK(EN_KEYPAD_RIGHT))  encoderPosition = 0;
              #else
                     if (RRK(EN_KEYPAD_UP)   || RRK(EN_KEYPAD_LEFT))  encoderPosition -= epps;
                else if (RRK(EN_KEYPAD_DOWN) || RRK(EN_KEYPAD_RIGHT)) encoderPosition += epps;
              #endif
            }
          #endif
          next_button_update_ms = millis() + ADC_MIN_KEY_DELAY;
          return true;
        }

      #else // !HAS_ADC_BUTTONS

        static uint8_t keypad_debounce = 0;

        if (!RRK( EN_KEYPAD_F1    | EN_KEYPAD_F2
                | EN_KEYPAD_F3    | EN_KEYPAD_DOWN
                | EN_KEYPAD_RIGHT | EN_KEYPAD_MIDDLE
                | EN_KEYPAD_UP    | EN_KEYPAD_LEFT )
        ) {
          if (keypad_debounce > 0) keypad_debounce--;
        }
        else if (!keypad_debounce) {
          keypad_debounce = 2;

          const bool homed = all_axes_homed();

          #if HAS_MARLINUI_MENU

            if (RRK(EN_KEYPAD_MIDDLE))  goto_screen(menu_move);

            #if NONE(DELTA, Z_HOME_TO_MAX)
              if (RRK(EN_KEYPAD_F2))    _reprapworld_keypad_move(Z_AXIS,  1);
            #endif

            if (homed) {
              #if ANY(DELTA, Z_HOME_TO_MAX)
                if (RRK(EN_KEYPAD_F2))  _reprapworld_keypad_move(Z_AXIS,  1);
              #endif
              if (RRK(EN_KEYPAD_F3))    _reprapworld_keypad_move(Z_AXIS, -1);
              if (RRK(EN_KEYPAD_LEFT))  _reprapworld_keypad_move(X_AXIS, -1);
              if (RRK(EN_KEYPAD_RIGHT)) _reprapworld_keypad_move(X_AXIS,  1);
              if (RRK(EN_KEYPAD_DOWN))  _reprapworld_keypad_move(Y_AXIS,  1);
              if (RRK(EN_KEYPAD_UP))    _reprapworld_keypad_move(Y_AXIS, -1);
            }

          #endif // HAS_MARLINUI_MENU

          if (!homed && RRK(EN_KEYPAD_F1)) queue.inject_P(G28_STR);
          return true;
        }

      #endif // !HAS_ADC_BUTTONS

      return false;
    }

  #endif // IS_RRW_KEYPAD && HAS_ENCODER_ACTION

  /**
   * Status Screen
   *
   * This is very display-dependent, so the lcd implementation draws this.
   */

  #if HAS_BASIC_PROGRESS_BAR
    millis_t MarlinUI::progress_bar_ms; // = 0
    #if PROGRESS_MSG_EXPIRE > 0
      millis_t MarlinUI::expire_status_ms; // = 0
    #endif
  #endif

  void MarlinUI::status_screen() {

    #if HAS_BASIC_PROGRESS_BAR

      //
      // HD44780 implements the following message blinking and
      // message expiration because Status Line and Progress Bar
      // share the same line on the display.
      //

      #if DISABLED(PROGRESS_MSG_ONCE) || PROGRESS_MSG_EXPIRE > 0
        #define GOT_MS
        const millis_t ms = millis();
      #endif

      // If the message will blink rather than expire...
      #if DISABLED(PROGRESS_MSG_ONCE)
        if (ELAPSED(ms, progress_bar_ms + PROGRESS_BAR_MSG_TIME + PROGRESS_BAR_BAR_TIME))
          progress_bar_ms = ms;
      #endif

      #if PROGRESS_MSG_EXPIRE > 0

        // Handle message expire
        if (expire_status_ms) {

          // Expire the message if a job is active and the bar has ticks
          if (get_progress_percent() > 2 && !print_job_timer.isPaused()) {
            if (ELAPSED(ms, expire_status_ms)) {
              status_message.clear();
              expire_status_ms = 0;
            }
          }
          else {
            // Defer message expiration before bar appears
            // and during any pause (not just SD)
            expire_status_ms += LCD_UPDATE_INTERVAL;
          }
        }

      #endif // PROGRESS_MSG_EXPIRE

    #endif // HAS_BASIC_PROGRESS_BAR

    bool did_expire = status_reset_callback && (*status_reset_callback)();

    #if HAS_STATUS_MESSAGE_TIMEOUT
      #ifndef GOT_MS
        #define GOT_MS
        const millis_t ms = millis();
      #endif
      did_expire |= status_message_expire_ms && ELAPSED(ms, status_message_expire_ms);
    #endif

    if (did_expire) reset_status();

    #if HAS_MARLINUI_MENU
      if (use_click()) {
        #if ALL(FILAMENT_LCD_DISPLAY, HAS_MEDIA)
          pause_filament_display();
        #endif
        goto_screen(menu_main);
        reinit_lcd(); // Revive a noisy shared SPI LCD
        return;
      }

    #endif

    #if ENABLED(ULTIPANEL_FEEDMULTIPLY)

      const int16_t old_frm = feedrate_percentage;
            int16_t new_frm = old_frm + int16_t(encoderPosition);

      // Dead zone at 100% feedrate
      if (old_frm == 100) {
        if (int16_t(encoderPosition) > ENCODER_FEEDRATE_DEADZONE)
          new_frm -= ENCODER_FEEDRATE_DEADZONE;
        else if (int16_t(encoderPosition) < -(ENCODER_FEEDRATE_DEADZONE))
          new_frm += ENCODER_FEEDRATE_DEADZONE;
        else
          new_frm = old_frm;
      }
      else if ((old_frm < 100) == (new_frm > 100)) // Crossing 100? Stick at 100.
        new_frm = 100;

      LIMIT(new_frm, SPEED_EDIT_MIN, SPEED_EDIT_MAX);

      if (old_frm != new_frm) {
        feedrate_percentage = new_frm;
        encoderPosition = 0;
        #if ALL(HAS_SOUND, BEEP_ON_FEEDRATE_CHANGE)
          static millis_t next_beep;
          #ifndef GOT_MS
            const millis_t ms = millis();
          #endif
          if (ELAPSED(ms, next_beep)) {
            BUZZ(FEEDRATE_CHANGE_BEEP_DURATION, FEEDRATE_CHANGE_BEEP_FREQUENCY);
            next_beep = ms + 500UL;
          }
        #endif
      }

    #elif ENABLED(ULTIPANEL_FLOWPERCENT)

      const int16_t old_fp = planner.flow_percentage[active_extruder];
            int16_t new_fp = old_fp + int16_t(encoderPosition);

      // Dead zone at 100% flow percentage
      if (old_fp == 100) {
        if (int16_t(encoderPosition) > ENCODER_FEEDRATE_DEADZONE)
          new_fp -= ENCODER_FEEDRATE_DEADZONE;
        else if (int16_t(encoderPosition) < -(ENCODER_FEEDRATE_DEADZONE))
          new_fp += ENCODER_FEEDRATE_DEADZONE;
        else
          new_fp = old_fp;
      }
      else if ((old_fp < 100) == (new_fp > 100)) // Crossing 100? Stick at 100.
        new_fp = 100;

      LIMIT(new_fp, FLOW_EDIT_MIN, FLOW_EDIT_MAX);

      if (old_fp != new_fp) {
        planner.set_flow(active_extruder, new_fp);
        encoderPosition = 0;
      }

    #endif // ULTIPANEL_FLOWPERCENT

    draw_status_screen();
  }

  void MarlinUI::kill_screen(FSTR_P const lcd_error, FSTR_P const lcd_component) {
    init();
    status_printf(1, F(S_FMT ": " S_FMT), lcd_error, lcd_component);
    TERN_(HAS_MARLINUI_MENU, return_to_status());

    // RED ALERT. RED ALERT.
    #if ENABLED(PRINTER_EVENT_LEDS)
      leds.set_color(LEDColorRed());
      #ifdef NEOPIXEL_BKGD_INDEX_FIRST
        neo.set_background_color(255, 0, 0, 0);
        neo.show();
      #endif
    #endif

    draw_kill_screen();
  }

  void MarlinUI::quick_feedback(const bool clear_buttons/*=true*/) {
    wake_display(); // Wake the screen for any click sound

    TERN_(HAS_MARLINUI_MENU, refresh());

    #if HAS_ENCODER_ACTION
      if (clear_buttons)
        TERN_(HAS_ADC_BUTTONS, keypad_buttons =) buttons = 0;
      next_button_update_ms = millis() + 500;
    #else
      UNUSED(clear_buttons);
    #endif

    chirp();  // Buzz and wait. Is the delay needed for buttons to settle?

    #if HAS_CHIRP && HAS_MARLINUI_MENU
      #if HAS_BEEPER
        for (int8_t i = 5; i--;) { buzzer.tick(); delay(2); }
      #else
        delay(10);
      #endif
    #endif
  }

  #if HAS_MARLINUI_MENU

    ManualMove MarlinUI::manual_move{};

    millis_t ManualMove::start_time = 0;
    float ManualMove::menu_scale = 1;
    screenFunc_t ManualMove::screen_ptr;
    #if IS_KINEMATIC
      float ManualMove::offset = 0;
      xyze_pos_t ManualMove::all_axes_destination = { 0 };
      bool ManualMove::processing = false;
    #endif
    #if MULTI_E_MANUAL
      int8_t ManualMove::e_index = 0;
    #endif
    AxisEnum ManualMove::axis = NO_AXIS_ENUM;
    #if ENABLED(MANUAL_E_MOVES_RELATIVE)
      float ManualMove::e_origin = 0;
    #endif

    /**
     * If a manual move has been posted and its time has arrived, and if the planner
     * has a space for it, then add a linear move to current_position the planner.
     *
     * If any manual move needs to be interrupted, make sure to force a manual move
     * by setting manual_move.start_time to millis() after updating current_position.
     *
     * To post a manual move:
     *   - Update current_position to the new place you want to go.
     *   - Set manual_move.axis to an axis like X_AXIS. Use ALL_AXES_ENUM for diagonal moves.
     *   - Set manual_move.start_time to a point in the future (in ms) when the move should be done.
     *
     * For kinematic machines:
     *   - Set manual_move.offset to modify one axis and post the move.
     *     This is used to achieve more rapid stepping on kinematic machines.
     */
    void ManualMove::task() {

      if (processing) return;   // Prevent re-entry from idle() calls

      // Add a manual move to the queue?
      if (axis == NO_AXIS_ENUM || PENDING(millis(), start_time) || planner.is_full()) return;

      const feedRate_t fr_mm_s = (axis < LOGICAL_AXES) ? manual_feedrate_mm_s[axis] : PLANNER_XY_FEEDRATE_MM_S;

      /**
       * For a rotational axis apply the "inch" to "mm" conversion factor. This mimics behaviour of the G-code G1
       * (see get_destination_from_command). For moves involving only rotational axes, the planner will convert
       * back to the feedrate in degrees-per-time unit.
       */
      const feedRate_t fr = parser.axis_is_rotational(axis) && parser.using_inch_units() ? IN_TO_MM(fr_mm_s) : fr_mm_s;

      #if IS_KINEMATIC

        #if HAS_MULTI_EXTRUDER
          REMEMBER(ae, active_extruder);
          #if MULTI_E_MANUAL
            if (axis == E_AXIS) active_extruder = e_index;
          #endif
        #endif

        // Apply a linear offset to a single axis
        if (axis == ALL_AXES_ENUM)
          destination = all_axes_destination;
        else if (axis <= LOGICAL_AXES) {
          destination = current_position;
          destination[axis] += offset;
        }

        // Reset for the next move
        offset = 0;
        axis = NO_AXIS_ENUM;

        // DELTA and SCARA machines use segmented moves, which could fill the planner during the call to
        // move_to_destination. This will cause idle() to be called, which can then call this function while the
        // previous invocation is being blocked. Modifications to offset shouldn't be made while
        // processing is true or the planner will get out of sync.
        processing = true;
        prepare_internal_move_to_destination(fr);  // will set current_position from destination
        processing = false;

      #else

        // For Cartesian / Core motion simply move to the current_position
        planner.buffer_line(current_position, fr,
          TERN_(MULTI_E_MANUAL, axis == E_AXIS ? e_index :) active_extruder
        );

        //SERIAL_ECHOLNPGM("Add planner.move with Axis ", C(AXIS_CHAR(axis)), " at FR ", fr_mm_s);

        axis = NO_AXIS_ENUM;

      #endif
    }

    //
    // Tell ui.update() to start a move to current_position after a short delay.
    //
    void ManualMove::soon(const AxisEnum move_axis
      OPTARG(MULTI_E_MANUAL, const int8_t eindex/*=active_extruder*/)
    ) {
      TERN_(MULTI_E_MANUAL, if (move_axis == E_AXIS) e_index = eindex);
      start_time = millis() + (menu_scale < 0.99f ? 0UL : 250UL); // delay for bigger moves
      axis = move_axis;
      //SERIAL_ECHOLNPGM("Post Move with Axis ", C(AXIS_CHAR(axis)), " soon.");
    }

    #if ENABLED(AUTO_BED_LEVELING_UBL)

      void MarlinUI::external_encoder() {
        if (external_control && encoderDiff) {
          bedlevel.encoder_diff += encoderDiff; // Encoder for UBL G29 mesh editing
          encoderDiff = 0;                      // Hide encoder events from the screen handler
          refresh(LCDVIEW_REDRAW_NOW);          // ...but keep the refresh.
        }
      }

    #endif

  #endif // HAS_MARLINUI_MENU

  /**
   * Update the LCD, read encoder buttons, etc.
   *   - Read button states
   *   - Check the SD Card slot state
   *   - Act on RepRap World keypad input
   *   - Update the encoder position
   *   - Apply acceleration to the encoder position
   *   - Do refresh(LCDVIEW_CALL_REDRAW_NOW) on controller events
   *   - Reset the Info Screen timeout if there's any input
   *   - Update status indicators, if any
   *
   *   Run the current LCD menu handler callback function:
   *   - Call the handler only if lcdDrawUpdate != LCDVIEW_NONE
   *   - Before calling the handler, LCDVIEW_CALL_NO_REDRAW => LCDVIEW_NONE
   *   - Call the menu handler. Menu handlers should do the following:
   *     - If a value changes, set lcdDrawUpdate to LCDVIEW_REDRAW_NOW and draw the value
   *       (Encoder events automatically set lcdDrawUpdate for you.)
   *     - if (should_draw()) { redraw }
   *     - Before exiting the handler set lcdDrawUpdate to:
   *       - LCDVIEW_CLEAR_CALL_REDRAW to clear screen and set LCDVIEW_CALL_REDRAW_NEXT.
   *       - LCDVIEW_REDRAW_NOW to draw now (including remaining stripes).
   *       - LCDVIEW_CALL_REDRAW_NEXT to draw now and get LCDVIEW_REDRAW_NOW on the next loop.
   *       - LCDVIEW_CALL_NO_REDRAW to draw now and get LCDVIEW_NONE on the next loop.
   *     - NOTE: For graphical displays menu handlers may be called 2 or more times per loop,
   *             so don't change lcdDrawUpdate without considering this.
   *
   *   After the menu handler callback runs (or not):
   *   - Clear the LCD if lcdDrawUpdate == LCDVIEW_CLEAR_CALL_REDRAW
   *   - Update lcdDrawUpdate for the next loop (i.e., move one state down, usually)
   *
   * This function is only called from the main thread.
   */

  LCDViewAction MarlinUI::lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
  millis_t next_lcd_update_ms;

  inline bool can_encode() {
    return !BUTTON_PRESSED(ENC_EN); // Update encoder only when ENC_EN is not LOW (pressed)
  }

  void MarlinUI::update() {

    static uint16_t max_display_update_time = 0;
    const millis_t ms = millis();

    TERN_(HAS_LED_POWEROFF_TIMEOUT, leds.update_timeout(powerManager.psu_on));

    #if HAS_MARLINUI_MENU

      // Handle any queued Move Axis motion
      manual_move.task();

      // Update button states for button_pressed(), etc.
      // If the state changes the next update may be delayed 300-500ms.
      update_buttons();

      // If the action button is pressed...
      static bool wait_for_unclick; // = false

      // Set lcd_clicked for most clicks.
      // Ignore the click when clearing wait_for_user or waking the screen.
      auto do_click = [&]{
        wait_for_unclick = true;
        lcd_clicked = !wait_for_user && !display_is_asleep();
        wait_for_user = false;
        quick_feedback();
      };

      #if HAS_TOUCH_BUTTONS
        if (touch_buttons) {
          reset_status_timeout(ms);
          if (touch_buttons & (EN_A | EN_B)) {              // Menu arrows, in priority
            if (ELAPSED(ms, next_button_update_ms)) {
              encoderDiff = (ENCODER_STEPS_PER_MENU_ITEM) * epps * encoderDirection;
              if (touch_buttons & EN_A) encoderDiff *= -1;
              TERN_(AUTO_BED_LEVELING_UBL, external_encoder());
              next_button_update_ms = ms + repeat_delay;    // Assume the repeat delay
              if (!wait_for_unclick) {
                next_button_update_ms += 250;               // Longer delay on first press
                wait_for_unclick = true;                    // Avoid Back/Select click while repeating
                chirp();
              }
            }
          }
          else if (!wait_for_unclick && (buttons & EN_C))   // OK button, if not waiting for a debounce release:
            do_click();
        }
        // keep wait_for_unclick value
      #endif

      if (!touch_buttons) {
        // Integrated LCD click handling via button_pressed
        if (!external_control && button_pressed()) {
          if (!wait_for_unclick) do_click();              // Handle the click
        }
        else
          wait_for_unclick = false;
      }

      if (LCD_BACK_CLICKED()) {
        quick_feedback();
        goto_previous_screen();
      }

    #endif // HAS_MARLINUI_MENU

    const bool lcd_update_ms_elapsed = ELAPSED(ms, next_lcd_update_ms);
    if (lcd_update_ms_elapsed) {
      next_lcd_update_ms = ms + LCD_UPDATE_INTERVAL;

      #if HAS_TOUCH_BUTTONS
        if (on_status_screen()) next_lcd_update_ms += (LCD_UPDATE_INTERVAL) * 2;

        TERN_(HAS_ENCODER_ACTION, touch_buttons = touchBt.read_buttons());
      #endif

      TERN_(LCD_HAS_STATUS_INDICATORS, update_indicators());

      #if HAS_ENCODER_ACTION
        TERN_(HAS_SLOW_BUTTONS, slow_buttons = read_slow_buttons()); // Buttons that take too long to read in interrupt context

        if (TERN0(IS_RRW_KEYPAD, handle_keypad()))
          reset_status_timeout(ms);

        static int8_t lastEncoderDiff;
        if (lastEncoderDiff != encoderDiff) wake_display();
        lastEncoderDiff = encoderDiff;

        const uint8_t abs_diff = ABS(encoderDiff);
        const bool encoderPastThreshold = (abs_diff >= epps);
        if (encoderPastThreshold && TERN1(IS_TFTGLCD_PANEL, !external_control)) {

          int32_t encoder_multiplier = 1;

          #if ALL(HAS_MARLINUI_MENU, ENCODER_RATE_MULTIPLIER)

            if (encoder_multiplier_enabled) {
              // Note that the rate is always calculated between two passes through the
              // loop and that the abs of the encoderDiff value is tracked.
              static millis_t encoder_mult_prev_ms = 0;
              const float encoderStepRate = ((float(abs_diff) / float(epps)) * 1000.0f) / float(ms - encoder_mult_prev_ms);
              encoder_mult_prev_ms = ms;

              if (ENCODER_100X_STEPS_PER_SEC > 0 && encoderStepRate >= ENCODER_100X_STEPS_PER_SEC)
                encoder_multiplier = 100;
              else if (ENCODER_10X_STEPS_PER_SEC > 0 && encoderStepRate >= ENCODER_10X_STEPS_PER_SEC)
                encoder_multiplier = 10;
              else if (ENCODER_5X_STEPS_PER_SEC > 0 && encoderStepRate >= ENCODER_5X_STEPS_PER_SEC)
                encoder_multiplier = 5;

              // Enable to output the encoder steps per second value
              //#define ENCODER_RATE_MULTIPLIER_DEBUG
              #if ENABLED(ENCODER_RATE_MULTIPLIER_DEBUG)
                SERIAL_ECHO_MSG(
                  "Enc Step Rate: ", encoderStepRate,
                  "  Mult: ", encoder_multiplier,
                  "  5X Steps: ", ENCODER_5X_STEPS_PER_SEC,
                  "  10X Steps: ", ENCODER_10X_STEPS_PER_SEC,
                  "  100X Steps: ", ENCODER_100X_STEPS_PER_SEC
                );
              #endif
            }

          #endif // ENCODER_RATE_MULTIPLIER

          const int8_t fullSteps = encoderDiff / epps;
          if (fullSteps != 0) {
            encoderDiff -= fullSteps * epps;
            if (can_encode() && !lcd_clicked)
              encoderPosition += (fullSteps * encoder_multiplier);
          }
        }

        if (encoderPastThreshold || lcd_clicked) {
          reset_status_timeout(ms);

          #if HAS_BACKLIGHT_TIMEOUT
            refresh_backlight_timeout();
          #elif HAS_DISPLAY_SLEEP
            refresh_screen_timeout();
          #endif

          refresh(LCDVIEW_REDRAW_NOW);
          TERN_(HAS_MARLINUI_U8GLIB, drawing_screen = false);
          #if MARLINUI_SCROLL_NAME
            filename_scroll_max = 0;
            filename_scroll_pos = 0;
            lcd_status_update_delay = 9;
          #endif

          TERN_(HAS_LED_POWEROFF_TIMEOUT, if (!powerManager.psu_on) leds.reset_timeout(ms));

        } // encoder activity

      #endif // HAS_ENCODER_ACTION

      // This runs every ~100ms when idling often enough.
      // Instead of tracking changes just redraw the Status Screen once per second.
      if (on_status_screen() && !lcd_status_update_delay--) {
        lcd_status_update_delay = TERN(HAS_MARLINUI_U8GLIB, 12, 9);
        if (max_display_update_time) max_display_update_time--;  // Be sure never go to a very big number
        refresh(LCDVIEW_REDRAW_NOW);
        TERN_(HAS_MARLINUI_U8GLIB, drawing_screen = false);
      }

      #if MARLINUI_SCROLL_NAME
        // If scrolling of long file names is enabled and we are in the sd card menu,
        // cause a refresh to occur until all the text has scrolled into view.
        if (currentScreen == menu_media && filename_scroll_max && !lcd_status_update_delay--) {
          lcd_status_update_delay = ++filename_scroll_pos >= filename_scroll_max ? 12 : 4; // Long delay at end and start
          if (filename_scroll_pos > filename_scroll_max) filename_scroll_pos = 0;
          refresh(LCDVIEW_REDRAW_NOW);
          TERN_(HAS_MARLINUI_U8GLIB, drawing_screen = false);
          reset_status_timeout(ms);
        }
      #endif
    }

    if (lcd_update_ms_elapsed || drawing_screen) {
      // Then we want to use only 50% of the time
      const uint16_t bbr2 = planner.block_buffer_runtime() >> 1;

      if ((should_draw() || drawing_screen) && (!bbr2 || bbr2 > max_display_update_time)) {

        // Change state of drawing flag between screen updates
        if (!drawing_screen) switch (lcdDrawUpdate) {
          case LCDVIEW_CALL_NO_REDRAW:
            refresh(LCDVIEW_NONE);
            break;
          case LCDVIEW_CLEAR_CALL_REDRAW:
          case LCDVIEW_CALL_REDRAW_NEXT:
            refresh(LCDVIEW_REDRAW_NOW);
          case LCDVIEW_REDRAW_NOW:        // set above, or by a handler through LCDVIEW_CALL_REDRAW_NEXT
          case LCDVIEW_NONE:
            break;
        } // switch

        TERN_(HAS_ADC_BUTTONS, keypad_buttons = 0);

        #if HAS_MARLINUI_U8GLIB
          #if ENABLED(LIGHTWEIGHT_UI)
            const bool in_status = on_status_screen(),
                       do_u8g_loop = !in_status;
            lcd_in_status(in_status);
            if (in_status) status_screen();
          #else
            constexpr bool do_u8g_loop = true;
          #endif

          if (do_u8g_loop) {
            if (!drawing_screen) {                // If not already drawing pages
              u8g.firstPage();                    // Start the first page
              drawing_screen = first_page = true; // Flag as drawing pages
            }
            set_font(FONT_MENU);                  // Setup font for every page draw
            u8g.setColorIndex(1);                 // And reset the color
            run_current_screen();                 // Draw and process the current screen
            first_page = false;

            // The screen handler can clear drawing_screen for an action that changes the screen.
            // If still drawing and there's another page, update max-time and return now.
            // The nextPage will already be set up on the next call.
            if (drawing_screen && (drawing_screen = u8g.nextPage())) {
              if (on_status_screen())
                NOLESS(max_display_update_time, millis() - ms);
              return;
            }
          }
        #else
          run_current_screen();

          // Apply all DWIN drawing after processing
          TERN_(IS_DWIN_MARLINUI, dwinUpdateLCD());
        #endif

        TERN_(HAS_MARLINUI_MENU, lcd_clicked = false);

        // Keeping track of the longest time for an individual LCD update.
        // Used to do screen throttling when the planner starts to fill up.
        if (on_status_screen())
          NOLESS(max_display_update_time, millis() - ms);
      }

      #if HAS_SCREEN_TIMEOUT
        // Return to Status Screen after a timeout
        if (on_status_screen() || defer_return_to_status)
          reset_status_timeout(ms);
        else if (ELAPSED(ms, return_to_status_ms))
          return_to_status();
      #endif

      #if HAS_BACKLIGHT_TIMEOUT
        if (backlight_off_ms && ELAPSED(ms, backlight_off_ms)) {
          #ifdef NEOPIXEL_BKGD_INDEX_FIRST
            neo.set_background_off();
            neo.show();
          #elif PIN_EXIST(LCD_BACKLIGHT)
            WRITE(LCD_BACKLIGHT_PIN, LOW); // Backlight off
          #endif
          backlight_off_ms = 0;
        }
      #elif HAS_DISPLAY_SLEEP
        if (screen_timeout_ms && ELAPSED(ms, screen_timeout_ms))
          sleep_display();
      #endif

      // Change state of drawing flag between screen updates
      if (!drawing_screen) switch (lcdDrawUpdate) {
        case LCDVIEW_CLEAR_CALL_REDRAW:
          clear_for_drawing(); break;
        case LCDVIEW_REDRAW_NOW:
          refresh(LCDVIEW_NONE);
        case LCDVIEW_NONE:
        case LCDVIEW_CALL_REDRAW_NEXT:
        case LCDVIEW_CALL_NO_REDRAW:
        default: break;
      } // switch
    }

    TERN_(HAS_GRAPHICAL_TFT, tft_idle());
  }

  #if HAS_ADC_BUTTONS

    typedef struct {
      raw_adc_t ADCKeyValueMin, ADCKeyValueMax;
      uint8_t  ADCKeyNo;
    } _stADCKeypadTable_;

    #ifndef ADC_BUTTONS_VALUE_SCALE
      #define ADC_BUTTONS_VALUE_SCALE       1.0  // for the power voltage equal to the reference voltage
    #endif
    #ifndef ADC_BUTTONS_R_PULLUP
      #define ADC_BUTTONS_R_PULLUP          4.7  // common pull-up resistor in the voltage divider
    #endif
    #ifndef ADC_BUTTONS_LEFT_R_PULLDOWN
      #define ADC_BUTTONS_LEFT_R_PULLDOWN   0.47 // pull-down resistor for LEFT button voltage divider
    #endif
    #ifndef ADC_BUTTONS_RIGHT_R_PULLDOWN
      #define ADC_BUTTONS_RIGHT_R_PULLDOWN  4.7  // pull-down resistor for RIGHT button voltage divider
    #endif
    #ifndef ADC_BUTTONS_UP_R_PULLDOWN
      #define ADC_BUTTONS_UP_R_PULLDOWN     1.0  // pull-down resistor for UP button voltage divider
    #endif
    #ifndef ADC_BUTTONS_DOWN_R_PULLDOWN
      #define ADC_BUTTONS_DOWN_R_PULLDOWN   10.0 // pull-down resistor for DOWN button voltage divider
    #endif
    #ifndef ADC_BUTTONS_MIDDLE_R_PULLDOWN
      #define ADC_BUTTONS_MIDDLE_R_PULLDOWN 2.2  // pull-down resistor for MIDDLE button voltage divider
    #endif

    // Calculate the ADC value for the voltage divider with specified pull-down resistor value
    #define ADC_BUTTON_VALUE(r)  raw_adc_t(HAL_ADC_RANGE * (ADC_BUTTONS_VALUE_SCALE) * r / (r + ADC_BUTTONS_R_PULLUP))

    static constexpr raw_adc_t adc_button_tolerance = HAL_ADC_RANGE *   25 / 1024,
                                   adc_other_button = raw_adc_t(uint32_t(HAL_ADC_RANGE * 1000UL) / 1024UL);
    static const _stADCKeypadTable_ stADCKeyTable[] PROGMEM = {
      // VALUE_MIN, VALUE_MAX, KEY
      { adc_other_button, HAL_ADC_RANGE, 1 + BLEN_KEYPAD_F1     }, // F1
      { adc_other_button, HAL_ADC_RANGE, 1 + BLEN_KEYPAD_F2     }, // F2
      { adc_other_button, HAL_ADC_RANGE, 1 + BLEN_KEYPAD_F3     }, // F3
      {  ADC_BUTTON_VALUE(ADC_BUTTONS_LEFT_R_PULLDOWN)   - adc_button_tolerance,
         ADC_BUTTON_VALUE(ADC_BUTTONS_LEFT_R_PULLDOWN)   + adc_button_tolerance, 1 + BLEN_KEYPAD_LEFT   }, // LEFT  ( 272 ...  472)
      {  ADC_BUTTON_VALUE(ADC_BUTTONS_RIGHT_R_PULLDOWN)  - adc_button_tolerance,
         ADC_BUTTON_VALUE(ADC_BUTTONS_RIGHT_R_PULLDOWN)  + adc_button_tolerance, 1 + BLEN_KEYPAD_RIGHT  }, // RIGHT (1948 ... 2148)
      {  ADC_BUTTON_VALUE(ADC_BUTTONS_UP_R_PULLDOWN)     - adc_button_tolerance,
         ADC_BUTTON_VALUE(ADC_BUTTONS_UP_R_PULLDOWN)     + adc_button_tolerance, 1 + BLEN_KEYPAD_UP     }, // UP    ( 618 ...  818)
      {  ADC_BUTTON_VALUE(ADC_BUTTONS_DOWN_R_PULLDOWN)   - adc_button_tolerance,
         ADC_BUTTON_VALUE(ADC_BUTTONS_DOWN_R_PULLDOWN)   + adc_button_tolerance, 1 + BLEN_KEYPAD_DOWN   }, // DOWN  (2686 ... 2886)
      {  ADC_BUTTON_VALUE(ADC_BUTTONS_MIDDLE_R_PULLDOWN) - adc_button_tolerance,
         ADC_BUTTON_VALUE(ADC_BUTTONS_MIDDLE_R_PULLDOWN) + adc_button_tolerance, 1 + BLEN_KEYPAD_MIDDLE }, // ENTER (1205 ... 1405)
    };

    uint8_t get_ADC_keyValue() {
      if (thermalManager.ADCKey_count >= 16) {
        const raw_adc_t currentkpADCValue = thermalManager.current_ADCKey_raw;
        thermalManager.current_ADCKey_raw = HAL_ADC_RANGE;
        thermalManager.ADCKey_count = 0;
        if (currentkpADCValue < adc_other_button)
          for (uint8_t i = 0; i < ADC_KEY_NUM; ++i) {
            const raw_adc_t lo = pgm_read_word(&stADCKeyTable[i].ADCKeyValueMin),
                            hi = pgm_read_word(&stADCKeyTable[i].ADCKeyValueMax);
            if (WITHIN(currentkpADCValue, lo, hi)) return pgm_read_byte(&stADCKeyTable[i].ADCKeyNo);
          }
      }
      return 0;
    }

  #endif // HAS_ADC_BUTTONS

  #if HAS_ENCODER_ACTION

    /**
     * Read encoder buttons from the hardware registers
     * Warning: This function is called from interrupt context!
     */
    void MarlinUI::update_buttons() {
      const millis_t now = millis();

      #if HAS_MARLINUI_ENCODER

        const int8_t delta = get_encoder_delta(now);
        if (delta) {
          encoderDiff += delta * encoderDirection;
          #if ALL(HAS_MARLINUI_MENU, AUTO_BED_LEVELING_UBL)
            external_encoder();
          #endif
        }

      #endif

      if (PENDING(now, next_button_update_ms)) return;

      #if HAS_DIGITAL_BUTTONS

        uint8_t newbuttons = 0;
        #if ANY_BUTTON(ENC, BACK)
          if (can_encode() && BUTTON_PRESSED(ENC)) newbuttons |= EN_C;
          if (BUTTON_PRESSED(BACK))                newbuttons |= EN_D;
        #endif

        //
        // Directional buttons
        //
        #if ANY_BUTTON(UP, DOWN, LEFT, RIGHT)

          const int8_t pulses = epps * encoderDirection;

          if (BUTTON_PRESSED(UP)) {
            encoderDiff = (ENCODER_STEPS_PER_MENU_ITEM) * pulses;
            next_button_update_ms = now + 300;
          }
          else if (BUTTON_PRESSED(DOWN)) {
            encoderDiff = -(ENCODER_STEPS_PER_MENU_ITEM) * pulses;
            next_button_update_ms = now + 300;
          }
          else if (BUTTON_PRESSED(LEFT)) {
            encoderDiff = -pulses;
            next_button_update_ms = now + 300;
          }
          else if (BUTTON_PRESSED(RIGHT)) {
            encoderDiff = pulses;
            next_button_update_ms = now + 300;
          }

        #endif // UP || DOWN || LEFT || RIGHT

        buttons = (newbuttons | TERN0(HAS_SLOW_BUTTONS, slow_buttons)
          #if ALL(HAS_TOUCH_BUTTONS, HAS_ENCODER_ACTION)
            | (touch_buttons & TERN(HAS_MARLINUI_ENCODER, ~(EN_A | EN_B), 0xFF))
          #endif
        );

      #elif HAS_ADC_BUTTONS

        buttons = 0;

      #endif

      #if HAS_ADC_BUTTONS
        if (keypad_buttons == 0) {
          const uint8_t b = get_ADC_keyValue();
          if (WITHIN(b, 1, 8)) keypad_buttons = _BV(b - 1);
        }
      #endif

      #if HAS_SHIFT_ENCODER
        /**
         * Set up Rotary Encoder bit values (for two pin encoders to indicate movement).
         * These values are independent of which pins are used for EN_A / EN_B indications.
         * The rotary encoder part is also independent of the LCD chipset.
         */
        uint8_t val = 0;
        WRITE(SHIFT_LD_PIN, LOW);
        WRITE(SHIFT_LD_PIN, HIGH);
        for (uint8_t i = 0; i < 8; ++i) {
          val >>= 1;
          if (READ(SHIFT_OUT_PIN)) SBI(val, 7);
          WRITE(SHIFT_CLK_PIN, HIGH);
          WRITE(SHIFT_CLK_PIN, LOW);
        }
        TERN(REPRAPWORLD_KEYPAD, keypad_buttons, buttons) = ~val;
      #endif

      #if IS_TFTGLCD_PANEL
        next_button_update_ms = now + (LCD_UPDATE_INTERVAL / 2);
        buttons = slow_buttons;
        TERN_(AUTO_BED_LEVELING_UBL, external_encoder());
      #endif

    } // update_buttons

  #endif // HAS_ENCODER_ACTION

#endif // HAS_WIRED_LCD

#if MARLINUI_ENCODER_DELTA

  #define ENCODER_DEBOUNCE_MS 2

  /**
   * Get the encoder delta (-2 -1 0 +1 +2) since the last call, reading the live encoder state.
   * Pins may be debounced to filter noise.
   */
  int8_t MarlinUI::get_encoder_delta(const millis_t &now/*=millis()*/) {

    typedef struct { bool a:1, b:1; } enc_t;

    const enc_t live_enc = { BUTTON_PRESSED(EN1), BUTTON_PRESSED(EN2) };

    #if ENCODER_DEBOUNCE_MS

      static enc_t enc;
      static enc_t old_live;

      static millis_t en_A_bounce_ms;
      if (old_live.a != live_enc.a) en_A_bounce_ms = now + (ENCODER_DEBOUNCE_MS);
      else if (ELAPSED(now, en_A_bounce_ms)) enc.a = live_enc.a;

      static millis_t en_B_bounce_ms;
      if (old_live.b != live_enc.b) en_B_bounce_ms = now + (ENCODER_DEBOUNCE_MS);
      else if (ELAPSED(now, en_B_bounce_ms)) enc.b = live_enc.b;

      old_live = live_enc;

    #else

      const enc_t &enc = live_enc;

    #endif

    static uint8_t old_pos;
    const uint8_t pos = (enc.a ^ enc.b) | (enc.a << 1); // 0:00  1:10  2:11  3:01
    int8_t delta = 0;
    if (pos != old_pos) {
      delta = (pos - old_pos + 4 + 1) % 4 - 1;
      old_pos = pos;

      static int8_t last_dir;
      if (delta == 2) delta = last_dir * 2;
      else last_dir = delta;
    }
    return delta;

  } // get_encoder_delta

#endif // MARLINUI_ENCODER_DELTA

void MarlinUI::completion_feedback(const bool good/*=true*/) {
  wake_display(); // Wake the screen for all audio feedback
  #if HAS_SOUND
    if (good) OKAY_BUZZ(); else ERR_BUZZ();
  #endif
}

void MarlinUI::host_notify_P(PGM_P const pstr) {
  TERN_(HOST_STATUS_NOTIFICATIONS, hostui.notify_P(pstr));
}
void MarlinUI::host_notify(const char * const cstr) {
  TERN_(HOST_STATUS_NOTIFICATIONS, hostui.notify(cstr));
}

#include <stdarg.h>

#if HAS_STATUS_MESSAGE

  #if ENABLED(EXTENSIBLE_UI)
    #include "extui/ui_api.h"
  #endif

  /**
   * Reset the status message
   */
  void MarlinUI::reset_status(const bool no_welcome) {
    FSTR_P msg;
    if (printingIsPaused())
      msg = GET_TEXT_F(MSG_PRINT_PAUSED);
    #if HAS_MEDIA
      else if (IS_SD_PRINTING())
        return set_status_no_expire(card.longest_filename());
    #endif
    else if (print_job_timer.isRunning())
      msg = GET_TEXT_F(MSG_PRINTING);

    #if SERVICE_INTERVAL_1 > 0
      else if (print_job_timer.needsService(1)) msg = F("> " SERVICE_NAME_1 "!");
    #endif
    #if SERVICE_INTERVAL_2 > 0
      else if (print_job_timer.needsService(2)) msg = F("> " SERVICE_NAME_2 "!");
    #endif
    #if SERVICE_INTERVAL_3 > 0
      else if (print_job_timer.needsService(3)) msg = F("> " SERVICE_NAME_3 "!");
    #endif

    else if (!no_welcome) msg = GET_TEXT_F(WELCOME_MSG);

    else if (ENABLED(STATUS_DO_CLEAR_EMPTY))
      msg = F("");
    else
      return;

    set_min_status(msg);
  }

  /**
   * Try to set the alert level.
   * @param level Alert level. Negative to ignore and reset the level. Non-zero never expires.
   * @return      TRUE if the level could NOT be set.
   */
  bool MarlinUI::set_alert_level(int8_t &level) {
    if (level < 0) level = alert_level = 0;
    if (level < alert_level) return true;
    alert_level = level;
    return false;
  }

  /**
   * @brief Set Status with a C- or P-string and alert level.
   *
   * @param ustr  A C- or P-string, according to pgm.
   * @param level Alert level. Negative to ignore and reset the level. Non-zero never expires.
   * @param pgm   Program string flag. Only relevant on AVR.
   */
  void MarlinUI::_set_status_and_level(const char * const ustr, int8_t level, const bool pgm) {
    if (set_alert_level(level)) return;

    pgm ? host_notify_P(ustr) : host_notify(ustr);

    MString<30> msg;
    pgm ? msg.set_P(ustr) : msg.set(ustr);
    status_message.set(&msg).utrunc(MAX_MESSAGE_LENGTH);

    finish_status(level > 0); // Persist if the status has a level
  }

  /**
   * @brief Set Status with a C- or P-string and persistence flag.
   *
   * @param ustr    A C- or P-string, according to pgm.
   * @param persist Don't expire (Requires STATUS_EXPIRE_SECONDS) - and set alert level to 1.
   * @param pgm     Program string flag. Only relevant on AVR.
   */
  void MarlinUI::_set_status(const char * const ustr, const bool persist, const bool pgm) {
    if (alert_level) return;

    pgm ? host_notify_P(ustr) : host_notify(ustr);

    // Remove the last partial Unicode glyph, if any
    (pgm ? status_message.set_P(ustr) : status_message.set(ustr)).utrunc(MAX_MESSAGE_LENGTH);

    finish_status(persist);
  }

  /**
   * @brief Set Alert with a C- or P-string and alert level.
   *
   * @param ustr  A C- or P-string, according to pgm.
   * @param level Alert level. Negative to ignore and reset the level. Non-zero never expires.
   * @param pgm   Program string flag. Only relevant on AVR.
   */
  void MarlinUI::_set_alert(const char * const ustr, const int8_t level, const bool pgm) {
    pgm ? set_status_and_level_P(ustr, level) : set_status_and_level(ustr, level);
    wake_display();
    TERN_(HAS_MARLINUI_MENU, return_to_status());
  }

  /**
   * @brief Set a status with a format string and parameters.
   *
   * @param pfmt    A constant format P-string
   */
  void MarlinUI::status_printf_P(int8_t level, PGM_P const pfmt, ...) {
    if (set_alert_level(level)) return;

    va_list args;
    va_start(args, pfmt);
    vsnprintf_P(status_message, MAX_MESSAGE_LENGTH, pfmt, args);
    va_end(args);

    host_notify(status_message);

    finish_status(level > 0);
  }

  void MarlinUI::finish_status(const bool persist) {

    UNUSED(persist);

    set_status_reset_fn();

    TERN_(HAS_STATUS_MESSAGE_TIMEOUT, status_message_expire_ms = persist ? 0 : millis() + (STATUS_MESSAGE_TIMEOUT_SEC) * 1000UL);

    #if HAS_WIRED_LCD

      #if HAS_BASIC_PROGRESS_BAR || ALL(FILAMENT_LCD_DISPLAY, HAS_MEDIA)
        const millis_t ms = millis();
      #endif

      #if HAS_BASIC_PROGRESS_BAR
        progress_bar_ms = ms;
        #if PROGRESS_MSG_EXPIRE > 0
          expire_status_ms = persist ? 0 : ms + PROGRESS_MSG_EXPIRE;
        #endif
      #endif

      #if ALL(FILAMENT_LCD_DISPLAY, HAS_MEDIA)
        pause_filament_display(ms); // Show status message for 5s
      #endif

    #endif

    TERN_(STATUS_MESSAGE_SCROLLING, reset_status_scroll());

    TERN_(EXTENSIBLE_UI, ExtUI::onStatusChanged(status_message));
    TERN_(DWIN_CREALITY_LCD, dwinStatusChanged(status_message));
    TERN_(DWIN_CREALITY_LCD_JYERSUI, jyersDWIN.updateStatus(status_message));
  }

  #if ENABLED(STATUS_MESSAGE_SCROLLING)

    void MarlinUI::advance_status_scroll() {
      // Advance by one UTF8 code-word
      if (status_scroll_offset < status_message.glyphs())
        while (!START_OF_UTF8_CHAR(status_message[++status_scroll_offset]));
      else
        reset_status_scroll();
    }

    char* MarlinUI::status_and_len(uint8_t &len) {
      char *out = &status_message + status_scroll_offset;
      len = utf8_strlen(out);
      return out;
    }

  #endif

#else // !HAS_STATUS_MESSAGE

  //
  // Send the status line as a host notification
  //
  void MarlinUI::_set_status(const char * const ustr, const bool, const bool pgm) {
    host_notify(ustr);
  }
  void MarlinUI::_set_alert(const char * const ustr, const int8_t, const bool pgm) {
    host_notify(ustr);
  }
  void MarlinUI::_set_status_and_level(const char * const ustr, const int8_t=0, const bool pgm) {
    pgm ? host_notify_P(ustr) : host_notify(ustr);
  }
  void MarlinUI::status_printf_P(int8_t level, PGM_P const pfmt, ...) {
    MString<30> msg;

    va_list args;
    va_start(args, pfmt);
    vsnprintf_P(&msg, 30, pfmt, args);
    va_end(args);

    host_notify(msg);
  }

#endif // !HAS_STATUS_MESSAGE

#if HAS_DISPLAY

  #if HAS_MEDIA
    extern bool wait_for_user, wait_for_heatup;
  #endif

  void MarlinUI::abort_print() {
    #if HAS_MEDIA
      wait_for_heatup = wait_for_user = false;
      if (IS_SD_PRINTING())
        card.abortFilePrintSoon();
      else if (card.isMounted())
        card.closefile();
    #endif
    #ifdef ACTION_ON_CANCEL
      hostui.cancel();
    #endif
    print_job_timer.stop();
    TERN_(HOST_PROMPT_SUPPORT, hostui.prompt_open(PROMPT_INFO, F("UI Aborted"), FPSTR(DISMISS_STR)));
    LCD_MESSAGE(MSG_PRINT_ABORTED);
    TERN_(HAS_MARLINUI_MENU, return_to_status());
    TERN_(DWIN_LCD_PROUI, hmiFlag.abort_flag = true);
  }

  #if ALL(HAS_MARLINUI_MENU, PSU_CONTROL)

    void MarlinUI::poweroff() {
      queue.inject(F("M81" TERN_(POWER_OFF_WAIT_FOR_COOLDOWN, "S")));
      return_to_status();
    }

  #endif

  #if ENABLED(FLOWMETER_SAFETY)
    void MarlinUI::flow_fault() {
      LCD_ALERTMESSAGE(MSG_FLOWMETER_FAULT);
      BUZZ(1000, 440);
      TERN_(HAS_MARLINUI_MENU, return_to_status());
    }
  #endif

  /**
   * - Synchronize and put up a wait screen
   * - Wake the display
   * - Tell the host to present a "Resume" option
   * - Print "Paused" on the display
   * - One of three possibilities follow:
   *   - If Parking, Print a message, send M125 P (which parks and waits), then M24 which will resume SD printing.
   *   - If media exists call M25 to pause the SD print.
   *   - For a host-only printer tell the host to pause the print in progress.
   */
  void MarlinUI::pause_print() {
    #if HAS_MARLINUI_MENU
      synchronize(GET_TEXT_F(MSG_PAUSING));
      defer_status_screen();
    #endif

    wake_display();

    TERN_(HOST_PROMPT_SUPPORT, hostui.prompt_open(PROMPT_PAUSE_RESUME, F("UI Pause"), F("Resume")));

    LCD_MESSAGE(MSG_PRINT_PAUSED);

    #if ENABLED(PARK_HEAD_ON_PAUSE)
      pause_show_message(PAUSE_MESSAGE_PARKING, PAUSE_MODE_PAUSE_PRINT); // Show message immediately to let user know about pause in progress
      queue.inject(F("M25 P\nM24"));
    #elif HAS_MEDIA
      queue.inject(F("M25"));
    #elif defined(ACTION_ON_PAUSE)
      hostui.pause();
    #endif
  }

  void MarlinUI::resume_print() {
    reset_status();
    TERN_(PARK_HEAD_ON_PAUSE, wait_for_heatup = wait_for_user = false);
    TERN_(HAS_MEDIA, if (IS_SD_PAUSED()) queue.inject_P(M24_STR));
    #ifdef ACTION_ON_RESUME
      hostui.resume();
    #endif
    print_job_timer.start(); // Also called by M24
  }

  #if HAS_TOUCH_BUTTONS

    /**
     * Screen Click
     *  - On menu screens move directly to the touched item
     *  - On menu screens, right side (last 3 cols) acts like a scroll - half up => prev page, half down = next page
     *  - On select screens (and others) touch the Left Half for ←, Right Half for →
     *  - On edit screens, touch Top Half for ↑, Bottom Half for ↓
     */
    void MarlinUI::screen_click(const uint8_t row, const uint8_t col, const uint8_t, const uint8_t) {
      const millis_t now = millis();
      if (PENDING(now, next_button_update_ms)) return;
      next_button_update_ms = now + repeat_delay;    // Assume the repeat delay
      const int8_t xdir = col < (LCD_WIDTH ) / 2 ? -1 : 1,
                   ydir = row < (LCD_HEIGHT) / 2 ? -1 : 1;
      if (on_edit_screen)
        encoderDiff = epps * ydir;
      else if (screen_items > 0) {
        // Last 5 cols act as a scroll :-)
        if (col > (LCD_WIDTH) - 5)
          // 2 * LCD_HEIGHT to scroll to bottom of next page. (LCD_HEIGHT would only go 1 item down.)
          encoderDiff = epps * (encoderLine - encoderTopLine + 2 * (LCD_HEIGHT)) * ydir;
        else
          encoderDiff = epps * (row - encoderPosition + encoderTopLine);
      }
      else if (!on_status_screen())
        encoderDiff = epps * xdir;
    }

  #endif

#endif // HAS_DISPLAY

#if HAS_PRINT_PROGRESS

  MarlinUI::progress_t MarlinUI::_get_progress() {
    return (
      TERN0(SET_PROGRESS_PERCENT, (progress_override & PROGRESS_MASK))
      #if HAS_MEDIA
        ?: TERN(HAS_PRINT_PROGRESS_PERMYRIAD, card.permyriadDone(), card.percentDone())
      #endif
    );
  }

  #if LCD_WITH_BLINK && HAS_EXTRA_PROGRESS

    // Renew and redraw all enabled progress strings
    void MarlinUI::rotate_progress() {
      typedef void (*PrintProgress_t)();
      const PrintProgress_t progFunc[] = {
        OPTITEM(SHOW_PROGRESS_PERCENT, drawPercent)
        OPTITEM(SHOW_ELAPSED_TIME, drawElapsed)
        OPTITEM(SHOW_REMAINING_TIME, drawRemain)
        OPTITEM(SHOW_INTERACTION_TIME, drawInter)
      };
      static bool prev_blink;
      static uint8_t i;
      if (prev_blink != get_blink()) {
        prev_blink = get_blink();
        if (++i >= COUNT(progFunc)) i = 0;
        (*progFunc[i])();
      }
    }

  #endif // LCD_WITH_BLINK && HAS_EXTRA_PROGRESS

#endif // HAS_PRINT_PROGRESS

#if HAS_MEDIA

  #if ENABLED(EXTENSIBLE_UI)
    #include "extui/ui_api.h"
  #endif

  void MarlinUI::media_changed(const uint8_t old_status, const uint8_t status) {
    TERN_(HAS_DISPLAY_SLEEP, refresh_screen_timeout());
    if (old_status == status) {
      TERN_(EXTENSIBLE_UI, ExtUI::onMediaError()); // Failed to mount/unmount
      return;
    }

    if (old_status < 2) {   // Skip this section on first boot check
      if (status) {         // Media Mounted
        #if ENABLED(EXTENSIBLE_UI)
          ExtUI::onMediaMounted();
        #elif ENABLED(BROWSE_MEDIA_ON_INSERT)
          clear_menu_history();
          quick_feedback();
          goto_screen(MEDIA_MENU_GATEWAY);
        #else
          LCD_MESSAGE(MSG_MEDIA_INSERTED);
        #endif
      }
      else {                // Media Removed
        #if ENABLED(EXTENSIBLE_UI)
          ExtUI::onMediaRemoved();
        #elif HAS_SD_DETECT // Q: Does "Media Removed" need to be shown for manual release too?
          LCD_MESSAGE(MSG_MEDIA_REMOVED);
          #if HAS_MARLINUI_MENU
            if (ENABLED(HAS_WIRED_LCD) || !defer_return_to_status) return_to_status();
          #endif
        #elif HAS_WIRED_LCD
          return_to_status();
        #endif
      }
    }

    reinit_lcd(); // Revive a noisy shared SPI LCD

    refresh();

    #if HAS_WIRED_LCD || HAS_LED_POWEROFF_TIMEOUT
      const millis_t ms = millis();
      TERN_(HAS_WIRED_LCD, next_lcd_update_ms = ms + LCD_UPDATE_INTERVAL); // Delay LCD update for SD activity
      TERN_(HAS_LED_POWEROFF_TIMEOUT, leds.reset_timeout(ms));
    #endif
  }

#endif // HAS_MEDIA

#if HAS_MARLINUI_MENU
  void MarlinUI::reset_settings() {
    settings.reset();
    completion_feedback();
    #if ENABLED(TOUCH_SCREEN_CALIBRATION)
      if (touch_calibration.need_calibration()) ui.goto_screen(touch_screen_calibration);
    #endif
  }

  #if ANY(BABYSTEP_GFX_OVERLAY, MESH_EDIT_GFX_OVERLAY)
    void MarlinUI::zoffset_overlay(const_float_t zvalue) {
      // Determine whether the user is raising or lowering the nozzle.
      static int8_t dir;
      static float old_zvalue;
      if (zvalue != old_zvalue) {
        dir = zvalue ? zvalue < old_zvalue ? -1 : 1 : 0;
        old_zvalue = zvalue;
      }
      zoffset_overlay(dir);
    }
  #endif

#endif

#if ENABLED(EEPROM_SETTINGS)

  #if HAS_MARLINUI_MENU
    void MarlinUI::init_eeprom() {
      const bool good = settings.init_eeprom();
      completion_feedback(good);
      return_to_status();
    }
    void MarlinUI::load_settings() {
      const bool good = settings.load();
      completion_feedback(good);
    }
    void MarlinUI::store_settings() {
      const bool good = settings.save();
      completion_feedback(good);
    }
  #endif

  #if DISABLED(EEPROM_AUTO_INIT)

    static inline FSTR_P eeprom_err(const EEPROM_Error err) {
      switch (err) {
        case ERR_EEPROM_VERSION:  return GET_TEXT_F(MSG_ERR_EEPROM_VERSION);
        case ERR_EEPROM_SIZE:     return GET_TEXT_F(MSG_ERR_EEPROM_SIZE);
        case ERR_EEPROM_CRC:      return GET_TEXT_F(MSG_ERR_EEPROM_CRC);
        case ERR_EEPROM_CORRUPT:  return GET_TEXT_F(MSG_ERR_EEPROM_CORRUPT);
        default: return nullptr;
      }
    }

    void MarlinUI::eeprom_alert(const EEPROM_Error err) {
      FSTR_P const err_msg = eeprom_err(err);
      set_status(err_msg);
      TERN_(HOST_PROMPT_SUPPORT, hostui.notify(err_msg));
      #if HAS_MARLINUI_MENU
        editable.uint8 = err;
        goto_screen([]{
          FSTR_P const restore_msg = GET_TEXT_F(MSG_INIT_EEPROM);
          char msg[utf8_strlen(restore_msg) + 1];
          strcpy_P(msg, FTOP(restore_msg));
          MenuItem_confirm::select_screen(
            GET_TEXT_F(MSG_BUTTON_RESET), GET_TEXT_F(MSG_BUTTON_IGNORE),
            init_eeprom, return_to_status,
            eeprom_err((EEPROM_Error)editable.uint8), msg, F("?")
          );
        });
      #endif
    }

  #endif // EEPROM_AUTO_INIT

#endif // EEPROM_SETTINGS
