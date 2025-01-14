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

#if HAS_MARLINUI_HD44780

/**
 * marlinui_HD44780.cpp
 *
 * LCD display implementations for Hitachi HD44780.
 * These are the most common LCD character displays.
 */

#include "marlinui_HD44780.h"
#include "../marlinui.h"
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

#if ANY(HAS_COOLER, LASER_COOLANT_FLOW_METER)
  #include "../../feature/cooler.h"
#endif

#if ENABLED(I2C_AMMETER)
  #include "../../feature/ammeter.h"
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL)
  #include "../../feature/bedlevel/bedlevel.h"
#endif

#if HAS_CUTTER
  #include "../../feature/spindle_laser.h"
#endif

//
// Create LCD instance and chipset-specific information
//

#if ENABLED(LCD_I2C_TYPE_PCF8575)

  LCD_CLASS lcd(LCD_I2C_ADDRESS, LCD_I2C_PIN_EN, LCD_I2C_PIN_RW, LCD_I2C_PIN_RS, LCD_I2C_PIN_D4, LCD_I2C_PIN_D5, LCD_I2C_PIN_D6, LCD_I2C_PIN_D7);

#elif ANY(LCD_I2C_TYPE_MCP23017, LCD_I2C_TYPE_MCP23008)

  LCD_CLASS lcd(LCD_I2C_ADDRESS OPTARG(DETECT_I2C_LCD_DEVICE, 1));

#elif ENABLED(LCD_I2C_TYPE_PCA8574)

  LCD_CLASS lcd(LCD_I2C_ADDRESS, LCD_WIDTH, LCD_HEIGHT);

#elif ENABLED(SR_LCD_2W_NL)

  // 2 wire Non-latching LCD SR from:
  // https://github.com/fmalpartida/New-LiquidCrystal/wiki/schematics#user-content-ShiftRegister_connection

  LCD_CLASS lcd(SR_DATA_PIN, SR_CLK_PIN
    #if PIN_EXISTS(SR_STROBE)
      , SR_STROBE_PIN
    #endif
  );

#elif ENABLED(SR_LCD_3W_NL)

  // NewLiquidCrystal was not working
  // https://github.com/mikeshub/SailfishLCD
  // uses the code directly from Sailfish

  LCD_CLASS lcd(SR_STROBE_PIN, SR_DATA_PIN, SR_CLK_PIN);

#elif ENABLED(LCM1602)

  LCD_CLASS lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#elif ENABLED(YHCB2004)

  #ifndef YHCB2004_SS_PIN
    #define YHCB2004_SS_PIN   SS
  #endif
  #ifndef YHCB2004_SCK_PIN
    #define YHCB2004_SCK_PIN  SCK
  #endif
  #ifndef YHCB2004_MOSI_PIN
    #define YHCB2004_MOSI_PIN MOSI
  #endif
  #ifndef YHCB2004_MISO_PIN
    #define YHCB2004_MISO_PIN MISO
  #endif
  #if !PINS_EXIST(YHCB2004_SS, YHCB2004_SCK, YHCB2004_MOSI, YHCB2004_MISO)
    #error "YHCB2004 display requires YHCB2004_SS_PIN, YHCB2004_SCK_PIN, YHCB2004_MOSI_PIN, and YHCB2004_MISO_PIN."
  #endif

  LCD_CLASS lcd(YHCB2004_SS_PIN, 20, 4, YHCB2004_SCK_PIN, YHCB2004_MOSI_PIN, YHCB2004_MISO_PIN); // SS, cols, rows, SCK, MOSI, MISO

#else

  // Standard direct-connected LCD implementations
  LCD_CLASS lcd(LCD_PINS_RS, LCD_PINS_EN, LCD_PINS_D4, LCD_PINS_D5, LCD_PINS_D6, LCD_PINS_D7);

#endif

static void createChar_P(const char c, const byte * const ptr) {
  byte temp[8];
  for (uint8_t i = 0; i < 8; ++i)
    temp[i] = pgm_read_byte(&ptr[i]);
  lcd.createChar(c, temp);
}

#if ENABLED(LCD_PROGRESS_BAR)
  #define LCD_STR_PROGRESS  "\x03\x04\x05"
#endif

#if ENABLED(LCD_USE_I2C_BUZZER)

  void MarlinUI::buzz(const long duration, const uint16_t freq/*=0*/) {
    if (sound_on) lcd.buzz(duration, freq);
  }

#endif

void MarlinUI::set_custom_characters(const HD44780CharSet screen_charset/*=CHARSET_INFO*/) {
  #if NONE(LCD_PROGRESS_BAR, SHOW_BOOTSCREEN)
    UNUSED(screen_charset);
  #endif

  // CHARSET_BOOT
  #if ENABLED(SHOW_BOOTSCREEN)
    const static PROGMEM byte corner[4][8] = { {
      B00000,
      B00000,
      B00000,
      B00000,
      B00001,
      B00010,
      B00100,
      B00100
    }, {
      B00000,
      B00000,
      B00000,
      B11100,
      B11100,
      B01100,
      B00100,
      B00100
    }, {
      B00100,
      B00010,
      B00001,
      B00000,
      B00000,
      B00000,
      B00000,
      B00000
    }, {
      B00100,
      B01000,
      B10000,
      B00000,
      B00000,
      B00000,
      B00000,
      B00000
    } };
  #endif // SHOW_BOOTSCREEN

  // CHARSET_INFO
  const static PROGMEM byte bedTemp[8] = {
    B00000,
    B11111,
    B10101,
    B10001,
    B10101,
    B11111,
    B00000,
    B00000
  };

  const static PROGMEM byte degree[8] = {
    B01100,
    B10010,
    B10010,
    B01100,
    B00000,
    B00000,
    B00000,
    B00000
  };

  const static PROGMEM byte thermometer[8] = {
    B00100,
    B01010,
    B01010,
    B01010,
    B01010,
    B10001,
    B10001,
    B01110
  };

  const static PROGMEM byte uplevel[8] = {
    B00100,
    B01110,
    B11111,
    B00100,
    B11100,
    B00000,
    B00000,
    B00000
  };

  const static PROGMEM byte feedrate[8] = {
    #if LCD_INFO_SCREEN_STYLE == 1
      B00000,
      B00100,
      B10010,
      B01001,
      B10010,
      B00100,
      B00000,
      B00000
    #else
      B11100,
      B10000,
      B11000,
      B10111,
      B00101,
      B00110,
      B00101,
      B00000
    #endif
  };

  const static PROGMEM byte clock[8] = {
    B00000,
    B01110,
    B10011,
    B10101,
    B10001,
    B01110,
    B00000,
    B00000
  };

  #if ENABLED(LCD_PROGRESS_BAR)

    // CHARSET_INFO
    const static PROGMEM byte progress[3][8] = { {
      B00000,
      B10000,
      B10000,
      B10000,
      B10000,
      B10000,
      B10000,
      B00000
    }, {
      B00000,
      B10100,
      B10100,
      B10100,
      B10100,
      B10100,
      B10100,
      B00000
    }, {
      B00000,
      B10101,
      B10101,
      B10101,
      B10101,
      B10101,
      B10101,
      B00000
    } };

  #endif // LCD_PROGRESS_BAR

  #if ALL(HAS_MEDIA, HAS_MARLINUI_MENU)

    // CHARSET_MENU
    const static PROGMEM byte refresh[8] = {
      B00000,
      B00110,
      B11001,
      B11000,
      B00011,
      B10011,
      B01100,
      B00000,
    };
    const static PROGMEM byte folder[8] = {
      B00000,
      B11100,
      B11111,
      B10001,
      B10001,
      B11111,
      B00000,
      B00000
    };

  #endif // HAS_MEDIA

  switch (screen_charset) {

    #if ENABLED(SHOW_BOOTSCREEN)
      case CHARSET_BOOT: {
        // Set boot screen corner characters
        for (uint8_t i = 4; i--;) createChar_P(i, corner[i]);
      } break;
    #endif

    #if ENABLED(SHOW_CUSTOM_BOOTSCREEN)
      case CHARSET_BOOT_CUSTOM: {
        for (uint8_t i = COUNT(customBootChars); i--;)
          createChar_P(i, customBootChars[i]);
      } break;
    #endif

    default: {
      // Info Screen uses 5 special characters
      createChar_P(LCD_STR_BEDTEMP[0], bedTemp);
      createChar_P(LCD_STR_DEGREE[0], degree);
      createChar_P(LCD_STR_THERMOMETER[0], thermometer);
      createChar_P(LCD_STR_FEEDRATE[0], feedrate);
      createChar_P(LCD_STR_CLOCK[0], clock);

      #if ENABLED(LCD_PROGRESS_BAR)
        if (screen_charset == CHARSET_INFO) { // 3 Progress bar characters for info screen
          for (int16_t i = 3; i--;)
            createChar_P(LCD_STR_PROGRESS[i], progress[i]);
        }
        else
      #endif
        {
          createChar_P(LCD_STR_UPLEVEL[0], uplevel);
          #if ALL(HAS_MEDIA, HAS_MARLINUI_MENU)
            // SD Card sub-menu special characters
            createChar_P(LCD_STR_REFRESH[0], refresh);
            createChar_P(LCD_STR_FOLDER[0], folder);
          #endif
        }
    } break;

  }

}

void MarlinUI::init_lcd() {

  #if ENABLED(LCD_I2C_TYPE_PCF8575)
    lcd.begin(LCD_WIDTH, LCD_HEIGHT);
    #ifdef LCD_I2C_PIN_BL
      lcd.setBacklightPin(LCD_I2C_PIN_BL, POSITIVE);
      lcd.setBacklight(HIGH);
    #endif

  #elif ENABLED(LCD_I2C_TYPE_MCP23017)
    lcd.setMCPType(LTI_TYPE_MCP23017);
    lcd.begin(LCD_WIDTH, LCD_HEIGHT);
    update_indicators();

  #elif ENABLED(LCD_I2C_TYPE_MCP23008)
    lcd.setMCPType(LTI_TYPE_MCP23008);
    lcd.begin(LCD_WIDTH, LCD_HEIGHT);

  #elif ENABLED(LCD_I2C_TYPE_PCA8574)
    lcd.init();
    lcd.backlight();

  #else
    lcd.begin(LCD_WIDTH, LCD_HEIGHT);
  #endif

  set_custom_characters(on_status_screen() ? CHARSET_INFO : CHARSET_MENU);

  lcd.clear();
}

bool MarlinUI::detected() {
  return TERN1(DETECT_I2C_LCD_DEVICE, lcd.LcdDetected() == 1);
}

#if ENABLED(SHOW_CUSTOM_BOOTSCREEN)

  #ifndef CUSTOM_BOOTSCREEN_X
    #define CUSTOM_BOOTSCREEN_X -1
  #endif
  #ifndef CUSTOM_BOOTSCREEN_Y
    #define CUSTOM_BOOTSCREEN_Y ((LCD_HEIGHT - COUNT(custom_boot_lines)) / 2)
  #endif
  #ifndef CUSTOM_BOOTSCREEN_TIMEOUT
    #define CUSTOM_BOOTSCREEN_TIMEOUT 2500
  #endif

  void MarlinUI::draw_custom_bootscreen(const uint8_t/*=0*/) {
    set_custom_characters(CHARSET_BOOT_CUSTOM);
    lcd.clear();
    const int8_t sx = CUSTOM_BOOTSCREEN_X;
    const uint8_t sy = CUSTOM_BOOTSCREEN_Y;
    for (lcd_uint_t i = 0; i < COUNT(custom_boot_lines); ++i) {
      PGM_P const pstr = (PGM_P)pgm_read_ptr(&custom_boot_lines[i]);
      const uint8_t clen = utf8_strlen_P(pstr);
      const lcd_uint_t x = sx >= 0 ? sx : (LCD_WIDTH - clen) / 2;
      for (lcd_uint_t j = 0; j < clen; ++j) {
        const lchar_t c = pgm_read_byte(&pstr[j]);
        lcd_put_lchar(x + j, sy + i, c == '\x08' ? '\x00' : c);
      }
    }
  }

  // Shows the custom bootscreen and delays
  void MarlinUI::show_custom_bootscreen() {
    draw_custom_bootscreen();
    safe_delay(CUSTOM_BOOTSCREEN_TIMEOUT);
  }

#endif // SHOW_CUSTOM_BOOTSCREEN

#if HAS_SLOW_BUTTONS
  uint8_t MarlinUI::read_slow_buttons() {
    #if ENABLED(LCD_I2C_TYPE_MCP23017)
      // Reading these buttons is too slow for interrupt context
      // so they are read during LCD update in the main loop.
      uint8_t slow_bits = lcd.readButtons()
        #if !BUTTON_EXISTS(ENC)
          << B_I2C_BTN_OFFSET
        #endif
      ;
      #if ENABLED(LCD_I2C_VIKI)
        if ((slow_bits & (B_MI | B_RI)) && PENDING(millis(), next_button_update_ms)) // LCD clicked
          slow_bits &= ~(B_MI | B_RI); // Disable LCD clicked buttons if screen is updated
      #endif
      return slow_bits;
    #endif // LCD_I2C_TYPE_MCP23017
  }
#endif

void MarlinUI::clear_lcd() { lcd.clear(); }
void MarlinUI::clear_for_drawing() { clear_lcd(); }

#if ENABLED(SHOW_BOOTSCREEN)

  void lcd_erase_line(const lcd_uint_t line) {
    lcd_moveto(0, line);
    for (uint8_t i = LCD_WIDTH + 1; --i;)
      lcd_put_u8str(F(" "));
  }

  // Scroll the PSTR 'text' in a 'len' wide field for 'time' milliseconds at position col,line
  void lcd_scroll(const lcd_uint_t col, const lcd_uint_t line, FSTR_P const ftxt, const uint8_t len, const int16_t time) {
    uint8_t slen = utf8_strlen(ftxt);
    if (slen < len) {
      lcd_put_u8str_max(col, line, ftxt, len);
      for (; slen < len; ++slen) lcd_put_u8str(F(" "));
      safe_delay(time);
    }
    else {
      PGM_P p = FTOP(ftxt);
      int dly = time / _MAX(slen, 1);
      for (uint8_t i = 0; i <= slen; ++i) {

        // Print the text at the correct place
        lcd_put_u8str_max_P(col, line, p, len);

        // Fill with spaces
        for (uint8_t ix = slen - i; ix < len; ++ix) lcd_put_u8str(F(" "));

        // Delay
        safe_delay(dly);

        // Advance to the next UTF8 valid position
        p++;
        while (!START_OF_UTF8_CHAR(pgm_read_byte(p))) p++;
      }
    }
  }

  static void logo_lines(FSTR_P const extra) {
    int16_t indent = (LCD_WIDTH - 8 - utf8_strlen(extra)) / 2;
    lcd_put_lchar(indent, 0, '\x00'); lcd_put_u8str(F( "------" )); lcd_put_u8str(F("\x01"));
    lcd_put_u8str(indent, 1, F("|Marlin|")); lcd_put_u8str(extra);
    lcd_put_lchar(indent, 2, '\x02'); lcd_put_u8str(F( "------" )); lcd_put_u8str(F("\x03"));
  }

  void MarlinUI::show_bootscreen() {
    TERN_(SHOW_CUSTOM_BOOTSCREEN, show_custom_bootscreen());

    set_custom_characters(CHARSET_BOOT);
    lcd.clear();

    #define LCD_EXTRA_SPACE (LCD_WIDTH-8)

    #define CENTER_OR_SCROLL(STRING,DELAY) { \
      lcd_erase_line(3); \
      const int len = utf8_strlen(STRING); \
      if (len <= LCD_WIDTH) { \
        lcd_put_u8str((LCD_WIDTH - len) / 2, 3, F(STRING)); \
        safe_delay(DELAY); \
      } \
      else \
        lcd_scroll(0, 3, F(STRING), LCD_WIDTH, DELAY); \
    }

    //
    // Show the Marlin logo with splash line 1
    //
    if (LCD_EXTRA_SPACE >= utf8_strlen(SHORT_BUILD_VERSION) + 1) {
      //
      // Show the Marlin logo, splash line1, and splash line 2
      //
      logo_lines(F(" " SHORT_BUILD_VERSION));
      CENTER_OR_SCROLL(MARLIN_WEBSITE_URL, 2000);
    }
    else {
      //
      // Show the Marlin logo and short build version
      // After a delay show the website URL
      //
      logo_lines(FPSTR(NUL_STR));
      CENTER_OR_SCROLL(SHORT_BUILD_VERSION, 1500);
      CENTER_OR_SCROLL(MARLIN_WEBSITE_URL, 1500);
      #ifdef STRING_SPLASH_LINE3
        CENTER_OR_SCROLL(STRING_SPLASH_LINE3, 1500);
      #endif
    }
  }

  void MarlinUI::bootscreen_completion(const millis_t) {
    lcd.clear();
    safe_delay(100);
    set_custom_characters(CHARSET_INFO);
    lcd.clear();
  }

#endif // SHOW_BOOTSCREEN

void MarlinUI::draw_kill_screen() {
  lcd_uint_t x = 0, y = 0;
  lcd_put_u8str(x, y, status_message);
  y = 2;
  #if LCD_HEIGHT >= 4
    lcd_put_u8str(x, y++, GET_TEXT_F(MSG_HALTED));
  #endif
  lcd_put_u8str(x, y, GET_TEXT_F(MSG_PLEASE_RESET));
}

//
// Before homing, blink '123' <-> '???'.
// Homed but unknown... '123' <-> '   '.
// Homed and known, display constantly.
//
FORCE_INLINE void _draw_axis_value(const AxisEnum axis, const char *value, const bool blink) {
  lcd_put_lchar('X' + uint8_t(axis));
  if (blink)
    lcd_put_u8str(value);
  else if (axis_should_home(axis))
    while (const char c = *value++) lcd_put_lchar(c <= '.' ? c : '?');
  else if (NONE(HOME_AFTER_DEACTIVATE, DISABLE_REDUCED_ACCURACY_WARNING) && !axis_is_trusted(axis))
    lcd_put_u8str(TERN0(HAS_Z_AXIS, axis == Z_AXIS) ? F("       ") : F("    "));
  else
    lcd_put_u8str(value);
}

/**
 * @brief Draw current and target for a heater/cooler
 * @details Print at the current LCD position the current/target for a single heater,
 *          blinking the target temperature of an idle heater has timed out.
 *
 * @param heater_id The heater ID, such as 0, 1, ..., H_BED, H_CHAMBER, etc.
 * @param prefix A char to draw in front (e.g., a thermometer or icon)
 * @param blink Flag to show the blink state instead of the regular state
 */
FORCE_INLINE void _draw_heater_status(const heater_id_t heater_id, const char prefix, const bool blink) {
  #if HAS_HEATED_BED
    const bool isBed = heater_id == H_BED;
    const celsius_t t1 = (isBed ? thermalManager.wholeDegBed()  : thermalManager.wholeDegHotend(heater_id)),
                    t2 = (isBed ? thermalManager.degTargetBed() : thermalManager.degTargetHotend(heater_id));
  #else
    const celsius_t t1 = thermalManager.wholeDegHotend(heater_id), t2 = thermalManager.degTargetHotend(heater_id);
  #endif

  if (prefix >= 0) lcd_put_lchar(prefix);

  if (t1 >= 0)
    lcd_put_u8str(ui16tostr3rj(t1));
  else {
    #if ENABLED(SHOW_TEMPERATURE_BELOW_ZERO)
      char * const str = i16tostr3rj(t1);
      lcd_put_u8str(&str[1]);
    #else
      lcd_put_u8str(F("err"));
    #endif
  }

  lcd_put_u8str(F("/"));

  #if !HEATER_IDLE_HANDLER
    UNUSED(blink);
  #else
    if (!blink && thermalManager.heater_idle[thermalManager.idle_index_for_id(heater_id)].timed_out)
      lcd_put_u8str(F("   "));
    else
  #endif
      lcd_put_u8str(i16tostr3left(t2));

  if (prefix >= 0) {
    lcd_put_lchar(LCD_STR_DEGREE[0]);
    lcd_put_u8str(F(" "));
    if (t2 < 10) lcd_put_u8str(F(" "));
  }
}

#if HAS_COOLER

  FORCE_INLINE void _draw_cooler_status(const char prefix, const bool blink) {
    const celsius_t t2 = thermalManager.degTargetCooler();

    if (prefix >= 0) lcd_put_lchar(prefix);

    lcd_put_u8str(i16tostr3rj(thermalManager.wholeDegCooler()));
    lcd_put_u8str(F("/"));

    #if !HEATER_IDLE_HANDLER
      UNUSED(blink);
    #else
      if (!blink && thermalManager.heater_idle[thermalManager.idle_index_for_id(heater_id)].timed_out) {
        lcd_put_u8str(F(" "));
        if (t2 >= 10) lcd_put_u8str(F(" "));
        if (t2 >= 100) lcd_put_u8str(F(" "));
      }
      else
    #endif
        lcd_put_u8str(i16tostr3left(t2));

    if (prefix >= 0) {
      lcd_put_lchar(LCD_STR_DEGREE[0]);
      lcd_put_u8str(F(" "));
      if (t2 < 10) lcd_put_u8str(F(" "));
    }
  }

#endif // HAS_COOLER

#if ENABLED(LASER_COOLANT_FLOW_METER)
  FORCE_INLINE void _draw_flowmeter_status() {
    lcd_put_u8str(F("~"));
    lcd_put_u8str(ftostr11ns(cooler.flowrate));
    lcd_put_u8str(F("L"));
  }
#endif

#if ENABLED(I2C_AMMETER)
  FORCE_INLINE void _draw_ammeter_status() {
    lcd_put_u8str(F(" "));
    ammeter.read();
    if (ammeter.current <= 0.999f) {
      lcd_put_u8str(ui16tostr3rj(uint16_t(ammeter.current * 1000 + 0.5f)));
      lcd_put_u8str(F("mA"));
    }
    else {
      lcd_put_u8str(ftostr12ns(ammeter.current));
      lcd_put_u8str(F("A"));
    }
  }
#endif

FORCE_INLINE void _draw_bed_status(const bool blink) {
  _draw_heater_status(H_BED, TERN0(HAS_LEVELING, blink && planner.leveling_active) ? '_' : LCD_STR_BEDTEMP[0], blink);
}

#if HAS_CUTTER

  FORCE_INLINE void _draw_cutter_status() {
    lcd_put_u8str(TERN(LASER_FEATURE, GET_TEXT_F(MSG_LASER), GET_TEXT_F(MSG_CUTTER)));
    lcd_put_u8str(F(": "));

    #if CUTTER_UNIT_IS(RPM)
      lcd_put_u8str(ftostr61rj(float(cutter.unitPower) / 1000));
      lcd_put_u8str(F("K"));
    #else
      lcd_put_u8str(cutter_power2str(cutter.unitPower));
    #endif

    lcd_put_u8str(F(" "));
    lcd_put_u8str(cutter.enabled() ? GET_TEXT_F(MSG_LCD_ON) : GET_TEXT_F(MSG_LCD_OFF));
    lcd_put_u8str(F(" "));

    switch (cutter.cutter_mode) {
      case CUTTER_MODE_STANDARD:   lcd_put_u8str(F("S")); break;
      case CUTTER_MODE_CONTINUOUS: lcd_put_u8str(F("C")); break;
      case CUTTER_MODE_DYNAMIC:    lcd_put_u8str(F("D")); break;
      case CUTTER_MODE_ERROR:      lcd_put_u8str(F("!")); break;
    }
  }

#endif // HAS_CUTTER

#if ENABLED(LCD_PROGRESS_BAR)

  void MarlinUI::draw_progress_bar(const uint8_t percent) {
    const int16_t tix = int16_t(percent * (LCD_WIDTH) * 3) / 100,
                  cel = tix / 3,
                  rem = tix % 3;
    uint8_t i = LCD_WIDTH;
    char msg[LCD_WIDTH + 1], b = ' ';
    msg[LCD_WIDTH] = '\0';
    while (i--) {
      if (i == cel - 1)
        b = LCD_STR_PROGRESS[2];
      else if (i == cel && rem != 0)
        b = LCD_STR_PROGRESS[rem - 1];
      msg[i] = b;
    }
    lcd_put_u8str(msg);
  }

#endif // LCD_PROGRESS_BAR

void MarlinUI::draw_status_message(const bool blink) {

  lcd_moveto(0, LCD_HEIGHT - 1);

  #if ENABLED(LCD_PROGRESS_BAR)

    // Draw the progress bar if the message has shown long enough
    // or if there is no message set.
    if (ELAPSED(millis(), progress_bar_ms + PROGRESS_BAR_MSG_TIME) || !has_status()) {
      const uint8_t progress = get_progress_percent();
      if (progress > 2) return draw_progress_bar(progress);
    }

  #elif ALL(FILAMENT_LCD_DISPLAY, HAS_MEDIA)

    // Alternate Status message and Filament display
    if (ELAPSED(millis(), next_filament_display)) {
      lcd_put_u8str(F("Dia "));
      lcd_put_u8str(ftostr12ns(filwidth.measured_mm));
      lcd_put_u8str(F(" V"));
      lcd_put_u8str(i16tostr3rj(planner.volumetric_percent(parser.volumetric_enabled)));
      lcd_put_u8str(F("%"));
      return;
    }

  #endif // FILAMENT_LCD_DISPLAY && HAS_MEDIA

  #if ENABLED(STATUS_MESSAGE_SCROLLING)
    static bool last_blink = false;

    // Get the UTF8 character count of the string
    uint8_t slen = status_message.glyphs();

    // If the string fits into the LCD, just print it and do not scroll it
    if (slen <= LCD_WIDTH) {

      // The string isn't scrolling and may not fill the screen
      lcd_put_u8str(status_message);

      // Fill the rest with spaces
      while (slen < LCD_WIDTH) { lcd_put_u8str(F(" ")); ++slen; }
    }
    else {
      // String is larger than the available space in screen.

      // Get a pointer to the next valid UTF8 character
      // and the string remaining length
      uint8_t rlen;
      const char *stat = status_and_len(rlen);
      lcd_put_u8str_max(stat, LCD_WIDTH);     // The string leaves space

      // If the remaining string doesn't completely fill the screen
      if (rlen < LCD_WIDTH) {
        uint8_t chars = LCD_WIDTH - rlen;     // Amount of space left in characters
        lcd_put_u8str(F(" "));                   // Always at 1+ spaces left, draw a space
        if (--chars) {                        // Draw a second space if there's room
          lcd_put_u8str(F(" "));
          if (--chars) {                      // Draw a third space if there's room
            lcd_put_u8str(F(" "));
            if (--chars)
              lcd_put_u8str_max(status_message, chars); // Print a second copy of the message
          }
        }
      }
      if (last_blink != blink) {
        last_blink = blink;
        advance_status_scroll();
      }
    }
  #else
    UNUSED(blink);

    // Get the UTF8 character count of the string
    uint8_t slen = status_message.glyphs();

    // Just print the string to the LCD
    lcd_put_u8str_max(status_message, LCD_WIDTH);

    // Fill the rest with spaces if there are missing spaces
    for (; slen < LCD_WIDTH; ++slen) lcd_put_u8str(F(" "));
  #endif
}

#if HAS_PRINT_PROGRESS

  #define TPOFFSET (LCD_WIDTH - 1)

  #if ENABLED(SHOW_PROGRESS_PERCENT)
    static lcd_uint_t pc = 0, pr = 2;
    inline void setPercentPos(const lcd_uint_t c, const lcd_uint_t r) { pc = c; pr = r; }
    void MarlinUI::drawPercent() {
      const uint8_t progress = get_progress_percent();
      if (progress) {
        lcd_moveto(pc, pr);
        lcd_put_u8str(F(TERN(IS_SD_PRINTING, "SD", "P:")));
        lcd_put_u8str(TERN(PRINT_PROGRESS_SHOW_DECIMALS, permyriadtostr4(get_progress_permyriad()), ui8tostr3rj(progress)));
        lcd_put_u8str(F("%"));
      }
    }
  #endif

  #if ENABLED(SHOW_REMAINING_TIME)
    void MarlinUI::drawRemain() {
      if (printJobOngoing()) {
        char buffer[8];
        const duration_t remaint = get_remaining_time();
        const uint8_t timepos = TPOFFSET - remaint.toDigital(buffer);
        IF_DISABLED(LCD_INFO_SCREEN_STYLE, lcd_put_lchar(timepos - 1, 2, 0x20));
        lcd_put_lchar(TERN(LCD_INFO_SCREEN_STYLE, 11, timepos), 2, 'R');
        lcd_put_u8str(buffer);
      }
    }
  #endif

  #if ENABLED(SHOW_INTERACTION_TIME)
    void MarlinUI::drawInter() {
      const duration_t interactt = interaction_time;
      if (printingIsActive() && interactt.value) {
        char buffer[8];
        const uint8_t timepos = TPOFFSET - interactt.toDigital(buffer);
        IF_DISABLED(LCD_INFO_SCREEN_STYLE, lcd_put_lchar(timepos - 1, 2, 0x20));
        lcd_put_lchar(TERN(LCD_INFO_SCREEN_STYLE, 11, timepos), 2, 'C');
        lcd_put_u8str(buffer);
      }
    }
  #endif

  #if ENABLED(SHOW_ELAPSED_TIME)
    void MarlinUI::drawElapsed() {
      if (printJobOngoing()) {
        char buffer[8];
        const duration_t elapsedt = print_job_timer.duration();
        const uint8_t timepos = TPOFFSET - elapsedt.toDigital(buffer);
        IF_DISABLED(LCD_INFO_SCREEN_STYLE, lcd_put_lchar(timepos - 1, 2, 0x20));
        lcd_put_lchar(TERN(LCD_INFO_SCREEN_STYLE, 11, timepos), 2, 'E');
        lcd_put_u8str(buffer);
      }
    }
  #endif

#endif // HAS_PRINT_PROGRESS

/**
 *  LCD_INFO_SCREEN_STYLE 0 : Classic Status Screen
 *
 *  16x2   |000/000 B000/000|
 *         |0123456789012345|
 *
 *  16x4   |000/000 B000/000|
 *         |SD---%  Z 000.00|
 *         |F---%     T--:--|
 *         |0123456789012345|
 *
 *  20x2   |T000/000° B000/000° |
 *         |01234567890123456789|
 *
 *  20x4   |T000/000° B000/000° |
 *         |X 000 Y 000 Z000.000|
 *         |F---%  SD---% T--:--|
 *         |01234567890123456789|
 *
 *  LCD_INFO_SCREEN_STYLE 1 : Průša-style Status Screen
 *
 *  |T000/000°  Z 000.00 |
 *  |B000/000°  F---%    |
 *  |SD---%     T--:--   |
 *  |01234567890123456789|
 *
 *  |T000/000°  Z 000.00 |
 *  |T000/000°  F---%    |
 *  |B000/000°  SD---%   |
 *  |01234567890123456789|
 */

void MarlinUI::draw_status_screen() {

  const bool blink = get_blink();
  lcd_moveto(0, 0);

  #if LCD_INFO_SCREEN_STYLE == 0

    // ========== Line 1 ==========

    #if LCD_WIDTH < 20

      //
      // Hotend 0 Temperature
      //
      #if HAS_HOTEND
        _draw_heater_status(H_E0, -1, blink);

        //
        // Hotend 1 or Bed Temperature
        //
        #if HAS_MULTI_HOTEND
          lcd_moveto(8, 0);
          _draw_heater_status(H_E1, LCD_STR_THERMOMETER[0], blink);
        #elif HAS_HEATED_BED
          lcd_moveto(8, 0);
          _draw_bed_status(blink);
        #endif

      #elif HAS_CUTTER

        //
        // Cutter Status
        //
        lcd_moveto(0, 0);
        _draw_cutter_status();

      #endif

    #else // LCD_WIDTH >= 20

      //
      // Hotend 0 Temperature
      //
      #if HAS_HOTEND
        _draw_heater_status(H_E0, LCD_STR_THERMOMETER[0], blink);

        //
        // Hotend 1 or Bed Temperature
        //
        #if HAS_MULTI_HOTEND
          lcd_moveto(10, 0);
          _draw_heater_status(H_E1, LCD_STR_THERMOMETER[0], blink);
        #elif HAS_HEATED_BED
          lcd_moveto(10, 0);
          _draw_bed_status(blink);
        #endif

      #elif HAS_CUTTER

        //
        // Cutter Status
        //
        lcd_moveto(0, 0);
        _draw_cutter_status();

      #endif

      TERN_(HAS_COOLER, _draw_cooler_status('*', blink));
      TERN_(LASER_COOLANT_FLOW_METER, _draw_flowmeter_status());
      TERN_(I2C_AMMETER, _draw_ammeter_status());

    #endif // LCD_WIDTH >= 20

    // ========== Line 2 ==========

    #if LCD_HEIGHT > 2

      #if LCD_WIDTH < 20

        #if HAS_PRINT_PROGRESS
          TERN_(SHOW_PROGRESS_PERCENT, setPercentPos(0, 2));
          rotate_progress();
        #endif

      #else // LCD_WIDTH >= 20

        lcd_moveto(0, 1);

        // If the first line has two extruder temps,
        // show more temperatures on the next line

        #if HOTENDS > 2 || (HAS_MULTI_HOTEND && HAS_HEATED_BED)

          #if HOTENDS > 2
            _draw_heater_status(H_E2, LCD_STR_THERMOMETER[0], blink);
            lcd_moveto(10, 1);
          #endif

          _draw_bed_status(blink);

        #else // HOTENDS <= 2 && (HOTENDS <= 1 || !HAS_HEATED_BED)

          #if HAS_DUAL_MIXING

            // Two-component mix / gradient instead of XY

            char mixer_messages[15];
            PGM_P mix_label;
            #if ENABLED(GRADIENT_MIX)
              if (mixer.gradient.enabled) {
                mixer.update_mix_from_gradient();
                mix_label = PSTR("Gr");
              }
              else
            #endif
              {
                mixer.update_mix_from_vtool();
                mix_label = PSTR("Mx");
              }
            sprintf_P(mixer_messages, PSTR(S_FMT " %d;%d%% "), mix_label, int(mixer.mix[0]), int(mixer.mix[1]));
            lcd_put_u8str(mixer_messages);

          #else // !HAS_DUAL_MIXING

            const bool show_e_total = TERN1(HAS_X_AXIS, TERN0(LCD_SHOW_E_TOTAL, printingIsActive()));

            if (show_e_total) {
              #if ENABLED(LCD_SHOW_E_TOTAL)
                char tmp[20];
                const uint8_t escale = e_move_accumulator >= 100000.0f ? 10 : 1; // After 100m switch to cm
                sprintf_P(tmp, PSTR("E %ld%cm       "), uint32_t(_MAX(e_move_accumulator, 0.0f)) / escale, escale == 10 ? 'c' : 'm'); // 1234567mm
                lcd_put_u8str(tmp);
              #endif
            }
            else {
              #if HAS_X_AXIS
                const xy_pos_t lpos = current_position.asLogical();
                _draw_axis_value(X_AXIS, ftostr4sign(lpos.x), blink);
              #endif
              #if HAS_Y_AXIS
                TERN_(HAS_X_AXIS, lcd_put_u8str(F(" ")));
                _draw_axis_value(Y_AXIS, ftostr4sign(lpos.y), blink);
              #endif
            }

          #endif // !HAS_DUAL_MIXING

        #endif // HOTENDS <= 2 && (HOTENDS <= 1 || !HAS_HEATED_BED)

      #endif // LCD_WIDTH >= 20

      #if HAS_Z_AXIS
        lcd_moveto(LCD_WIDTH - 8, 1);
        _draw_axis_value(Z_AXIS, ftostr52sp(LOGICAL_Z_POSITION(current_position.z)), blink);
        #if HAS_LEVELING && !HAS_HEATED_BED
          lcd_put_lchar(planner.leveling_active || blink ? '_' : ' ');
        #endif
      #endif

    #endif // LCD_HEIGHT > 2

    // ========== Line 3 ==========

    #if LCD_HEIGHT > 3

      lcd_put_lchar(0, 2, LCD_STR_FEEDRATE[0]);
      lcd_put_u8str(i16tostr3rj(feedrate_percentage));
      lcd_put_u8str(F("%"));

      #if LCD_WIDTH >= 20

        #if HAS_PRINT_PROGRESS
          TERN_(SHOW_PROGRESS_PERCENT, setPercentPos(6, 2));
          rotate_progress();
        #else
          char c;
          uint16_t per;
          #if HAS_FAN0
            if (true
              #if ALL(HAS_EXTRUDERS, ADAPTIVE_FAN_SLOWING)
                && (blink || thermalManager.fan_speed_scaler[0] < 128)
              #endif
            ) {
              uint16_t spd = thermalManager.fan_speed[0];
              if (blink) c = 'F';
              #if ENABLED(ADAPTIVE_FAN_SLOWING)
                else { c = '*'; spd = thermalManager.scaledFanSpeed(0, spd); }
              #endif
              per = thermalManager.pwmToPercent(spd);
            }
            else
          #endif
            {
              #if HAS_EXTRUDERS
                c = 'E';
                per = planner.flow_percentage[0];
              #endif
            }
          lcd_put_lchar(c);
          lcd_put_u8str(i16tostr3rj(per));
          lcd_put_u8str(F("%"));
        #endif
      #endif

    #endif // LCD_HEIGHT > 3

  #elif LCD_INFO_SCREEN_STYLE == 1

    // ========== Line 1 ==========

    //
    // Hotend 0 Temperature
    //
    _draw_heater_status(H_E0, LCD_STR_THERMOMETER[0], blink);

    //
    // Z Coordinate
    //
    #if HAS_Z_AXIS
      lcd_moveto(LCD_WIDTH - 9, 0);
      _draw_axis_value(Z_AXIS, ftostr52sp(LOGICAL_Z_POSITION(current_position.z)), blink);
    #endif

    #if HAS_LEVELING && (HAS_MULTI_HOTEND || !HAS_HEATED_BED)
      lcd_put_lchar(LCD_WIDTH - 1, 0, planner.leveling_active || blink ? '_' : ' ');
    #endif

    // ========== Line 2 ==========

    //
    // Hotend 1 or Bed Temperature
    //
    lcd_moveto(0, 1);
    #if HAS_MULTI_HOTEND
      _draw_heater_status(H_E1, LCD_STR_THERMOMETER[0], blink);
    #elif HAS_HEATED_BED
      _draw_bed_status(blink);
    #endif

    lcd_put_lchar(LCD_WIDTH - 9, 1, LCD_STR_FEEDRATE[0]);
    lcd_put_u8str(i16tostr3rj(feedrate_percentage));
    lcd_put_u8str(F("%"));

    // ========== Line 3 ==========

    //
    // Progress percent, Hotend 2, or Bed
    //
    lcd_moveto(0, 2);
    #if HOTENDS > 2
      _draw_heater_status(H_E2, LCD_STR_THERMOMETER[0], blink);
    #elif HAS_MULTI_HOTEND && HAS_HEATED_BED
      _draw_bed_status(blink);
    #elif HAS_PRINT_PROGRESS
      #define DREW_PRINT_PROGRESS 1
      TERN_(SHOW_PROGRESS_PERCENT, setPercentPos(0, 2));
      rotate_progress();
    #endif

    //
    // All progress strings
    //
    #if HAS_PRINT_PROGRESS && !DREW_PRINT_PROGRESS
      TERN_(SHOW_PROGRESS_PERCENT, setPercentPos(LCD_WIDTH - 9, 2));
      rotate_progress();
    #endif

  #elif LCD_INFO_SCREEN_STYLE == 2

    // ========== Line 1 ==========

    //
    // X Coordinate
    //
    lcd_moveto(0, 0);
    _draw_axis_value(X_AXIS, ftostr52sp(LOGICAL_X_POSITION(current_position.x)), blink);

    //
    // Y Coordinate
    //
    lcd_moveto(LCD_WIDTH - 9, 0);
    _draw_axis_value(Y_AXIS, ftostr52sp(LOGICAL_Y_POSITION(current_position.y)), blink);

    // ========== Line 2 ==========
    lcd_moveto(0, 1);
    _draw_axis_value(Z_AXIS, ftostr52sp(LOGICAL_Z_POSITION(current_position.z)), blink);

    lcd_moveto(LCD_WIDTH - 9, 1);
    _draw_axis_value(I_AXIS, ftostr52sp(LOGICAL_I_POSITION(current_position.i)), blink);

    // ========== Line 3 ==========
    lcd_moveto(0, 2);
    lcd_put_lchar('F');

    lcd_moveto(LCD_WIDTH - 9, 2);
    lcd_put_lchar('S');


  #endif // LCD_INFO_SCREEN_STYLE

  // ========= Last Line ========

  //
  // Status Message (which may be a Progress Bar or Filament display)
  //
  draw_status_message(blink);
}

#if HAS_MARLINUI_MENU

  #include "../menu/menu.h"

  #if ENABLED(ADVANCED_PAUSE_FEATURE)

    void MarlinUI::draw_hotend_status(const uint8_t row, const uint8_t extruder) {
      if (row < LCD_HEIGHT) {
        lcd_moveto(LCD_WIDTH - 9, row);
        _draw_heater_status((heater_id_t)extruder, LCD_STR_THERMOMETER[0], get_blink());
      }
    }

  #endif // ADVANCED_PAUSE_FEATURE

  // Draw a static item with no left-right margin required. Centered by default.
  void MenuItem_static::draw(const uint8_t row, FSTR_P const ftpl, const uint8_t style/*=SS_DEFAULT*/, const char *vstr/*=nullptr*/) {
    lcd_moveto(0, row);

    int8_t n = LCD_WIDTH;
    const bool center = bool(style & SS_CENTER), full = bool(style & SS_FULL);

    // Value length, if any
    int8_t vlen = vstr ? utf8_strlen(vstr) : 0;

    // Expanded label string and width in chars
    char estr[calculateWidth(ftpl) + 3] = "\0";
    int8_t llen = ftpl ? expand_u8str(estr, ftpl, itemIndex, itemStringC, itemStringF, n - vlen) : 0;

    bool mv_colon = false;
    if (vlen && !center) {
      // Move the leading colon from the value to the label below
      mv_colon = (*vstr == ':');
      // Shorter value, wider label
      if (mv_colon) { vstr++; vlen--; llen++; }
      // Remove leading spaces from the value and shorten
      while (*vstr == ' ') { vstr++; vlen--; }
    }

    // Padding for center or full justification
    int8_t pad = (center || full) ? n - llen - vlen : 0;

    // SS_CENTER: Pad with half of the unused space first
    if (center) for (int8_t lpad = pad / 2; lpad > 0; --lpad, --pad, --n) lcd_put_u8str(F(" "));

    // Draw as much of the label as fits (without the relocated colon, drawn below)
    if (llen) n -= lcd_put_u8str_max(estr, n - vlen);

    if (vlen && n > 0) {
      // SS_FULL: Pad with enough space to justify the value
      if (full && !center) {
        // Move the leading colon from the value to the label
        if (mv_colon) n -= lcd_put_u8str(F(":"));
        // Pad in-between
        for (; pad > 0; --pad) { lcd_put_u8str(F(" ")); n--; }
      }
      n -= lcd_put_u8str_max(vstr, n);
    }

    for (; n > 0; --n) lcd_put_u8str(F(" "));
  }

  // Draw a generic menu item with pre_char (if selected) and post_char
  void MenuItemBase::_draw(const bool sel, const uint8_t row, FSTR_P const ftpl, const char pre_char, const char post_char) {
    lcd_put_lchar(0, row, sel ? pre_char : ' ');
    uint8_t n = LCD_WIDTH - 2;
    n -= lcd_put_u8str(ftpl, itemIndex, itemStringC, itemStringF, n);
    for (; n; --n) lcd_put_u8str(F(" "));
    lcd_put_lchar(post_char);
  }

  // Draw a menu item with a (potentially) editable value
  void MenuEditItemBase::draw(const bool sel, const uint8_t row, FSTR_P const ftpl, const char * const inStr, const bool pgm) {
    const uint8_t vlen = inStr ? (pgm ? utf8_strlen_P(inStr) : utf8_strlen(inStr)) : 0;
    lcd_put_lchar(0, row, sel ? LCD_STR_ARROW_RIGHT[0] : ' ');
    uint8_t n = LCD_WIDTH - 2 - vlen;
    n -= lcd_put_u8str(ftpl, itemIndex, itemStringC, itemStringF, n);
    if (vlen) {
      lcd_put_u8str(F(":"));
      for (; n; --n) lcd_put_u8str(F(" "));
      if (pgm) lcd_put_u8str_P(inStr); else lcd_put_u8str(inStr);
    }
  }

  // Low-level draw_edit_screen can be used to draw an edit screen from anyplace
  void MenuEditItemBase::draw_edit_screen(FSTR_P const ftpl, const char * const value/*=nullptr*/) {
    ui.encoder_direction_normal();
    uint8_t n = LCD_WIDTH - 1;
    n -= lcd_put_u8str(0, 1, ftpl, itemIndex, itemStringC, itemStringF, n);
    if (value) {
      lcd_put_u8str(F(":")); n--;
      const uint8_t len = utf8_strlen(value) + 1;   // Plus one for a leading space
      const lcd_uint_t valrow = n < len ? 2 : 1;    // Value on the next row if it won't fit
      lcd_put_lchar(LCD_WIDTH - len, valrow, ' ');  // Right-justified, padded, leading space
      lcd_put_u8str(value);
    }
  }

  // The Select Screen presents a prompt and two "buttons"
  void MenuItem_confirm::draw_select_screen(FSTR_P const yes, FSTR_P const no, const bool yesno, FSTR_P const fpre, const char * const string/*=nullptr*/, FSTR_P const fsuf/*=nullptr*/) {
    ui.draw_select_screen_prompt(fpre, string, fsuf);
    if (no) {
      SETCURSOR(0, LCD_HEIGHT - 1);
      lcd_put_lchar(yesno ? ' ' : '['); lcd_put_u8str(no); lcd_put_lchar(yesno ? ' ' : ']');
    }
    if (yes) {
      SETCURSOR_RJ(utf8_strlen(yes) + 2, LCD_HEIGHT - 1);
      lcd_put_lchar(yesno ? '[' : ' '); lcd_put_u8str(yes); lcd_put_lchar(yesno ? ']' : ' ');
    }
  }

  #if HAS_MEDIA

    void MenuItem_sdbase::draw(const bool sel, const uint8_t row, FSTR_P const, CardReader &theCard, const bool isDir) {
      lcd_put_lchar(0, row, sel ? LCD_STR_ARROW_RIGHT[0] : ' ');
      uint8_t n = LCD_WIDTH - 2;
      n -= lcd_put_u8str_max(ui.scrolled_filename(theCard, n, sel), n);
      for (; n; --n) lcd_put_u8str(F(" "));
      lcd_put_lchar(isDir ? LCD_STR_FOLDER[0] : ' ');
    }

  #endif

  #if ENABLED(LCD_HAS_STATUS_INDICATORS)

    void MarlinUI::update_indicators() {
      // Set the LEDS - referred to as backlights by the LiquidTWI2 library
      static uint8_t ledsprev = 0;
      uint8_t leds = 0;

      if (TERN0(HAS_HEATED_BED, thermalManager.degTargetBed() > 0)) leds |= LED_A;
      if (TERN0(HAS_HOTEND, thermalManager.degTargetHotend(0) > 0)) leds |= LED_B;

      #if HAS_FAN
        if ( TERN0(HAS_FAN0, thermalManager.fan_speed[0])
          || TERN0(HAS_FAN1, thermalManager.fan_speed[1])
          || TERN0(HAS_FAN2, thermalManager.fan_speed[2])
          || TERN0(HAS_FAN3, thermalManager.fan_speed[3])
          || TERN0(HAS_FAN4, thermalManager.fan_speed[4])
          || TERN0(HAS_FAN5, thermalManager.fan_speed[5])
          || TERN0(HAS_FAN6, thermalManager.fan_speed[6])
          || TERN0(HAS_FAN7, thermalManager.fan_speed[7])
        ) leds |= LED_C;
      #endif // HAS_FAN

      if (TERN0(HAS_MULTI_HOTEND, thermalManager.degTargetHotend(1) > 0)) leds |= LED_C;

      if (leds != ledsprev) {
        lcd.setBacklight(leds);
        ledsprev = leds;
      }
    }

  #endif // LCD_HAS_STATUS_INDICATORS

  #if ENABLED(AUTO_BED_LEVELING_UBL)

    #define HD44780_CHAR_WIDTH    5
    #define HD44780_CHAR_HEIGHT   8
    #define MESH_MAP_COLS         7
    #define MESH_MAP_ROWS         4

    #define CHAR_LINE_TOP         0
    #define CHAR_LINE_BOT         1
    #define CHAR_EDGE_L           2
    #define CHAR_EDGE_R           3
    #define CHAR_UL_UL            4
    #define CHAR_LR_UL            5
    #define CHAR_UL_LR            6
    #define CHAR_LR_LR            7

    #define TOP_LEFT         _BV(0)
    #define TOP_RIGHT        _BV(1)
    #define LOWER_LEFT       _BV(2)
    #define LOWER_RIGHT      _BV(3)

    /**
     * Possible map screens:
     *
     * 16x2   |X000.00  Y000.00|
     *        |(00,00)  Z00.000|
     *
     * 20x2   | X:000.00  Y:000.00 |
     *        | (00,00)   Z:00.000 |
     *
     * 16x4   |+-------+(00,00)|
     *        ||       |X000.00|
     *        ||       |Y000.00|
     *        |+-------+Z00.000|
     *
     * 20x4   | +-------+  (00,00) |
     *        | |       |  X:000.00|
     *        | |       |  Y:000.00|
     *        | +-------+  Z:00.000|
     */

    typedef struct {
      uint8_t custom_char_bits[HD44780_CHAR_HEIGHT];
    } custom_char;

    typedef struct {
      lcd_uint_t column, row,
                 x_pixel_offset, y_pixel_offset;
      uint8_t x_pixel_mask;
    } coordinate;

    void add_edges_to_custom_char(custom_char &custom, const coordinate &ul, const coordinate &lr, const coordinate &brc, const uint8_t cell_location);
    FORCE_INLINE static void clear_custom_char(custom_char * const cc) { ZERO(cc->custom_char_bits); }

    coordinate pixel_location(int16_t x, int16_t y) {
      coordinate ret_val;
      int16_t xp, yp, r, c;

      x++; y++; // +1 because lines on the left and top

      c = x / (HD44780_CHAR_WIDTH);
      r = y / (HD44780_CHAR_HEIGHT);

      ret_val.column = c;
      ret_val.row    = r;

      xp = x - c * (HD44780_CHAR_WIDTH);                                    // Get the pixel offsets into the character cell
      xp = HD44780_CHAR_WIDTH - 1 - xp;                                     // Column within relevant character cell (0 on the right)
      yp = y - r * (HD44780_CHAR_HEIGHT);

      ret_val.x_pixel_mask   = _BV(xp);
      ret_val.x_pixel_offset = xp;
      ret_val.y_pixel_offset = yp;
      return ret_val;
    }

    inline coordinate pixel_location(const lcd_uint_t x, const lcd_uint_t y) { return pixel_location((int16_t)x, (int16_t)y); }

    void prep_and_put_map_char(custom_char &chrdata, const coordinate &ul, const coordinate &lr, const coordinate &brc, const uint8_t cl, const char c, const lcd_uint_t x, const lcd_uint_t y) {
      add_edges_to_custom_char(chrdata, ul, lr, brc, cl);
      lcd.createChar(c, (uint8_t*)&chrdata);
      lcd_put_lchar(x, y, c);
    }

    void MarlinUI::ubl_plot(const uint8_t x_plot, const uint8_t y_plot) {

      #if LCD_WIDTH >= 20
        #define _LCD_W_POS 12
        #define _PLOT_X 1
        #define _MAP_X 3
        #define _LABEL(C,X,Y) lcd_put_u8str_P(X, Y, C)
        #define _XLABEL(X,Y) _LABEL(X_LBL,X,Y)
        #define _YLABEL(X,Y) _LABEL(Y_LBL,X,Y)
        #define _ZLABEL(X,Y) _LABEL(Z_LBL,X,Y)
      #else
        #define _LCD_W_POS 8
        #define _PLOT_X 0
        #define _MAP_X 1
        #define _LABEL(X,Y,C) lcd_put_lchar(X, Y, C)
        #define _XLABEL(X,Y) _LABEL('X',X,Y)
        #define _YLABEL(X,Y) _LABEL('Y',X,Y)
        #define _ZLABEL(X,Y) _LABEL('Z',X,Y)
      #endif

      #if LCD_HEIGHT <= 3   // 16x2 or 20x2 display

        /**
         * Show X and Y positions
         */
        _XLABEL(_PLOT_X, 0);
        lcd_put_u8str(ftostr52(LOGICAL_X_POSITION(bedlevel.get_mesh_x(x_plot))));
        _YLABEL(_LCD_W_POS, 0);
        lcd_put_u8str(ftostr52(LOGICAL_Y_POSITION(bedlevel.get_mesh_y(y_plot))));

        lcd_moveto(_PLOT_X, 0);

      #else // 16x4 or 20x4 display

        coordinate upper_left, lower_right, bottom_right_corner;
        custom_char new_char;
        uint8_t i, n, n_rows, n_cols;
        lcd_uint_t j, k, l, m, bottom_line, right_edge,
                   x_map_pixels, y_map_pixels,
                   pixels_per_x_mesh_pnt, pixels_per_y_mesh_pnt,
                   suppress_x_offset = 0, suppress_y_offset = 0;

        const uint8_t y_plot_inv = (GRID_MAX_POINTS_Y) - 1 - y_plot;

        upper_left.column  = 0;
        upper_left.row     = 0;
        lower_right.column = 0;
        lower_right.row    = 0;

        clear_for_drawing();

        x_map_pixels = (HD44780_CHAR_WIDTH) * (MESH_MAP_COLS) - 2;          // Minus 2 because we are drawing a box around the map
        y_map_pixels = (HD44780_CHAR_HEIGHT) * (MESH_MAP_ROWS) - 2;

        pixels_per_x_mesh_pnt = x_map_pixels / (GRID_MAX_POINTS_X);
        pixels_per_y_mesh_pnt = y_map_pixels / (GRID_MAX_POINTS_Y);

        if (pixels_per_x_mesh_pnt >= HD44780_CHAR_WIDTH) {                  // There are only 2 custom characters available, so the X
          pixels_per_x_mesh_pnt = HD44780_CHAR_WIDTH;                       // Size of the mesh point needs to fit within them independent
          suppress_x_offset = 1;                                            // Of where the starting pixel is located.
        }

        if (pixels_per_y_mesh_pnt >= HD44780_CHAR_HEIGHT) {                 // There are only 2 custom characters available, so the Y
          pixels_per_y_mesh_pnt = HD44780_CHAR_HEIGHT;                      // Size of the mesh point needs to fit within them independent
          suppress_y_offset = 1;                                            // Of where the starting pixel is located.
        }

        x_map_pixels = pixels_per_x_mesh_pnt * (GRID_MAX_POINTS_X);         // Now we have the right number of pixels to make both
        y_map_pixels = pixels_per_y_mesh_pnt * (GRID_MAX_POINTS_Y);         // Directions fit nicely

        right_edge   = pixels_per_x_mesh_pnt * (GRID_MAX_POINTS_X) + 1;     // Find location of right edge within the character cell
        bottom_line  = pixels_per_y_mesh_pnt * (GRID_MAX_POINTS_Y) + 1;     // Find location of bottom line within the character cell

        n_rows = bottom_line / (HD44780_CHAR_HEIGHT) + 1;
        n_cols = right_edge / (HD44780_CHAR_WIDTH) + 1;

        for (i = 0; i < n_cols; i++) {
          lcd_put_lchar(i, 0, CHAR_LINE_TOP);                               // Box Top line
          lcd_put_lchar(i, n_rows - 1, CHAR_LINE_BOT);                      // Box Bottom line
        }

        for (j = 0; j < n_rows; j++) {
          lcd_put_lchar(0, j, CHAR_EDGE_L);                                 // Box Left edge
          lcd_put_lchar(n_cols - 1, j, CHAR_EDGE_R);                        // Box Right edge
        }

        /**
         * If the entire 4th row is not in use, do not put vertical bars all the way down to the bottom of the display
         */

        k = pixels_per_y_mesh_pnt * (GRID_MAX_POINTS_Y) + 2;
        l = (HD44780_CHAR_HEIGHT) * n_rows;
        if (l > k && l - k >= (HD44780_CHAR_HEIGHT) / 2) {
          lcd_put_lchar(0, n_rows - 1, ' ');                                // Box Left edge
          lcd_put_lchar(n_cols - 1, n_rows - 1, ' ');                       // Box Right edge
        }

        clear_custom_char(&new_char);
        new_char.custom_char_bits[0] = 0b11111U;                            // Char #0 is used for the box top line
        lcd.createChar(CHAR_LINE_TOP, (uint8_t*)&new_char);

        clear_custom_char(&new_char);
        k = (GRID_MAX_POINTS_Y) * pixels_per_y_mesh_pnt + 1;                // Row of pixels for the bottom box line
        l = k % (HD44780_CHAR_HEIGHT);                                      // Row within relevant character cell
        new_char.custom_char_bits[l] = 0b11111U;                            // Char #1 is used for the box bottom line
        lcd.createChar(CHAR_LINE_BOT, (uint8_t*)&new_char);

        clear_custom_char(&new_char);
        for (j = 0; j < HD44780_CHAR_HEIGHT; j++)
          new_char.custom_char_bits[j] = 0b10000U;                          // Char #2 is used for the box left edge
        lcd.createChar(CHAR_EDGE_L, (uint8_t*)&new_char);

        clear_custom_char(&new_char);
        m = (GRID_MAX_POINTS_X) * pixels_per_x_mesh_pnt + 1;                // Column of pixels for the right box line
        n = m % (HD44780_CHAR_WIDTH);                                       // Column within relevant character cell
        i = HD44780_CHAR_WIDTH - 1 - n;                                     // Column within relevant character cell (0 on the right)
        for (j = 0; j < HD44780_CHAR_HEIGHT; j++)
          new_char.custom_char_bits[j] = (uint8_t)_BV(i);                   // Char #3 is used for the box right edge
        lcd.createChar(CHAR_EDGE_R, (uint8_t*)&new_char);

        i = x_plot * pixels_per_x_mesh_pnt - suppress_x_offset;
        j = y_plot_inv * pixels_per_y_mesh_pnt - suppress_y_offset;
        upper_left = pixel_location(i, j);

        k = (x_plot + 1) * pixels_per_x_mesh_pnt - 1 - suppress_x_offset;
        l = (y_plot_inv + 1) * pixels_per_y_mesh_pnt - 1 - suppress_y_offset;
        lower_right = pixel_location(k, l);

        bottom_right_corner = pixel_location(x_map_pixels, y_map_pixels);

        /**
         * First, handle the simple case where everything is within a single character cell.
         * If part of the Mesh Plot is outside of this character cell, we will follow up
         * and deal with that next.
         */

        clear_custom_char(&new_char);
        const lcd_uint_t ypix = _MIN(upper_left.y_pixel_offset + pixels_per_y_mesh_pnt, HD44780_CHAR_HEIGHT);
        for (j = upper_left.y_pixel_offset; j < ypix; j++) {
          i = upper_left.x_pixel_mask;
          for (k = 0; k < pixels_per_x_mesh_pnt; k++) {
            new_char.custom_char_bits[j] |= i;
            i >>= 1;
          }
        }

        prep_and_put_map_char(new_char, upper_left, lower_right, bottom_right_corner, TOP_LEFT, CHAR_UL_UL, upper_left.column, upper_left.row);

        /**
         * Next, check for two side by side character cells being used to display the Mesh Point
         * If found...  do the right hand character cell next.
         */
        if (upper_left.column == lower_right.column - 1) {
          l = upper_left.x_pixel_offset;
          clear_custom_char(&new_char);
          for (j = upper_left.y_pixel_offset; j < ypix; j++) {
            i = _BV(HD44780_CHAR_WIDTH - 1);                                // Fill in the left side of the right character cell
            for (k = 0; k < pixels_per_x_mesh_pnt - 1 - l; k++) {
              new_char.custom_char_bits[j] |= i;
              i >>= 1;
            }
          }
          prep_and_put_map_char(new_char, upper_left, lower_right, bottom_right_corner, TOP_RIGHT, CHAR_LR_UL, lower_right.column, upper_left.row);
        }

        /**
         * Next, check for two character cells stacked on top of each other being used to display the Mesh Point
         */
        if (upper_left.row == lower_right.row - 1) {
          l = HD44780_CHAR_HEIGHT - upper_left.y_pixel_offset;              // Number of pixel rows in top character cell
          k = pixels_per_y_mesh_pnt - l;                                    // Number of pixel rows in bottom character cell
          clear_custom_char(&new_char);
          for (j = 0; j < k; j++) {
            i = upper_left.x_pixel_mask;
            for (m = 0; m < pixels_per_x_mesh_pnt; m++) {                   // Fill in the top side of the bottom character cell
              new_char.custom_char_bits[j] |= i;
              if (!(i >>= 1)) break;
            }
          }
          prep_and_put_map_char(new_char, upper_left, lower_right, bottom_right_corner, LOWER_LEFT, CHAR_UL_LR, upper_left.column, lower_right.row);
        }

        /**
         * Next, check for four character cells being used to display the Mesh Point.  If that is
         * what is here, we work to fill in the character cell that is down one and to the right one
         * from the upper_left character cell.
         */

        if (upper_left.column == lower_right.column - 1 && upper_left.row == lower_right.row - 1) {
          l = HD44780_CHAR_HEIGHT - upper_left.y_pixel_offset;              // Number of pixel rows in top character cell
          k = pixels_per_y_mesh_pnt - l;                                    // Number of pixel rows in bottom character cell
          clear_custom_char(&new_char);
          for (j = 0; j < k; j++) {
            l = upper_left.x_pixel_offset;
            i = _BV(HD44780_CHAR_WIDTH - 1);                                // Fill in the left side of the right character cell
            for (m = 0; m < pixels_per_x_mesh_pnt - 1 - l; m++) {           // Fill in the top side of the bottom character cell
              new_char.custom_char_bits[j] |= i;
              i >>= 1;
            }
          }
          prep_and_put_map_char(new_char, upper_left, lower_right, bottom_right_corner, LOWER_RIGHT, CHAR_LR_LR, lower_right.column, lower_right.row);
        }

      #endif

      /**
       * Print plot position
       */
      lcd_put_lchar(_LCD_W_POS, 0, '(');
      lcd_put_u8str(ui8tostr3rj(x_plot));
      lcd_put_u8str(F(","));
      lcd_put_u8str(ui8tostr3rj(y_plot));
      lcd_put_u8str(F(")"));

      #if LCD_HEIGHT <= 3   // 16x2 or 20x2 display

        /**
         * Print Z values
         */
        _ZLABEL(_LCD_W_POS, 1);
        if (!isnan(bedlevel.z_values[x_plot][y_plot]))
          lcd_put_u8str(ftostr43sign(bedlevel.z_values[x_plot][y_plot]));
        else
          lcd_put_u8str(F(" -----"));

      #else                 // 16x4 or 20x4 display

        /**
         * Show all values at right of screen
         */
        _XLABEL(_LCD_W_POS, 1);
        lcd_put_u8str(ftostr52(LOGICAL_X_POSITION(bedlevel.get_mesh_x(x_plot))));
        _YLABEL(_LCD_W_POS, 2);
        lcd_put_u8str(ftostr52(LOGICAL_Y_POSITION(bedlevel.get_mesh_y(y_plot))));

        /**
         * Show the location value
         */
        _ZLABEL(_LCD_W_POS, 3);
        if (!isnan(bedlevel.z_values[x_plot][y_plot]))
          lcd_put_u8str(ftostr43sign(bedlevel.z_values[x_plot][y_plot]));
        else
          lcd_put_u8str(F(" -----"));

      #endif // LCD_HEIGHT > 3
    }

    void add_edges_to_custom_char(custom_char &custom, const coordinate &ul, const coordinate &lr, const coordinate &brc, const uint8_t cell_location) {
      uint8_t i, k;
      int16_t n_rows = lr.row    - ul.row    + 1,
              n_cols = lr.column - ul.column + 1;

      /**
       * Check if Top line of box needs to be filled in
       */

      if (ul.row == 0 && (cell_location & (TOP_LEFT|TOP_RIGHT))) {   // Only fill in the top line for the top character cells

        if (n_cols == 1) {
          if (ul.column != brc.column)
            custom.custom_char_bits[0] = 0xFF;                              // Single column in middle
          else
            for (i = brc.x_pixel_offset; i < HD44780_CHAR_WIDTH; i++)       // Single column on right side
              SBI(custom.custom_char_bits[0], i);
        }
        else if ((cell_location & TOP_LEFT) || lr.column != brc.column)     // Multiple column in the middle or with right cell in middle
          custom.custom_char_bits[0] = 0xFF;
        else
          for (i = brc.x_pixel_offset; i < HD44780_CHAR_WIDTH; i++)
            SBI(custom.custom_char_bits[0], i);
      }

      /**
       * Check if left line of box needs to be filled in
       */
      if (cell_location & (TOP_LEFT|LOWER_LEFT)) {
        if (ul.column == 0) {                                               // Left column of characters on LCD Display
          k = ul.row == brc.row ? brc.y_pixel_offset : HD44780_CHAR_HEIGHT; // If it isn't the last row... do the full character cell
          for (i = 0; i < k; i++)
            SBI(custom.custom_char_bits[i], HD44780_CHAR_WIDTH - 1);
        }
      }

      /**
       * Check if bottom line of box needs to be filled in
       */

      // Single row of mesh plot cells
      if (n_rows == 1 /* && (cell_location & (TOP_LEFT|TOP_RIGHT)) */ && ul.row == brc.row) {
        if (n_cols == 1)                                                    // Single row, single column case
          k = ul.column == brc.column ? brc.x_pixel_mask : 0x01;
        else if (cell_location & TOP_RIGHT)                                 // Single row, multiple column case
          k = lr.column == brc.column ? brc.x_pixel_mask : 0x01;
        else                                                                // Single row, left of multiple columns
          k = 0x01;
        while (k < _BV(HD44780_CHAR_WIDTH)) {
          custom.custom_char_bits[brc.y_pixel_offset] |= k;
          k <<= 1;
        }
      }

      // Double row of characters on LCD Display
      // And this is a bottom custom character
      if (n_rows == 2 && (cell_location & (LOWER_LEFT|LOWER_RIGHT)) && lr.row == brc.row) {
        if (n_cols == 1)                                                    // Double row, single column case
          k = ul.column == brc.column ? brc.x_pixel_mask : 0x01;
        else if (cell_location & LOWER_RIGHT)                               // Double row, multiple column case
          k = lr.column == brc.column ? brc.x_pixel_mask : 0x01;
        else                                                                // Double row, left of multiple columns
          k = 0x01;
        while (k < _BV(HD44780_CHAR_WIDTH)) {
          custom.custom_char_bits[brc.y_pixel_offset] |= k;
          k <<= 1;
        }
      }

      /**
       * Check if right line of box needs to be filled in
       */

      // Nothing to do if the lower right part of the mesh pnt isn't in the same column as the box line
      if (lr.column == brc.column) {
        // This mesh point is in the same character cell as the right box line
        if (ul.column == brc.column || (cell_location & (TOP_RIGHT|LOWER_RIGHT))) {
          // If not the last row... do the full character cell
          k = ul.row == brc.row ? brc.y_pixel_offset : HD44780_CHAR_HEIGHT;
          for (i = 0; i < k; i++) custom.custom_char_bits[i] |= brc.x_pixel_mask;
        }
      }
    }

  #endif // AUTO_BED_LEVELING_UBL

#endif // HAS_MARLINUI_MENU

#endif // HAS_MARLINUI_HD44780
