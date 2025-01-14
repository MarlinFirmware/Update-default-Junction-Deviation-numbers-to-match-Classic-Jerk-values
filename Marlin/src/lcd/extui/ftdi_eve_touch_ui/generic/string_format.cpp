/*********************
 * string_format.cpp *
 *********************/

/****************************************************************************
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

#include "../config.h"

#if ENABLED(TOUCH_UI_FTDI_EVE)

#include "../screens.h"

#define ROUND(val) uint16_t((val)+0.5)

#pragma GCC diagnostic push
#if GCC_VERSION <= 50000
  #pragma GCC diagnostic ignored "-Wno-format"
#endif

/**
 * Formats a temperature string (e.g. "100°C")
 */
void format_temp(char *str, const_celsius_float_t t1) {
  #ifdef TOUCH_UI_LCD_TEMP_PRECISION
    char num1[7];
    dtostrf(t1, 4 + TOUCH_UI_LCD_TEMP_PRECISION, TOUCH_UI_LCD_TEMP_PRECISION, num1);
    sprintf_P(str, PSTR("%s" S_FMT), num1, GET_TEXT(MSG_UNITS_C));
  #else
    sprintf_P(str, PSTR("%3d" S_FMT), ROUND(t1), GET_TEXT(MSG_UNITS_C));
  #endif
}

/**
 * Formats a temperature string for an idle heater (e.g. "100 °C / idle")
 */
void format_temp_and_idle(char *str, const_celsius_float_t t1) {
  #ifdef TOUCH_UI_LCD_TEMP_PRECISION
    char num1[7];
    dtostrf(t1, 4 + TOUCH_UI_LCD_TEMP_PRECISION, TOUCH_UI_LCD_TEMP_PRECISION, num1);
    sprintf_P(str, PSTR("%s" S_FMT " / " S_FMT), num1, GET_TEXT(MSG_UNITS_C), GET_TEXT(MSG_IDLE));
  #else
    sprintf_P(str, PSTR("%3d" S_FMT " / " S_FMT), ROUND(t1), GET_TEXT(MSG_UNITS_C), GET_TEXT(MSG_IDLE));
  #endif
}

/**
 * Formats a temperature string for an active heater (e.g. "100 / 200°C")
 */
void format_temp_and_temp(char *str, const_celsius_float_t t1, const_celsius_float_t t2) {
  #ifdef TOUCH_UI_LCD_TEMP_PRECISION
    char num1[7], num2[7];
    dtostrf(t1, 4 + TOUCH_UI_LCD_TEMP_PRECISION, TOUCH_UI_LCD_TEMP_PRECISION, num1);
    dtostrf(t2, 4 + TOUCH_UI_LCD_TEMP_PRECISION, TOUCH_UI_LCD_TEMP_PRECISION, num2);
    sprintf_P(str, PSTR("%s / %s" S_FMT), num1, num2, GET_TEXT(MSG_UNITS_C));
  #else
    sprintf_P(str, PSTR("%3d / %3d" S_FMT), ROUND(t1), ROUND(t2), GET_TEXT(MSG_UNITS_C));
  #endif
}

/**
 * Formats a temperature string for a material (e.g. "100°C (PLA)")
 */
void format_temp_and_material(char *str, const_celsius_float_t t1, const char *material) {
  #ifdef TOUCH_UI_LCD_TEMP_PRECISION
    char num1[7];
    dtostrf(t1, 4 + TOUCH_UI_LCD_TEMP_PRECISION, TOUCH_UI_LCD_TEMP_PRECISION, num1);
    sprintf_P(str, PSTR("%s" S_FMT " (" S_FMT ")"), num1, GET_TEXT(MSG_UNITS_C), material);
  #else
    sprintf_P(str, PSTR("%3d" S_FMT " (" S_FMT ")"), ROUND(t1), GET_TEXT(MSG_UNITS_C), material);
  #endif
}

/**
 * Formats a position value (e.g. "10 mm")
 */
void format_position(char *str, float p, uint8_t decimals) {
  dtostrf(p, 4 + decimals, decimals, str);
  strcat_P(str, PSTR(" "));
  //strcat_P(str, GET_TEXT(MSG_UNITS_MM));
}

/**
 * Formats a position vector (e.g. "10; 20; 30 mm")
 */
void format_position(char *str, float x, float y, float z) {
  char num1[7], num2[7], num3[7];
  dtostrf(x, 4, 2, num1);
  dtostrf(y, 4, 2, num2);
  dtostrf(z, 4, 2, num3);
  sprintf_P(str, PSTR("%s; %s; %s " S_FMT), num1, num2, num3, GET_TEXT(MSG_UNITS_MM));
}

#pragma GCC diagnostic pop

#endif // TOUCH_UI_FTDI_EVE
