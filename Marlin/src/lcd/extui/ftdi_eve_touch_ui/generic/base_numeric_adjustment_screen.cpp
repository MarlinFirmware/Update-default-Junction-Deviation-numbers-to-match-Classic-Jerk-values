/**************************************
 * base_numeric_adjustment_screen.cpp *
 **************************************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
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
#include "../screens.h"
#include "../screen_data.h"

#ifdef FTDI_BASE_NUMERIC_ADJ_SCREEN

using namespace FTDI;
using namespace Theme;

constexpr static BaseNumericAdjustmentScreenData &mydata = screen_data.BaseNumericAdjustmentScreen;

#if ENABLED(TOUCH_UI_PORTRAIT)
  #define GRID_COLS 13
  #define GRID_ROWS (8+EXTRUDERS)
  #define LAYOUT_FONT font_small
#else
  #define GRID_COLS 18
  #define GRID_ROWS  7
  #define LAYOUT_FONT font_medium
#endif

BaseNumericAdjustmentScreen::widgets_t::widgets_t(draw_mode_t what) : _what(what) {
  CommandProcessor cmd;

  if (what & BACKGROUND) {
    cmd.cmd(CLEAR_COLOR_RGB(bg_color))
       .cmd(CLEAR(true,true,true))
       .colors(normal_btn)
       .cmd(COLOR_RGB(bg_text_enabled))
       .tag(0);
  }
  else
    cmd.colors(normal_btn);

  cmd.font(font_medium);
  _button(cmd, 1,
    #if ENABLED(TOUCH_UI_PORTRAIT)
      BTN_POS(1,GRID_ROWS), BTN_SIZE(13,1),
    #else
      BTN_POS(15,7), BTN_SIZE(4,1),
    #endif
    GET_TEXT_F(MSG_BUTTON_DONE), true, true
  );

  _line = 1;
  _units = F("");
}

/**
 * Speed optimization for changing button style.
 */
void BaseNumericAdjustmentScreen::widgets_t::_button_style(CommandProcessor &cmd, BaseNumericAdjustmentScreen::widgets_t::style_t style) {
  if (_style != style) {
    const btn_colors *old_colors = &normal_btn;
    const btn_colors *new_colors = &normal_btn;

    switch (_style) {
      case BTN_ACTION:   old_colors = &action_btn;   break;
      case BTN_TOGGLE:   old_colors = &ui_toggle;    break;
      case BTN_DISABLED: old_colors = &disabled_btn; break;
      default: break;
    }
    switch (style) {
      case BTN_ACTION:   new_colors = &action_btn;   break;
      case BTN_TOGGLE:   new_colors = &ui_toggle;    break;
      case BTN_DISABLED: new_colors = &disabled_btn; break;
      default: break;
    }

    const bool rgb_changed  =  (old_colors->rgb  != new_colors->rgb) ||
                               (_style == TEXT_LABEL && style != TEXT_LABEL) ||
                               (_style != TEXT_LABEL && style == TEXT_LABEL);
    const bool grad_changed =  old_colors->grad != new_colors->grad;
    const bool fg_changed   = (old_colors->fg   != new_colors->fg) || (_style == TEXT_AREA);
    const bool bg_changed   =  old_colors->bg   != new_colors->bg;

    if (rgb_changed)  cmd.cmd(COLOR_RGB(style == TEXT_LABEL ? bg_text_enabled : new_colors->rgb));
    if (grad_changed) cmd.gradcolor(new_colors->grad);
    if (fg_changed)   cmd.fgcolor(new_colors->fg);
    if (bg_changed)   cmd.bgcolor(new_colors->bg);

    _style = style;
  }
}

/**
 * Speed optimization for drawing buttons. Draw all unpressed buttons in the
 * background layer and draw only the pressed button in the foreground layer.
 */
void BaseNumericAdjustmentScreen::widgets_t::_button(CommandProcessor &cmd, uint8_t tag, int16_t x, int16_t y, int16_t w, int16_t h, FSTR_P text, bool enabled, bool highlight) {
  if (_what & BACKGROUND) enabled = true;
  if ((_what & BACKGROUND) || buttonIsPressed(tag) || highlight || !enabled) {
    _button_style(cmd, (!enabled) ? BTN_DISABLED : (highlight ? BTN_ACTION : BTN_NORMAL));
    cmd.tag(enabled ? tag : 0).button(x, y, w, h, text);
  }
}

BaseNumericAdjustmentScreen::widgets_t &BaseNumericAdjustmentScreen::widgets_t::precision(uint8_t decimals, precision_default_t initial) {
  _decimals = decimals;
  if (mydata.increment == 0) {
    mydata.increment = 243 + (initial - DEFAULT_LOWEST) - _decimals;
  }
  return *this;
}

void BaseNumericAdjustmentScreen::widgets_t::heading(FSTR_P label) {
  if (_what & BACKGROUND) {
    CommandProcessor cmd;
    _button_style(cmd, TEXT_LABEL);
    cmd.font(font_medium)
       .tag(0)
       .text(
         #if ENABLED(TOUCH_UI_PORTRAIT)
           BTN_POS(1, _line), BTN_SIZE(12,1),
         #else
           BTN_POS(5, _line), BTN_SIZE(8,1),
         #endif
         label
       );
  }

  _line++;
}

#if ENABLED(TOUCH_UI_PORTRAIT)
  #ifdef TOUCH_UI_800x480
    #undef EDGE_R
    #define EDGE_R 20
  #else
    #undef EDGE_R
    #define EDGE_R 10
  #endif
#endif

void BaseNumericAdjustmentScreen::widgets_t::_draw_increment_btn(CommandProcessor &cmd, uint8_t, const uint8_t tag) {
  const char        *label = PSTR("?");
  uint8_t            pos;
  uint8_t &          increment = mydata.increment;

  if (increment == 0) {
    increment = tag; // Set the default value to be the first.
  }

  switch (tag) {
    case 240: label = PSTR(   ".001"); pos = _decimals - 3; break;
    case 241: label = PSTR(   ".01" ); pos = _decimals - 2; break;
    case 242: label = PSTR(  "0.1"  ); pos = _decimals - 1; break;
    case 243: label = PSTR(  "1"    ); pos = _decimals + 0; break;
    case 244: label = PSTR( "10"    ); pos = _decimals + 1; break;
    default:  label = PSTR("100"    ); pos = _decimals + 2; break;
  }

  const bool highlight = (_what & FOREGROUND) && (increment == tag);

  switch (pos) {
    #if ENABLED(TOUCH_UI_PORTRAIT)
      case 0: _button(cmd, tag, BTN_POS(5,_line), BTN_SIZE(2,1), FPSTR(label), true, highlight); break;
      case 1: _button(cmd, tag, BTN_POS(7,_line), BTN_SIZE(2,1), FPSTR(label), true, highlight); break;
      case 2: _button(cmd, tag, BTN_POS(9,_line), BTN_SIZE(2,1), FPSTR(label), true, highlight); break;
    #else
      case 0: _button(cmd, tag, BTN_POS(15,2),    BTN_SIZE(4,1), FPSTR(label), true, highlight); break;
      case 1: _button(cmd, tag, BTN_POS(15,3),    BTN_SIZE(4,1), FPSTR(label), true, highlight); break;
      case 2: _button(cmd, tag, BTN_POS(15,4),    BTN_SIZE(4,1), FPSTR(label), true, highlight); break;
    #endif
  }
}

void BaseNumericAdjustmentScreen::widgets_t::increments() {
  CommandProcessor cmd;

  cmd.font(LAYOUT_FONT);

  if (_what & BACKGROUND) {
    _button_style(cmd, TEXT_LABEL);
    cmd.tag(0).text(
      #if ENABLED(TOUCH_UI_PORTRAIT)
        BTN_POS(1, _line), BTN_SIZE(4,1),
      #else
        BTN_POS(15,    1), BTN_SIZE(4,1),
      #endif
      GET_TEXT_F(MSG_INCREMENT)
    );
  }

  _draw_increment_btn(cmd, _line+1, 245 - _decimals);
  _draw_increment_btn(cmd, _line+1, 244 - _decimals);
  _draw_increment_btn(cmd, _line+1, 243 - _decimals);

  #if ENABLED(TOUCH_UI_PORTRAIT)
    _line++;
  #endif
}

void BaseNumericAdjustmentScreen::widgets_t::adjuster_sram_val(uint8_t tag, FSTR_P label, const char *value, bool is_enabled) {
  CommandProcessor cmd;

  if (_what & BACKGROUND) {
    _button_style(cmd, TEXT_LABEL);
    cmd.tag(0)
       .font(font_small)
       .text( BTN_POS(1,_line), BTN_SIZE(4,1), label);
    _button_style(cmd, TEXT_AREA);
    cmd.fgcolor(_color).button(BTN_POS(5,_line), BTN_SIZE(5,1), F(""), OPT_FLAT);
  }

  cmd.font(font_medium);
  _button(cmd, tag,     BTN_POS(10,_line), BTN_SIZE(2,1),  F("-"), is_enabled);
  _button(cmd, tag + 1, BTN_POS(12,_line), BTN_SIZE(2,1),  F("+"), is_enabled);

  if ((_what & FOREGROUND) && is_enabled) {
    _button_style(cmd, BTN_NORMAL);
    cmd.tag(0)
       .font(font_small)
       .text(BTN_POS(5,_line), BTN_SIZE(5,1), value);
  }

  _line++;
}

void BaseNumericAdjustmentScreen::widgets_t::adjuster(uint8_t tag, FSTR_P label, const char *value, bool is_enabled) {
  if (_what & BACKGROUND) {
    adjuster_sram_val(tag, label, nullptr);
  }

  if (_what & FOREGROUND) {
    char b[strlen(value) + 1];
    strcpy(b, value);
    adjuster_sram_val(tag, label, b, is_enabled);
  }
}

void BaseNumericAdjustmentScreen::widgets_t::adjuster(uint8_t tag, FSTR_P label, float value, bool is_enabled) {
  if (_what & BACKGROUND) {
    adjuster_sram_val(tag, label, nullptr);
  }

  if (_what & FOREGROUND) {
    char b[32];
    dtostrf(value, 5, _decimals, b);
    strcat_P(b, PSTR(" "));
    strcat_P(b, (const char*) _units);
    adjuster_sram_val(tag, label, b, is_enabled);
  }
}

void BaseNumericAdjustmentScreen::widgets_t::button(uint8_t tag, FSTR_P label, bool is_enabled) {
  CommandProcessor cmd;
  cmd.font(LAYOUT_FONT);
  _button(cmd, tag, BTN_POS(5,_line), BTN_SIZE(9,1), label, is_enabled);

  _line++;
}

void BaseNumericAdjustmentScreen::widgets_t::text_field(uint8_t tag, FSTR_P label, const char *value, bool is_enabled) {
  CommandProcessor cmd;

  if (_what & BACKGROUND) {
    _button_style(cmd, TEXT_LABEL);
    cmd.enabled(1)
       .tag(0)
       .font(font_small)
       .text(   BTN_POS(1,_line), BTN_SIZE(4,1), label);
    _button_style(cmd, TEXT_AREA);
    cmd.fgcolor(_color)
       .tag(tag)
       .button(BTN_POS(5,_line), BTN_SIZE(9,1), F(""), OPT_FLAT);
  }

  if (_what & FOREGROUND) {
    cmd.font(font_small).text( BTN_POS(5,_line), BTN_SIZE(9,1), is_enabled ? value : "-");
  }

  _line++;
}

void BaseNumericAdjustmentScreen::widgets_t::two_buttons(uint8_t tag1, FSTR_P label1, uint8_t tag2, FSTR_P label2, bool is_enabled) {
  CommandProcessor cmd;
  cmd.font(LAYOUT_FONT);
  _button(cmd, tag1, BTN_POS(5,_line),   BTN_SIZE(4.5,1), label1, is_enabled);
  _button(cmd, tag2, BTN_POS(9.5,_line), BTN_SIZE(4.5,1), label2, is_enabled);

  _line++;
}

void BaseNumericAdjustmentScreen::widgets_t::toggle(uint8_t tag, FSTR_P label, bool value, bool is_enabled) {
  CommandProcessor cmd;

  if (_what & BACKGROUND) {
    _button_style(cmd, TEXT_LABEL);
    cmd.font(font_small)
       .text(
        #if ENABLED(TOUCH_UI_PORTRAIT)
          BTN_POS(1, _line), BTN_SIZE( 8,1),
        #else
          BTN_POS(1, _line), BTN_SIZE(10,1),
        #endif
        label
       );
  }

  if (_what & FOREGROUND) {
    _button_style(cmd, is_enabled ? BTN_TOGGLE : BTN_DISABLED);
    cmd.tag(is_enabled ? tag   : 0)
       .enabled(is_enabled)
       .font(font_small)
       .toggle2(
        #if ENABLED(TOUCH_UI_PORTRAIT)
          BTN_POS( 9,_line), BTN_SIZE(5,1),
        #else
          BTN_POS(10,_line), BTN_SIZE(4,1),
        #endif
        GET_TEXT_F(MSG_NO), GET_TEXT_F(MSG_YES), value
       );
  }

  _line++;
}

void BaseNumericAdjustmentScreen::widgets_t::home_buttons(uint8_t tag) {
  CommandProcessor cmd;

  if (_what & BACKGROUND) {
    _button_style(cmd, TEXT_LABEL);
    cmd.font(font_small)
       .text(BTN_POS(1, _line), BTN_SIZE(4,1), GET_TEXT_F(MSG_HOME));
  }

  cmd.font(LAYOUT_FONT);
 _button(cmd, tag+0, BTN_POS(5,_line),    BTN_SIZE(2,1), GET_TEXT_F(MSG_AXIS_X));
 _button(cmd, tag+1, BTN_POS(7,_line),    BTN_SIZE(2,1), GET_TEXT_F(MSG_AXIS_Y));
 #if DISABLED(Z_SAFE_HOMING)
   _button(cmd, tag+2, BTN_POS(9,_line),  BTN_SIZE(2,1), GET_TEXT_F(MSG_AXIS_Z));
   _button(cmd, tag+3, BTN_POS(11,_line), BTN_SIZE(3,1), GET_TEXT_F(MSG_AXIS_ALL));
 #else
   _button(cmd, tag+3, BTN_POS(9,_line),  BTN_SIZE(3,1), GET_TEXT_F(MSG_AXIS_ALL));
 #endif

  _line++;
}

void BaseNumericAdjustmentScreen::onEntry() {
  mydata.increment = 0; // This will force the increment to be picked while drawing.
  BaseScreen::onEntry();
  CommandProcessor cmd;
  cmd.set_button_style_callback(nullptr);
}

bool BaseNumericAdjustmentScreen::onTouchEnd(uint8_t tag) {
  switch (tag) {
    case 1:           GOTO_PREVIOUS(); return true;
    case 240 ... 245: mydata.increment = tag; break;
    default:          return current_screen.onTouchHeld(tag);
  }
  return true;
}

float BaseNumericAdjustmentScreen::getIncrement() {
  switch (mydata.increment) {
    case 240: return   0.001;
    case 241: return   0.01;
    case 242: return   0.1;
    case 243: return   1.0;
    case 244: return  10.0;
    case 245: return 100.0;
    default:  return   0.0;
  }
}

#endif // FTDI_BASE_NUMERIC_ADJ_SCREEN
