/************
 * colors.h *
 ************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2019 - Cocoa Press                          *
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

#pragma once

namespace Theme {
  #if ENABLED(TOUCH_UI_COCOA_THEME)
    constexpr int      accent_hue           = 23;

    // Browns and Oranges
    constexpr uint32_t accent_color_1       = hsl_to_rgb(12.8,0.597,0.263); // Darkest
    constexpr uint32_t accent_color_2       = hsl_to_rgb(12.8,0.597,0.263);
    constexpr uint32_t accent_color_3       = hsl_to_rgb( 9.6,0.664,0.443);
    constexpr uint32_t accent_color_4       = hsl_to_rgb(16.3,0.873,0.537);
    constexpr uint32_t accent_color_5       = hsl_to_rgb(23.0,0.889,0.539);
    constexpr uint32_t accent_color_6       = hsl_to_rgb(23.0,0.889,0.539); // Lightest
  #else
    // Use linear accent colors

    #if ANY(TOUCH_UI_ROYAL_THEME, TOUCH_UI_FROZEN_THEME)
      // Dark blue accent colors
      constexpr int      accent_hue       = 216;
      constexpr float    accent_sat       = 0.7;
    #else
      // Green accent colors
      constexpr int      accent_hue       = 68;
      constexpr float    accent_sat       = 0.68;
    #endif

    // Shades of accent color
    constexpr uint32_t accent_color_0       = hsl_to_rgb(accent_hue, accent_sat, 0.15); // Darkest
    constexpr uint32_t accent_color_1       = hsl_to_rgb(accent_hue, accent_sat, 0.26);
    constexpr uint32_t accent_color_2       = hsl_to_rgb(accent_hue, accent_sat, 0.39);
    constexpr uint32_t accent_color_3       = hsl_to_rgb(accent_hue, accent_sat, 0.52);
    constexpr uint32_t accent_color_4       = hsl_to_rgb(accent_hue, accent_sat, 0.65);
    constexpr uint32_t accent_color_5       = hsl_to_rgb(accent_hue, accent_sat, 0.78);
    constexpr uint32_t accent_color_6       = hsl_to_rgb(accent_hue, accent_sat, 0.91); // Lightest
  #endif

  // Shades of gray

  constexpr float    gray_sat               = 0.14;
  constexpr uint32_t gray_color_0           = hsl_to_rgb(accent_hue, gray_sat, 0.10); // Darkest
  constexpr uint32_t gray_color_1           = hsl_to_rgb(accent_hue, gray_sat, 0.26);
  constexpr uint32_t gray_color_2           = hsl_to_rgb(accent_hue, gray_sat, 0.39);
  constexpr uint32_t gray_color_3           = hsl_to_rgb(accent_hue, gray_sat, 0.52);
  constexpr uint32_t gray_color_4           = hsl_to_rgb(accent_hue, gray_sat, 0.65);
  constexpr uint32_t gray_color_5           = hsl_to_rgb(accent_hue, gray_sat, 0.78);
  constexpr uint32_t gray_color_6           = hsl_to_rgb(accent_hue, gray_sat, 0.91); // Lightest

  #if ENABLED(TOUCH_UI_ROYAL_THEME)
    constexpr uint32_t theme_darkest        = accent_color_1;
    constexpr uint32_t theme_dark           = accent_color_4;

    constexpr uint32_t bg_color             = gray_color_0;
    constexpr uint32_t axis_label           = gray_color_1;

    constexpr uint32_t bg_text_enabled      = accent_color_6;
    constexpr uint32_t bg_text_disabled     = gray_color_0;
    constexpr uint32_t bg_normal            = accent_color_4;
    constexpr uint32_t fg_disabled          = gray_color_0;
    constexpr uint32_t fg_normal            = accent_color_0;
    constexpr uint32_t fg_action            = accent_color_1;

    constexpr uint32_t logo_bg_rgb          = accent_color_1;
    constexpr uint32_t logo_fill_rgb        = accent_color_0;
    constexpr uint32_t logo_stroke_rgb      = accent_color_4;

    constexpr uint32_t bed_mesh_lines_rgb   = 0xFFFFFF;
    constexpr uint32_t bed_mesh_shadow_rgb  = 0x444444;
  #elif ANY(TOUCH_UI_COCOA_THEME, TOUCH_UI_FROZEN_THEME)
    constexpr uint32_t theme_darkest        = accent_color_1;
    constexpr uint32_t theme_dark           = accent_color_4;

    constexpr uint32_t bg_color             = 0xFFFFFF;
    constexpr uint32_t axis_label           = gray_color_5;

    constexpr uint32_t bg_text_enabled      = accent_color_1;
    constexpr uint32_t bg_text_disabled     = gray_color_1;
    constexpr uint32_t bg_normal            = accent_color_4;
    constexpr uint32_t fg_disabled          = gray_color_6;
    constexpr uint32_t fg_normal            = accent_color_1;
    constexpr uint32_t fg_action            = accent_color_4;

    constexpr uint32_t logo_bg_rgb          = accent_color_5;
    constexpr uint32_t logo_fill_rgb        = accent_color_6;
    constexpr uint32_t logo_stroke_rgb      = accent_color_2;

    constexpr uint32_t bed_mesh_lines_rgb   = accent_color_6;
    constexpr uint32_t bed_mesh_shadow_rgb  = 0x444444;
    #define BED_MESH_POINTS_GRAY
  #else
    constexpr uint32_t theme_darkest        = gray_color_1;
    constexpr uint32_t theme_dark           = gray_color_2;

    constexpr uint32_t bg_dark_color        = gray_color_0;
    constexpr uint32_t bg_color             = gray_color_1;
    constexpr uint32_t axis_label           = gray_color_2;
    constexpr uint32_t bg_light_color       = gray_color_3;

    constexpr uint32_t bg_text_enabled      = 0xFFFFFF;
    constexpr uint32_t bg_text_disabled     = gray_color_2;
    constexpr uint32_t bg_normal            = gray_color_1;
    constexpr uint32_t fg_disabled          = gray_color_1;
    constexpr uint32_t fg_normal            = gray_color_2;
    constexpr uint32_t fg_action            = accent_color_2;

    constexpr uint32_t logo_bg_rgb          = accent_color_4;
    constexpr uint32_t logo_fill_rgb        = accent_color_3;
    constexpr uint32_t logo_stroke_rgb      = 0x000000;
    constexpr uint32_t lulzbot_accent       = 0xA8A845;

    constexpr uint32_t bed_mesh_lines_rgb   = 0xFFFFFF;
    constexpr uint32_t bed_mesh_shadow_rgb  = 0x444444;
  #endif

  constexpr uint32_t shadow_rgb             = gray_color_6;
  constexpr uint32_t stroke_rgb             = accent_color_1;
  constexpr uint32_t fill_rgb               = accent_color_3;
  #if ENABLED(TOUCH_UI_COCOA_PRESS)
    constexpr uint32_t syringe_rgb          = 0xFFFFFF;
    constexpr uint32_t fluid_rgb            = accent_color_5;
  #else
    constexpr uint32_t syringe_rgb          = accent_color_5;
    constexpr uint32_t fluid_rgb            = accent_color_3;
  #endif

  #if ENABLED(TOUCH_UI_ROYAL_THEME)
    constexpr uint32_t x_axis               = hsl_to_rgb(0,   1.00, 0.26);
    constexpr uint32_t y_axis               = hsl_to_rgb(120, 1.00, 0.13);
    constexpr uint32_t z_axis               = hsl_to_rgb(240, 1.00, 0.10);
  #else
    constexpr uint32_t x_axis               = hsl_to_rgb(0,   1.00, 0.5);
    constexpr uint32_t y_axis               = hsl_to_rgb(120, 1.00, 0.37);
    constexpr uint32_t z_axis               = hsl_to_rgb(240, 1.00, 0.37);
  #endif
  constexpr uint32_t e_axis                 = axis_label;
  constexpr uint32_t feedrate               = axis_label;
  constexpr uint32_t other                  = axis_label;
  constexpr uint32_t cold_pull              = hsl_to_rgb(190, 0.5, 0.50);
  constexpr uint32_t cancel                 = hsl_to_rgb( 25, 0.7, 0.50);

  // Status screen
  constexpr uint32_t progress               = axis_label;
  constexpr uint32_t status_msg             = axis_label;
  #if ENABLED(TOUCH_UI_ROYAL_THEME)
    constexpr uint32_t fan_speed            = hsl_to_rgb(240, 0.5, 0.13);
    constexpr uint32_t temp                 = hsl_to_rgb(343, 1.0, 0.23);
  #else
    constexpr uint32_t fan_speed            = hsl_to_rgb(30, 0.85, 0.50);
    constexpr uint32_t fan_speed_button     = hsl_to_rgb(30, 0.85, 0.50);
    constexpr uint32_t temp                 = hsl_to_rgb(accent_hue, accent_sat, 0.30);
    constexpr uint32_t temp_button          = hsl_to_rgb(accent_hue, accent_sat, 0.39);
  #endif

  constexpr uint32_t disabled_icon          = gray_color_1;

  // Calibration Registers Screen
  constexpr uint32_t transformA             = 0x3010D0;
  constexpr uint32_t transformB             = 0x4010D0;
  constexpr uint32_t transformC             = 0x5010D0;
  constexpr uint32_t transformD             = 0x6010D0;
  constexpr uint32_t transformE             = 0x7010D0;
  constexpr uint32_t transformF             = 0x8010D0;
  constexpr uint32_t transformVal           = 0x104010;

  constexpr btn_colors disabled_btn         = {.bg = bg_color,      .grad = fg_disabled, .fg = fg_disabled,     .rgb = fg_disabled };
  constexpr btn_colors normal_btn           = {.bg = fg_action,     .grad = 0xFFFFFF,    .fg = fg_normal,       .rgb = 0xFFFFFF };
  constexpr btn_colors accent_btn           = {.bg = fg_action,     .grad = 0xFFFFFF,    .fg = lulzbot_accent,  .rgb = 0xFFFFFF };
  constexpr btn_colors action_btn           = {.bg = bg_color,      .grad = 0xFFFFFF,    .fg = fg_action,       .rgb = 0xFFFFFF };
  constexpr btn_colors red_btn              = {.bg = 0xFF5555,      .grad = 0xFFFFFF,    .fg = 0xFF0000,        .rgb = 0xFFFFFF };
  constexpr btn_colors ui_slider            = {.bg = theme_darkest, .grad = 0xFFFFFF,    .fg = theme_dark,      .rgb = accent_color_3 };
  constexpr btn_colors ui_toggle            = {.bg = theme_darkest, .grad = 0xFFFFFF,    .fg = theme_dark,      .rgb = 0xFFFFFF };
  constexpr btn_colors text_x_axis_btn      = {.bg = fg_action,     .grad = 0xFFFFFF,    .fg = fg_normal,       .rgb = x_axis };
  constexpr btn_colors text_y_axis_btn      = {.bg = fg_action,     .grad = 0xFFFFFF,    .fg = fg_normal,       .rgb = y_axis };
  constexpr btn_colors text_z_axis_btn      = {.bg = fg_action,     .grad = 0xFFFFFF,    .fg = fg_normal,       .rgb = z_axis };
  constexpr btn_colors normal_text          = {.bg = fg_action,     .grad = fg_normal,   .fg = fg_normal,       .rgb = 0xFFFFFF };
  constexpr btn_colors text_x_axis          = {.bg = fg_action,     .grad = fg_normal,   .fg = fg_normal,       .rgb = x_axis };
  constexpr btn_colors text_y_axis          = {.bg = fg_action,     .grad = fg_normal,   .fg = fg_normal,       .rgb = y_axis };
  constexpr btn_colors text_z_axis          = {.bg = fg_action,     .grad = fg_normal,   .fg = fg_normal,       .rgb = z_axis };
  constexpr btn_colors cold_pull_btn        = {.bg = fg_action,     .grad = fg_normal,   .fg = cold_pull,       .rgb = 0xFFFFFF };
  constexpr btn_colors cancel_btn           = {.bg = fg_action,     .grad = fg_normal,   .fg = cancel,          .rgb = 0xFFFFFF };
  constexpr btn_colors temp_btn             = {.bg = bg_color,      .grad = fg_disabled, .fg = fg_disabled,     .rgb = 0xFFFFFF };
  constexpr btn_colors temp_disabled_btn    = {.bg = bg_color,      .grad = bg_dark_color, .fg = bg_dark_color, .rgb = bg_light_color };

  // Temperature color scale

  const rgb_t cool_rgb (100, 100,  80);
  const rgb_t low_rgb  (128,  50,   0);
  const rgb_t med_rgb  (200,  85,   0);
  const rgb_t high_rgb (255,  85,   0);
};
