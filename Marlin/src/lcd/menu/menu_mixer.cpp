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
// Mixer Menu
//

#include "../../inc/MarlinConfigPre.h"

#if ALL(HAS_MARLINUI_MENU, MIXING_EXTRUDER)

#include "menu_item.h"
#include "menu_addon.h"

#include "../../feature/mixing.h"

#if HAS_GRAPHICAL_TFT
  #include "../tft/tft.h"
#endif

#define CHANNEL_MIX_EDITING !HAS_DUAL_MIXING

#if ENABLED(GRADIENT_MIX)

  void _lcd_mixer_gradient_z_edit(const bool isend) {
    ui.defer_status_screen();
    ui.enable_encoder_multiplier(true);

    float &zvar = isend ? mixer.gradient.end_z : mixer.gradient.start_z;

    if (ui.encoderPosition) {
      zvar += float(int32_t(ui.encoderPosition)) * 0.1;
      ui.encoderPosition = 0;
      LIMIT(zvar, 0, Z_MAX_POS);
    }

    if (ui.should_draw()) {
      char tmp[16];
      SETCURSOR(1, (LCD_HEIGHT - 1) / 2);
      lcd_put_u8str(isend ? GET_TEXT_F(MSG_END_Z) : GET_TEXT_F(MSG_START_Z));
      sprintf_P(tmp, PSTR("%4d.%d mm"), int(zvar), int(zvar * 10) % 10);
      SETCURSOR_RJ(9, (LCD_HEIGHT - 1) / 2);
      lcd_put_u8str(tmp);
    }

    if (ui.lcd_clicked) {
      if (isend && zvar < mixer.gradient.start_z)
        mixer.gradient.start_z = zvar;
      else if (!isend && zvar > mixer.gradient.end_z)
        mixer.gradient.end_z = zvar;
      mixer.refresh_gradient();
      ui.goto_previous_screen();
    }
    else {
      TERN_(HAS_GRAPHICAL_TFT, tft.draw_edit_screen_buttons());
    }
  }

  void lcd_mixer_edit_gradient_menu() {
    START_MENU();
    BACK_ITEM(MSG_MIXER);

    EDIT_ITEM(int8, MSG_START_VTOOL, &mixer.gradient.start_vtool, 0, MIXING_VIRTUAL_TOOLS - 1, mixer.refresh_gradient);
    EDIT_ITEM(int8, MSG_END_VTOOL, &mixer.gradient.end_vtool, 0, MIXING_VIRTUAL_TOOLS - 1, mixer.refresh_gradient);

    #if ENABLED(GRADIENT_VTOOL)
      EDIT_ITEM(int8, MSG_GRADIENT_ALIAS, &mixer.gradient.vtool_index, -1, MIXING_VIRTUAL_TOOLS - 1, mixer.refresh_gradient);
    #endif

    char tmp[18];

    FSTR_P const slabel = GET_TEXT_F(MSG_START_Z);
    SUBMENU_F(slabel, []{ _lcd_mixer_gradient_z_edit(false); });
    MENU_ITEM_ADDON_START_RJ(11);
      sprintf_P(tmp, PSTR("%4d.%d mm"), int(mixer.gradient.start_z), int(mixer.gradient.start_z * 10) % 10);
      lcd_put_u8str(tmp);
    MENU_ITEM_ADDON_END();

    FSTR_P const elabel = GET_TEXT_F(MSG_END_Z);
    SUBMENU_F(elabel, []{ _lcd_mixer_gradient_z_edit(true); });
    MENU_ITEM_ADDON_START_RJ(11);
      sprintf_P(tmp, PSTR("%4d.%d mm"), int(mixer.gradient.end_z), int(mixer.gradient.end_z * 10) % 10);
      lcd_put_u8str(tmp);
    MENU_ITEM_ADDON_END();

    END_MENU();
  }

#endif // GRADIENT_MIX

static uint8_t v_index;

#if HAS_DUAL_MIXING
  void _lcd_draw_mix(const uint8_t y) {
    char tmp[20]; // "100%_100%"
    sprintf_P(tmp, PSTR("%3d%% %3d%% "), int(mixer.mix[0]), int(mixer.mix[1]));
    SETCURSOR(2, y); lcd_put_u8str(GET_TEXT_F(MSG_MIX));
    SETCURSOR_RJ(10, y); lcd_put_u8str(tmp);
  }
#endif

void _lcd_mixer_select_vtool() {
  mixer.T(v_index);
  TERN_(HAS_DUAL_MIXING, _lcd_draw_mix(LCD_HEIGHT - 1));
}

#if CHANNEL_MIX_EDITING

  void _lcd_mixer_cycle_mix() {
    uint16_t bits = 0;
    MIXER_STEPPER_LOOP(i) if (mixer.collector[i]) SBI(bits, i);
    bits = (bits + 1) & (_BV(MIXING_STEPPERS) - 1);
    if (!bits) ++bits;
    MIXER_STEPPER_LOOP(i) mixer.collector[i] = TEST(bits, i) ? 10.0f : 0.0f;
    ui.refresh();
  }

  void _lcd_mixer_commit_vtool() {
    mixer.normalize();
    ui.goto_previous_screen();
  }

#endif

void lcd_mixer_mix_edit() {

  #if HAS_DUAL_MIXING && !CHANNEL_MIX_EDITING

    // Adjust 2-channel mix from the encoder
    if (ui.encoderPosition) {
      mixer.mix[0] += int32_t(ui.encoderPosition);
      ui.encoderPosition = 0;
      if (mixer.mix[0] < 0) mixer.mix[0] += 101;
      if (mixer.mix[0] > 100) mixer.mix[0] -= 101;
      mixer.mix[1] = 100 - mixer.mix[0];
    }
    _lcd_draw_mix((LCD_HEIGHT - 1) / 2);

    // Click to commit the change
    if (ui.lcd_clicked) {
      mixer.update_vtool_from_mix();
      ui.goto_previous_screen();
    }

    TERN_(HAS_GRAPHICAL_TFT, tft.draw_edit_screen_buttons());

  #else

    START_MENU();
    BACK_ITEM(MSG_MIXER);

    #if CHANNEL_MIX_EDITING

      for (uint_fast8_t n = 1; n <= MIXING_STEPPERS; ++n)
        EDIT_ITEM_FAST_N(float42_52, n, MSG_MIX_COMPONENT_N, &mixer.collector[n-1], 0, 10);

      ACTION_ITEM(MSG_CYCLE_MIX, _lcd_mixer_cycle_mix);
      ACTION_ITEM(MSG_COMMIT_VTOOL, _lcd_mixer_commit_vtool);

    #endif

    END_MENU();

  #endif
}

#if HAS_DUAL_MIXING

  //
  // Toggle Dual-Mix
  //
  inline void _lcd_mixer_toggle_mix() {
    mixer.mix[0] = mixer.mix[0] == 100 ? 0 : 100;
    mixer.mix[1] = 100 - mixer.mix[0];
    mixer.update_vtool_from_mix();
    ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
  }

#endif

#if CHANNEL_MIX_EDITING

  //
  // Prepare and Edit Mix
  //
  inline void _lcd_goto_mix_edit() {
    mixer.refresh_collector(10, v_index);
    ui.goto_screen(lcd_mixer_mix_edit);
    lcd_mixer_mix_edit();
  }

#endif

#if ENABLED(GRADIENT_MIX)
  //
  // Reverse Gradient
  //
  inline void _lcd_mixer_reverse_gradient() {
    const uint8_t sv = mixer.gradient.start_vtool;
    mixer.gradient.start_vtool = mixer.gradient.end_vtool;
    mixer.gradient.end_vtool = sv;
    mixer.refresh_gradient();
    ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
  }
#endif

void menu_mixer() {
  START_MENU();
  BACK_ITEM(MSG_MAIN_MENU);

  v_index = mixer.get_current_vtool();
  EDIT_ITEM(uint8, MSG_ACTIVE_VTOOL, &v_index, 0, MIXING_VIRTUAL_TOOLS - 1, _lcd_mixer_select_vtool, ENABLED(HAS_DUAL_MIXING));

  #if HAS_DUAL_MIXING
  {
    char tmp[11];
    SUBMENU(MSG_MIX, lcd_mixer_mix_edit);
    MENU_ITEM_ADDON_START_RJ(9);
      mixer.update_mix_from_vtool();
      sprintf_P(tmp, PSTR("%3d;%3d%%"), int(mixer.mix[0]), int(mixer.mix[1]));
      lcd_put_u8str(tmp);
    MENU_ITEM_ADDON_END();
    ACTION_ITEM(MSG_TOGGLE_MIX, _lcd_mixer_toggle_mix);
  }
  #else
    SUBMENU(MSG_MIX, _lcd_goto_mix_edit);
  #endif

  //
  // Reset All V-Tools
  //
  CONFIRM_ITEM(MSG_RESET_VTOOLS,
    MSG_BUTTON_RESET, MSG_BUTTON_CANCEL,
    []{
      mixer.reset_vtools();
      LCD_MESSAGE(MSG_VTOOLS_RESET);
      ui.return_to_status();
    },
    nullptr,
    GET_TEXT_F(MSG_RESET_VTOOLS), (const char *)nullptr, F("?")
  );

  #if ENABLED(GRADIENT_MIX)
  {
    char tmp[13];
    SUBMENU(MSG_GRADIENT, lcd_mixer_edit_gradient_menu);
    MENU_ITEM_ADDON_START_RJ(9);
      sprintf_P(tmp, PSTR("T%i->T%i"), mixer.gradient.start_vtool, mixer.gradient.end_vtool);
      lcd_put_u8str(tmp);
    MENU_ITEM_ADDON_END();
    ACTION_ITEM(MSG_REVERSE_GRADIENT, _lcd_mixer_reverse_gradient);
  }
  #endif

  END_MENU();
}

#endif // HAS_MARLINUI_MENU && MIXING_EXTRUDER
