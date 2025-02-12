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
// Motion Menu
//

#include "../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_MENU

#include "menu_item.h"
#include "menu_addon.h"

#include "../../module/motion.h"
#include "../../gcode/parser.h" // for inch support
#include "../../module/temperature.h"

#if ENABLED(DELTA)
  #include "../../module/delta.h"
#endif

// Always show configurable options regardless of FT Motion active
//#define FT_MOTION_NO_MENU_TOGGLE

constexpr bool has_large_area() {
  return TERN0(HAS_X_AXIS, (X_BED_SIZE) >= 1000) || TERN0(HAS_Y_AXIS, (Y_BED_SIZE) >= 1000) || TERN0(HAS_Z_AXIS, (Z_MAX_POS) >= 1000);
}

//
// "Motion" > "Move Axis" submenu
//

void lcd_move_axis(const AxisEnum axis) {
  if (ui.use_click()) return ui.goto_previous_screen_no_defer();
  if (ui.encoderPosition && !ui.manual_move.processing) {
    // Get motion limit from software endstops, if any
    float min, max;
    soft_endstop.get_manual_axis_limits(axis, min, max);

    // Delta limits XY based on the current offset from center
    // This assumes the center is 0,0
    #if ENABLED(DELTA)
      if (axis != Z_AXIS) {
        max = SQRT(FLOAT_SQ(PRINTABLE_RADIUS) - sq(current_position[Y_AXIS - axis])); // (Y_AXIS - axis) == the other axis
        min = -max;
      }
    #endif

    // Get the new position
    const float diff = float(int32_t(ui.encoderPosition)) * ui.manual_move.menu_scale;
    (void)ui.manual_move.apply_diff(axis, diff, min, max);
    ui.manual_move.soon(axis);
    ui.refresh(LCDVIEW_REDRAW_NOW);
  }
  ui.encoderPosition = 0;
  if (ui.should_draw()) {
    MenuEditItemBase::itemIndex = axis;
    const float pos = ui.manual_move.axis_value(axis);
    if (parser.using_inch_units() && !parser.axis_is_rotational(axis)) {
      const float imp_pos = parser.per_axis_value(axis, pos);
      MenuEditItemBase::draw_edit_screen(GET_TEXT_F(MSG_MOVE_N), ftostr63(imp_pos));
    }
    else
      MenuEditItemBase::draw_edit_screen(GET_TEXT_F(MSG_MOVE_N), ui.manual_move.menu_scale >= 0.1f ? (has_large_area() ? ftostr51sign(pos) : ftostr41sign(pos)) : ftostr63(pos));
  }
}

#if E_MANUAL

  static void lcd_move_e(TERN_(MULTI_E_MANUAL, const int8_t eindex=active_extruder)) {
    if (ui.use_click()) return ui.goto_previous_screen_no_defer();
    if (ui.encoderPosition) {
      if (!ui.manual_move.processing) {
        const float diff = float(int32_t(ui.encoderPosition)) * ui.manual_move.menu_scale;
        TERN(IS_KINEMATIC, ui.manual_move.offset, current_position.e) += diff;
        ui.manual_move.soon(E_AXIS OPTARG(MULTI_E_MANUAL, eindex));
        ui.refresh(LCDVIEW_REDRAW_NOW);
      }
      ui.encoderPosition = 0;
    }
    if (ui.should_draw()) {
      TERN_(MULTI_E_MANUAL, MenuItemBase::init(eindex));
      MenuEditItemBase::draw_edit_screen(
        GET_TEXT_F(TERN(MULTI_E_MANUAL, MSG_MOVE_EN, MSG_MOVE_E)),
        ftostr41sign(current_position.e
          PLUS_TERN0(IS_KINEMATIC, ui.manual_move.offset)
          MINUS_TERN0(MANUAL_E_MOVES_RELATIVE, ui.manual_move.e_origin)
        )
      );
    } // should_draw
  }

#endif // E_MANUAL

#if ANY(PROBE_OFFSET_WIZARD, X_AXIS_TWIST_COMPENSATION)

  void _goto_manual_move_z(const_float_t scale) {
    ui.manual_move.menu_scale = scale;
    ui.goto_screen([]{ lcd_move_axis(Z_AXIS); });
  }

#endif

//
// "Motion" > "Move Xmm" > "Move XYZ" submenu
//

#ifndef FINE_MANUAL_MOVE
  #define FINE_MANUAL_MOVE 0.025
#endif

void _goto_manual_move(const_float_t scale) {
  ui.defer_status_screen();
  ui.manual_move.menu_scale = scale;
  ui.goto_screen(ui.manual_move.screen_ptr);
  thermalManager.set_menu_cold_override(true);
}

void _menu_move_distance(const AxisEnum axis, const screenFunc_t func, const int8_t eindex=active_extruder) {
  ui.manual_move.screen_ptr = func;
  START_MENU();
  if (LCD_HEIGHT >= 4) {
    if (axis < NUM_AXES)
      STATIC_ITEM_N(axis, MSG_MOVE_N, SS_DEFAULT|SS_INVERT);
    else {
      TERN_(MANUAL_E_MOVES_RELATIVE, ui.manual_move.e_origin = current_position.e);
      STATIC_ITEM_N(eindex, MSG_MOVE_EN, SS_DEFAULT|SS_INVERT);
    }
  }

  BACK_ITEM(MSG_MOVE_AXIS);

  #define __LINEAR_LIMIT(D) ((D) < max_length(axis) / 2 + 1)
  #if HAS_EXTRUDERS
    #ifndef EXTRUDE_MAXLENGTH
      #define EXTRUDE_MAXLENGTH 50
    #endif
    #define _LINEAR_LIMIT(D) ((axis < E_AXIS) ? __LINEAR_LIMIT(D) : ((D) < (EXTRUDE_MAXLENGTH) / 2 + 1))
  #else
    #define _LINEAR_LIMIT __LINEAR_LIMIT
  #endif
  #define __MOVE_SUB(L,T,D) if (rotational[axis] || _LINEAR_LIMIT(D)) SUBMENU_S(F(T), L, []{ _goto_manual_move(D); })

  if (rotational[axis]) {
    #ifdef MANUAL_MOVE_DISTANCE_DEG
      #define _MOVE_DEG(D) __MOVE_SUB(MSG_MOVE_N_DEG, STRINGIFY(D), D);
      MAP(_MOVE_DEG, MANUAL_MOVE_DISTANCE_DEG)
    #endif
  }
  else if (parser.using_inch_units()) {
    #ifdef MANUAL_MOVE_DISTANCE_IN
      #define _MOVE_IN(I) __MOVE_SUB(MSG_MOVE_N_MM, STRINGIFY(I), IN_TO_MM(I));
      MAP(_MOVE_IN, MANUAL_MOVE_DISTANCE_IN)
    #endif
  }
  else {
    #ifdef MANUAL_MOVE_DISTANCE_MM
      #define _MOVE_MM(M) __MOVE_SUB(MSG_MOVE_N_MM, STRINGIFY(M), M);
      MAP(_MOVE_MM, MANUAL_MOVE_DISTANCE_MM)
    #endif
    #if HAS_Z_AXIS
      if (axis == Z_AXIS && (FINE_MANUAL_MOVE) > 0.0f && (FINE_MANUAL_MOVE) < 0.1f)
        SUBMENU_f(F(STRINGIFY(FINE_MANUAL_MOVE)), MSG_MOVE_N_MM, []{ _goto_manual_move(float(FINE_MANUAL_MOVE)); });
    #endif
  }
  END_MENU();
}

#if E_MANUAL

  inline void _goto_menu_move_distance_e() {
    ui.goto_screen([]{ _menu_move_distance(E_AXIS, []{ lcd_move_e(); }); });
  }

  inline void _menu_move_distance_e_maybe() {
    if (thermalManager.tooColdToExtrude(active_extruder)) {
      ui.goto_screen([]{
        MenuItem_confirm::select_screen(
          GET_TEXT_F(MSG_BUTTON_PROCEED), GET_TEXT_F(MSG_BACK),
          _goto_menu_move_distance_e, nullptr,
          GET_TEXT_F(MSG_HOTEND_TOO_COLD), (const char *)nullptr, F("!")
        );
      });
    }
    else
      _goto_menu_move_distance_e();
  }

#endif

void menu_move() {
  START_MENU();
  BACK_ITEM(MSG_MOTION);

  #if ALL(HAS_SOFTWARE_ENDSTOPS, SOFT_ENDSTOPS_MENU_ITEM)
    EDIT_ITEM(bool, MSG_LCD_SOFT_ENDSTOPS, &soft_endstop._enabled);
  #endif

  // Move submenu for each axis
  if (NONE(IS_KINEMATIC, NO_MOTION_BEFORE_HOMING) || all_axes_homed()) {
    if (TERN1(DELTA, current_position.z <= delta_clip_start_height)) {
      #if HAS_X_AXIS
        SUBMENU_N(X_AXIS, MSG_MOVE_N, []{ _menu_move_distance(X_AXIS, []{ lcd_move_axis(X_AXIS); }); });
      #endif
      #if HAS_Y_AXIS
        SUBMENU_N(Y_AXIS, MSG_MOVE_N, []{ _menu_move_distance(Y_AXIS, []{ lcd_move_axis(Y_AXIS); }); });
      #endif
    }
    else {
      #if ENABLED(DELTA)
        ACTION_ITEM(MSG_FREE_XY, []{ line_to_z(delta_clip_start_height); ui.synchronize(); });
      #endif
    }
    #if HAS_Z_AXIS
      #define _AXIS_MOVE(N) SUBMENU_N(N, MSG_MOVE_N, []{ _menu_move_distance(AxisEnum(N), []{ lcd_move_axis(AxisEnum(N)); }); });
      REPEAT_S(2, NUM_AXES, _AXIS_MOVE);
    #endif
  }
  else
    GCODES_ITEM(MSG_AUTO_HOME, FPSTR(G28_STR));

  #if ANY(HAS_SWITCHING_EXTRUDER, HAS_SWITCHING_NOZZLE, MAGNETIC_SWITCHING_TOOLHEAD)

    #if EXTRUDERS >= 4
      switch (active_extruder) {
        case 0: GCODES_ITEM_N(1, MSG_SELECT_E, F("T1")); break;
        case 1: GCODES_ITEM_N(0, MSG_SELECT_E, F("T0")); break;
        case 2: GCODES_ITEM_N(3, MSG_SELECT_E, F("T3")); break;
        case 3: GCODES_ITEM_N(2, MSG_SELECT_E, F("T2")); break;
        #if EXTRUDERS == 6
          case 4: GCODES_ITEM_N(5, MSG_SELECT_E, F("T5")); break;
          case 5: GCODES_ITEM_N(4, MSG_SELECT_E, F("T4")); break;
        #endif
      }
    #elif EXTRUDERS == 3
      if (active_extruder < 2)
        GCODES_ITEM_N(1 - active_extruder, MSG_SELECT_E, active_extruder ? F("T0") : F("T1"));
    #else
      GCODES_ITEM_N(1 - active_extruder, MSG_SELECT_E, active_extruder ? F("T0") : F("T1"));
    #endif

  #elif ENABLED(DUAL_X_CARRIAGE)

    GCODES_ITEM_N(1 - active_extruder, MSG_SELECT_E, active_extruder ? F("T0") : F("T1"));

  #endif

  #if E_MANUAL

    // The current extruder
    SUBMENU(MSG_MOVE_E, _menu_move_distance_e_maybe);

    #define SUBMENU_MOVE_E(N) SUBMENU_N(N, MSG_MOVE_EN, []{ _menu_move_distance(E_AXIS, []{ lcd_move_e(N); }, N); });

    #if HAS_SWITCHING_EXTRUDER || HAS_SWITCHING_NOZZLE

      // ...and the non-switching
      #if E_MANUAL == 7 || E_MANUAL == 5 || E_MANUAL == 3
        SUBMENU_MOVE_E(E_MANUAL - 1);
      #endif

    #elif MULTI_E_MANUAL

      // Independent extruders with one E stepper per hotend
      REPEAT(E_MANUAL, SUBMENU_MOVE_E);

    #endif

  #endif // E_MANUAL

  END_MENU();
}

#define _HOME_ITEM(N) GCODES_ITEM_N(N##_AXIS, MSG_AUTO_HOME_N, F("G28" STR_##N));

#if ENABLED(INDIVIDUAL_AXIS_HOMING_SUBMENU)
  //
  // "Motion" > "Homing" submenu
  //
  void menu_home() {
    START_MENU();
    BACK_ITEM(MSG_MOTION);

    GCODES_ITEM(MSG_AUTO_HOME, FPSTR(G28_STR));
    MAIN_AXIS_MAP(_HOME_ITEM);

    END_MENU();
  }
#endif

#if ENABLED(FT_MOTION_MENU)

  #include "../../module/ft_motion.h"

  FSTR_P get_shaper_name(const AxisEnum axis=X_AXIS) {
    switch (ftMotion.cfg.shaper[axis]) {
      default: return nullptr;
      case ftMotionShaper_NONE:  return GET_TEXT_F(MSG_LCD_OFF);
      case ftMotionShaper_ZV:    return GET_TEXT_F(MSG_FTM_ZV);
      case ftMotionShaper_ZVD:   return GET_TEXT_F(MSG_FTM_ZVD);
      case ftMotionShaper_ZVDD:  return GET_TEXT_F(MSG_FTM_ZVDD);
      case ftMotionShaper_ZVDDD: return GET_TEXT_F(MSG_FTM_ZVDDD);
      case ftMotionShaper_EI:    return GET_TEXT_F(MSG_FTM_EI);
      case ftMotionShaper_2HEI:  return GET_TEXT_F(MSG_FTM_2HEI);
      case ftMotionShaper_3HEI:  return GET_TEXT_F(MSG_FTM_3HEI);
      case ftMotionShaper_MZV:   return GET_TEXT_F(MSG_FTM_MZV);
    }
  }

  #if HAS_DYNAMIC_FREQ
    FSTR_P get_dyn_freq_mode_name() {
      switch (ftMotion.cfg.dynFreqMode) {
        default:
        case dynFreqMode_DISABLED:   return GET_TEXT_F(MSG_LCD_OFF);
        case dynFreqMode_Z_BASED:    return GET_TEXT_F(MSG_FTM_Z_BASED);
        case dynFreqMode_MASS_BASED: return GET_TEXT_F(MSG_FTM_MASS_BASED);
      }
    }
  #endif

  void ftm_menu_set_shaper(const AxisEnum axis, const ftMotionShaper_t s) {
    ftMotion.cfg.shaper[axis] = s;
    ftMotion.update_shaping_params();
    ui.go_back();
  }

  inline void menu_ftm_shaper_x() {
    const ftMotionShaper_t shaper = ftMotion.cfg.shaper.x;
    START_MENU();
    BACK_ITEM(MSG_FIXED_TIME_MOTION);

    if (shaper != ftMotionShaper_NONE)   ACTION_ITEM(MSG_LCD_OFF,  []{ ftm_menu_set_shaper(X_AXIS, ftMotionShaper_NONE); });
    if (shaper != ftMotionShaper_ZV)     ACTION_ITEM(MSG_FTM_ZV,   []{ ftm_menu_set_shaper(X_AXIS, ftMotionShaper_ZV); });
    if (shaper != ftMotionShaper_ZVD)    ACTION_ITEM(MSG_FTM_ZVD,  []{ ftm_menu_set_shaper(X_AXIS, ftMotionShaper_ZVD); });
    if (shaper != ftMotionShaper_ZVDD)   ACTION_ITEM(MSG_FTM_ZVDD, []{ ftm_menu_set_shaper(X_AXIS, ftMotionShaper_ZVDD); });
    if (shaper != ftMotionShaper_ZVDDD)  ACTION_ITEM(MSG_FTM_ZVDDD,[]{ ftm_menu_set_shaper(X_AXIS, ftMotionShaper_ZVDDD); });
    if (shaper != ftMotionShaper_EI)     ACTION_ITEM(MSG_FTM_EI,   []{ ftm_menu_set_shaper(X_AXIS, ftMotionShaper_EI); });
    if (shaper != ftMotionShaper_2HEI)   ACTION_ITEM(MSG_FTM_2HEI, []{ ftm_menu_set_shaper(X_AXIS, ftMotionShaper_2HEI); });
    if (shaper != ftMotionShaper_3HEI)   ACTION_ITEM(MSG_FTM_3HEI, []{ ftm_menu_set_shaper(X_AXIS, ftMotionShaper_3HEI); });
    if (shaper != ftMotionShaper_MZV)    ACTION_ITEM(MSG_FTM_MZV,  []{ ftm_menu_set_shaper(X_AXIS, ftMotionShaper_MZV); });

    END_MENU();
  }

  inline void menu_ftm_shaper_y() {
    const ftMotionShaper_t shaper = ftMotion.cfg.shaper.y;
    START_MENU();
    BACK_ITEM(MSG_FIXED_TIME_MOTION);

    if (shaper != ftMotionShaper_NONE)   ACTION_ITEM(MSG_LCD_OFF,  []{ ftm_menu_set_shaper(Y_AXIS, ftMotionShaper_NONE); });
    if (shaper != ftMotionShaper_ZV)     ACTION_ITEM(MSG_FTM_ZV,   []{ ftm_menu_set_shaper(Y_AXIS, ftMotionShaper_ZV); });
    if (shaper != ftMotionShaper_ZVD)    ACTION_ITEM(MSG_FTM_ZVD,  []{ ftm_menu_set_shaper(Y_AXIS, ftMotionShaper_ZVD); });
    if (shaper != ftMotionShaper_ZVDD)   ACTION_ITEM(MSG_FTM_ZVDD, []{ ftm_menu_set_shaper(Y_AXIS, ftMotionShaper_ZVDD); });
    if (shaper != ftMotionShaper_ZVDDD)  ACTION_ITEM(MSG_FTM_ZVDDD,[]{ ftm_menu_set_shaper(Y_AXIS, ftMotionShaper_ZVDDD); });
    if (shaper != ftMotionShaper_EI)     ACTION_ITEM(MSG_FTM_EI,   []{ ftm_menu_set_shaper(Y_AXIS, ftMotionShaper_EI); });
    if (shaper != ftMotionShaper_2HEI)   ACTION_ITEM(MSG_FTM_2HEI, []{ ftm_menu_set_shaper(Y_AXIS, ftMotionShaper_2HEI); });
    if (shaper != ftMotionShaper_3HEI)   ACTION_ITEM(MSG_FTM_3HEI, []{ ftm_menu_set_shaper(Y_AXIS, ftMotionShaper_3HEI); });
    if (shaper != ftMotionShaper_MZV)    ACTION_ITEM(MSG_FTM_MZV,  []{ ftm_menu_set_shaper(Y_AXIS, ftMotionShaper_MZV); });

    END_MENU();
  }

  #if HAS_DYNAMIC_FREQ

    void menu_ftm_dyn_mode() {
      const dynFreqMode_t dmode = ftMotion.cfg.dynFreqMode;

      START_MENU();
      BACK_ITEM(MSG_FIXED_TIME_MOTION);

      if (dmode != dynFreqMode_DISABLED) ACTION_ITEM(MSG_LCD_OFF, []{ ftMotion.cfg.dynFreqMode = dynFreqMode_DISABLED; ui.go_back(); });
      #if HAS_DYNAMIC_FREQ_MM
        if (dmode != dynFreqMode_Z_BASED) ACTION_ITEM(MSG_FTM_Z_BASED, []{ ftMotion.cfg.dynFreqMode = dynFreqMode_Z_BASED; ui.go_back(); });
      #endif
      #if HAS_DYNAMIC_FREQ_G
        if (dmode != dynFreqMode_MASS_BASED) ACTION_ITEM(MSG_FTM_MASS_BASED, []{ ftMotion.cfg.dynFreqMode = dynFreqMode_MASS_BASED; ui.go_back(); });
      #endif

      END_MENU();
    }

  #endif // HAS_DYNAMIC_FREQ

  void menu_ft_motion() {
    // Define stuff ahead of the menu loop
    MString<20> shaper_name[NUM_AXES_SHAPED] {};
    #if HAS_X_AXIS
      for (uint_fast8_t a = X_AXIS; a < NUM_AXES_SHAPED; ++a)
        shaper_name[a] = get_shaper_name(AxisEnum(a));
    #endif
    #if HAS_DYNAMIC_FREQ
      MString<20> dmode = get_dyn_freq_mode_name();
    #endif

    ft_config_t &c = ftMotion.cfg;

    START_MENU();
    BACK_ITEM(MSG_MOTION);

    bool show_state = c.active;
    EDIT_ITEM(bool, MSG_FIXED_TIME_MOTION, &show_state, []{
      FLIP(ftMotion.cfg.active);
      ftMotion.update_shaping_params();
    });

    // Show only when FT Motion is active (or optionally always show)
    if (c.active || ENABLED(FT_MOTION_NO_MENU_TOGGLE)) {
      #if HAS_X_AXIS
        SUBMENU_N(X_AXIS, MSG_FTM_CMPN_MODE, menu_ftm_shaper_x);
        MENU_ITEM_ADDON_START_RJ(5); lcd_put_u8str(shaper_name[X_AXIS]); MENU_ITEM_ADDON_END();

        if (AXIS_HAS_SHAPER(X)) {
          EDIT_ITEM_FAST_N(float42_52, X_AXIS, MSG_FTM_BASE_FREQ_N, &c.baseFreq.x, FTM_MIN_SHAPE_FREQ, (FTM_FS) / 2, ftMotion.update_shaping_params);
          EDIT_ITEM_FAST_N(float42_52, X_AXIS, MSG_FTM_ZETA_N, &c.zeta.x, 0.0f, 1.0f, ftMotion.update_shaping_params);
          if (AXIS_HAS_EISHAPER(X))
            EDIT_ITEM_FAST_N(float42_52, X_AXIS, MSG_FTM_VTOL_N, &c.vtol.x, 0.0f, 1.0f, ftMotion.update_shaping_params);
        }
      #endif
      #if HAS_Y_AXIS
        SUBMENU_N(Y_AXIS, MSG_FTM_CMPN_MODE, menu_ftm_shaper_y);
        MENU_ITEM_ADDON_START_RJ(5); lcd_put_u8str(shaper_name[Y_AXIS]); MENU_ITEM_ADDON_END();

        if (AXIS_HAS_SHAPER(Y)) {
          EDIT_ITEM_FAST_N(float42_52, Y_AXIS, MSG_FTM_BASE_FREQ_N, &c.baseFreq.y, FTM_MIN_SHAPE_FREQ, (FTM_FS) / 2, ftMotion.update_shaping_params);
          EDIT_ITEM_FAST_N(float42_52, Y_AXIS, MSG_FTM_ZETA_N, &c.zeta.y, 0.0f, 1.0f, ftMotion.update_shaping_params);
          if (AXIS_HAS_EISHAPER(Y))
            EDIT_ITEM_FAST_N(float42_52, Y_AXIS, MSG_FTM_VTOL_N, &c.vtol.y, 0.0f, 1.0f, ftMotion.update_shaping_params);
        }
      #endif

      #if HAS_DYNAMIC_FREQ
        SUBMENU(MSG_FTM_DYN_MODE, menu_ftm_dyn_mode);
        MENU_ITEM_ADDON_START_RJ(11); lcd_put_u8str(dmode); MENU_ITEM_ADDON_END();
        if (c.dynFreqMode != dynFreqMode_DISABLED) {
          #if HAS_X_AXIS
            EDIT_ITEM_FAST_N(float42_52, X_AXIS, MSG_FTM_DFREQ_K_N, &c.dynFreqK.x, 0.0f, 20.0f);
          #endif
          #if HAS_Y_AXIS
            EDIT_ITEM_FAST_N(float42_52, Y_AXIS, MSG_FTM_DFREQ_K_N, &c.dynFreqK.y, 0.0f, 20.0f);
          #endif
        }
      #endif

      #if HAS_EXTRUDERS
        EDIT_ITEM(bool, MSG_LINEAR_ADVANCE, &c.linearAdvEna);
        if (c.linearAdvEna || ENABLED(FT_MOTION_NO_MENU_TOGGLE))
          EDIT_ITEM(float62, MSG_ADVANCE_K, &c.linearAdvK, 0.0f, 1000.0f);
      #endif
    }
    END_MENU();
  }

  void menu_tune_ft_motion() {
    // Define stuff ahead of the menu loop
    MString<20> shaper_name[NUM_AXES_SHAPED] {};
    #if HAS_X_AXIS
      for (uint_fast8_t a = X_AXIS; a < NUM_AXES_SHAPED; ++a)
        shaper_name[a] = get_shaper_name(AxisEnum(a));
    #endif
    #if HAS_DYNAMIC_FREQ
      MString<20> dmode = get_dyn_freq_mode_name();
    #endif

    #if HAS_EXTRUDERS
      ft_config_t &c = ftMotion.cfg;
    #endif

    START_MENU();
    BACK_ITEM(MSG_TUNE);

    #if HAS_X_AXIS
      SUBMENU_N(X_AXIS, MSG_FTM_CMPN_MODE, menu_ftm_shaper_x);
      MENU_ITEM_ADDON_START_RJ(5); lcd_put_u8str(shaper_name[X_AXIS]); MENU_ITEM_ADDON_END();
    #endif
    #if HAS_Y_AXIS
      SUBMENU_N(Y_AXIS, MSG_FTM_CMPN_MODE, menu_ftm_shaper_y);
      MENU_ITEM_ADDON_START_RJ(5); lcd_put_u8str(shaper_name[Y_AXIS]); MENU_ITEM_ADDON_END();
    #endif
    #if HAS_DYNAMIC_FREQ
      SUBMENU(MSG_FTM_DYN_MODE, menu_ftm_dyn_mode);
      MENU_ITEM_ADDON_START_RJ(dmode.length()); lcd_put_u8str(dmode); MENU_ITEM_ADDON_END();
    #endif
    #if HAS_EXTRUDERS
      EDIT_ITEM(bool, MSG_LINEAR_ADVANCE, &c.linearAdvEna);
      if (c.linearAdvEna || ENABLED(FT_MOTION_NO_MENU_TOGGLE))
        EDIT_ITEM(float42_52, MSG_ADVANCE_K, &c.linearAdvK, 0, 10);
    #endif

    END_MENU();
  }

#endif // FT_MOTION_MENU

void menu_motion() {

  START_MENU();

  //
  // ^ Main
  //
  BACK_ITEM(MSG_MAIN_MENU);

  //
  // Move Axis
  //
  if (TERN1(DELTA, all_axes_homed()))
    SUBMENU(MSG_MOVE_AXIS, menu_move);

  //
  // Auto Home
  //
  #if ENABLED(INDIVIDUAL_AXIS_HOMING_SUBMENU)
    SUBMENU(MSG_HOMING, menu_home);
  #else
    GCODES_ITEM(MSG_AUTO_HOME, FPSTR(G28_STR));
    #if ENABLED(INDIVIDUAL_AXIS_HOMING_MENU)
      MAIN_AXIS_MAP(_HOME_ITEM);
    #endif
  #endif

  //
  // M493 - Fixed-Time Motion
  //
  #if ENABLED(FT_MOTION_MENU)
    SUBMENU(MSG_FIXED_TIME_MOTION, menu_ft_motion);
  #endif

  //
  // Pen up/down menu
  //
  #if ENABLED(PEN_UP_DOWN_MENU)
    GCODES_ITEM(MSG_MANUAL_PENUP, F("M280 P0 S90"));
    GCODES_ITEM(MSG_MANUAL_PENDOWN, F("M280 P0 S50"));
  #endif

  //
  // Auto-calibration with Object
  //
  #if ENABLED(CALIBRATION_GCODE)
    GCODES_ITEM(MSG_AUTO_CALIBRATE, F("G425"));
  #endif

  //
  // Disable Steppers
  //
  GCODES_ITEM(MSG_DISABLE_STEPPERS, F("M84"));

  END_MENU();
}

#endif // HAS_MARLINUI_MENU
