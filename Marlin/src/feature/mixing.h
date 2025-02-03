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

/**
 * feature/mixing.h
 */

#include "../inc/MarlinConfig.h"

//#define MIXER_NORMALIZER_DEBUG

#ifndef __AVR__ // || HAS_DUAL_MIXING
  // Use 16-bit (or fastest) data for the integer mix factors
  typedef uint_fast16_t mixer_comp_t;
  typedef uint_fast16_t mixer_accu_t;
  #define COLOR_A_MASK 0x8000
  #define COLOR_MASK 0x7FFF
#else
  // Use 8-bit data for the integer mix factors
  // Exactness is sacrificed for speed
  #define MIXER_ACCU_SIGNED
  typedef uint8_t mixer_comp_t;
  typedef int8_t mixer_accu_t;
  #define COLOR_A_MASK 0x80
  #define COLOR_MASK 0x7F
#endif

typedef int8_t mixer_perc_t;

enum MixTool {
    FIRST_USER_VIRTUAL_TOOL = 0
  , LAST_USER_VIRTUAL_TOOL = MIXING_VIRTUAL_TOOLS - 1
  , NR_USER_VIRTUAL_TOOLS
  , MIXER_DIRECT_SET_TOOL = NR_USER_VIRTUAL_TOOLS
  #if HAS_MIXER_SYNC_CHANNEL
    , MIXER_AUTORETRACT_TOOL
  #endif
  , NR_MIXING_VIRTUAL_TOOLS
};

#define MAX_VTOOLS TERN(HAS_MIXER_SYNC_CHANNEL, 254, 255)
static_assert(NR_MIXING_VIRTUAL_TOOLS <= MAX_VTOOLS, "MIXING_VIRTUAL_TOOLS must be <= " STRINGIFY(MAX_VTOOLS) "!");

#define MIXER_STEPPER_LOOP(VAR) for (uint_fast8_t VAR = 0; VAR < MIXING_STEPPERS; VAR++)

#if ENABLED(GRADIENT_MIX)

  typedef struct {
    bool enabled;                            // This gradient is enabled
    mixer_comp_t color[MIXING_STEPPERS];     // The current gradient color
    float start_z, end_z;                    // Region for gradient
    int8_t start_vtool, end_vtool;           // Start and end virtual tools
    mixer_perc_t start_mix[MIXING_STEPPERS], // Start and end mixes from those tools
                 end_mix[MIXING_STEPPERS];
    #if ENABLED(GRADIENT_VTOOL)
      int8_t vtool_index;                    // Use this virtual tool number as index
    #endif
  } gradient_t;

#endif

/**
 * @brief Mixer class
 * @details Contains data and behaviors for a Mixing Extruder
 */
class Mixer {
  public:

  static float collector[MIXING_STEPPERS]; // M163 components, also editable from LCD

  static void init(); // Populate colors at boot time

  static void reset_vtools();
  static void refresh_collector(const float proportion=1.0, const uint8_t t=selected_vtool, float (&c)[MIXING_STEPPERS]=collector);

  // Used up to Planner level
  FORCE_INLINE static void set_collector(const uint8_t c, const float f) { collector[c] = _MAX(f, 0.0f); }

  static void normalize(const uint8_t tool_index);
  FORCE_INLINE static void normalize() { normalize(selected_vtool); }

  FORCE_INLINE static uint8_t get_current_vtool() { return selected_vtool; }

  FORCE_INLINE static void T(const uint_fast8_t c) {
    selected_vtool = c;
    TERN_(GRADIENT_VTOOL, refresh_gradient());
    TERN_(HAS_DUAL_MIXING, update_mix_from_vtool());
  }

  // Used when dealing with blocks
  FORCE_INLINE static void populate_block(mixer_comp_t (&b_color)[MIXING_STEPPERS]) {
    #if ENABLED(GRADIENT_MIX)
      if (gradient.enabled) {
        MIXER_STEPPER_LOOP(i) b_color[i] = gradient.color[i];
        return;
      }
    #endif
    MIXER_STEPPER_LOOP(i) b_color[i] = color[selected_vtool][i];
  }

  FORCE_INLINE static void stepper_setup(mixer_comp_t (&b_color)[MIXING_STEPPERS]) {
    MIXER_STEPPER_LOOP(i) s_color[i] = b_color[i];
  }

  #if ANY(HAS_DUAL_MIXING, GRADIENT_MIX)

    static mixer_perc_t mix[MIXING_STEPPERS]; // Scratch array for the Mix in proportion to 100

    static void copy_mix_to_color(mixer_comp_t (&tcolor)[MIXING_STEPPERS]) {
      // Scale each component to the largest one in terms of COLOR_A_MASK
      // So the largest component will be COLOR_A_MASK and the other will be in proportion to it
      const float scale = (COLOR_A_MASK) * RECIPROCAL(_MAX(
        LIST_N(MIXING_STEPPERS, mix[0], mix[1], mix[2], mix[3], mix[4], mix[5])
      ));

      // Scale all values so their maximum is COLOR_A_MASK
      MIXER_STEPPER_LOOP(i) tcolor[i] = mix[i] * scale;

      #ifdef MIXER_NORMALIZER_DEBUG
        SERIAL_ECHOLN(
          F("Mix [ "), LIST_N(MIXING_STEPPERS, mix[0], mix[1], mix[2], mix[3], mix[4], mix[5]),
          F(" ] to Color [ "), LIST_N(MIXING_STEPPERS, tcolor[0], tcolor[1], tcolor[2], tcolor[3], tcolor[4], tcolor[5]),
          F(" ]")
        );
      #endif
    }

    static void update_mix_from_vtool(const uint8_t j=selected_vtool) {
      float ctot = 0;
      MIXER_STEPPER_LOOP(i) ctot += color[j][i];
      MIXER_STEPPER_LOOP(i) mix[i] = mixer_perc_t(100.0f * color[j][i] / ctot + 0.5f);

      #ifdef MIXER_NORMALIZER_DEBUG
        SERIAL_ECHOLN(F("V-tool "), j,
          F(" [ "), LIST_N(MIXING_STEPPERS, color[j][0], color[j][1], color[j][2], color[j][3], color[j][4], color[j][5]),
          F(" ] to Mix [ "), LIST_N(MIXING_STEPPERS, mix[0], mix[1], mix[2], mix[3], mix[4], mix[5]), F(" ]")
        );
      #endif
    }

  #endif // HAS_DUAL_MIXING || GRADIENT_MIX

  #if HAS_DUAL_MIXING

    // Update the virtual tool from an edited mix
    static void update_vtool_from_mix() {
      copy_mix_to_color(color[selected_vtool]);
      TERN_(GRADIENT_MIX, refresh_gradient());
      // MIXER_STEPPER_LOOP(i) collector[i] = mix[i];
      // normalize();
    }

  #endif // HAS_DUAL_MIXING

  #if ENABLED(GRADIENT_MIX)

    static gradient_t gradient;
    static float prev_z;

    // Update the current mix from the gradient for a given Z
    static void update_gradient_for_z(const_float_t z);
    static void update_gradient_for_planner_z();
    static void gradient_control(const_float_t z) {
      if (gradient.enabled) {
        if (z >= gradient.end_z)
          T(gradient.end_vtool);
        else
          update_gradient_for_z(z);
      }
    }

    static void update_mix_from_gradient() {
      float ctot = 0;
      MIXER_STEPPER_LOOP(i) ctot += gradient.color[i];
      MIXER_STEPPER_LOOP(i) mix[i] = (mixer_perc_t)CEIL(100.0f * gradient.color[i] / ctot);

      #ifdef MIXER_NORMALIZER_DEBUG
        SERIAL_ECHOLN(
          F("Gradient [ "), LIST_N(MIXING_STEPPERS, gradient.color[0], gradient.color[1], gradient.color[2], gradient.color[3], gradient.color[4], gradient.color[5]),
          F(" ] to Mix [ "), LIST_N(MIXING_STEPPERS, mix[0], mix[1], mix[2], mix[3], mix[4], mix[5]), F(" ]")
        );
      #endif
    }

    // Refresh the gradient after a change
    static void refresh_gradient() {
      #if ENABLED(GRADIENT_VTOOL)
        const bool is_grd = (gradient.vtool_index == -1 || selected_vtool == (uint8_t)gradient.vtool_index);
      #else
        constexpr bool is_grd = true;
      #endif
      gradient.enabled = is_grd && gradient.start_vtool != gradient.end_vtool && gradient.start_z < gradient.end_z;
      if (gradient.enabled) {
        mixer_perc_t mix_bak[MIXING_STEPPERS];
        COPY(mix_bak, mix);
        update_mix_from_vtool(gradient.start_vtool);
        COPY(gradient.start_mix, mix);
        update_mix_from_vtool(gradient.end_vtool);
        COPY(gradient.end_mix, mix);
        update_gradient_for_planner_z();
        COPY(mix, mix_bak);
        prev_z = -1;
      }
    }

  #endif // GRADIENT_MIX

  // Used in Stepper
  FORCE_INLINE static uint8_t get_stepper() { return runner; }
  FORCE_INLINE static uint8_t get_next_stepper() {
    for (;;) {
      if (--runner < 0) runner = MIXING_STEPPERS - 1;
      accu[runner] += s_color[runner];
      if (TERN(MIXER_ACCU_SIGNED, accu[runner] < 0, accu[runner] & COLOR_A_MASK)) {
        accu[runner] &= COLOR_MASK;
        return runner;
      }
    }
  }

  private:

  // Used up to Planner level
  static uint_fast8_t selected_vtool;
  static mixer_comp_t color[NR_MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];

  // Used in Stepper
  static int_fast8_t  runner;
  static mixer_comp_t s_color[MIXING_STEPPERS];
  static mixer_accu_t accu[MIXING_STEPPERS];
};

extern Mixer mixer;
