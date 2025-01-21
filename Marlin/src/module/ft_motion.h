/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../inc/MarlinConfigPre.h" // Access the top level configurations.
#include "../module/planner.h"      // Access block type from planner.
#include "../module/stepper.h"      // For stepper motion and direction

#include "ft_types.h"

#if HAS_X_AXIS && (HAS_Z_AXIS || HAS_EXTRUDERS)
  #define HAS_DYNAMIC_FREQ 1
  #if HAS_Z_AXIS
    #define HAS_DYNAMIC_FREQ_MM 1
  #endif
  #if HAS_EXTRUDERS
    #define HAS_DYNAMIC_FREQ_G 1
  #endif
#endif

typedef struct FTConfig {
  bool active = ENABLED(FTM_IS_DEFAULT_MOTION);           // Active (else standard motion)

  #if HAS_FTM_SHAPING
    ft_shaped_shaper_t shaper =                           // Shaper type
      { SHAPED_ELEM(FTM_DEFAULT_SHAPER_X, FTM_DEFAULT_SHAPER_Y) };
    ft_shaped_float_t baseFreq =                          // Base frequency. [Hz]
      { SHAPED_ELEM(FTM_SHAPING_DEFAULT_FREQ_X, FTM_SHAPING_DEFAULT_FREQ_Y) };
    ft_shaped_float_t zeta =                              // Damping factor
      { SHAPED_ELEM(FTM_SHAPING_ZETA_X, FTM_SHAPING_ZETA_Y) };
    ft_shaped_float_t vtol =                              // Vibration Level
      { SHAPED_ELEM(FTM_SHAPING_V_TOL_X, FTM_SHAPING_V_TOL_Y) };

    #if HAS_DYNAMIC_FREQ
      dynFreqMode_t dynFreqMode = FTM_DEFAULT_DYNFREQ_MODE; // Dynamic frequency mode configuration.
      ft_shaped_float_t dynFreqK = { 0.0f };                // Scaling / gain for dynamic frequency. [Hz/mm] or [Hz/g]
    #else
      static constexpr dynFreqMode_t dynFreqMode = dynFreqMode_DISABLED;
    #endif
  #endif // HAS_FTM_SHAPING

  #if HAS_EXTRUDERS
    bool linearAdvEna = FTM_LINEAR_ADV_DEFAULT_ENA;       // Linear advance enable configuration.
    float linearAdvK = FTM_LINEAR_ADV_DEFAULT_K;          // Linear advance gain.
  #endif
} ft_config_t;

class FTMotion {

  public:

    // Public variables
    static ft_config_t cfg;
    static bool busy;

    static void set_defaults() {
      cfg.active = ENABLED(FTM_IS_DEFAULT_MOTION);

      #if HAS_FTM_SHAPING

        #if HAS_X_AXIS
          cfg.shaper.x = FTM_DEFAULT_SHAPER_X;
          cfg.baseFreq.x = FTM_SHAPING_DEFAULT_FREQ_X;
          cfg.zeta.x = FTM_SHAPING_ZETA_X;
          cfg.vtol.x = FTM_SHAPING_V_TOL_X;
        #endif

        #if HAS_Y_AXIS
          cfg.shaper.y = FTM_DEFAULT_SHAPER_Y;
          cfg.baseFreq.y = FTM_SHAPING_DEFAULT_FREQ_Y;
          cfg.zeta.y = FTM_SHAPING_ZETA_Y;
          cfg.vtol.y = FTM_SHAPING_V_TOL_Y;
        #endif

        #if HAS_DYNAMIC_FREQ
          cfg.dynFreqMode = FTM_DEFAULT_DYNFREQ_MODE;
          TERN_(HAS_X_AXIS, cfg.dynFreqK.x = 0.0f);
          TERN_(HAS_Y_AXIS, cfg.dynFreqK.y = 0.0f);
        #endif

        update_shaping_params();

      #endif // HAS_FTM_SHAPING

      #if HAS_EXTRUDERS
        cfg.linearAdvEna = FTM_LINEAR_ADV_DEFAULT_ENA;
        cfg.linearAdvK = FTM_LINEAR_ADV_DEFAULT_K;
      #endif

      reset();
    }

    static ft_command_t stepperCmdBuff[FTM_STEPPERCMD_BUFF_SIZE]; // Buffer of stepper commands.
    static int32_t stepperCmdBuff_produceIdx,             // Index of next stepper command write to the buffer.
                   stepperCmdBuff_consumeIdx;             // Index of next stepper command read from the buffer.

    static bool sts_stepperBusy;                          // The stepper buffer has items and is in use.

    static XYZEval<millis_t> axis_move_end_ti;
    static AxisBits axis_move_dir;

    // Public methods
    static void init();
    static void loop();                                   // Controller main, to be invoked from non-isr task.

    #if HAS_FTM_SHAPING
      // Refresh gains and indices used by shaping functions.
      static void update_shaping_params(void);
    #endif

    static void reset();                                  // Reset all states of the fixed time conversion to defaults.

    FORCE_INLINE static bool axis_is_moving(const AxisEnum axis) {
      return cfg.active ? PENDING(millis(), axis_move_end_ti[axis]) : stepper.axis_is_moving(axis);
    }
    FORCE_INLINE static bool motor_direction(const AxisEnum axis) {
      return cfg.active ? axis_move_dir[axis] : stepper.last_direction_bits[axis];
    }

  private:

    static xyze_trajectory_t traj;
    static xyze_trajectoryMod_t trajMod;

    static bool blockProcRdy;
    static bool batchRdy, batchRdyForInterp;

    // Trapezoid data variables.
    static xyze_pos_t   startPosn,          // (mm) Start position of block
                        endPosn_prevBlock;  // (mm) End position of previous block
    static xyze_float_t ratio;              // (ratio) Axis move ratio of block
    static float accel_P, decel_P,
                 F_P,
                 f_s,
                 s_1e,
                 s_2e;

    static uint32_t N1, N2, N3;
    static uint32_t max_intervals;

    // Number of batches needed to propagate the current trajectory to the stepper.
    static constexpr uint32_t PROP_BATCHES = CEIL((FTM_WINDOW_SIZE) / (FTM_BATCH_SIZE)) - 1;

    // Make vector variables.
    static uint32_t makeVector_idx,
                    makeVector_batchIdx;

    // Interpolation variables.
    static uint32_t interpIdx;

    static xyze_long_t steps;

    // Shaping variables.
    #if HAS_FTM_SHAPING

      typedef struct AxisShaping {
        bool ena = false;                 // Enabled indication.
        float d_zi[FTM_ZMAX] = { 0.0f };  // Data point delay vector.
        float Ai[5];                      // Shaping gain vector.
        uint32_t Ni[5];                   // Shaping time index vector.
        uint32_t max_i;                   // Vector length for the selected shaper.

        void set_axis_shaping_N(const ftMotionShaper_t shaper, const_float_t f, const_float_t zeta);    // Sets the gains used by shaping functions.
        void set_axis_shaping_A(const ftMotionShaper_t shaper, const_float_t zeta, const_float_t vtol); // Sets the indices used by shaping functions.

      } axis_shaping_t;

      typedef struct Shaping {
        uint32_t zi_idx;           // Index of storage in the data point delay vectors.
        #if HAS_X_AXIS
          axis_shaping_t x;
        #endif
        #if HAS_Y_AXIS
          axis_shaping_t y;
        #endif
      } shaping_t;

      static shaping_t shaping; // Shaping data

    #endif // HAS_FTM_SHAPING

    // Linear advance variables.
    #if HAS_EXTRUDERS
      static float e_raw_z1, e_advanced_z1;
    #endif

    // Private methods
    static void discard_planner_block_protected();
    static void runoutBlock();
    static int32_t stepperCmdBuffItems();
    static void loadBlockData(block_t *const current_block);
    static void makeVector();
    static void convertToSteps(const uint32_t idx);

    FORCE_INLINE static int32_t num_samples_shaper_settle() { return ( shaping.x.ena || shaping.y.ena ) ? FTM_ZMAX : 0; }

}; // class FTMotion

extern FTMotion ftMotion;

typedef struct FTMotionDisableInScope {
  bool isactive;
  FTMotionDisableInScope() {
    isactive = ftMotion.cfg.active;
    ftMotion.cfg.active = false;
  }
  ~FTMotionDisableInScope() {
    ftMotion.cfg.active = isactive;
    if (isactive) ftMotion.init();
  }
} FTMotionDisableInScope_t;
