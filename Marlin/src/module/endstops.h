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
 * endstops.h - manages endstops
 */

#include "../inc/MarlinConfig.h"
#include <stdint.h>

#define _ES_ENUM(A,M) A##_##M
#define ES_ENUM(A,M) _ES_ENUM(A,M)

#define _ES_ITEM(N) , N
#define ES_ITEM(K,N) TERN(K,_ES_ITEM,_IF_1_ELSE)(N)

#define _ESN_ITEM(K,A,M) ES_ITEM(K,ES_ENUM(A,M))
#define ES_MINMAX(A) ES_ITEM(HAS_##A##_MIN_STATE, ES_ENUM(A,MIN)) ES_ITEM(HAS_##A##_MAX_STATE, ES_ENUM(A,MAX))

/**
 * Basic Endstop Flag Bits:
 * - Each axis with an endstop gets a flag for its homing direction.
 *   (The use of "MIN" or "MAX" makes it easier to pair with similarly-named endstop pins.)
 * - Bed probes with a single pin get a Z_MIN_PROBE flag. This includes Sensorless Z Probe.
 *
 * Extended Flag Bits:
 * - Multi-stepper axes may have multi-endstops such as X2_MIN, Y2_MAX, etc.
 * - DELTA gets X_MAX, Y_MAX, and Z_MAX corresponding to its "A", "B", "C" towers.
 * - For DUAL_X_CARRIAGE the X axis has both X_MIN and X_MAX flags.
 * - The Z axis may have both MIN and MAX when homing to MAX and the probe is Z_MIN.
 * - DELTA Sensorless Probe uses X/Y/Z_MAX but sets the Z_MIN flag.
 *
 * Endstop Flag Bit Aliases:
 * - Each *_MIN or *_MAX flag is aliased to *_ENDSTOP.
 * - Z_MIN_PROBE is an alias to Z_MIN when the Z_MIN_PIN is being used as the probe pin.
 * - When homing with the probe Z_ENDSTOP is a Z_MIN_PROBE alias, otherwise a Z_MIN/MAX alias.
 */
enum EndstopEnum : int8_t {
  _ES_START_ = -1

  // Common XYZ (ABC) endstops.
  ES_MINMAX(X) ES_MINMAX(Y) ES_MINMAX(Z)
  ES_MINMAX(I) ES_MINMAX(J) ES_MINMAX(K)
  ES_MINMAX(U) ES_MINMAX(V) ES_MINMAX(W)

  // Extra Endstops for XYZ
  ES_MINMAX(X2) ES_MINMAX(Y2) ES_MINMAX(Z2) ES_MINMAX(Z3) ES_MINMAX(Z4)

  // Calibration pin state
  ES_ITEM(HAS_CALIBRATION_STATE, CALIBRATION)

  // Bed Probe state is distinct or shared with Z_MIN (i.e., when the probe is the only Z endstop)
  #if HAS_Z_PROBE_STATE && USE_Z_MIN_PROBE
    , Z_MIN_PROBE
  #endif

  // The total number of distinct states
  , NUM_ENDSTOP_STATES

  // Endstop aliases
  #if HAS_Z_PROBE_STATE && !USE_Z_MIN_PROBE
    , Z_MIN_PROBE = Z_MIN
  #endif

  #if HAS_X_STATE
    , X_ENDSTOP = TERN(X_HOME_TO_MAX, X_MAX, X_MIN)
  #endif
  #if HAS_X2_STATE
    , X2_ENDSTOP = TERN(X_HOME_TO_MAX, X2_MAX, X2_MIN)
  #endif
  #if HAS_Y_STATE
    , Y_ENDSTOP = TERN(Y_HOME_TO_MAX, Y_MAX, Y_MIN)
  #endif
  #if HAS_Y2_STATE
    , Y2_ENDSTOP = TERN(Y_HOME_TO_MAX, Y2_MAX, Y2_MIN)
  #endif
  #if HOMING_Z_WITH_PROBE
    , Z_ENDSTOP = Z_MIN_PROBE // "Z" endstop alias when homing with the probe
  #elif HAS_Z_STATE
    , Z_ENDSTOP = TERN(Z_HOME_TO_MAX, Z_MAX, Z_MIN)
  #endif
  #if HAS_Z2_STATE
    , Z2_ENDSTOP = TERN(Z_HOME_TO_MAX, Z2_MAX, Z2_MIN)
  #endif
  #if HAS_Z3_STATE
    , Z3_ENDSTOP = TERN(Z_HOME_TO_MAX, Z3_MAX, Z3_MIN)
  #endif
  #if HAS_Z4_STATE
    , Z4_ENDSTOP = TERN(Z_HOME_TO_MAX, Z4_MAX, Z4_MIN)
  #endif
  #if HAS_I_STATE
    , I_ENDSTOP = TERN(I_HOME_TO_MAX, I_MAX, I_MIN)
  #endif
  #if HAS_J_STATE
    , J_ENDSTOP = TERN(J_HOME_TO_MAX, J_MAX, J_MIN)
  #endif
  #if HAS_K_STATE
    , K_ENDSTOP = TERN(K_HOME_TO_MAX, K_MAX, K_MIN)
  #endif
  #if HAS_U_STATE
    , U_ENDSTOP = TERN(U_HOME_TO_MAX, U_MAX, U_MIN)
  #endif
  #if HAS_V_STATE
    , V_ENDSTOP = TERN(V_HOME_TO_MAX, V_MAX, V_MIN)
  #endif
  #if HAS_W_STATE
    , W_ENDSTOP = TERN(W_HOME_TO_MAX, W_MAX, W_MIN)
  #endif
};

#undef _ES_ITEM
#undef ES_ITEM
#undef _ESN_ITEM
#undef ES_MINMAX

class Endstops {
  public:

    typedef bits_t(NUM_ENDSTOP_STATES) endstop_mask_t;

    #if ENABLED(X_DUAL_ENDSTOPS)
      static float x2_endstop_adj;
    #endif
    #if ENABLED(Y_DUAL_ENDSTOPS)
      static float y2_endstop_adj;
    #endif
    #if ENABLED(Z_MULTI_ENDSTOPS)
      static float z2_endstop_adj;
    #endif
    #if ENABLED(Z_MULTI_ENDSTOPS) && NUM_Z_STEPPERS >= 3
      static float z3_endstop_adj;
    #endif
    #if ENABLED(Z_MULTI_ENDSTOPS) && NUM_Z_STEPPERS >= 4
      static float z4_endstop_adj;
    #endif

  private:
    static bool enabled, enabled_globally;
    static endstop_mask_t live_state;
    static volatile endstop_mask_t hit_state; // Use X_MIN, Y_MIN, Z_MIN and Z_MIN_PROBE as BIT index

    #if ENDSTOP_NOISE_THRESHOLD
      static endstop_mask_t validated_live_state;
      static uint8_t endstop_poll_count;    // Countdown from threshold for polling
    #endif

  public:
    Endstops() {};

    /**
     * Initialize the endstop pins
     */
    static void init();

    /**
     * Are endstops or the Z min probe or the CALIBRATION probe set to abort the move?
     */
    FORCE_INLINE static bool abort_enabled() {
      return enabled || TERN0(HAS_BED_PROBE, z_probe_enabled) || TERN0(CALIBRATION_GCODE, calibration_probe_enabled);
    }

    static bool global_enabled() { return enabled_globally; }

    /**
     * Periodic call to poll endstops if required. Called from temperature ISR
     */
    static void poll();

    /**
     * Update endstops bits from the pins. Apply filtering to get a verified state.
     * If abort_enabled() and moving towards a triggered switch, abort the current move.
     * Called from ISR contexts.
     */
    static void update();

    #if ENABLED(BD_SENSOR)
      static bool bdp_state;
      static void bdp_state_update(const bool z_state) { bdp_state = z_state; }
    #endif

    /**
     * Get Endstop hit state.
     */
    FORCE_INLINE static endstop_mask_t trigger_state() { return hit_state; }

    /**
     * Get current endstops state
     */
    FORCE_INLINE static endstop_mask_t state() {
      return
        #if ENDSTOP_NOISE_THRESHOLD
          validated_live_state
        #else
          live_state
        #endif
      ;
    }

    static bool probe_switch_activated() {
      return (true
        #if ENABLED(PROBE_ACTIVATION_SWITCH)
          && READ(PROBE_ACTIVATION_SWITCH_PIN) == PROBE_ACTIVATION_SWITCH_STATE
        #endif
      );
    }

    /**
     * Report endstop hits to serial. Called from loop().
     */
    static void event_handler();

    /**
     * Report endstop states in response to M119
     */
    static void report_states();

    // Enable / disable endstop checking globally
    static void enable_globally(const bool onoff=true);

    // Enable / disable endstop checking
    static void enable(const bool onoff=true);

    // Disable / Enable endstops based on ENSTOPS_ONLY_FOR_HOMING and global enable
    static void not_homing();

    #if ENABLED(VALIDATE_HOMING_ENDSTOPS)
      // If the last move failed to trigger an endstop, call kill
      static void validate_homing_move();
    #else
      FORCE_INLINE static void validate_homing_move() { hit_on_purpose(); }
    #endif

    // Clear endstops (i.e., they were hit intentionally) to suppress the report
    FORCE_INLINE static void hit_on_purpose() { hit_state = 0; }

    // Enable / disable endstop z-probe checking
    #if HAS_BED_PROBE
      static volatile bool z_probe_enabled;
      static void enable_z_probe(const bool onoff=true);
    #endif

    // Enable / disable calibration probe checking
    #if ENABLED(CALIBRATION_GCODE)
      static volatile bool calibration_probe_enabled;
      static volatile bool calibration_stop_state;
      static void enable_calibration_probe(const bool onoff, const bool stop_state=true);
    #endif

    static void resync();

    // Debugging of endstops
    #if ENABLED(PINS_DEBUGGING)
      static bool monitor_flag;
      static void monitor();
      static void run_monitor();
    #endif

    #if ENABLED(SPI_ENDSTOPS)
      typedef struct {
        union {
          bool any;
          struct { bool NUM_AXIS_LIST(x:1, y:1, z:1, i:1, j:1, k:1); };
        };
      } tmc_spi_homing_t;
      static tmc_spi_homing_t tmc_spi_homing;
      static void clear_endstop_state();
      static bool tmc_spi_homing_check();
    #endif
};

extern Endstops endstops;

/**
 * A class to save and change the endstop state,
 * then restore it when it goes out of scope.
 */
class TemporaryGlobalEndstopsState {
  bool saved;

  public:
    TemporaryGlobalEndstopsState(const bool enable) : saved(endstops.global_enabled()) {
      endstops.enable_globally(enable);
    }
    ~TemporaryGlobalEndstopsState() { endstops.enable_globally(saved); }
};
