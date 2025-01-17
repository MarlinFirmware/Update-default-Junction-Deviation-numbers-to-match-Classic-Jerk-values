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

#include "../inc/MarlinConfig.h"

#if ENABLED(FT_MOTION)

#include "ft_motion.h"
#include "stepper.h" // Access stepper block queue function and abort status.
#include "endstops.h"

FTMotion ftMotion;

//-----------------------------------------------------------------
// Variables.
//-----------------------------------------------------------------

// Public variables.

ft_config_t FTMotion::cfg;
bool FTMotion::busy; // = false
ft_command_t FTMotion::stepperCmdBuff[FTM_STEPPERCMD_BUFF_SIZE] = {0U}; // Stepper commands buffer.
int32_t FTMotion::stepperCmdBuff_produceIdx = 0, // Index of next stepper command write to the buffer.
        FTMotion::stepperCmdBuff_consumeIdx = 0; // Index of next stepper command read from the buffer.

bool FTMotion::sts_stepperBusy = false;         // The stepper buffer has items and is in use.

XYZEval<millis_t> FTMotion::axis_move_end_ti = { 0 };
AxisBits FTMotion::axis_move_dir;

// Private variables.

// NOTE: These are sized for Ulendo FBS use.
xyze_trajectory_t    FTMotion::traj;            // = {0.0f} Storage for fixed-time-based trajectory.
xyze_trajectoryMod_t FTMotion::trajMod;         // = {0.0f} Storage for fixed time trajectory window.

bool FTMotion::blockProcRdy = false;            // Set when new block data is loaded from stepper module into FTM, ...
                                                // ... and reset when block is completely converted to FTM trajectory.
bool FTMotion::batchRdy = false;                // Indicates a batch of the fixed time trajectory...
                                                // ... has been generated, is now available in the upper -
                                                // batch of traj.x[], y, z ... e vectors, and is ready to be
                                                // post processed, if applicable, then interpolated. Reset when the
                                                // data has been shifted out.
bool FTMotion::batchRdyForInterp = false;       // Indicates the batch is done being post processed...
                                                // ... if applicable, and is ready to be converted to step commands.

// Trapezoid data variables.
xyze_pos_t   FTMotion::startPosn,                     // (mm) Start position of block
             FTMotion::endPosn_prevBlock = { 0.0f };  // (mm) End position of previous block
xyze_float_t FTMotion::ratio;                         // (ratio) Axis move ratio of block
float FTMotion::accel_P,                        // Acceleration prime of block. [mm/sec/sec]
      FTMotion::decel_P,                        // Deceleration prime of block. [mm/sec/sec]
      FTMotion::F_P,                            // Feedrate prime of block. [mm/sec]
      FTMotion::f_s,                            // Starting feedrate of block. [mm/sec]
      FTMotion::s_1e,                           // Position after acceleration phase of block.
      FTMotion::s_2e;                           // Position after acceleration and coasting phase of block.

uint32_t FTMotion::N1,                          // Number of data points in the acceleration phase.
         FTMotion::N2,                          // Number of data points in the coasting phase.
         FTMotion::N3;                          // Number of data points in the deceleration phase.

uint32_t FTMotion::max_intervals;               // Total number of data points that will be generated from block.

// Make vector variables.
uint32_t FTMotion::makeVector_idx = 0,          // Index of fixed time trajectory generation of the overall block.
         FTMotion::makeVector_batchIdx = 0;     // Index of fixed time trajectory generation within the batch.

// Interpolation variables.
xyze_long_t FTMotion::steps = { 0 };            // Step count accumulator.

uint32_t FTMotion::interpIdx = 0;               // Index of current data point being interpolated.

// Shaping variables.
#if HAS_FTM_SHAPING
  FTMotion::shaping_t FTMotion::shaping = {
    zi_idx: 0
    #if HAS_X_AXIS
      , x:{ false, { 0.0f }, { 0.0f }, { 0 }, 0 } // ena, d_zi[], Ai[], Ni[], max_i
    #endif
    #if HAS_Y_AXIS
      , y:{ false, { 0.0f }, { 0.0f }, { 0 }, 0 } // ena, d_zi[], Ai[], Ni[], max_i
    #endif
  };
#endif

#if HAS_EXTRUDERS
  // Linear advance variables.
  float FTMotion::e_raw_z1 = 0.0f;        // (ms) Unit delay of raw extruder position.
  float FTMotion::e_advanced_z1 = 0.0f;   // (ms) Unit delay of advanced extruder position.
#endif

constexpr uint32_t BATCH_SIDX_IN_WINDOW = (FTM_WINDOW_SIZE) - (FTM_BATCH_SIZE); // Batch start index in window.

//-----------------------------------------------------------------
// Function definitions.
//-----------------------------------------------------------------

// Public functions.

// Controller main, to be invoked from non-isr task.
void FTMotion::loop() {

  if (!cfg.active) return;

  /**
   * Handle block abort with the following sequence:
   * 1. Zero out commands in stepper ISR.
   * 2. Drain the motion buffer, stop processing until they are emptied.
   * 3. Reset all the states / memory.
   * 4. Signal ready for new block.
   */
  if (stepper.abort_current_block) {
    if (sts_stepperBusy) return;          // Wait until motion buffers are emptied
    discard_planner_block_protected();
    reset();
    stepper.abort_current_block = false;  // Abort finished.
  }

  while (!blockProcRdy && (stepper.current_block = planner.get_current_block())) {
    if (stepper.current_block->is_sync()) {     // Sync block?
      if (stepper.current_block->is_sync_pos()) // Position sync? Set the position.
        stepper._set_position(stepper.current_block->position);
      discard_planner_block_protected();
      continue;
    }
    loadBlockData(stepper.current_block);
    blockProcRdy = true;
    // Some kinematics track axis motion in HX, HY, HZ
    #if ANY(CORE_IS_XY, CORE_IS_XZ, MARKFORGED_XY, MARKFORGED_YX)
      stepper.last_direction_bits.hx = stepper.current_block->direction_bits.hx;
    #endif
    #if ANY(CORE_IS_XY, CORE_IS_YZ, MARKFORGED_XY, MARKFORGED_YX)
      stepper.last_direction_bits.hy = stepper.current_block->direction_bits.hy;
    #endif
    #if ANY(CORE_IS_XZ, CORE_IS_YZ)
      stepper.last_direction_bits.hz = stepper.current_block->direction_bits.hz;
    #endif
  }

  if (blockProcRdy) {

    if (!batchRdy) makeVector(); // Caution: Do not consolidate checks on blockProcRdy/batchRdy, as they are written by makeVector().
    // When makeVector is finished: either blockProcRdy has been set false (because the block is
    // done being processed) or batchRdy is set true, or both.

    // Check if the block has been completely converted:
    if (!blockProcRdy) {
      discard_planner_block_protected();

      // Check if the block needs to be runout:
      if (!batchRdy && !planner.has_blocks_queued()) {
        runoutBlock();
        makeVector(); // Do an additional makeVector call to guarantee batchRdy set this loop.
      }
    }
  }

  // FBS / post processing.
  if (batchRdy && !batchRdyForInterp) {

    // Call Ulendo FBS here.

    #if ENABLED(FTM_UNIFIED_BWS)
      trajMod = traj; // Move the window to traj
    #else
      // Copy the uncompensated vectors.
      #define TCOPY(A) memcpy(trajMod.A, traj.A, sizeof(trajMod.A));
      LOGICAL_AXIS_MAP_LC(TCOPY);

      // Shift the time series back in the window
      #define TSHIFT(A) memcpy(traj.A, &traj.A[FTM_BATCH_SIZE], BATCH_SIDX_IN_WINDOW * sizeof(traj.A[0]));
      LOGICAL_AXIS_MAP_LC(TSHIFT);
    #endif

    // ... data is ready in trajMod.
    batchRdyForInterp = true;

    batchRdy = false; // Clear so makeVector() can resume generating points.
  }

  // Interpolation (generation of step commands from fixed time trajectory).
  while (batchRdyForInterp
    && (stepperCmdBuffItems() < (FTM_STEPPERCMD_BUFF_SIZE) - (FTM_STEPS_PER_UNIT_TIME))) {
    convertToSteps(interpIdx);
    if (++interpIdx == FTM_BATCH_SIZE) {
      batchRdyForInterp = false;
      interpIdx = 0;
    }
  }

  // Report busy status to planner.
  busy = (sts_stepperBusy || blockProcRdy || batchRdy || batchRdyForInterp);

}

#if HAS_FTM_SHAPING

  // Refresh the gains used by shaping functions.
  void FTMotion::AxisShaping::set_axis_shaping_A(const ftMotionShaper_t shaper, const_float_t zeta, const_float_t vtol) {

    const float K = exp(-zeta * M_PI / sqrt(1.f - sq(zeta))),
                K2 = sq(K),
                K3 = K2 * K,
                K4 = K3 * K;

    switch (shaper) {

      case ftMotionShaper_ZV:
        max_i = 1U;
        Ai[0] = 1.0f / (1.0f + K);
        Ai[1] = Ai[0] * K;
        break;

      case ftMotionShaper_ZVD:
        max_i = 2U;
        Ai[0] = 1.0f / (1.0f + 2.0f * K + K2);
        Ai[1] = Ai[0] * 2.0f * K;
        Ai[2] = Ai[0] * K2;
        break;

      case ftMotionShaper_ZVDD:
        max_i = 3U;
        Ai[0] = 1.0f / (1.0f + 3.0f * K + 3.0f * K2 + K3);
        Ai[1] = Ai[0] * 3.0f * K;
        Ai[2] = Ai[0] * 3.0f * K2;
        Ai[3] = Ai[0] * K3;
        break;

      case ftMotionShaper_ZVDDD:
        max_i = 4U;
        Ai[0] = 1.0f / (1.0f + 4.0f * K + 6.0f * K2 + 4.0f * K3 + K4);
        Ai[1] = Ai[0] * 4.0f * K;
        Ai[2] = Ai[0] * 6.0f * K2;
        Ai[3] = Ai[0] * 4.0f * K3;
        Ai[4] = Ai[0] * K4;
        break;

      case ftMotionShaper_EI: {
        max_i = 2U;
        Ai[0] = 0.25f * (1.0f + vtol);
        Ai[1] = 0.50f * (1.0f - vtol) * K;
        Ai[2] = Ai[0] * K2;

        const float adj = 1.0f / (Ai[0] + Ai[1] + Ai[2]);
        for (uint32_t i = 0U; i < 3U; i++) {
          Ai[i] *= adj;
        }
      }
      break;

      case ftMotionShaper_2HEI: {
        max_i = 3U;
        const float vtolx2 = sq(vtol);
        const float X = pow(vtolx2 * (sqrt(1.0f - vtolx2) + 1.0f), 1.0f / 3.0f);
        Ai[0] = (3.0f * sq(X) + 2.0f * X + 3.0f * vtolx2) / (16.0f * X);
        Ai[1] = (0.5f - Ai[0]) * K;
        Ai[2] = Ai[1] * K;
        Ai[3] = Ai[0] * K3;

        const float adj = 1.0f / (Ai[0] + Ai[1] + Ai[2] + Ai[3]);
        for (uint32_t i = 0U; i < 4U; i++) {
          Ai[i] *= adj;
        }
      }
      break;

      case ftMotionShaper_3HEI: {
        max_i = 4U;
        Ai[0] = 0.0625f * ( 1.0f + 3.0f * vtol + 2.0f * sqrt( 2.0f * ( vtol + 1.0f ) * vtol ) );
        Ai[1] = 0.25f * ( 1.0f - vtol ) * K;
        Ai[2] = ( 0.5f * ( 1.0f + vtol ) - 2.0f * Ai[0] ) * K2;
        Ai[3] = Ai[1] * K2;
        Ai[4] = Ai[0] * K4;

        const float adj = 1.0f / (Ai[0] + Ai[1] + Ai[2] + Ai[3] + Ai[4]);
        for (uint32_t i = 0U; i < 5U; i++) {
          Ai[i] *= adj;
        }
      }
      break;

      case ftMotionShaper_MZV: {
        max_i = 2U;
        const float Bx = 1.4142135623730950488016887242097f * K;
        Ai[0] = 1.0f / (1.0f + Bx + K2);
        Ai[1] = Ai[0] * Bx;
        Ai[2] = Ai[0] * K2;
      }
      break;

      default:
        ZERO(Ai);
        max_i = 0;
    }

  }

  // Refresh the indices used by shaping functions.
  void FTMotion::AxisShaping::set_axis_shaping_N(const ftMotionShaper_t shaper, const_float_t f, const_float_t zeta) {
    // Note that protections are omitted for DBZ and for index exceeding array length.
    const float df = sqrt ( 1.f - sq(zeta) );
    switch (shaper) {
      case ftMotionShaper_ZV:
        Ni[1] = round((0.5f / f / df) * (FTM_FS));
        break;
      case ftMotionShaper_ZVD:
      case ftMotionShaper_EI:
        Ni[1] = round((0.5f / f / df) * (FTM_FS));
        Ni[2] = Ni[1] + Ni[1];
        break;
      case ftMotionShaper_ZVDD:
      case ftMotionShaper_2HEI:
        Ni[1] = round((0.5f / f / df) * (FTM_FS));
        Ni[2] = Ni[1] + Ni[1];
        Ni[3] = Ni[2] + Ni[1];
        break;
      case ftMotionShaper_ZVDDD:
      case ftMotionShaper_3HEI:
        Ni[1] = round((0.5f / f / df) * (FTM_FS));
        Ni[2] = Ni[1] + Ni[1];
        Ni[3] = Ni[2] + Ni[1];
        Ni[4] = Ni[3] + Ni[1];
        break;
      case ftMotionShaper_MZV:
        Ni[1] = round((0.375f / f / df) * (FTM_FS));
        Ni[2] = Ni[1] + Ni[1];
        break;
      default: ZERO(Ni);
    }
  }

  void FTMotion::update_shaping_params() {
    #if HAS_X_AXIS
      if ((shaping.x.ena = AXIS_HAS_SHAPER(X))) {
        shaping.x.set_axis_shaping_A(cfg.shaper.x, cfg.zeta.x, cfg.vtol.x);
        shaping.x.set_axis_shaping_N(cfg.shaper.x, cfg.baseFreq.x, cfg.zeta.x);
      }
    #endif
    #if HAS_Y_AXIS
      if ((shaping.y.ena = AXIS_HAS_SHAPER(Y))) {
        shaping.y.set_axis_shaping_A(cfg.shaper.y, cfg.zeta.y, cfg.vtol.y);
        shaping.y.set_axis_shaping_N(cfg.shaper.y, cfg.baseFreq.y, cfg.zeta.y);
      }
    #endif
  }

#endif // HAS_FTM_SHAPING

// Reset all trajectory processing variables.
void FTMotion::reset() {

  stepperCmdBuff_produceIdx = stepperCmdBuff_consumeIdx = 0;

  traj.reset();

  blockProcRdy = batchRdy = batchRdyForInterp = false;

  endPosn_prevBlock.reset();

  makeVector_idx = 0;
  makeVector_batchIdx = TERN(FTM_UNIFIED_BWS, 0, _MIN(BATCH_SIDX_IN_WINDOW, FTM_BATCH_SIZE));

  steps.reset();
  interpIdx = 0;

  #if HAS_FTM_SHAPING
    TERN_(HAS_X_AXIS, ZERO(shaping.x.d_zi));
    TERN_(HAS_Y_AXIS, ZERO(shaping.y.d_zi));
    shaping.zi_idx = 0;
  #endif

  TERN_(HAS_EXTRUDERS, e_raw_z1 = e_advanced_z1 = 0.0f);

  axis_move_end_ti.reset();
}

// Private functions.

void FTMotion::discard_planner_block_protected() {
  if (stepper.current_block) {  // Safeguard in case current_block must not be null (it will
                                // be null when the "block" is a runout or generated) in order
                                // to use planner.release_current_block().
    stepper.current_block = nullptr;
    planner.release_current_block();  // FTM uses release_current_block() instead of discard_current_block(),
                                      // as in block_phase_isr(). This change is to avoid invoking axis_did_move.reset().
                                      // current_block = nullptr is added to replicate discard without axis_did_move reset.
                                      // Note invoking axis_did_move.reset() causes no issue since FTM's stepper refreshes
                                      // its values every ISR.
  }
}

/**
 * Set up a pseudo block to allow motion to settle and buffers to empty.
 * Called when the planner has one block left. The buffers will be filled
 * with the last commanded position by setting the startPosn block variable to
 * the last position of the previous block and all ratios to zero such that no
 * axes' positions are incremented.
 */
void FTMotion::runoutBlock() {

  startPosn = endPosn_prevBlock;
  ratio.reset();

  const int32_t n_to_fill_batch = (FTM_WINDOW_SIZE) - makeVector_batchIdx;

  // This line or function is to be modified for FBS use; do not optimize out.
  const int32_t n_to_settle_shaper = num_samples_shaper_settle();

  const int32_t n_diff = n_to_settle_shaper - n_to_fill_batch,
                n_to_fill_batch_after_settling = n_diff > 0 ? (FTM_BATCH_SIZE) - (n_diff % (FTM_BATCH_SIZE)) : -n_diff;

  const int32_t n_to_settle_and_fill_batch = n_to_settle_shaper + n_to_fill_batch_after_settling;

  const int32_t N_needed_to_propagate_to_stepper = PROP_BATCHES;

  const int32_t n_to_use = N_needed_to_propagate_to_stepper * (FTM_BATCH_SIZE) + n_to_settle_and_fill_batch;

  max_intervals = n_to_use;

  blockProcRdy = true;
}

// Auxiliary function to get number of step commands in the buffer.
int32_t FTMotion::stepperCmdBuffItems() {
  const int32_t udiff = stepperCmdBuff_produceIdx - stepperCmdBuff_consumeIdx;
  return (udiff < 0) ? udiff + (FTM_STEPPERCMD_BUFF_SIZE) : udiff;
}

// Initializes storage variables before startup.
void FTMotion::init() {
  update_shaping_params();
  reset(); // Precautionary.
}

// Load / convert block data from planner to fixed-time control variables.
void FTMotion::loadBlockData(block_t * const current_block) {

  const float totalLength = current_block->millimeters,
              oneOverLength = 1.0f / totalLength;

  startPosn = endPosn_prevBlock;
  const xyze_pos_t moveDist = LOGICAL_AXIS_ARRAY(
    current_block->steps.e * planner.mm_per_step[E_AXIS_N(current_block->extruder)] * (current_block->direction_bits.e ? 1 : -1),
    current_block->steps.x * planner.mm_per_step[X_AXIS] * (current_block->direction_bits.x ? 1 : -1),
    current_block->steps.y * planner.mm_per_step[Y_AXIS] * (current_block->direction_bits.y ? 1 : -1),
    current_block->steps.z * planner.mm_per_step[Z_AXIS] * (current_block->direction_bits.z ? 1 : -1),
    current_block->steps.i * planner.mm_per_step[I_AXIS] * (current_block->direction_bits.i ? 1 : -1),
    current_block->steps.j * planner.mm_per_step[J_AXIS] * (current_block->direction_bits.j ? 1 : -1),
    current_block->steps.k * planner.mm_per_step[K_AXIS] * (current_block->direction_bits.k ? 1 : -1),
    current_block->steps.u * planner.mm_per_step[U_AXIS] * (current_block->direction_bits.u ? 1 : -1),
    current_block->steps.v * planner.mm_per_step[V_AXIS] * (current_block->direction_bits.v ? 1 : -1),
    current_block->steps.w * planner.mm_per_step[W_AXIS] * (current_block->direction_bits.w ? 1 : -1)
  );

  ratio = moveDist * oneOverLength;

  const float spm = totalLength / current_block->step_event_count;  // (steps/mm) Distance for each step

  f_s = spm * current_block->initial_rate;              // (steps/s) Start feedrate

  const float f_e = spm * current_block->final_rate;    // (steps/s) End feedrate

  /* Keep for comprehension
  const float a = current_block->acceleration,          // (mm/s^2) Same magnitude for acceleration or deceleration
              oneby2a = 1.0f / (2.0f * a),              // (s/mm) Time to accelerate or decelerate one mm (i.e., oneby2a * 2
              oneby2d = -oneby2a;                       // (s/mm) Time to accelerate or decelerate one mm (i.e., oneby2a * 2
  const float fsSqByTwoA = sq(f_s) * oneby2a,           // (mm) Distance to accelerate from start speed to nominal speed
              feSqByTwoD = sq(f_e) * oneby2d;           // (mm) Distance to decelerate from nominal speed to end speed

  float F_n = current_block->nominal_speed;             // (mm/s) Speed we hope to achieve, if possible
  const float fdiff = feSqByTwoD - fsSqByTwoA,          // (mm) Coasting distance if nominal speed is reached
              odiff = oneby2a - oneby2d,                // (i.e., oneby2a * 2) (mm/s) Change in speed for one second of acceleration
              ldiff = totalLength - fdiff;              // (mm) Distance to travel if nominal speed is reached

  float T2 = (1.0f / F_n) * (ldiff - odiff * sq(F_n));  // (s) Coasting duration after nominal speed reached
  if (T2 < 0.0f) {
    T2 = 0.0f;
    F_n = SQRT(ldiff / odiff);                          // Clip by intersection if nominal speed can't be reached.
  }

  const float T1 = (F_n - f_s) / a,                     // (s) Accel Time = difference in feedrate over acceleration
              T3 = (F_n - f_e) / a;                     // (s) Decel Time = difference in feedrate over acceleration
  */

  const float accel = current_block->acceleration,
              oneOverAccel = 1.0f / accel;

  float F_n = current_block->nominal_speed;
  const float ldiff = totalLength + 0.5f * oneOverAccel * (sq(f_s) + sq(f_e));

  float T2 = ldiff / F_n - oneOverAccel * F_n;
  if (T2 < 0.0f) {
    T2 = 0.0f;
    F_n = SQRT(ldiff * accel);
  }

  const float T1 = (F_n - f_s) * oneOverAccel,
              T3 = (F_n - f_e) * oneOverAccel;

  N1 = CEIL(T1 * (FTM_FS));         // Accel datapoints based on Hz frequency
  N2 = CEIL(T2 * (FTM_FS));         // Coast
  N3 = CEIL(T3 * (FTM_FS));         // Decel

  const float T1_P = N1 * (FTM_TS), // (s) Accel datapoints x timestep resolution
              T2_P = N2 * (FTM_TS), // (s) Coast
              T3_P = N3 * (FTM_TS); // (s) Decel

  /**
   * Calculate the reachable feedrate at the end of the accel phase.
   *  totalLength is the total distance to travel in mm
   *  f_s        : (mm/s) Starting feedrate
   *  f_e        : (mm/s) Ending feedrate
   *  T1_P       : (sec) Time spent accelerating
   *  T2_P       : (sec) Time spent coasting
   *  T3_P       : (sec) Time spent decelerating
   *  f_s * T1_P : (mm) Distance traveled during the accel phase
   *  f_e * T3_P : (mm) Distance traveled during the decel phase
   */
  const float adist = f_s * T1_P;
  F_P = (2.0f * totalLength - adist - f_e * T3_P) / (T1_P + 2.0f * T2_P + T3_P); // (mm/s) Feedrate at the end of the accel phase

  // Calculate the acceleration and deceleration rates
  accel_P = N1 ? ((F_P - f_s) / T1_P) : 0.0f;

  decel_P = (f_e - F_P) / T3_P;

  // Calculate the distance traveled during the accel phase
  s_1e = adist + 0.5f * accel_P * sq(T1_P);

  // Calculate the distance traveled during the decel phase
  s_2e = s_1e + F_P * T2_P;

  // Accel + Coasting + Decel datapoints
  max_intervals = N1 + N2 + N3;

  endPosn_prevBlock += moveDist;

  // Watch endstops until the move ends
  const millis_t move_end_ti = millis() + SEC_TO_MS((FTM_TS) * float(max_intervals + num_samples_shaper_settle() + ((PROP_BATCHES) + 1) * (FTM_BATCH_SIZE)) + (float(FTM_STEPPERCMD_BUFF_SIZE) / float(FTM_STEPPER_FS)));

  #define __SET_MOVE_END(A,V) do{ if (V) { axis_move_end_ti.A = move_end_ti; axis_move_dir.A = (V > 0); } }while(0);
  #define _SET_MOVE_END(A) __SET_MOVE_END(A, moveDist[_AXIS(A)])
  #if CORE_IS_XY
    __SET_MOVE_END(X, moveDist.x + moveDist.y);
    __SET_MOVE_END(Y, moveDist.x - moveDist.y);
  #else
    _SET_MOVE_END(X);
    _SET_MOVE_END(Y);
  #endif
  TERN_(HAS_Z_AXIS, _SET_MOVE_END(Z));
  SECONDARY_AXIS_MAP(_SET_MOVE_END);

  // If the endstop is already pressed, endstop interrupts won't invoke
  // endstop_triggered and the move will grind. So check here for a
  // triggered endstop, which shortly marks the block for discard.
  endstops.update();

}

// Generate data points of the trajectory.
void FTMotion::makeVector() {
  float accel_k = 0.0f;                                 // (mm/s^2) Acceleration K factor
  float tau = (makeVector_idx + 1) * (FTM_TS);          // (s) Time since start of block
  float dist = 0.0f;                                    // (mm) Distance traveled

  if (makeVector_idx < N1) {
    // Acceleration phase
    dist = (f_s * tau) + (0.5f * accel_P * sq(tau));    // (mm) Distance traveled for acceleration phase since start of block
    accel_k = accel_P;                                  // (mm/s^2) Acceleration K factor from Accel phase
  }
  else if (makeVector_idx < (N1 + N2)) {
    // Coasting phase
    dist = s_1e + F_P * (tau - N1 * (FTM_TS));          // (mm) Distance traveled for coasting phase since start of block
    //accel_k = 0.0f;
  }
  else {
    // Deceleration phase
    tau -= (N1 + N2) * (FTM_TS);                        // (s) Time since start of decel phase
    dist = s_2e + F_P * tau + 0.5f * decel_P * sq(tau); // (mm) Distance traveled for deceleration phase since start of block
    accel_k = decel_P;                                  // (mm/s^2) Acceleration K factor from Decel phase
  }

  #define _SET_TRAJ(q) traj.q[makeVector_batchIdx] = startPosn.q + ratio.q * dist;
  LOGICAL_AXIS_MAP_LC(_SET_TRAJ);

  #if HAS_EXTRUDERS
    if (cfg.linearAdvEna) {
      float dedt_adj = (traj.e[makeVector_batchIdx] - e_raw_z1) * (FTM_FS);
      if (ratio.e > 0.0f) dedt_adj += accel_k * cfg.linearAdvK * 0.0001f;

      e_raw_z1 = traj.e[makeVector_batchIdx];
      e_advanced_z1 += dedt_adj * (FTM_TS);
      traj.e[makeVector_batchIdx] = e_advanced_z1;
    }
  #endif

  // Update shaping parameters if needed.

  switch (cfg.dynFreqMode) {

    #if HAS_DYNAMIC_FREQ_MM
      case dynFreqMode_Z_BASED: {
        static float oldz = 0.0f;
        const float z = traj.z[makeVector_batchIdx];
        if (z != oldz) { // Only update if Z changed.
          oldz = z;
          #if HAS_X_AXIS
            const float xf = cfg.baseFreq.x + cfg.dynFreqK.x * z;
            shaping.x.set_axis_shaping_N(cfg.shaper.x, _MAX(xf, FTM_MIN_SHAPE_FREQ), cfg.zeta.x);
          #endif
          #if HAS_Y_AXIS
            const float yf = cfg.baseFreq.y + cfg.dynFreqK.y * z;
            shaping.y.set_axis_shaping_N(cfg.shaper.y, _MAX(yf, FTM_MIN_SHAPE_FREQ), cfg.zeta.y);
          #endif
        }
      } break;
    #endif

    #if HAS_DYNAMIC_FREQ_G
      case dynFreqMode_MASS_BASED:
        // Update constantly. The optimization done for Z value makes
        // less sense for E, as E is expected to constantly change.
        #if HAS_X_AXIS
          shaping.x.set_axis_shaping_N(cfg.shaper.x, cfg.baseFreq.x + cfg.dynFreqK.x * traj.e[makeVector_batchIdx], cfg.zeta.x);
        #endif
        #if HAS_Y_AXIS
          shaping.y.set_axis_shaping_N(cfg.shaper.y, cfg.baseFreq.y + cfg.dynFreqK.y * traj.e[makeVector_batchIdx], cfg.zeta.y);
        #endif
        break;
    #endif

    default: break;
  }

  // Apply shaping if active on each axis
  #if HAS_FTM_SHAPING
    #if HAS_X_AXIS
      if (shaping.x.ena) {
        shaping.x.d_zi[shaping.zi_idx] = traj.x[makeVector_batchIdx];
        traj.x[makeVector_batchIdx] *= shaping.x.Ai[0];
        for (uint32_t i = 1U; i <= shaping.x.max_i; i++) {
          const uint32_t udiffx = shaping.zi_idx - shaping.x.Ni[i];
          traj.x[makeVector_batchIdx] += shaping.x.Ai[i] * shaping.x.d_zi[shaping.x.Ni[i] > shaping.zi_idx ? (FTM_ZMAX) + udiffx : udiffx];
        }
      }
    #endif

    #if HAS_Y_AXIS
      if (shaping.y.ena) {
        shaping.y.d_zi[shaping.zi_idx] = traj.y[makeVector_batchIdx];
        traj.y[makeVector_batchIdx] *= shaping.y.Ai[0];
        for (uint32_t i = 1U; i <= shaping.y.max_i; i++) {
          const uint32_t udiffy = shaping.zi_idx - shaping.y.Ni[i];
          traj.y[makeVector_batchIdx] += shaping.y.Ai[i] * shaping.y.d_zi[shaping.y.Ni[i] > shaping.zi_idx ? (FTM_ZMAX) + udiffy : udiffy];
        }
      }
    #endif
    if (++shaping.zi_idx == (FTM_ZMAX)) shaping.zi_idx = 0;
  #endif // HAS_FTM_SHAPING

  // Filled up the queue with regular and shaped steps
  if (++makeVector_batchIdx == FTM_WINDOW_SIZE) {
    makeVector_batchIdx = BATCH_SIDX_IN_WINDOW;
    batchRdy = true;
  }

  if (++makeVector_idx == max_intervals) {
    blockProcRdy = false;
    makeVector_idx = 0;
  }
}

/**
 * Convert to steps
 * - Commands are written in a bitmask with step and dir as single bits.
 * - Tests for delta are moved outside the loop.
 * - Two functions are used for command computation with an array of function pointers.
 */
static void (*command_set[LOGICAL_AXES])(int32_t&, int32_t&, ft_command_t&, int32_t, int32_t);

static void command_set_pos(int32_t &e, int32_t &s, ft_command_t &b, int32_t bd, int32_t bs) {
  if (e < FTM_CTS_COMPARE_VAL) return;
  s++;
  b |= bd | bs;
  e -= FTM_STEPS_PER_UNIT_TIME;
}

static void command_set_neg(int32_t &e, int32_t &s, ft_command_t &b, int32_t bd, int32_t bs) {
  if (e > -(FTM_CTS_COMPARE_VAL)) return;
  s--;
  b |= bs;
  e += FTM_STEPS_PER_UNIT_TIME;
}

// Interpolates single data point to stepper commands.
void FTMotion::convertToSteps(const uint32_t idx) {
  xyze_long_t err_P = { 0 };

  //#define STEPS_ROUNDING
  #if ENABLED(STEPS_ROUNDING)
    #define TOSTEPS(A,B) int32_t(trajMod.A[idx] * planner.settings.axis_steps_per_mm[B] + (trajMod.A[idx] < 0.0f ? -0.5f : 0.5f))
    const xyze_long_t steps_tar = LOGICAL_AXIS_ARRAY(
      TOSTEPS(e, E_AXIS_N(stepper.current_block->extruder)), // May be eliminated if guaranteed positive.
      TOSTEPS(x, X_AXIS), TOSTEPS(y, Y_AXIS), TOSTEPS(z, Z_AXIS),
      TOSTEPS(i, I_AXIS), TOSTEPS(j, J_AXIS), TOSTEPS(k, K_AXIS),
      TOSTEPS(u, U_AXIS), TOSTEPS(v, V_AXIS), TOSTEPS(w, W_AXIS)
    );
    xyze_long_t delta = steps_tar - steps;
  #else
    #define TOSTEPS(A,B) int32_t(trajMod.A[idx] * planner.settings.axis_steps_per_mm[B]) - steps.A
    xyze_long_t delta = LOGICAL_AXIS_ARRAY(
      TOSTEPS(e, E_AXIS_N(stepper.current_block->extruder)),
      TOSTEPS(x, X_AXIS), TOSTEPS(y, Y_AXIS), TOSTEPS(z, Z_AXIS),
      TOSTEPS(i, I_AXIS), TOSTEPS(j, J_AXIS), TOSTEPS(k, K_AXIS),
      TOSTEPS(u, U_AXIS), TOSTEPS(v, V_AXIS), TOSTEPS(w, W_AXIS)
    );
  #endif

  #define _COMMAND_SET(AXIS) command_set[_AXIS(AXIS)] = delta[_AXIS(AXIS)] >= 0 ? command_set_pos : command_set_neg;
  LOGICAL_AXIS_MAP(_COMMAND_SET);

  for (uint32_t i = 0U; i < (FTM_STEPS_PER_UNIT_TIME); i++) {

    ft_command_t &cmd = stepperCmdBuff[stepperCmdBuff_produceIdx];

    // Init all step/dir bits to 0 (defaulting to reverse/negative motion)
    cmd = 0;

    // Accumulate the errors for all axes
    err_P += delta;

    // Set up step/dir bits for all axes
    #define _COMMAND_RUN(A) command_set[_AXIS(A)](err_P.A, steps.A, cmd, _BV(FT_BIT_DIR_##A), _BV(FT_BIT_STEP_##A));
    LOGICAL_AXIS_MAP(_COMMAND_RUN);

    // Next circular buffer index
    if (++stepperCmdBuff_produceIdx == (FTM_STEPPERCMD_BUFF_SIZE))
      stepperCmdBuff_produceIdx = 0;

  } // FTM_STEPS_PER_UNIT_TIME loop
}

#endif // FT_MOTION
