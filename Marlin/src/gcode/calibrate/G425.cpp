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

#include "../../MarlinCore.h"

#if ENABLED(CALIBRATION_GCODE)

#include "../gcode.h"

#if ENABLED(BACKLASH_GCODE)
  #include "../../feature/backlash.h"
#endif

#include "../../lcd/marlinui.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/endstops.h"
#include "../../feature/bedlevel/bedlevel.h"

#if HAS_MULTI_HOTEND
  #include "../../module/tool_change.h"
#endif

#if !AXIS_CAN_CALIBRATE(X)
  #undef CALIBRATION_MEASURE_LEFT
  #undef CALIBRATION_MEASURE_RIGHT
#endif

#if !AXIS_CAN_CALIBRATE(Y)
  #undef CALIBRATION_MEASURE_FRONT
  #undef CALIBRATION_MEASURE_BACK
#endif

#if !AXIS_CAN_CALIBRATE(Z)
  #undef CALIBRATION_MEASURE_AT_TOP_EDGES
#endif

/**
 * G425 backs away from the calibration object by various distances
 * depending on the confidence level:
 *
 *   UNKNOWN   - No real notion on where the calibration object is on the bed
 *   UNCERTAIN - Measurement may be uncertain due to backlash
 *   CERTAIN   - Measurement obtained with backlash compensation
 */

#ifndef CALIBRATION_MEASUREMENT_UNKNOWN
  #define CALIBRATION_MEASUREMENT_UNKNOWN   5.0 // mm
#endif
#ifndef CALIBRATION_MEASUREMENT_UNCERTAIN
  #define CALIBRATION_MEASUREMENT_UNCERTAIN 1.0 // mm
#endif
#ifndef CALIBRATION_MEASUREMENT_CERTAIN
  #define CALIBRATION_MEASUREMENT_CERTAIN   0.5 // mm
#endif

#if ALL(HAS_X_AXIS, CALIBRATION_MEASURE_LEFT, CALIBRATION_MEASURE_RIGHT)
  #define HAS_X_CENTER 1
#endif
#if ALL(HAS_Y_AXIS, CALIBRATION_MEASURE_FRONT, CALIBRATION_MEASURE_BACK)
  #define HAS_Y_CENTER 1
#endif
#if ALL(HAS_I_AXIS, CALIBRATION_MEASURE_IMIN, CALIBRATION_MEASURE_IMAX)
  #define HAS_I_CENTER 1
#endif
#if ALL(HAS_J_AXIS, CALIBRATION_MEASURE_JMIN, CALIBRATION_MEASURE_JMAX)
  #define HAS_J_CENTER 1
#endif
#if ALL(HAS_K_AXIS, CALIBRATION_MEASURE_KMIN, CALIBRATION_MEASURE_KMAX)
  #define HAS_K_CENTER 1
#endif
#if ALL(HAS_U_AXIS, CALIBRATION_MEASURE_UMIN, CALIBRATION_MEASURE_UMAX)
  #define HAS_U_CENTER 1
#endif
#if ALL(HAS_V_AXIS, CALIBRATION_MEASURE_VMIN, CALIBRATION_MEASURE_VMAX)
  #define HAS_V_CENTER 1
#endif
#if ALL(HAS_W_AXIS, CALIBRATION_MEASURE_WMIN, CALIBRATION_MEASURE_WMAX)
  #define HAS_W_CENTER 1
#endif

enum side_t : uint8_t {
  TOP, RIGHT, FRONT, LEFT, BACK, NUM_SIDES,
  LIST_N(DOUBLE(SECONDARY_AXES), IMINIMUM, IMAXIMUM, JMINIMUM, JMAXIMUM, KMINIMUM, KMAXIMUM, UMINIMUM, UMAXIMUM, VMINIMUM, VMAXIMUM, WMINIMUM, WMAXIMUM)
};

static constexpr xyz_pos_t true_center CALIBRATION_OBJECT_CENTER;
static constexpr xyz_float_t dimensions CALIBRATION_OBJECT_DIMENSIONS;
static constexpr xy_float_t nod = { CALIBRATION_NOZZLE_OUTER_DIAMETER, CALIBRATION_NOZZLE_OUTER_DIAMETER };

struct measurements_t {
  xyz_pos_t obj_center = true_center; // Non-static must be assigned from xyz_pos_t

  float obj_side[NUM_SIDES], backlash[NUM_SIDES];
  xyz_float_t pos_error;

  xy_float_t nozzle_outer_dimension = nod;
};

#if ENABLED(BACKLASH_GCODE)
  class restorer_correction {
    const uint8_t val_;
  public:
    restorer_correction(const uint8_t temp_val) : val_(backlash.get_correction_uint8()) { backlash.set_correction_uint8(temp_val); }
    ~restorer_correction() { backlash.set_correction_uint8(val_); }
  };

  #define TEMPORARY_BACKLASH_CORRECTION(value) restorer_correction restorer_tbst(value)
#else
  #define TEMPORARY_BACKLASH_CORRECTION(value)
#endif

#if ENABLED(BACKLASH_GCODE) && defined(BACKLASH_SMOOTHING_MM)
  class restorer_smoothing {
    const float val_;
  public:
    restorer_smoothing(const float temp_val) : val_(backlash.get_smoothing_mm()) { backlash.set_smoothing_mm(temp_val); }
    ~restorer_smoothing() { backlash.set_smoothing_mm(val_); }
  };

  #define TEMPORARY_BACKLASH_SMOOTHING(value) restorer_smoothing restorer_tbsm(value)
#else
  #define TEMPORARY_BACKLASH_SMOOTHING(value)
#endif

inline void calibration_move() {
  do_blocking_move_to((xyz_pos_t)current_position, MMM_TO_MMS(CALIBRATION_FEEDRATE_TRAVEL));
}

/**
 * Move to the exact center above the calibration object
 *
 *   m                  in     - Measurement record
 *   uncertainty        in     - How far away from the object top to park
 */
inline void park_above_object(measurements_t &m, const float uncertainty) {
  // Move to safe distance above calibration object
  current_position.z = m.obj_center.z + dimensions.z / 2 + uncertainty;
  calibration_move();

  // Move to center of calibration object in XY
  current_position = xy_pos_t(m.obj_center);
  calibration_move();
}

#if HAS_MULTI_HOTEND
  inline void set_nozzle(measurements_t &m, const uint8_t extruder) {
    if (extruder != active_extruder) {
      park_above_object(m, CALIBRATION_MEASUREMENT_UNKNOWN);
      tool_change(extruder);
    }
  }
#endif

#if HAS_HOTEND_OFFSET

  inline void normalize_hotend_offsets() {
    for (uint8_t e = 1; e < HOTENDS; ++e)
      hotend_offset[e] -= hotend_offset[0];
    hotend_offset[0].reset();
  }

#endif

/**
 * Move along axis in the specified dir until the probe value becomes stop_state,
 * then return the axis value.
 *
 *   axis         in - Axis along which the measurement will take place
 *   dir          in - Direction along that axis (-1 or 1)
 *   stop_state   in - Move until probe pin becomes this value
 *   fast         in - Fast vs. precise measurement
 */
float measuring_movement(const AxisEnum axis, const int dir, const bool stop_state, const bool fast) {
  const feedRate_t mms = fast ? MMM_TO_MMS(CALIBRATION_FEEDRATE_FAST) : MMM_TO_MMS(CALIBRATION_FEEDRATE_SLOW);
  const float limit    = fast ? 50 : 5;

  destination = current_position;
  destination[axis] += dir * limit;
  endstops.enable_calibration_probe(true, stop_state);
  do_blocking_move_to((xyz_pos_t)destination, mms);
  endstops.enable_calibration_probe(false);
  endstops.hit_on_purpose();
  set_current_from_steppers_for_axis(axis);
  sync_plan_position();
  return current_position[axis];
}

/**
 * Move along axis until the probe is triggered. Move toolhead to its starting
 * point and return the measured value.
 *
 *   axis               in     - Axis along which the measurement will take place
 *   dir                in     - Direction along that axis (-1 or 1)
 *   stop_state         in     - Move until probe pin becomes this value
 *   backlash_ptr       in/out - When not nullptr, measure and record axis backlash
 *   uncertainty        in     - If uncertainty is CALIBRATION_MEASUREMENT_UNKNOWN, do a fast probe.
 */
inline float measure(const AxisEnum axis, const int dir, const bool stop_state, float * const backlash_ptr, const float uncertainty) {
  const bool fast = uncertainty == CALIBRATION_MEASUREMENT_UNKNOWN;

  // Save the current position of the specified axis
  const float start_pos = current_position[axis];

  // Take a measurement. Only the specified axis will be affected.
  const float measured_pos = measuring_movement(axis, dir, stop_state, fast);

  // Measure backlash
  if (backlash_ptr && !fast) {
    const float release_pos = measuring_movement(axis, -dir, !stop_state, fast);
    *backlash_ptr = ABS(release_pos - measured_pos);
  }

  // Move back to the starting position
  destination = current_position;
  destination[axis] = start_pos;
  do_blocking_move_to((xyz_pos_t)destination, MMM_TO_MMS(CALIBRATION_FEEDRATE_TRAVEL));
  return measured_pos;
}

/**
 * Probe one side of the calibration object
 *
 *   m                  in/out - Measurement record, m.obj_center and m.obj_side will be updated.
 *   uncertainty        in     - How far away from the calibration object to begin probing
 *   side               in     - Side of probe where probe will occur
 *   probe_top_at_edge  in     - When probing sides, probe top of calibration object nearest edge
 *                               to find out height of edge
 */
inline void probe_side(measurements_t &m, const float uncertainty, const side_t side, const bool probe_top_at_edge=false) {
  const xyz_float_t dimensions = CALIBRATION_OBJECT_DIMENSIONS;
  AxisEnum axis;
  float dir = 1;

  park_above_object(m, uncertainty);

  #define _ACASE(N,A,B) case A: dir = -1; case B: axis = N##_AXIS; break
  #define _PCASE(N) _ACASE(N, N##MINIMUM, N##MAXIMUM)

  switch (side) {
    #if AXIS_CAN_CALIBRATE(X)
      _ACASE(X, RIGHT, LEFT);
    #endif
    #if AXIS_CAN_CALIBRATE(Y)
      _ACASE(Y, BACK, FRONT);
    #endif
    #if AXIS_CAN_CALIBRATE(Z)
      case TOP: {
        const float measurement = measure(Z_AXIS, -1, true, &m.backlash[TOP], uncertainty);
        m.obj_center.z = measurement - dimensions.z / 2;
        m.obj_side[TOP] = measurement;
        return;
      }
    #endif
    #if AXIS_CAN_CALIBRATE(I)
      _PCASE(I);
    #endif
    #if AXIS_CAN_CALIBRATE(J)
      _PCASE(J);
    #endif
    #if AXIS_CAN_CALIBRATE(K)
      _PCASE(K);
    #endif
    #if AXIS_CAN_CALIBRATE(U)
      _PCASE(U);
    #endif
    #if AXIS_CAN_CALIBRATE(V)
      _PCASE(V);
    #endif
    #if AXIS_CAN_CALIBRATE(W)
      _PCASE(W);
    #endif
    default: return;
  }

  if (probe_top_at_edge) {
    #if AXIS_CAN_CALIBRATE(Z)
      // Probe top nearest the side we are probing
      current_position[axis] = m.obj_center[axis] + (-dir) * (dimensions[axis] / 2 - m.nozzle_outer_dimension[axis]);
      calibration_move();
      m.obj_side[TOP] = measure(Z_AXIS, -1, true, &m.backlash[TOP], uncertainty);
      m.obj_center.z = m.obj_side[TOP] - dimensions.z / 2;
    #endif
  }

  if ((AXIS_CAN_CALIBRATE(X) && axis == X_AXIS) || (AXIS_CAN_CALIBRATE(Y) && axis == Y_AXIS)) {
    // Move to safe distance to the side of the calibration object
    current_position[axis] = m.obj_center[axis] + (-dir) * (dimensions[axis] / 2 + m.nozzle_outer_dimension[axis] / 2 + uncertainty);
    calibration_move();

    // Plunge below the side of the calibration object and measure
    current_position.z = m.obj_side[TOP] - (CALIBRATION_NOZZLE_TIP_HEIGHT) * 0.7f;
    calibration_move();
    const float measurement = measure(axis, dir, true, &m.backlash[side], uncertainty);
    m.obj_center[axis] = measurement + dir * (dimensions[axis] / 2 + m.nozzle_outer_dimension[axis] / 2);
    m.obj_side[side] = measurement;
  }
}

/**
 * Probe all sides of the calibration calibration object
 *
 *   m                  in/out - Measurement record: center, backlash and error values be updated.
 *   uncertainty        in     - How far away from the calibration object to begin probing
 */
inline void probe_sides(measurements_t &m, const float uncertainty) {
  #if ENABLED(CALIBRATION_MEASURE_AT_TOP_EDGES)
    constexpr bool probe_top_at_edge = true;
  #else
    // Probing at the exact center only works if the center is flat. Probing on a washer
    // or bolt will require probing the top near the side edges, away from the center.
    constexpr bool probe_top_at_edge = false;
    probe_side(m, uncertainty, TOP);
  #endif

  /**
   * Allow Y axis to probe and compute values before X axis (or remaining arbitrary axes)
   * to assist with centering in calibration object. Lulzbot saw issues with higher uncertainty
   * values where the nozzle was catching on the edges of the cube, and this was intended to help
   * ensure the probe object remained centered.
   */
  TERN_(CALIBRATION_MEASURE_FRONT, probe_side(m, uncertainty, FRONT,    probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_BACK,  probe_side(m, uncertainty, BACK,     probe_top_at_edge));

  #if HAS_Y_CENTER
    m.obj_center.y = (m.obj_side[FRONT] + m.obj_side[BACK]) / 2;
    m.nozzle_outer_dimension.y = m.obj_side[BACK] - m.obj_side[FRONT] - dimensions.y;
  #endif

  TERN_(CALIBRATION_MEASURE_LEFT,  probe_side(m, uncertainty, LEFT,     probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_RIGHT, probe_side(m, uncertainty, RIGHT,    probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_IMIN,  probe_side(m, uncertainty, IMINIMUM, probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_IMAX,  probe_side(m, uncertainty, IMAXIMUM, probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_JMIN,  probe_side(m, uncertainty, JMINIMUM, probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_JMAX,  probe_side(m, uncertainty, JMAXIMUM, probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_KMIN,  probe_side(m, uncertainty, KMINIMUM, probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_KMAX,  probe_side(m, uncertainty, KMAXIMUM, probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_UMIN,  probe_side(m, uncertainty, UMINIMUM, probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_UMAX,  probe_side(m, uncertainty, UMAXIMUM, probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_VMIN,  probe_side(m, uncertainty, VMINIMUM, probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_VMAX,  probe_side(m, uncertainty, VMAXIMUM, probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_WMIN,  probe_side(m, uncertainty, WMINIMUM, probe_top_at_edge));
  TERN_(CALIBRATION_MEASURE_WMAX,  probe_side(m, uncertainty, WMAXIMUM, probe_top_at_edge));

  // Compute the measured center of the calibration object.
  TERN_(HAS_X_CENTER, m.obj_center.x = (m.obj_side[LEFT]     + m.obj_side[RIGHT])    / 2);
  TERN_(HAS_I_CENTER, m.obj_center.i = (m.obj_side[IMINIMUM] + m.obj_side[IMAXIMUM]) / 2);
  TERN_(HAS_J_CENTER, m.obj_center.j = (m.obj_side[JMINIMUM] + m.obj_side[JMAXIMUM]) / 2);
  TERN_(HAS_K_CENTER, m.obj_center.k = (m.obj_side[KMINIMUM] + m.obj_side[KMAXIMUM]) / 2);
  TERN_(HAS_U_CENTER, m.obj_center.u = (m.obj_side[UMINIMUM] + m.obj_side[UMAXIMUM]) / 2);
  TERN_(HAS_V_CENTER, m.obj_center.v = (m.obj_side[VMINIMUM] + m.obj_side[VMAXIMUM]) / 2);
  TERN_(HAS_W_CENTER, m.obj_center.w = (m.obj_side[WMINIMUM] + m.obj_side[WMAXIMUM]) / 2);

  // Compute the outside diameter of the nozzle at the height
  // at which it makes contact with the calibration object
  TERN_(HAS_X_CENTER, m.nozzle_outer_dimension.x = m.obj_side[RIGHT] - m.obj_side[LEFT] - dimensions.x);

  park_above_object(m, uncertainty);

  // The difference between the known and the measured location
  // of the calibration object is the positional error
  NUM_AXIS_CODE(
    m.pos_error.x = TERN0(HAS_X_CENTER, true_center.x - m.obj_center.x),
    m.pos_error.y = TERN0(HAS_Y_CENTER, true_center.y - m.obj_center.y),
    m.pos_error.z = true_center.z - m.obj_center.z,
    m.pos_error.i = TERN0(HAS_I_CENTER, true_center.i - m.obj_center.i),
    m.pos_error.j = TERN0(HAS_J_CENTER, true_center.j - m.obj_center.j),
    m.pos_error.k = TERN0(HAS_K_CENTER, true_center.k - m.obj_center.k),
    m.pos_error.u = TERN0(HAS_U_CENTER, true_center.u - m.obj_center.u),
    m.pos_error.v = TERN0(HAS_V_CENTER, true_center.v - m.obj_center.v),
    m.pos_error.w = TERN0(HAS_W_CENTER, true_center.w - m.obj_center.w)
  );
}

#if ENABLED(CALIBRATION_REPORTING)
  inline void report_measured_faces(const measurements_t &m) {
    SERIAL_ECHOLNPGM("Sides:");
    #if AXIS_CAN_CALIBRATE(Z)
      SERIAL_ECHOLNPGM("  Top: ", m.obj_side[TOP]);
    #endif
    #if HAS_X_AXIS
      #if ENABLED(CALIBRATION_MEASURE_LEFT)
        SERIAL_ECHOLNPGM("  Left: ", m.obj_side[LEFT]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_RIGHT)
        SERIAL_ECHOLNPGM("  Right: ", m.obj_side[RIGHT]);
      #endif
    #endif
    #if HAS_Y_AXIS
      #if ENABLED(CALIBRATION_MEASURE_FRONT)
        SERIAL_ECHOLNPGM("  Front: ", m.obj_side[FRONT]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_BACK)
        SERIAL_ECHOLNPGM("  Back: ", m.obj_side[BACK]);
      #endif
    #endif
    #if HAS_I_AXIS
      #if ENABLED(CALIBRATION_MEASURE_IMIN)
        SERIAL_ECHOLNPGM("  " STR_I_MIN ": ", m.obj_side[IMINIMUM]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_IMAX)
        SERIAL_ECHOLNPGM("  " STR_I_MAX ": ", m.obj_side[IMAXIMUM]);
      #endif
    #endif
    #if HAS_J_AXIS
      #if ENABLED(CALIBRATION_MEASURE_JMIN)
        SERIAL_ECHOLNPGM("  " STR_J_MIN ": ", m.obj_side[JMINIMUM]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_JMAX)
        SERIAL_ECHOLNPGM("  " STR_J_MAX ": ", m.obj_side[JMAXIMUM]);
      #endif
    #endif
    #if HAS_K_AXIS
      #if ENABLED(CALIBRATION_MEASURE_KMIN)
        SERIAL_ECHOLNPGM("  " STR_K_MIN ": ", m.obj_side[KMINIMUM]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_KMAX)
        SERIAL_ECHOLNPGM("  " STR_K_MAX ": ", m.obj_side[KMAXIMUM]);
      #endif
    #endif
    #if HAS_U_AXIS
      #if ENABLED(CALIBRATION_MEASURE_UMIN)
        SERIAL_ECHOLNPGM("  " STR_U_MIN ": ", m.obj_side[UMINIMUM]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_UMAX)
        SERIAL_ECHOLNPGM("  " STR_U_MAX ": ", m.obj_side[UMAXIMUM]);
      #endif
    #endif
    #if HAS_V_AXIS
      #if ENABLED(CALIBRATION_MEASURE_VMIN)
        SERIAL_ECHOLNPGM("  " STR_V_MIN ": ", m.obj_side[VMINIMUM]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_VMAX)
        SERIAL_ECHOLNPGM("  " STR_V_MAX ": ", m.obj_side[VMAXIMUM]);
      #endif
    #endif
    #if HAS_W_AXIS
      #if ENABLED(CALIBRATION_MEASURE_WMIN)
        SERIAL_ECHOLNPGM("  " STR_W_MIN ": ", m.obj_side[WMINIMUM]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_WMAX)
        SERIAL_ECHOLNPGM("  " STR_W_MAX ": ", m.obj_side[WMAXIMUM]);
      #endif
    #endif
    SERIAL_EOL();
  }

  inline void report_measured_center(const measurements_t &m) {
    SERIAL_ECHOLNPGM("Center:");
    #if HAS_X_CENTER
      SERIAL_ECHOLNPGM_P(SP_X_STR, m.obj_center.x);
    #endif
    #if HAS_Y_CENTER
      SERIAL_ECHOLNPGM_P(SP_Y_STR, m.obj_center.y);
    #endif
    SERIAL_ECHOLNPGM_P(SP_Z_STR, m.obj_center.z);
    #if HAS_I_CENTER
      SERIAL_ECHOLNPGM_P(SP_I_STR, m.obj_center.i);
    #endif
    #if HAS_J_CENTER
      SERIAL_ECHOLNPGM_P(SP_J_STR, m.obj_center.j);
    #endif
    #if HAS_K_CENTER
      SERIAL_ECHOLNPGM_P(SP_K_STR, m.obj_center.k);
    #endif
    #if HAS_U_CENTER
      SERIAL_ECHOLNPGM_P(SP_U_STR, m.obj_center.u);
    #endif
    #if HAS_V_CENTER
      SERIAL_ECHOLNPGM_P(SP_V_STR, m.obj_center.v);
    #endif
    #if HAS_W_CENTER
      SERIAL_ECHOLNPGM_P(SP_W_STR, m.obj_center.w);
    #endif
    SERIAL_EOL();
  }

  inline void report_measured_backlash(const measurements_t &m) {
    SERIAL_ECHOLNPGM("Backlash:");
    #if AXIS_CAN_CALIBRATE(X)
      #if ENABLED(CALIBRATION_MEASURE_LEFT)
        SERIAL_ECHOLNPGM("  Left: ", m.backlash[LEFT]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_RIGHT)
        SERIAL_ECHOLNPGM("  Right: ", m.backlash[RIGHT]);
      #endif
    #endif
    #if AXIS_CAN_CALIBRATE(Y)
      #if ENABLED(CALIBRATION_MEASURE_FRONT)
        SERIAL_ECHOLNPGM("  Front: ", m.backlash[FRONT]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_BACK)
        SERIAL_ECHOLNPGM("  Back: ", m.backlash[BACK]);
      #endif
    #endif
    #if AXIS_CAN_CALIBRATE(Z)
      SERIAL_ECHOLNPGM("  Top: ", m.backlash[TOP]);
    #endif
    #if AXIS_CAN_CALIBRATE(I)
      #if ENABLED(CALIBRATION_MEASURE_IMIN)
        SERIAL_ECHOLNPGM("  " STR_I_MIN ": ", m.backlash[IMINIMUM]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_IMAX)
        SERIAL_ECHOLNPGM("  " STR_I_MAX ": ", m.backlash[IMAXIMUM]);
      #endif
    #endif
    #if AXIS_CAN_CALIBRATE(J)
      #if ENABLED(CALIBRATION_MEASURE_JMIN)
        SERIAL_ECHOLNPGM("  " STR_J_MIN ": ", m.backlash[JMINIMUM]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_JMAX)
        SERIAL_ECHOLNPGM("  " STR_J_MAX ": ", m.backlash[JMAXIMUM]);
      #endif
    #endif
    #if AXIS_CAN_CALIBRATE(K)
      #if ENABLED(CALIBRATION_MEASURE_KMIN)
        SERIAL_ECHOLNPGM("  " STR_K_MIN ": ", m.backlash[KMINIMUM]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_KMAX)
        SERIAL_ECHOLNPGM("  " STR_K_MAX ": ", m.backlash[KMAXIMUM]);
      #endif
    #endif
    #if AXIS_CAN_CALIBRATE(U)
      #if ENABLED(CALIBRATION_MEASURE_UMIN)
        SERIAL_ECHOLNPGM("  " STR_U_MIN ": ", m.backlash[UMINIMUM]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_UMAX)
        SERIAL_ECHOLNPGM("  " STR_U_MAX ": ", m.backlash[UMAXIMUM]);
      #endif
    #endif
    #if AXIS_CAN_CALIBRATE(V)
      #if ENABLED(CALIBRATION_MEASURE_VMIN)
        SERIAL_ECHOLNPGM("  " STR_V_MIN ": ", m.backlash[VMINIMUM]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_VMAX)
        SERIAL_ECHOLNPGM("  " STR_V_MAX ": ", m.backlash[VMAXIMUM]);
      #endif
    #endif
    #if AXIS_CAN_CALIBRATE(W)
      #if ENABLED(CALIBRATION_MEASURE_WMIN)
        SERIAL_ECHOLNPGM("  " STR_W_MIN ": ", m.backlash[WMINIMUM]);
      #endif
      #if ENABLED(CALIBRATION_MEASURE_WMAX)
        SERIAL_ECHOLNPGM("  " STR_W_MAX ": ", m.backlash[WMAXIMUM]);
      #endif
    #endif
    SERIAL_EOL();
  }

  inline void report_measured_positional_error(const measurements_t &m) {
    SERIAL_CHAR('T');
    SERIAL_ECHO(active_extruder);
    SERIAL_ECHOLNPGM(" Positional Error:");
    #if HAS_X_CENTER && AXIS_CAN_CALIBRATE(X)
      SERIAL_ECHOLNPGM_P(SP_X_STR, m.pos_error.x);
    #endif
    #if HAS_Y_CENTER && AXIS_CAN_CALIBRATE(Y)
      SERIAL_ECHOLNPGM_P(SP_Y_STR, m.pos_error.y);
    #endif
    #if AXIS_CAN_CALIBRATE(Z)
      SERIAL_ECHOLNPGM_P(SP_Z_STR, m.pos_error.z);
    #endif
    #if HAS_I_CENTER && AXIS_CAN_CALIBRATE(I)
      SERIAL_ECHOLNPGM_P(SP_I_STR, m.pos_error.i);
    #endif
    #if HAS_J_CENTER && AXIS_CAN_CALIBRATE(J)
      SERIAL_ECHOLNPGM_P(SP_J_STR, m.pos_error.j);
    #endif
    #if HAS_K_CENTER && AXIS_CAN_CALIBRATE(K)
      SERIAL_ECHOLNPGM_P(SP_K_STR, m.pos_error.k);
    #endif
    #if HAS_U_CENTER && AXIS_CAN_CALIBRATE(U)
      SERIAL_ECHOLNPGM_P(SP_U_STR, m.pos_error.u);
    #endif
    #if HAS_V_CENTER && AXIS_CAN_CALIBRATE(V)
      SERIAL_ECHOLNPGM_P(SP_V_STR, m.pos_error.v);
    #endif
    #if HAS_W_CENTER && AXIS_CAN_CALIBRATE(W)
      SERIAL_ECHOLNPGM_P(SP_W_STR, m.pos_error.w);
    #endif
    SERIAL_EOL();
  }

  inline void report_measured_nozzle_dimensions(const measurements_t &m) {
    SERIAL_ECHOLNPGM("Nozzle Tip Outer Dimensions:");
    #if HAS_X_CENTER
      SERIAL_ECHOLNPGM_P(SP_X_STR, m.nozzle_outer_dimension.x);
    #endif
    #if HAS_Y_CENTER
      SERIAL_ECHOLNPGM_P(SP_Y_STR, m.nozzle_outer_dimension.y);
    #endif
    SERIAL_EOL();
    UNUSED(m);
  }

  #if HAS_HOTEND_OFFSET
    //
    // This function requires normalize_hotend_offsets() to be called
    //
    inline void report_hotend_offsets() {
      for (uint8_t e = 1; e < HOTENDS; ++e)
        SERIAL_ECHOLNPGM_P(PSTR("T"), e, PSTR(" Hotend Offset X"), hotend_offset[e].x, SP_Y_STR, hotend_offset[e].y, SP_Z_STR, hotend_offset[e].z);
    }
  #endif

#endif // CALIBRATION_REPORTING

/**
 * Probe around the calibration object to measure backlash
 *
 *   m              in/out - Measurement record, updated with new readings
 *   uncertainty    in     - How far away from the object to begin probing
 */
inline void calibrate_backlash(measurements_t &m, const float uncertainty) {
  // Backlash compensation should be off while measuring backlash

  {
    // New scope for TEMPORARY_BACKLASH_CORRECTION
    TEMPORARY_BACKLASH_CORRECTION(backlash.all_off);
    TEMPORARY_BACKLASH_SMOOTHING(0.0f);

    probe_sides(m, uncertainty);

    #if ENABLED(BACKLASH_GCODE)

      #if HAS_X_CENTER
        backlash.set_distance_mm(X_AXIS, (m.backlash[LEFT] + m.backlash[RIGHT]) / 2);
      #elif ENABLED(CALIBRATION_MEASURE_LEFT)
        backlash.set_distance_mm(X_AXIS, m.backlash[LEFT]);
      #elif ENABLED(CALIBRATION_MEASURE_RIGHT)
        backlash.set_distance_mm(X_AXIS, m.backlash[RIGHT]);
      #endif

      #if HAS_Y_CENTER
        backlash.set_distance_mm(Y_AXIS, (m.backlash[FRONT] + m.backlash[BACK]) / 2);
      #elif ENABLED(CALIBRATION_MEASURE_FRONT)
        backlash.set_distance_mm(Y_AXIS, m.backlash[FRONT]);
      #elif ENABLED(CALIBRATION_MEASURE_BACK)
        backlash.set_distance_mm(Y_AXIS, m.backlash[BACK]);
      #endif

      TERN_(HAS_Z_AXIS, if (AXIS_CAN_CALIBRATE(Z)) backlash.set_distance_mm(Z_AXIS, m.backlash[TOP]));

      #if HAS_I_CENTER
        backlash.set_distance_mm(I_AXIS, (m.backlash[IMINIMUM] + m.backlash[IMAXIMUM]) / 2);
      #elif ENABLED(CALIBRATION_MEASURE_IMIN)
        backlash.set_distance_mm(I_AXIS, m.backlash[IMINIMUM]);
      #elif ENABLED(CALIBRATION_MEASURE_IMAX)
        backlash.set_distance_mm(I_AXIS, m.backlash[IMAXIMUM]);
      #endif

      #if HAS_J_CENTER
        backlash.set_distance_mm(J_AXIS, (m.backlash[JMINIMUM] + m.backlash[JMAXIMUM]) / 2);
      #elif ENABLED(CALIBRATION_MEASURE_JMIN)
        backlash.set_distance_mm(J_AXIS, m.backlash[JMINIMUM]);
      #elif ENABLED(CALIBRATION_MEASURE_JMAX)
        backlash.set_distance_mm(J_AXIS, m.backlash[JMAXIMUM]);
      #endif

      #if HAS_K_CENTER
        backlash.set_distance_mm(K_AXIS, (m.backlash[KMINIMUM] + m.backlash[KMAXIMUM]) / 2);
      #elif ENABLED(CALIBRATION_MEASURE_KMIN)
        backlash.set_distance_mm(K_AXIS, m.backlash[KMINIMUM]);
      #elif ENABLED(CALIBRATION_MEASURE_KMAX)
        backlash.set_distance_mm(K_AXIS, m.backlash[KMAXIMUM]);
      #endif

      #if HAS_U_CENTER
        backlash.distance_mm.u = (m.backlash[UMINIMUM] + m.backlash[UMAXIMUM]) / 2;
      #elif ENABLED(CALIBRATION_MEASURE_UMIN)
        backlash.distance_mm.u = m.backlash[UMINIMUM];
      #elif ENABLED(CALIBRATION_MEASURE_UMAX)
        backlash.distance_mm.u = m.backlash[UMAXIMUM];
      #endif

      #if HAS_V_CENTER
        backlash.distance_mm.v = (m.backlash[VMINIMUM] + m.backlash[VMAXIMUM]) / 2;
      #elif ENABLED(CALIBRATION_MEASURE_VMIN)
        backlash.distance_mm.v = m.backlash[VMINIMUM];
      #elif ENABLED(CALIBRATION_MEASURE_UMAX)
        backlash.distance_mm.v = m.backlash[VMAXIMUM];
      #endif

      #if HAS_W_CENTER
        backlash.distance_mm.w = (m.backlash[WMINIMUM] + m.backlash[WMAXIMUM]) / 2;
      #elif ENABLED(CALIBRATION_MEASURE_WMIN)
        backlash.distance_mm.w = m.backlash[WMINIMUM];
      #elif ENABLED(CALIBRATION_MEASURE_WMAX)
        backlash.distance_mm.w = m.backlash[WMAXIMUM];
      #endif

    #endif // BACKLASH_GCODE
  }

  #if ENABLED(BACKLASH_GCODE)
    // Turn on backlash compensation and move in all
    // allowed directions to take up any backlash
    {
      // New scope for TEMPORARY_BACKLASH_CORRECTION
      TEMPORARY_BACKLASH_CORRECTION(backlash.all_on);
      TEMPORARY_BACKLASH_SMOOTHING(0.0f);
      const xyz_float_t move = NUM_AXIS_ARRAY(
        AXIS_CAN_CALIBRATE(X) * 3, AXIS_CAN_CALIBRATE(Y) * 3, AXIS_CAN_CALIBRATE(Z) * 3,
        AXIS_CAN_CALIBRATE(I) * 3, AXIS_CAN_CALIBRATE(J) * 3, AXIS_CAN_CALIBRATE(K) * 3,
        AXIS_CAN_CALIBRATE(U) * 3, AXIS_CAN_CALIBRATE(V) * 3, AXIS_CAN_CALIBRATE(W) * 3
      );
      current_position += move; calibration_move();
      current_position -= move; calibration_move();
    }
  #endif
}

inline void update_measurements(measurements_t &m, const AxisEnum axis) {
  current_position[axis] += m.pos_error[axis];
  m.obj_center[axis] = true_center[axis];
  m.pos_error[axis] = 0;
}

/**
 * Probe around the calibration object. Adjust the position and toolhead offset
 * using the deviation from the known position of the calibration object.
 *
 *   m              in/out - Measurement record, updated with new readings
 *   uncertainty    in     - How far away from the object to begin probing
 *   extruder       in     - What extruder to probe
 *
 * Prerequisites:
 *    - Call calibrate_backlash() beforehand for best accuracy
 */
inline void calibrate_toolhead(measurements_t &m, const float uncertainty, const uint8_t extruder) {
  TEMPORARY_BACKLASH_CORRECTION(backlash.all_on);
  TEMPORARY_BACKLASH_SMOOTHING(0.0f);

  TERN(HAS_MULTI_HOTEND, set_nozzle(m, extruder), UNUSED(extruder));

  probe_sides(m, uncertainty);

  // Adjust the hotend offset
  #if HAS_HOTEND_OFFSET
    if (ENABLED(HAS_X_CENTER) && AXIS_CAN_CALIBRATE(X)) hotend_offset[extruder].x += m.pos_error.x;
    if (ENABLED(HAS_Y_CENTER) && AXIS_CAN_CALIBRATE(Y)) hotend_offset[extruder].y += m.pos_error.y;
                             if (AXIS_CAN_CALIBRATE(Z)) hotend_offset[extruder].z += m.pos_error.z;
    normalize_hotend_offsets();
  #endif

  // Correct for positional error, so the object
  // is at the known actual spot
  planner.synchronize();
  if (ENABLED(HAS_X_CENTER) && AXIS_CAN_CALIBRATE(X)) update_measurements(m, X_AXIS);
  if (ENABLED(HAS_Y_CENTER) && AXIS_CAN_CALIBRATE(Y)) update_measurements(m, Y_AXIS);
                           if (AXIS_CAN_CALIBRATE(Z)) update_measurements(m, Z_AXIS);

  TERN_(HAS_I_CENTER, update_measurements(m, I_AXIS));
  TERN_(HAS_J_CENTER, update_measurements(m, J_AXIS));
  TERN_(HAS_K_CENTER, update_measurements(m, K_AXIS));
  TERN_(HAS_U_CENTER, update_measurements(m, U_AXIS));
  TERN_(HAS_V_CENTER, update_measurements(m, V_AXIS));
  TERN_(HAS_W_CENTER, update_measurements(m, W_AXIS));

  sync_plan_position();
}

/**
 * Probe around the calibration object for all toolheads, adjusting the coordinate
 * system for the first nozzle and the nozzle offset for subsequent nozzles.
 *
 *   m              in/out - Measurement record, updated with new readings
 *   uncertainty    in     - How far away from the object to begin probing
 */
inline void calibrate_all_toolheads(measurements_t &m, const float uncertainty) {
  TEMPORARY_BACKLASH_CORRECTION(backlash.all_on);
  TEMPORARY_BACKLASH_SMOOTHING(0.0f);

  HOTEND_LOOP() calibrate_toolhead(m, uncertainty, e);

  TERN_(HAS_HOTEND_OFFSET, normalize_hotend_offsets());

  TERN_(HAS_MULTI_HOTEND, set_nozzle(m, 0));
}

/**
 * Perform a full auto-calibration routine:
 *
 *   1) For each nozzle, touch top and sides of object to determine object position and
 *      nozzle offsets. Do a fast but rough search over a wider area.
 *   2) With the first nozzle, touch top and sides of object to determine backlash values
 *      for all axes (if BACKLASH_GCODE is enabled)
 *   3) For each nozzle, touch top and sides of object slowly to determine precise
 *      position of object. Adjust coordinate system and nozzle offsets so probed object
 *      location corresponds to known object location with a high degree of precision.
 */
inline void calibrate_all() {
  measurements_t m;

  TERN_(HAS_HOTEND_OFFSET, reset_hotend_offsets());

  TEMPORARY_BACKLASH_CORRECTION(backlash.all_on);
  TEMPORARY_BACKLASH_SMOOTHING(0.0f);

  // Do a fast and rough calibration of the toolheads
  calibrate_all_toolheads(m, CALIBRATION_MEASUREMENT_UNKNOWN);

  TERN_(BACKLASH_GCODE, calibrate_backlash(m, CALIBRATION_MEASUREMENT_UNCERTAIN));

  // Cycle the toolheads so the servos settle into their "natural" positions
  #if HAS_MULTI_HOTEND
    HOTEND_LOOP() set_nozzle(m, e);
  #endif

  // Do a slow and precise calibration of the toolheads
  calibrate_all_toolheads(m, CALIBRATION_MEASUREMENT_UNCERTAIN);

  current_position.x = X_CENTER;
  calibration_move();         // Park nozzle away from calibration object
}

/**
 * G425: Perform calibration with calibration object.
 *
 *   B           - Perform calibration of backlash only.
 *   T<extruder> - Perform calibration of toolhead only.
 *   V           - Probe object and print position, error, backlash and hotend offset.
 *   U           - Uncertainty, how far to start probe away from the object (mm)
 *
 *   no args     - Perform entire calibration sequence (backlash + position on all toolheads)
 */
void GcodeSuite::G425() {

  #ifdef CALIBRATION_SCRIPT_PRE
    process_subcommands_now(F(CALIBRATION_SCRIPT_PRE));
  #endif

  if (homing_needed_error()) return;

  TEMPORARY_BED_LEVELING_STATE(false);
  SET_SOFT_ENDSTOP_LOOSE(true);

  measurements_t m;
  const float uncertainty = parser.floatval('U', CALIBRATION_MEASUREMENT_UNCERTAIN);

  if (parser.seen_test('B'))
    calibrate_backlash(m, uncertainty);
  else if (parser.seen_test('T'))
    calibrate_toolhead(m, uncertainty, parser.intval('T', active_extruder));
  #if ENABLED(CALIBRATION_REPORTING)
    else if (parser.seen('V')) {
      probe_sides(m, uncertainty);
      SERIAL_EOL();
      report_measured_faces(m);
      report_measured_center(m);
      report_measured_backlash(m);
      report_measured_nozzle_dimensions(m);
      report_measured_positional_error(m);
      #if HAS_HOTEND_OFFSET
        normalize_hotend_offsets();
        report_hotend_offsets();
      #endif
    }
  #endif
  else
    calibrate_all();

  SET_SOFT_ENDSTOP_LOOSE(false);

  #ifdef CALIBRATION_SCRIPT_POST
    process_subcommands_now(F(CALIBRATION_SCRIPT_POST));
  #endif
}

#endif // CALIBRATION_GCODE
