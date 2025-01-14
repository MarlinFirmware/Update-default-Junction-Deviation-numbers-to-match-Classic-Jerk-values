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

//todo:  add support for multiple encoders on a single axis
//todo:    add z axis auto-leveling
//todo:  consolidate some of the related M codes?
//todo:  add endstop-replacement mode?
//todo:  try faster I2C speed; tweak TWI_FREQ (400000L, or faster?); or just TWBR = ((CPU_FREQ / 400000L) - 16) / 2;
//todo:    consider Marlin-optimized Wire library; i.e. MarlinWire, like MarlinSerial

#include "../inc/MarlinConfig.h"

#if ENABLED(I2C_POSITION_ENCODERS)

#include "encoder_i2c.h"

#include "../module/stepper.h"
#include "../gcode/parser.h"

#include "babystep.h"

#include <Wire.h>

I2CPositionEncodersMgr I2CPEM;

void I2CPositionEncoder::init(const uint8_t address, const AxisEnum axis) {
  encoderAxis = axis;
  i2cAddress = address;

  initialized = true;

  SERIAL_ECHOLNPGM("Setting up encoder on ", C(AXIS_CHAR(encoderAxis)), " axis, addr = ", address);

  position = get_position();
}

void I2CPositionEncoder::update() {
  if (!initialized || !homed || !active) return; //check encoder is set up and active

  position = get_position();

  //we don't want to stop things just because the encoder missed a message,
  //so we only care about responses that indicate bad magnetic strength

  if (!passes_test(false)) { //check encoder data is good
    lastErrorTime = millis();
    /*
    if (trusted) { //commented out as part of the note below
      trusted = false;
      SERIAL_ECHOLNPGM("Fault detected on ", C(AXIS_CHAR(encoderAxis)), " axis encoder. Disengaging error correction until module is trusted again.");
    }
    */
    return;
  }

  if (!trusted) {
    /**
     * This is commented out because it introduces error and can cause bad print quality.
     *
     * This code is intended to manage situations where the encoder has reported bad magnetic strength.
     * This indicates that the magnetic strip was too far away from the sensor to reliably track position.
     * When this happens, this code resets the offset based on where the printer thinks it is. This has been
     * shown to introduce errors in actual position which result in drifting prints and poor print quality.
     * Perhaps a better method would be to disable correction on the axis with a problem, report it to the
     * user via the status leds on the encoder module and prompt the user to re-home the axis at which point
     * the encoder would be re-enabled.
     */

    #if 0
      // If the magnetic strength has been good for a certain time, start trusting the module again

      if (millis() - lastErrorTime > I2CPE_TIME_TRUSTED) {
        trusted = true;

        SERIAL_ECHOLNPGM("Untrusted encoder module on ", C(AXIS_CHAR(encoderAxis)), " axis has been fault-free for set duration, reinstating error correction.");

        //the encoder likely lost its place when the error occurred, so we'll reset and use the printer's
        //idea of where it the axis is to re-initialize
        const float pos = planner.get_axis_position_mm(encoderAxis);
        int32_t positionInTicks = pos * get_ticks_unit();

        //shift position from previous to current position
        zeroOffset -= (positionInTicks - get_position());

        #ifdef I2CPE_DEBUG
          SERIAL_ECHOLNPGM("Current position is ", pos);
          SERIAL_ECHOLNPGM("Position in encoder ticks is ", positionInTicks);
          SERIAL_ECHOLNPGM("New zero-offset of ", zeroOffset);
          SERIAL_ECHOLN(F("New position reads as "), get_position(), C('('), mm_from_count(get_position()), C(')'));
        #endif
      }
    #endif
    return;
  }

  lastPosition = position;
  const millis_t positionTime = millis();

  //only do error correction if setup and enabled
  if (ec && ecMethod != I2CPE_ECM_NONE) {

    #ifdef I2CPE_EC_THRESH_PROPORTIONAL
      const millis_t deltaTime = positionTime - lastPositionTime;
      const uint32_t distance = ABS(position - lastPosition),
                     speed = distance / deltaTime;
      const float threshold = constrain((speed / 50), 1, 50) * ecThreshold;
    #else
      const float threshold = get_error_correct_threshold();
    #endif

    //check error
    #if ENABLED(I2CPE_ERR_ROLLING_AVERAGE)
      float sum = 0, diffSum = 0;

      errIdx = (errIdx >= I2CPE_ERR_ARRAY_SIZE - 1) ? 0 : errIdx + 1;
      err[errIdx] = get_axis_error_steps(false);

      for (uint8_t i = 0; i < I2CPE_ERR_ARRAY_SIZE; ++i) {
        sum += err[i];
        if (i) diffSum += ABS(err[i-1] - err[i]);
      }

      const int32_t error = int32_t(sum / (I2CPE_ERR_ARRAY_SIZE + 1)); //calculate average for error

    #else
      const int32_t error = get_axis_error_steps(false);
    #endif

    //SERIAL_ECHOLNPGM("Axis error steps: ", error);

    #ifdef I2CPE_ERR_THRESH_ABORT
      if (ABS(error) > I2CPE_ERR_THRESH_ABORT * planner.settings.axis_steps_per_mm[encoderAxis]) {
        //kill(F("Significant Error"));
        SERIAL_ECHOLNPGM("Axis error over threshold, aborting!", error);
        safe_delay(5000);
      }
    #endif

    #if ENABLED(I2CPE_ERR_ROLLING_AVERAGE)
      if (errIdx == 0) {
        // In order to correct for "error" but avoid correcting for noise and non-skips
        // it must be > threshold and have a difference average of < 10 and be < 2000 steps
        if (ABS(error) > threshold * planner.settings.axis_steps_per_mm[encoderAxis]
            && diffSum < 10 * (I2CPE_ERR_ARRAY_SIZE - 1)
            && ABS(error) < 2000
        ) {                              // Check for persistent error (skip)
          errPrst[errPrstIdx++] = error; // Error must persist for I2CPE_ERR_PRST_ARRAY_SIZE error cycles. This also serves to improve the average accuracy
          if (errPrstIdx >= I2CPE_ERR_PRST_ARRAY_SIZE) {
            float sumP = 0;
            for (uint8_t i = 0; i < I2CPE_ERR_PRST_ARRAY_SIZE; ++i) sumP += errPrst[i];
            const int32_t errorP = int32_t(sumP * RECIPROCAL(I2CPE_ERR_PRST_ARRAY_SIZE));
            SERIAL_CHAR(AXIS_CHAR(encoderAxis));
            SERIAL_ECHOLNPGM(" : CORRECT ERR ", errorP * planner.mm_per_step[encoderAxis], "mm");
            babystep.add_steps(encoderAxis, -LROUND(errorP));
            errPrstIdx = 0;
          }
        }
        else
          errPrstIdx = 0;
      }
    #else
      if (ABS(error) > threshold * planner.settings.axis_steps_per_mm[encoderAxis]) {
        //SERIAL_ECHOLN(error);
        //SERIAL_ECHOLN(position);
        babystep.add_steps(encoderAxis, -LROUND(error / 2));
      }
    #endif

    if (ABS(error) > I2CPE_ERR_CNT_THRESH * planner.settings.axis_steps_per_mm[encoderAxis]) {
      const millis_t ms = millis();
      if (ELAPSED(ms, nextErrorCountTime)) {
        SERIAL_CHAR(AXIS_CHAR(encoderAxis));
        SERIAL_ECHOLNPGM(" : LARGE ERR ", error, "; diffSum=", diffSum);
        errorCount++;
        nextErrorCountTime = ms + I2CPE_ERR_CNT_DEBOUNCE_MS;
      }
    }
  }

  lastPositionTime = positionTime;
}

void I2CPositionEncoder::set_homed() {
  if (active) {
    reset();  // Reset module's offset to zero (so current position is homed / zero)
    delay(10);

    zeroOffset = get_raw_count();
    homed = trusted = true;

    #ifdef I2CPE_DEBUG
      SERIAL_CHAR(AXIS_CHAR(encoderAxis));
      SERIAL_ECHOLNPGM(" axis encoder homed, offset of ", zeroOffset, " ticks.");
    #endif
  }
}

void I2CPositionEncoder::set_unhomed() {
  zeroOffset = 0;
  homed = trusted = false;

  #ifdef I2CPE_DEBUG
    SERIAL_CHAR(AXIS_CHAR(encoderAxis));
    SERIAL_ECHOLNPGM(" axis encoder unhomed.");
  #endif
}

bool I2CPositionEncoder::passes_test(const bool report) {
  if (report) {
    if (H != I2CPE_MAG_SIG_GOOD) SERIAL_ECHOPGM("Warning. ");
    SERIAL_CHAR(AXIS_CHAR(encoderAxis));
    serial_ternary(F(" axis "), H == I2CPE_MAG_SIG_BAD, F("magnetic strip "), F("encoder "));
    switch (H) {
      case I2CPE_MAG_SIG_GOOD:
      case I2CPE_MAG_SIG_MID:
        SERIAL_ECHO_TERNARY(H == I2CPE_MAG_SIG_GOOD, "passes test; field strength ", "good", "fair", ".\n");
        break;
      default:
        SERIAL_ECHOLNPGM("not detected!");
    }
  }
  return (H == I2CPE_MAG_SIG_GOOD || H == I2CPE_MAG_SIG_MID);
}

float I2CPositionEncoder::get_axis_error_mm(const bool report) {
  const float target = planner.get_axis_position_mm(encoderAxis),
              actual = mm_from_count(position),
              diff = actual - target,
              error = ABS(diff) > 10000 ? 0 : diff; // Huge error is a bad reading

  if (report) {
    SERIAL_CHAR(AXIS_CHAR(encoderAxis));
    SERIAL_ECHOLNPGM(" axis target=", target, "mm; actual=", actual, "mm; err=", error, "mm");
  }

  return error;
}

int32_t I2CPositionEncoder::get_axis_error_steps(const bool report) {
  if (!active) {
    if (report) {
      SERIAL_CHAR(AXIS_CHAR(encoderAxis));
      SERIAL_ECHOLNPGM(" axis encoder not active!");
    }
    return 0;
  }

  float stepperTicksPerUnit;
  int32_t encoderTicks = position, encoderCountInStepperTicksScaled;
  //int32_t stepperTicks = stepper.position(encoderAxis);

  // With a rotary encoder we're concerned with ticks/rev; whereas with a linear we're concerned with ticks/mm
  stepperTicksPerUnit = (type == I2CPE_ENC_TYPE_ROTARY) ? stepperTicks : planner.settings.axis_steps_per_mm[encoderAxis];

  //convert both 'ticks' into same units / base
  encoderCountInStepperTicksScaled = LROUND((stepperTicksPerUnit * encoderTicks) / encoderTicksPerUnit);

  const int32_t target = stepper.position(encoderAxis);
  int32_t error = encoderCountInStepperTicksScaled - target;

  //suppress discontinuities (might be caused by bad I2C readings...?)
  const bool suppressOutput = (ABS(error - errorPrev) > 100);

  errorPrev = error;

  if (report) {
    SERIAL_CHAR(AXIS_CHAR(encoderAxis));
    SERIAL_ECHOLNPGM(" axis target=", target, "; actual=", encoderCountInStepperTicksScaled, "; err=", error);
  }

  if (suppressOutput) {
    if (report) SERIAL_ECHOLNPGM("!Discontinuity. Suppressing error.");
    error = 0;
  }

  return error;
}

int32_t I2CPositionEncoder::get_raw_count() {
  uint8_t index = 0;
  i2cLong encoderCount;

  encoderCount.val = 0x00;

  if (Wire.requestFrom(I2C_ADDRESS(i2cAddress), uint8_t(3)) != 3) {
    //houston, we have a problem...
    H = I2CPE_MAG_SIG_NF;
    return 0;
  }

  while (Wire.available())
    encoderCount.bval[index++] = (uint8_t)Wire.read();

  //extract the magnetic strength
  H = (B00000011 & (encoderCount.bval[2] >> 6));

  //extract sign bit; sign = (encoderCount.bval[2] & B00100000);
  //set all upper bits to the sign value to overwrite H
  encoderCount.val = (encoderCount.bval[2] & B00100000) ? (encoderCount.val | 0xFFC00000) : (encoderCount.val & 0x003FFFFF);

  if (invert) encoderCount.val *= -1;

  return encoderCount.val;
}

bool I2CPositionEncoder::test_axis() {
  // Only works on XYZ Cartesian machines for the time being
  if (!(encoderAxis == X_AXIS || encoderAxis == Y_AXIS || encoderAxis == Z_AXIS)) return false;

  const float startPosition = soft_endstop.min[encoderAxis] + 10,
              endPosition = soft_endstop.max[encoderAxis] - 10;
  const feedRate_t fr_mm_s = FLOOR(homing_feedrate(encoderAxis));

  ec = false;

  xyze_pos_t startCoord, endCoord;
  LOOP_NUM_AXES(a) {
    startCoord[a] = planner.get_axis_position_mm((AxisEnum)a);
    endCoord[a] = planner.get_axis_position_mm((AxisEnum)a);
  }
  startCoord[encoderAxis] = startPosition;
  endCoord[encoderAxis] = endPosition;

  planner.synchronize();

  #if HAS_EXTRUDERS
    startCoord.e = planner.get_axis_position_mm(E_AXIS);
    planner.buffer_line(startCoord, fr_mm_s, 0);
    planner.synchronize();
  #endif

  // if the module isn't currently trusted, wait until it is (or until it should be if things are working)
  if (!trusted) {
    int32_t startWaitingTime = millis();
    while (!trusted && millis() - startWaitingTime < I2CPE_TIME_TRUSTED)
      safe_delay(500);
  }

  if (trusted) { // if trusted, commence test
    TERN_(HAS_EXTRUDERS, endCoord.e = planner.get_axis_position_mm(E_AXIS));
    planner.buffer_line(endCoord, fr_mm_s, 0);
    planner.synchronize();
  }

  return trusted;
}

void I2CPositionEncoder::calibrate_steps_mm(const uint8_t iter) {
  if (type != I2CPE_ENC_TYPE_LINEAR) {
    SERIAL_ECHOLNPGM("Steps/mm calibration requires linear encoder.");
    return;
  }

  if (!(encoderAxis == X_AXIS || encoderAxis == Y_AXIS || encoderAxis == Z_AXIS)) {
    SERIAL_ECHOLNPGM("Steps/mm calibration not supported for this axis.");
    return;
  }

  float old_steps_mm, new_steps_mm,
        startDistance, endDistance,
        travelDistance, travelledDistance, total = 0;

  int32_t startCount, stopCount;

  const feedRate_t fr_mm_s = homing_feedrate(encoderAxis);

  bool oldec = ec;
  ec = false;

  startDistance = 20;
  endDistance = soft_endstop.max[encoderAxis] - 20;
  travelDistance = endDistance - startDistance;

  xyze_pos_t startCoord, endCoord;
  LOOP_NUM_AXES(a) {
    startCoord[a] = planner.get_axis_position_mm((AxisEnum)a);
    endCoord[a] = planner.get_axis_position_mm((AxisEnum)a);
  }
  startCoord[encoderAxis] = startDistance;
  endCoord[encoderAxis] = endDistance;

  planner.synchronize();

  for (uint8_t i = 0; i < iter; ++i) {
    TERN_(HAS_EXTRUDERS, startCoord.e = planner.get_axis_position_mm(E_AXIS));
    planner.buffer_line(startCoord, fr_mm_s, 0);
    planner.synchronize();

    delay(250);
    startCount = get_position();

    //do_blocking_move_to(endCoord);

    TERN_(HAS_EXTRUDERS, endCoord.e = planner.get_axis_position_mm(E_AXIS));
    planner.buffer_line(endCoord, fr_mm_s, 0);
    planner.synchronize();

    //Read encoder distance
    delay(250);
    stopCount = get_position();

    travelledDistance = mm_from_count(ABS(stopCount - startCount));

    SERIAL_ECHOLNPGM("Attempted travel: ", travelDistance, "mm");
    SERIAL_ECHOLNPGM("   Actual travel: ", travelledDistance, "mm");

    // Calculate new axis steps per unit
    old_steps_mm = planner.settings.axis_steps_per_mm[encoderAxis];
    new_steps_mm = (old_steps_mm * travelDistance) / travelledDistance;

    SERIAL_ECHOLNPGM("Old steps/mm: ", old_steps_mm);
    SERIAL_ECHOLNPGM("New steps/mm: ", new_steps_mm);

    // Save new value
    planner.settings.axis_steps_per_mm[encoderAxis] = new_steps_mm;

    if (iter > 1) {
      total += new_steps_mm;

      // Swap start and end points so next loop runs from current position
      const float tempCoord = startCoord[encoderAxis];
      startCoord[encoderAxis] = endCoord[encoderAxis];
      endCoord[encoderAxis] = tempCoord;
    }
  }

  if (iter > 1) {
    total /= (float)iter;
    SERIAL_ECHOLNPGM("Average steps/mm: ", total);
  }

  ec = oldec;

  SERIAL_ECHOLNPGM("Calculated steps/mm set. Use M500 to save to EEPROM.");
}

void I2CPositionEncoder::reset() {
  Wire.beginTransmission(I2C_ADDRESS(i2cAddress));
  Wire.write(I2CPE_RESET_COUNT);
  Wire.endTransmission();

  TERN_(I2CPE_ERR_ROLLING_AVERAGE, ZERO(err));
}

bool I2CPositionEncodersMgr::I2CPE_anyaxis;
uint8_t I2CPositionEncodersMgr::I2CPE_addr,
        I2CPositionEncodersMgr::I2CPE_idx;
I2CPositionEncoder I2CPositionEncodersMgr::encoders[I2CPE_ENCODER_CNT];

void I2CPositionEncodersMgr::init() {
  Wire.begin();

  #if I2CPE_ENCODER_CNT > 0
    uint8_t i = 0;

    encoders[i].init(I2CPE_ENC_1_ADDR, I2CPE_ENC_1_AXIS);

    #ifdef I2CPE_ENC_1_TYPE
      encoders[i].set_type(I2CPE_ENC_1_TYPE);
    #endif
    #ifdef I2CPE_ENC_1_TICKS_UNIT
      encoders[i].set_ticks_unit(I2CPE_ENC_1_TICKS_UNIT);
    #endif
    #ifdef I2CPE_ENC_1_TICKS_REV
      encoders[i].set_stepper_ticks(I2CPE_ENC_1_TICKS_REV);
    #endif
    #ifdef I2CPE_ENC_1_INVERT
      encoders[i].set_inverted(ENABLED(I2CPE_ENC_1_INVERT));
    #endif
    #ifdef I2CPE_ENC_1_EC_METHOD
      encoders[i].set_ec_method(I2CPE_ENC_1_EC_METHOD);
    #endif
    #ifdef I2CPE_ENC_1_EC_THRESH
      encoders[i].set_ec_threshold(I2CPE_ENC_1_EC_THRESH);
    #endif

    encoders[i].set_active(encoders[i].passes_test(true));

    TERN_(HAS_EXTRUDERS, if (I2CPE_ENC_1_AXIS == E_AXIS) encoders[i].set_homed());
  #endif

  #if I2CPE_ENCODER_CNT > 1
    i++;

    encoders[i].init(I2CPE_ENC_2_ADDR, I2CPE_ENC_2_AXIS);

    #ifdef I2CPE_ENC_2_TYPE
      encoders[i].set_type(I2CPE_ENC_2_TYPE);
    #endif
    #ifdef I2CPE_ENC_2_TICKS_UNIT
      encoders[i].set_ticks_unit(I2CPE_ENC_2_TICKS_UNIT);
    #endif
    #ifdef I2CPE_ENC_2_TICKS_REV
      encoders[i].set_stepper_ticks(I2CPE_ENC_2_TICKS_REV);
    #endif
    #ifdef I2CPE_ENC_2_INVERT
      encoders[i].set_inverted(ENABLED(I2CPE_ENC_2_INVERT));
    #endif
    #ifdef I2CPE_ENC_2_EC_METHOD
      encoders[i].set_ec_method(I2CPE_ENC_2_EC_METHOD);
    #endif
    #ifdef I2CPE_ENC_2_EC_THRESH
      encoders[i].set_ec_threshold(I2CPE_ENC_2_EC_THRESH);
    #endif

    encoders[i].set_active(encoders[i].passes_test(true));

    TERN_(HAS_EXTRUDERS, if (I2CPE_ENC_2_AXIS == E_AXIS) encoders[i].set_homed());
  #endif

  #if I2CPE_ENCODER_CNT > 2
    i++;

    encoders[i].init(I2CPE_ENC_3_ADDR, I2CPE_ENC_3_AXIS);

    #ifdef I2CPE_ENC_3_TYPE
      encoders[i].set_type(I2CPE_ENC_3_TYPE);
    #endif
    #ifdef I2CPE_ENC_3_TICKS_UNIT
      encoders[i].set_ticks_unit(I2CPE_ENC_3_TICKS_UNIT);
    #endif
    #ifdef I2CPE_ENC_3_TICKS_REV
      encoders[i].set_stepper_ticks(I2CPE_ENC_3_TICKS_REV);
    #endif
    #ifdef I2CPE_ENC_3_INVERT
      encoders[i].set_inverted(ENABLED(I2CPE_ENC_3_INVERT));
    #endif
    #ifdef I2CPE_ENC_3_EC_METHOD
      encoders[i].set_ec_method(I2CPE_ENC_3_EC_METHOD);
    #endif
    #ifdef I2CPE_ENC_3_EC_THRESH
      encoders[i].set_ec_threshold(I2CPE_ENC_3_EC_THRESH);
    #endif

    encoders[i].set_active(encoders[i].passes_test(true));

    TERN_(HAS_EXTRUDERS, if (I2CPE_ENC_3_AXIS == E_AXIS) encoders[i].set_homed());
  #endif

  #if I2CPE_ENCODER_CNT > 3
    i++;

    encoders[i].init(I2CPE_ENC_4_ADDR, I2CPE_ENC_4_AXIS);

    #ifdef I2CPE_ENC_4_TYPE
      encoders[i].set_type(I2CPE_ENC_4_TYPE);
    #endif
    #ifdef I2CPE_ENC_4_TICKS_UNIT
      encoders[i].set_ticks_unit(I2CPE_ENC_4_TICKS_UNIT);
    #endif
    #ifdef I2CPE_ENC_4_TICKS_REV
      encoders[i].set_stepper_ticks(I2CPE_ENC_4_TICKS_REV);
    #endif
    #ifdef I2CPE_ENC_4_INVERT
      encoders[i].set_inverted(ENABLED(I2CPE_ENC_4_INVERT));
    #endif
    #ifdef I2CPE_ENC_4_EC_METHOD
      encoders[i].set_ec_method(I2CPE_ENC_4_EC_METHOD);
    #endif
    #ifdef I2CPE_ENC_4_EC_THRESH
      encoders[i].set_ec_threshold(I2CPE_ENC_4_EC_THRESH);
    #endif

    encoders[i].set_active(encoders[i].passes_test(true));

    TERN_(HAS_EXTRUDERS, if (I2CPE_ENC_4_AXIS == E_AXIS) encoders[i].set_homed());
  #endif

  #if I2CPE_ENCODER_CNT > 4
    i++;

    encoders[i].init(I2CPE_ENC_5_ADDR, I2CPE_ENC_5_AXIS);

    #ifdef I2CPE_ENC_5_TYPE
      encoders[i].set_type(I2CPE_ENC_5_TYPE);
    #endif
    #ifdef I2CPE_ENC_5_TICKS_UNIT
      encoders[i].set_ticks_unit(I2CPE_ENC_5_TICKS_UNIT);
    #endif
    #ifdef I2CPE_ENC_5_TICKS_REV
      encoders[i].set_stepper_ticks(I2CPE_ENC_5_TICKS_REV);
    #endif
    #ifdef I2CPE_ENC_5_INVERT
      encoders[i].set_inverted(ENABLED(I2CPE_ENC_5_INVERT));
    #endif
    #ifdef I2CPE_ENC_5_EC_METHOD
      encoders[i].set_ec_method(I2CPE_ENC_5_EC_METHOD);
    #endif
    #ifdef I2CPE_ENC_5_EC_THRESH
      encoders[i].set_ec_threshold(I2CPE_ENC_5_EC_THRESH);
    #endif

    encoders[i].set_active(encoders[i].passes_test(true));

    TERN_(HAS_EXTRUDERS, if (I2CPE_ENC_5_AXIS == E_AXIS) encoders[i].set_homed());
  #endif

  #if I2CPE_ENCODER_CNT > 5
    i++;

    encoders[i].init(I2CPE_ENC_6_ADDR, I2CPE_ENC_6_AXIS);

    #ifdef I2CPE_ENC_6_TYPE
      encoders[i].set_type(I2CPE_ENC_6_TYPE);
    #endif
    #ifdef I2CPE_ENC_6_TICKS_UNIT
      encoders[i].set_ticks_unit(I2CPE_ENC_6_TICKS_UNIT);
    #endif
    #ifdef I2CPE_ENC_6_TICKS_REV
      encoders[i].set_stepper_ticks(I2CPE_ENC_6_TICKS_REV);
    #endif
    #ifdef I2CPE_ENC_6_INVERT
      encoders[i].set_inverted(ENABLED(I2CPE_ENC_6_INVERT));
    #endif
    #ifdef I2CPE_ENC_6_EC_METHOD
      encoders[i].set_ec_method(I2CPE_ENC_6_EC_METHOD);
    #endif
    #ifdef I2CPE_ENC_6_EC_THRESH
      encoders[i].set_ec_threshold(I2CPE_ENC_6_EC_THRESH);
    #endif

    encoders[i].set_active(encoders[i].passes_test(true));

    TERN_(HAS_EXTRUDERS, if (I2CPE_ENC_6_AXIS == E_AXIS) encoders[i].set_homed());
  #endif
}

void I2CPositionEncodersMgr::report_position(const int8_t idx, const bool units, const bool noOffset) {
  CHECK_IDX();

  if (units)
    SERIAL_ECHOLN(noOffset ? encoders[idx].mm_from_count(encoders[idx].get_raw_count()) : encoders[idx].get_position_mm());
  else {
    if (noOffset) {
      const int32_t raw_count = encoders[idx].get_raw_count();
      SERIAL_CHAR(AXIS_CHAR(encoders[idx).get_axis()], ' ');

      for (uint8_t j = 31; j > 0; j--)
        SERIAL_ECHO((bool)(0x00000001 & (raw_count >> j)));

      SERIAL_ECHO((bool)(0x00000001 & raw_count));
      SERIAL_CHAR(' ');
      SERIAL_ECHOLN(raw_count);
    }
    else
      SERIAL_ECHOLN(encoders[idx].get_position());
  }
}

void I2CPositionEncodersMgr::change_module_address(const uint8_t oldaddr, const uint8_t newaddr) {
  // First check 'new' address is not in use
  Wire.beginTransmission(I2C_ADDRESS(newaddr));
  if (!Wire.endTransmission()) {
    SERIAL_ECHOLNPGM("?There is already a device with that address on the I2C bus! (", newaddr, ")");
    return;
  }

  // Now check that we can find the module on the oldaddr address
  Wire.beginTransmission(I2C_ADDRESS(oldaddr));
  if (Wire.endTransmission()) {
    SERIAL_ECHOLNPGM("?No module detected at this address! (", oldaddr, ")");
    return;
  }

  SERIAL_ECHOLNPGM("Module found at ", oldaddr, ", changing address to ", newaddr);

  // Change the modules address
  Wire.beginTransmission(I2C_ADDRESS(oldaddr));
  Wire.write(I2CPE_SET_ADDR);
  Wire.write(newaddr);
  Wire.endTransmission();

  SERIAL_ECHOLNPGM("Address changed, resetting and waiting for confirmation..");

  // Wait for the module to reset (can probably be improved by polling address with a timeout).
  safe_delay(I2CPE_REBOOT_TIME);

  // Look for the module at the new address.
  Wire.beginTransmission(I2C_ADDRESS(newaddr));
  if (Wire.endTransmission()) {
    SERIAL_ECHOLNPGM("Address change failed! Check encoder module.");
    return;
  }

  SERIAL_ECHOLNPGM("Address change successful!");

  // Now, if this module is configured, find which encoder instance it's supposed to correspond to
  // and enable it (it will likely have failed initialization on power-up, before the address change).
  const int8_t idx = idx_from_addr(newaddr);
  if (idx >= 0 && !encoders[idx].get_active()) {
    SERIAL_CHAR(AXIS_CHAR(encoders[idx).get_axis()]);
    SERIAL_ECHOLNPGM(" axis encoder was not detected on printer startup. Trying again.");
    encoders[idx].set_active(encoders[idx].passes_test(true));
  }
}

void I2CPositionEncodersMgr::report_module_firmware(const uint8_t address) {
  // First check there is a module
  Wire.beginTransmission(I2C_ADDRESS(address));
  if (Wire.endTransmission()) {
    SERIAL_ECHOLNPGM("?No module detected at this address! (", address, ")");
    return;
  }

  SERIAL_ECHOLNPGM("Requesting version info from module at address ", address, ":");

  Wire.beginTransmission(I2C_ADDRESS(address));
  Wire.write(I2CPE_SET_REPORT_MODE);
  Wire.write(I2CPE_REPORT_VERSION);
  Wire.endTransmission();

  // Read value
  if (Wire.requestFrom(I2C_ADDRESS(address), uint8_t(32))) {
    char c;
    while (Wire.available() > 0 && (c = (char)Wire.read()) > 0)
      SERIAL_CHAR(c);
    SERIAL_EOL();
  }

  // Set module back to normal (distance) mode
  Wire.beginTransmission(I2C_ADDRESS(address));
  Wire.write(I2CPE_SET_REPORT_MODE);
  Wire.write(I2CPE_REPORT_DISTANCE);
  Wire.endTransmission();
}

int8_t I2CPositionEncodersMgr::parse() {
  I2CPE_addr = 0;

  if (parser.seen('A')) {

    if (!parser.has_value()) {
      SERIAL_ECHOLNPGM("?A seen, but no address specified! [30-200]");
      return I2CPE_PARSE_ERR;
    }

    I2CPE_addr = parser.value_byte();
    if (!WITHIN(I2CPE_addr, 30, 200)) { // reserve the first 30 and last 55
      SERIAL_ECHOLNPGM("?Address out of range. [30-200]");
      return I2CPE_PARSE_ERR;
    }

    I2CPE_idx = idx_from_addr(I2CPE_addr);
    if (I2CPE_idx >= I2CPE_ENCODER_CNT) {
      SERIAL_ECHOLNPGM("?No device with this address!");
      return I2CPE_PARSE_ERR;
    }
  }
  else if (parser.seenval('I')) {

    if (!parser.has_value()) {
      SERIAL_ECHOLNPGM("?I seen, but no index specified! [0-", I2CPE_ENCODER_CNT - 1, "]");
      return I2CPE_PARSE_ERR;
    }

    I2CPE_idx = parser.value_byte();
    if (I2CPE_idx >= I2CPE_ENCODER_CNT) {
      SERIAL_ECHOLNPGM("?Index out of range. [0-", I2CPE_ENCODER_CNT - 1, "]");
      return I2CPE_PARSE_ERR;
    }

    I2CPE_addr = encoders[I2CPE_idx].get_address();
  }
  else
    I2CPE_idx = 0xFF;

  I2CPE_anyaxis = parser.seen_axis();

  return I2CPE_PARSE_OK;
}

/**
 * M860:  Report the position(s) of position encoder module(s).
 *
 *   A<addr>  Module I2C address.  [30, 200].
 *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1]
 *   O        Include homed zero-offset in returned position.
 *   U        Units in mm or raw step count.
 *
 *   If A or I not specified:
 *    X       Report on X axis encoder, if present.
 *    Y       Report on Y axis encoder, if present.
 *    Z       Report on Z axis encoder, if present.
 *    E       Report on E axis encoder, if present.
 */
void I2CPositionEncodersMgr::M860() {
  if (parse()) return;

  const bool hasU = parser.seen_test('U'), hasO = parser.seen_test('O');

  if (I2CPE_idx == 0xFF) {
    LOOP_LOGICAL_AXES(i) {
      if (!I2CPE_anyaxis || parser.seen_test(AXIS_CHAR(i))) {
        const uint8_t idx = idx_from_axis(AxisEnum(i));
        if ((int8_t)idx >= 0) report_position(idx, hasU, hasO);
      }
    }
  }
  else
    report_position(I2CPE_idx, hasU, hasO);
}

/**
 * M861:  Report the status of position encoder modules.
 *
 *   A<addr>  Module I2C address.  [30, 200].
 *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1]
 *
 *   If A or I not specified:
 *    X       Report on X axis encoder, if present.
 *    Y       Report on Y axis encoder, if present.
 *    Z       Report on Z axis encoder, if present.
 *    E       Report on E axis encoder, if present.
 */
void I2CPositionEncodersMgr::M861() {
  if (parse()) return;

  if (I2CPE_idx == 0xFF) {
    LOOP_LOGICAL_AXES(i) {
      if (!I2CPE_anyaxis || parser.seen(AXIS_CHAR(i))) {
        const uint8_t idx = idx_from_axis(AxisEnum(i));
        if ((int8_t)idx >= 0) report_status(idx);
      }
    }
  }
  else
    report_status(I2CPE_idx);
}

/**
 * M862:  Perform an axis continuity test for position encoder
 *        modules.
 *
 *   A<addr>  Module I2C address.  [30, 200].
 *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1]
 *
 *   If A or I not specified:
 *    X       Report on X axis encoder, if present.
 *    Y       Report on Y axis encoder, if present.
 *    Z       Report on Z axis encoder, if present.
 *    E       Report on E axis encoder, if present.
 */
void I2CPositionEncodersMgr::M862() {
  if (parse()) return;

  if (I2CPE_idx == 0xFF) {
    LOOP_LOGICAL_AXES(i) {
      if (!I2CPE_anyaxis || parser.seen(AXIS_CHAR(i))) {
        const uint8_t idx = idx_from_axis(AxisEnum(i));
        if ((int8_t)idx >= 0) test_axis(idx);
      }
    }
  }
  else
    test_axis(I2CPE_idx);
}

/**
 * M863:  Perform steps-per-mm calibration for
 *        position encoder modules.
 *
 *   A<addr>  Module I2C address.  [30, 200].
 *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1]
 *   P        Number of rePeats/iterations.
 *
 *   If A or I not specified:
 *    X       Report on X axis encoder, if present.
 *    Y       Report on Y axis encoder, if present.
 *    Z       Report on Z axis encoder, if present.
 *    E       Report on E axis encoder, if present.
 */
void I2CPositionEncodersMgr::M863() {
  if (parse()) return;

  const uint8_t iterations = constrain(parser.byteval('P', 1), 1, 10);

  if (I2CPE_idx == 0xFF) {
    LOOP_LOGICAL_AXES(i) {
      if (!I2CPE_anyaxis || parser.seen(AXIS_CHAR(i))) {
        const uint8_t idx = idx_from_axis(AxisEnum(i));
        if ((int8_t)idx >= 0) calibrate_steps_mm(idx, iterations);
      }
    }
  }
  else
    calibrate_steps_mm(I2CPE_idx, iterations);
}

/**
 * M864:  Change position encoder module I2C address.
 *
 *   A<addr>  Module current/old I2C address.  If not present,
 *            assumes default address (030).  [30, 200].
 *   S<addr>  Module new I2C address. [30, 200].
 *
 *   If S is not specified:
 *    X       Use I2CPE_PRESET_ADDR_X (030).
 *    Y       Use I2CPE_PRESET_ADDR_Y (031).
 *    Z       Use I2CPE_PRESET_ADDR_Z (032).
 *    E       Use I2CPE_PRESET_ADDR_E (033).
 */
void I2CPositionEncodersMgr::M864() {
  uint8_t newAddress;

  if (parse()) return;

  if (!I2CPE_addr) I2CPE_addr = I2CPE_PRESET_ADDR_X;

  if (parser.seen('S')) {
    if (!parser.has_value()) {
      SERIAL_ECHOLNPGM("?S seen, but no address specified! [30-200]");
      return;
    }

    newAddress = parser.value_byte();
    if (!WITHIN(newAddress, 30, 200)) {
      SERIAL_ECHOLNPGM("?New address out of range. [30-200]");
      return;
    }
  }
  else if (!I2CPE_anyaxis) {
    SERIAL_ECHOLNPGM("?You must specify S or [XYZE].");
    return;
  }
  else {
         if (parser.seen_test('X')) newAddress = I2CPE_PRESET_ADDR_X;
    else if (parser.seen_test('Y')) newAddress = I2CPE_PRESET_ADDR_Y;
    else if (parser.seen_test('Z')) newAddress = I2CPE_PRESET_ADDR_Z;
    else if (parser.seen_test('E')) newAddress = I2CPE_PRESET_ADDR_E;
    else return;
  }

  SERIAL_ECHOLNPGM("Changing module at address ", I2CPE_addr, " to address ", newAddress);

  change_module_address(I2CPE_addr, newAddress);
}

/**
 * M865:  Check position encoder module firmware version.
 *
 *   A<addr>  Module I2C address.  [30, 200].
 *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1].
 *
 *   If A or I not specified:
 *    X       Check X axis encoder, if present.
 *    Y       Check Y axis encoder, if present.
 *    Z       Check Z axis encoder, if present.
 *    E       Check E axis encoder, if present.
 */
void I2CPositionEncodersMgr::M865() {
  if (parse()) return;

  if (!I2CPE_addr) {
    LOOP_LOGICAL_AXES(i) {
      if (!I2CPE_anyaxis || parser.seen(AXIS_CHAR(i))) {
        const uint8_t idx = idx_from_axis(AxisEnum(i));
        if ((int8_t)idx >= 0) report_module_firmware(encoders[idx].get_address());
      }
    }
  }
  else
    report_module_firmware(I2CPE_addr);
}

/**
 * M866:  Report or reset position encoder module error
 *        count.
 *
 *   A<addr>  Module I2C address.  [30, 200].
 *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1].
 *   R        Reset error counter.
 *
 *   If A or I not specified:
 *    X       Act on X axis encoder, if present.
 *    Y       Act on Y axis encoder, if present.
 *    Z       Act on Z axis encoder, if present.
 *    E       Act on E axis encoder, if present.
 */
void I2CPositionEncodersMgr::M866() {
  if (parse()) return;

  const bool hasR = parser.seen_test('R');

  if (I2CPE_idx == 0xFF) {
    LOOP_LOGICAL_AXES(i) {
      if (!I2CPE_anyaxis || parser.seen(AXIS_CHAR(i))) {
        const uint8_t idx = idx_from_axis(AxisEnum(i));
        if ((int8_t)idx >= 0) {
          if (hasR)
            reset_error_count(idx, AxisEnum(i));
          else
            report_error_count(idx, AxisEnum(i));
        }
      }
    }
  }
  else if (hasR)
    reset_error_count(I2CPE_idx, encoders[I2CPE_idx].get_axis());
  else
    report_error_count(I2CPE_idx, encoders[I2CPE_idx].get_axis());
}

/**
 * M867:  Enable/disable or toggle error correction for position encoder modules.
 *
 *   A<addr>  Module I2C address.  [30, 200].
 *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1].
 *   S<1|0>   Enable/disable error correction. 1 enables, 0 disables.  If not
 *            supplied, toggle.
 *
 *   If A or I not specified:
 *    X       Act on X axis encoder, if present.
 *    Y       Act on Y axis encoder, if present.
 *    Z       Act on Z axis encoder, if present.
 *    E       Act on E axis encoder, if present.
 */
void I2CPositionEncodersMgr::M867() {
  if (parse()) return;

  const int8_t onoff = parser.seenval('S') ? parser.value_int() : -1;

  if (I2CPE_idx == 0xFF) {
    LOOP_LOGICAL_AXES(i) {
      if (!I2CPE_anyaxis || parser.seen(AXIS_CHAR(i))) {
        const uint8_t idx = idx_from_axis(AxisEnum(i));
        if ((int8_t)idx >= 0) {
          const bool ena = onoff == -1 ? !encoders[I2CPE_idx].get_ec_enabled() : !!onoff;
          enable_ec(idx, ena, AxisEnum(i));
        }
      }
    }
  }
  else {
    const bool ena = onoff == -1 ? !encoders[I2CPE_idx].get_ec_enabled() : !!onoff;
    enable_ec(I2CPE_idx, ena, encoders[I2CPE_idx].get_axis());
  }
}

/**
 * M868:  Report or set position encoder module error correction
 *        threshold.
 *
 *   A<addr>  Module I2C address.  [30, 200].
 *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1].
 *   T        New error correction threshold.
 *
 *   If A not specified:
 *    X       Act on X axis encoder, if present.
 *    Y       Act on Y axis encoder, if present.
 *    Z       Act on Z axis encoder, if present.
 *    E       Act on E axis encoder, if present.
 */
void I2CPositionEncodersMgr::M868() {
  if (parse()) return;

  const float newThreshold = parser.seenval('T') ? parser.value_float() : -9999;

  if (I2CPE_idx == 0xFF) {
    LOOP_LOGICAL_AXES(i) {
      if (!I2CPE_anyaxis || parser.seen(AXIS_CHAR(i))) {
        const uint8_t idx = idx_from_axis(AxisEnum(i));
        if ((int8_t)idx >= 0) {
          if (newThreshold != -9999)
            set_ec_threshold(idx, newThreshold, encoders[idx].get_axis());
          else
            get_ec_threshold(idx, encoders[idx].get_axis());
        }
      }
    }
  }
  else if (newThreshold != -9999)
    set_ec_threshold(I2CPE_idx, newThreshold, encoders[I2CPE_idx].get_axis());
  else
    get_ec_threshold(I2CPE_idx, encoders[I2CPE_idx].get_axis());
}

/**
 * M869:  Report position encoder module error.
 *
 *   A<addr>  Module I2C address.  [30, 200].
 *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1].
 *
 *   If A not specified:
 *    X       Act on X axis encoder, if present.
 *    Y       Act on Y axis encoder, if present.
 *    Z       Act on Z axis encoder, if present.
 *    E       Act on E axis encoder, if present.
 */
void I2CPositionEncodersMgr::M869() {
  if (parse()) return;

  if (I2CPE_idx == 0xFF) {
    LOOP_LOGICAL_AXES(i) {
      if (!I2CPE_anyaxis || parser.seen(AXIS_CHAR(i))) {
        const uint8_t idx = idx_from_axis(AxisEnum(i));
        if ((int8_t)idx >= 0) report_error(idx);
      }
    }
  }
  else
    report_error(I2CPE_idx);
}

#endif // I2C_POSITION_ENCODERS
