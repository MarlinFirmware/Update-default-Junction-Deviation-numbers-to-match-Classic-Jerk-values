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

/**
 * servo.cpp - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
 * Copyright (c) 2009 Michael Margolis.  All right reserved.
 */

/**
 * A servo is activated by creating an instance of the Servo class passing the desired pin to the attach() method.
 * The servos are pulsed in the background using the value most recently written using the write() method
 *
 * Note that analogWrite of PWM on pins associated with the timer are disabled when the first servo is attached.
 * Timers are seized as needed in groups of 12 servos - 24 servos use two timers, 48 servos will use four.
 *
 * The methods are:
 *
 * Servo - Class for manipulating servo motors connected to Arduino pins.
 *
 * attach(pin)           - Attach a servo motor to an i/o pin.
 * attach(pin, min, max) - Attach to a pin, setting min and max values in microseconds
 *                         Default min is 544, max is 2400
 *
 * write()               - Set the servo angle in degrees. (Invalid angles —over MIN_PULSE_WIDTH— are treated as µs.)
 * writeMicroseconds()   - Set the servo pulse width in microseconds.
 * move(pin, angle)      - Sequence of attach(pin), write(angle), safe_delay(servo_delay[servoIndex]).
 *                         With DEACTIVATE_SERVOS_AFTER_MOVE it detaches after servo_delay[servoIndex].
 * read()                - Get the last-written servo pulse width as an angle between 0 and 180.
 * readMicroseconds()    - Get the last-written servo pulse width in microseconds.
 * attached()            - Return true if a servo is attached.
 * detach()              - Stop an attached servo from pulsing its i/o pin.
 */

#include "../../inc/MarlinConfig.h"

#if SHARED_SERVOS

#include "servo.h"
#include "servo_private.h"

ServoInfo_t servo_info[MAX_SERVOS];             // static array of servo info structures
uint8_t ServoCount = 0;                         // the total number of attached servos

#define SERVO_MIN_US(v) (MIN_PULSE_WIDTH - (v) * 4) // minimum value in uS for this servo
#define SERVO_MAX_US(v) (MAX_PULSE_WIDTH - (v) * 4) // maximum value in uS for this servo

/************ static functions common to all instances ***********************/

static bool anyTimerChannelActive(const timer16_Sequence_t timer) {
  // returns true if any servo is active on this timer
  for (uint_fast8_t channel = 0; channel < SERVOS_PER_TIMER; ++channel) {
    if (SERVO(timer, channel).Pin.isActive)
      return true;
  }
  return false;
}

/****************** end of static functions ******************************/

Servo::Servo() {
  if (ServoCount < MAX_SERVOS) {
    servoIndex = ServoCount++;                    // assign a servo index to this instance
    servo_info[servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);   // store default values  - 12 Aug 2009
  }
  else
    servoIndex = INVALID_SERVO;  // too many servos
}

int8_t Servo::attach(const int inPin) {
  return attach(inPin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

int8_t Servo::attach(const int inPin, const int inMin, const int inMax) {

  if (servoIndex >= MAX_SERVOS) return -1;

  if (inPin > 0) servo_info[servoIndex].Pin.nbr = inPin;
  pinMode(servo_info[servoIndex].Pin.nbr, OUTPUT); // set servo pin to output

  // TODO: min/max check: ABS(min - MIN_PULSE_WIDTH) / 4 < 128
  min = (MIN_PULSE_WIDTH - inMin) / 4; //resolution of min/max is 4 uS
  max = (MAX_PULSE_WIDTH - inMax) / 4;

  // initialize the timer if it has not already been initialized
  const timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
  if (!anyTimerChannelActive(timer)) initISR(timer);
  servo_info[servoIndex].Pin.isActive = true;  // this must be set after the check for anyTimerChannelActive

  return servoIndex;
}

void Servo::detach() {
  servo_info[servoIndex].Pin.isActive = false;
  const timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
  if (!anyTimerChannelActive(timer)) finISR(timer);
  //pinMode(servo_info[servoIndex].Pin.nbr, INPUT); // set servo pin to input
}

void Servo::write(int value) {
  if (value < MIN_PULSE_WIDTH)    // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
    value = map(constrain(value, 0, 180), 0, 180, SERVO_MIN_US(min), SERVO_MAX_US(max));
  writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value) {
  // calculate and store the values for the given channel
  byte channel = servoIndex;
  if (channel < MAX_SERVOS) {  // ensure channel is valid
    // ensure pulse width is valid
    LIMIT(value, SERVO_MIN_US(min), SERVO_MAX_US(max));
    value = usToTicks(value - (TRIM_DURATION));  // convert to ticks after compensating for interrupt overhead - 12 Aug 2009

    CRITICAL_SECTION_START();
    servo_info[channel].ticks = value;
    CRITICAL_SECTION_END();
  }
}

// return the value as degrees
int Servo::read() { return map(readMicroseconds() + 1, SERVO_MIN_US(min), SERVO_MAX_US(max), 0, 180); }

int Servo::readMicroseconds() {
  return (servoIndex == INVALID_SERVO) ? 0 : ticksToUs(servo_info[servoIndex].ticks) + (TRIM_DURATION);
}

bool Servo::attached() { return servo_info[servoIndex].Pin.isActive; }

void Servo::move(const int value) {
  constexpr uint16_t servo_delay[] = SERVO_DELAY;
  static_assert(COUNT(servo_delay) == NUM_SERVOS, "SERVO_DELAY must be an array NUM_SERVOS long.");
  if (attach(0) >= 0) {
    write(value);
    safe_delay(servo_delay[servoIndex]);
    TERN_(DEACTIVATE_SERVOS_AFTER_MOVE, detach());
  }
}

#endif // SHARED_SERVOS
