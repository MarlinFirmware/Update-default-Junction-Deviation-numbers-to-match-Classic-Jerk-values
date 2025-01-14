/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * progress_codes.h
 */

#include <stdint.h>

/**
 * A complete set of progress codes which may be reported while running a high-level command/operation.
 * This header file should be included in the printer's firmware as well as a reference, so the progress
 * codes are extracted to one place.
 */
enum class ProgressCode : uint_fast8_t {
  OK = 0, //!< finished ok

  EngagingIdler, // P1
  DisengagingIdler, // P2
  UnloadingToFinda, // P3
  UnloadingToPulley, // P4
  FeedingToFinda, // P5
  FeedingToExtruder, // P6
  FeedingToNozzle, // P7
  AvoidingGrind, // P8
  FinishingMoves, // P9

  ERRDisengagingIdler, // P10
  ERREngagingIdler, // P11
  ERRWaitingForUser, // P12
  ERRInternal, // P13
  ERRHelpingFilament, // P14
  ERRTMCFailed, // P15

  UnloadingFilament, // P16
  LoadingFilament, // P17
  SelectingFilamentSlot, // P18
  PreparingBlade, // P19
  PushingFilament, // P20
  PerformingCut, // P21
  ReturningSelector, // P22
  ParkingSelector, // P23
  EjectingFilament, // P24
  RetractingFromFinda, // P25

  Homing, // P26
  MovingSelector, // P27

  FeedingToFSensor, // P28

  Empty = 0xFF // dummy empty state
};
