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

#include "../../inc/MarlinConfigPre.h"

#define CAN_HOST_MAX_STRING_MSG_LENGTH                    128 // Max string message length to receive from the toolhead
#define CAN_HOST_GCODE_TIME_SYNC_NO                      7777 // A unique unused Gcode number to trigger a time sync
#define CAN_HOST_CONFIGURATION_COMPLETE                  7778 // Signal the configuration is complete
#define CAN_HOST_MAX_WAIT_TIME                             25 // Time in ms to wait for CAN FIFO buffers
#define CAN_HOST_E0_TEMP_UPDATE_WATCHDOG_TIME            3000 // An E0 temp update must be received withing this time
#define CAN_HOST_ERROR_REPEAT_TIME                      10000 // Time between report repeats of an error message

#define STDID_FIFO_TOGGLE_BIT                   0b10000000000
#define EXTID_FIFO_TOGGLE_BIT                      0x10000000

#define CAN_ID_IO_MASK                                0b11111 // Mask for the 5 virtual IO bits
#define CAN_ID_GCODE_MASK                   0b111111111111111 // Gcode type and number mask
#define CAN_ID_GCODE_NUMBER_MASK              0b1111111111111 // Gcode number mask
#define CAN_ID_PARAMETER_LETTER_MASK                  0b11111
#define CAN_ID_PARAMETER_COUNT_MASK                     0b111 // Parameter count
#define CAN_ID_GCODE_TYPE_MASK                           0b11 // Gcode type mask

#define CAN_ID_PROBE_BIT_MASK                         (1 << 0) // Virtual IO bit for Z-probe pin
#define CAN_ID_PROBE_BIT_POS                                0
#define CAN_ID_FILAMENT_BIT_MASK                      (1 << 1) // Virtual IO bit for filament pin
#define CAN_ID_FILAMENT_BIT_POS                             1
#define CAN_ID_X_ENDSTOP_BIT_MASK                     (1 << 2) // Virtual IO bit for X-min pin
#define CAN_ID_X_ENDSTOP_BIT_POS                            2
#define CAN_ID_Y_ENDSTOP_BIT_MASK                     (1 << 3) // Virtual IO bit for Y-min pin
#define CAN_ID_Y_ENDSTOP_BIT_POS                            3
#define CAN_ID_Z_ENDSTOP_BIT_MASK                     (1 << 4) // Virtual IO bit for Z-min pin
#define CAN_ID_Z_ENDSTOP_BIT_POS                            4
#define CAN_ID_STRING_MESSAGE_BIT_MASK                (1 << 5) // Signals the toolhead sent a string message
#define CAN_ID_STRING_MESSAGE_BIT_POS                       5
#define CAN_ID_REQUEST_SETUP_BIT_MASK                 (1 << 6) // Signals the toolhead requests setup information
#define CAN_ID_REQUEST_SETUP_BIT_POS                        6
#define CAN_ID_TMC_OT_BIT_MASK                        (1 << 7) // Signals the toolhead signals a TMC Over-Temp error
#define CAN_ID_TMC_OT_BIT_POS                               7
#define CAN_ID_REQUEST_TIME_SYNC_BIT_MASK             (1 << 8) // Signals the toolhead requested a time sync
#define CAN_ID_REQUEST_TIME_SYNC_BIT_POS                    8
#define CAN_ID_ERROR_BIT_MASK                         (1 << 9) // Signals the toolhead encountered an error
#define CAN_ID_ERROR_BIT_POS                                9

#define CAN_ID_PARAMETER1_BIT_POS                           0
#define CAN_ID_PARAMETER2_BIT_POS                           5
#define CAN_ID_GCODE_NUMBER_BIT_POS                        10
#define CAN_ID_GCODE_TYPE_BIT_POS                          23
#define CAN_ID_PARAMETER_COUNT_BIT_POS                     25

#define CAN_ID_GCODE_TYPE_D                                 0
#define CAN_ID_GCODE_TYPE_G                                 1
#define CAN_ID_GCODE_TYPE_M                                 2
#define CAN_ID_GCODE_TYPE_T                                 3

// Host error messages
#define CAN_ERROR_HOST_RX_FIFO_OVERFLOW               (1 << 0) // Incoming message lost
#define CAN_ERROR_HOST_TX_MSG_DROPPED                 (1 << 1) // Outgoing message dropped
#define CAN_ERROR_HOST_INVALID_GCODE                  (1 << 2) // Gcode could not be sent over CANBUS
#define CAN_ERROR_HOST_INVALID_BAUDRATE               (1 << 3) // Generated baudrate doesn't match CAN_BAUDRATE

// Toolhead error messages
#define CAN_ERROR_TOOLHEAD_RX_FIFO_OVERFLOW           (1 << 4)
#define CAN_ERROR_TOOLHEAD_TX_FIFO_OVERFLOW           (1 << 5)
#define CAN_ERROR_TOOLHEAD_INCOMPLETE_GCODE_RECEIVED  (1 << 6)
#define CAN_ERROR_TOOLHEAD_MARLIN_CMD_BUFFER_OVERFLOW (1 << 7)
#define CAN_ERROR_TOOLHEAD_INVALID_BAUDRATE           (1 << 8) // Generated baudrate doesn't match CAN_BAUDRATE

// CAN error messsages
#define CAN_ERROR_MSG_RX_FIFO_OVERFLOW               "CAN RX FIFO overflow"
#define CAN_ERROR_MSG_TX_FIFO_OVERFLOW               "CAN TX FIFO overflow"
#define CAN_ERROR_MSG_INCOMPLETE_GCODE               "Incomplete Gcode message received"
#define CAN_ERROR_MSG_MARLIN_CMM_BUF_OVERFLOW        "Marlin CMD buffer overflow"
#define CAN_ERROR_MSG_INVALID_BAUDRATE               "Incorrect CAN baudrate"

void CAN_host_idle();                                // CAN idle task
void CAN_host_send_setup(bool change_status);        // Send configuration to toolhead
uint32_t CAN_host_get_iostate();                     // Read the CAN virtual IO state
HAL_StatusTypeDef CAN_host_start();                  // Start the CAN device
HAL_StatusTypeDef CAN_host_stop();                   // Stop the CAN device
HAL_StatusTypeDef CAN_host_send_gcode();             // Send Gcode to the toolhead
HAL_StatusTypeDef CAN_host_send_gcode_2params(uint32_t Gcode_type, uint32_t Gcode_no, uint32_t parameter1, float value1, uint32_t parameter2, float value2);

HAL_StatusTypeDef CAN_toolhead_start();              // Start the CAN device
void CAN_toolhead_send_update(bool tempUpdate);      // Send an IO and temp update to the host
void CAN_toolhead_send_string(const char * message); // Send CAN string to host
void CAN_toolhead_idle();   