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

/**
 * Contributor Notes:
 * NOTE 1: Requires `HAL_FDCAN_MODULE_ENABLED`, e.g., with `-DFDHAL_CAN_MODULE_ENABLED`
 *         For Arduino IDE use "hal_conf_extra.h" with `#define HAL_FDCAN_MODULE_ENABLED`
 * NOTE 2: Serial communication in ISR causes issues! Hangs etc. so avoid this!
 */

#include "../../inc/MarlinConfigPre.h"

#if ALL(CAN_HOST, STM32H7xx)

#include "../platforms.h"
#include "../../gcode/parser.h"
#include "../../module/temperature.h"
#include "../../module/motion.h"  // For current_position variable
#include "../../module/planner.h" // For steps/mm parameters variables
#include "../../feature/tmc_util.h"
#include "../../module/endstops.h"
#include "../../feature/controllerfan.h" // For controllerFan settings
#include "../../libs/numtostr.h"  // For float to string conversion

#include "../shared/CAN.h"

// Interrupt handlers
extern "C" void FDCAN1_IT0_IRQHandler(void);
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);

#ifndef CAN_BAUDRATE
  #define CAN_BAUDRATE 1000000LU
#endif

#ifndef CAN_RD_PIN
  #define CAN_RD_PIN PB8
#endif
#ifndef CAN_TD_PIN
  #define CAN_TD_PIN PB9
#endif

#if (CAN_BAUDRATE != 1000000) && (CAN_BAUDRATE != 500000) && (CAN_BAUDRATE != 250000) && (CAN_BAUDRATE != 125000)
  #error ERROR: Select a valid CAN_BAUDRATE: 1000000, 500000, 250000 or 125000 baud
#endif

#define FDCAN_HOST_TX_FIFO_DEPTH                         8 // TX FIFO0 and FIFO1 depth 0-32
#define FDCAN_HOST_RX_FIFO_DEPTH                         8 // RX FIFO0 and FIFO1 depth 0-64

#define FDCAN_HOST_DATALENGTH_OFFSET                    16 // Bit offset of FDCAN_DLC_BYTES_1 in register

FDCAN_HandleTypeDef hCAN1                          = { 0 }; // The global FDCAN handle
FDCAN_TxHeaderTypeDef TxHeader                     = { 0 }; // Header to send a FDCAN message
volatile uint32_t CAN_io_state                         = 0; // Virtual IO state variable
volatile bool CAN_toolhead_error                       = 0; // Register if an error was reported by the toolhead
volatile bool CAN_toolhead_setup_request           = false; // Signals the toolhead requested setup information
volatile bool CAN_time_sync_request                = false; // Signals the toolhead requested a time sync
volatile uint32_t time_sync_request_time               = 0; // Record the time the time sync request was received

volatile uint32_t HAL_FDCAN_error_code                 = 0; // Store the HAL FDCAN error code
volatile uint32_t CAN_host_error_code                  = 0; // Store the CAN host error code

volatile bool first_E0_error                        = true; // First CAN bus error, show warning only once
volatile bool string_message_complete              = false; // Signals a complete string message was received
uint32_t CAN_next_temp_report_time                 = 10000; // Track when the next toolhead temperature report should arrive, delay at startup
uint32_t CAN_next_error_message_time                   = 0; // Track when to display the next repeat of an error message
volatile bool CAN_host_FIFO_toggle_bit             = false; // FIFO toggle flag for receiver FIFO filtering
SString <CAN_HOST_MAX_STRING_MSG_LENGTH> string_message;    // CAN string message buffer for incoming messages

uint32_t CAN_host_get_iostate() {
  return CAN_io_state;
}

uint32_t CAN_set_extended_id(int gcode_type, int gcode_no, int parameter1, int parameter2, int count) {

  CAN_host_FIFO_toggle_bit = !CAN_host_FIFO_toggle_bit;                                // FIFO toggle bit

  return (CAN_host_FIFO_toggle_bit ? EXTID_FIFO_TOGGLE_BIT : 0)                      | // FIFO toggle bit
         (count << CAN_ID_PARAMETER_COUNT_BIT_POS)                                   | // Parameter count
         (gcode_type << CAN_ID_GCODE_TYPE_BIT_POS)                                   | // G/M/T/D-code
         ((gcode_no & CAN_ID_GCODE_NUMBER_MASK) << CAN_ID_GCODE_NUMBER_BIT_POS)      | // Gcode number
         ((parameter1 & CAN_ID_PARAMETER_LETTER_MASK) << CAN_ID_PARAMETER1_BIT_POS)  | // First parameter
         ((parameter2 & CAN_ID_PARAMETER_LETTER_MASK) << CAN_ID_PARAMETER2_BIT_POS);   // Second parameter

}

// Send time sync timestamp of arrival and response time
void CAN_host_send_timestamp() { // Request receive timestamp + request response timestamp

  TxHeader.IdType     = FDCAN_EXTENDED_ID;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8; // Send sync time t1(receive time, uint32_t) and t2(response time, uint32_t)
  TxHeader.Identifier = CAN_set_extended_id(CAN_ID_GCODE_TYPE_M, CAN_HOST_GCODE_TIME_SYNC_NO, 1, 2, 2);

  uint8_t CAN_tx_buffer[8];                       // 8 bytes CAN data TX buffer
  uint32_t * uint32p = (uint32_t *)CAN_tx_buffer; // Point to TX buffer
  *uint32p++ = time_sync_request_time;

  uint32_t deadline = millis() + CAN_HOST_MAX_WAIT_TIME;
  while ((HAL_FDCAN_GetTxFifoFreeLevel(&hCAN1) < FDCAN_HOST_TX_FIFO_DEPTH) && PENDING(millis(), deadline)) { /* BLOCKING! Wait for empty TX buffer */ }

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hCAN1)) {
    *uint32p = micros(); // Only record the response time at the last possible moment
    HAL_FDCAN_AddMessageToTxFifoQ(&hCAN1, &TxHeader, CAN_tx_buffer); // Queue CAN message
  }
  else
    CAN_host_error_code |= CAN_ERROR_HOST_TX_MSG_DROPPED;

}

// Send specified Gcode with max 2 parameters and 2 values via CAN bus
HAL_StatusTypeDef CAN_host_send_gcode_2params(uint32_t Gcode_type, uint32_t Gcode_no, uint32_t parameter1, float value1, uint32_t parameter2, float value2) {

  HAL_StatusTypeDef status = HAL_OK;

  switch (Gcode_type) {

    case 'D':
      Gcode_type = CAN_ID_GCODE_TYPE_D;
      return HAL_ERROR;
    break;

    case 'G':
      Gcode_type = CAN_ID_GCODE_TYPE_G;
      return HAL_ERROR;
    break;

    case 'M':
      Gcode_type = CAN_ID_GCODE_TYPE_M;

      #if ENABLED(CAN_DEBUG)
        SERIAL_ECHOPGM("; MSG to toolhead: \"M", Gcode_no);
        if (parameter1) {
          SERIAL_CHAR(' ', parameter1);
          if (value1 == int(value1))
            SERIAL_ECHO(int(value1)); // Integer value
          else
            SERIAL_ECHO(p_float_t(value1, 4));  // Float with 4 digits
        }

        if (parameter2) {
          SERIAL_CHAR(' ', parameter2);
          if (value2 == int(value2))
            SERIAL_ECHO(int(value2)); // Integer value
          else
            SERIAL_ECHO(p_float_t(value2, 4));  // Float with 4 digits
        }
        SERIAL_ECHOLN("\"");
      #endif

    break;

    case 'T': Gcode_type = CAN_ID_GCODE_TYPE_T;
      return HAL_ERROR;
    break;

    default:
    return HAL_ERROR; // Unknown Gcode type
  }

  if (parameter1 > 31)
    parameter1 -= 64; // Format 'A' = 1, 'B' = 2, etc.

  if (parameter2 > 31)
    parameter2 -= 64; // Format 'A' = 1, 'B' = 2, etc.

  TxHeader.IdType = FDCAN_EXTENDED_ID;

  TxHeader.DataLength = 4 * (!!parameter1 + !!parameter2) << FDCAN_HOST_DATALENGTH_OFFSET;  // Amount of bytes to send (4 or 8)
  TxHeader.Identifier = CAN_set_extended_id(Gcode_type, Gcode_no, parameter1, parameter2, !!parameter1 + !!parameter2);
  
  uint8_t CAN_tx_buffer[8];            // 8 bytes CAN data TX buffer
  float * fp = (float *)CAN_tx_buffer; // Point to TX buffer
  *fp++ = value1;
  *fp   = value2;

  const uint32_t deadline = millis() + CAN_HOST_MAX_WAIT_TIME;
  while ((HAL_FDCAN_GetTxFifoFreeLevel(&hCAN1) == 0) && PENDING(millis(), deadline)) { /* BLOCKING! Wait for empty TX buffer */ }

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hCAN1))
    status = HAL_FDCAN_AddMessageToTxFifoQ(&hCAN1, &TxHeader, CAN_tx_buffer); // Queue CAN message
  else
    CAN_host_error_code |= CAN_ERROR_HOST_TX_MSG_DROPPED;

  return status;
}

void CAN_host_send_setup(bool changeStatus) { // Send setup to toolhead

  // NOTE: Sending many command too fast will cause a Marlin command buffer overrun at the toolhead, add delays if needed
  CAN_toolhead_setup_request = false;

  SERIAL_ECHOLNPGM(">>> CAN: Sending configuration to toolhead...");

  #if ENABLED(MPCTEMP)
    // M306 MPC settings (managed by host)
    MPC_t &mpc = thermalManager.temp_hotend[0].mpc;

    CAN_host_send_gcode_2params('M', 306, 'A', mpc.ambient_xfer_coeff_fan0, 'C', mpc.block_heat_capacity);          // M306 R<sensor_responsiveness> A<Ambient heat transfer coefficient>
    CAN_host_send_gcode_2params('M', 306, 'F', mpc.fanCoefficient(),        'H', mpc.filament_heat_capacity_permm); // M306 F<sensor_responsiveness> H<filament_heat_capacity_permm>
    CAN_host_send_gcode_2params('M', 306, 'P', mpc.heater_power,            'R', mpc.sensor_responsiveness);        // M306 P<heater_power> C<Heatblock Capacity (joules/kelvin)>

  #endif

  //CAN_host_send_gcode_2params('M', 150, 0, 0, 0, 0); // M150, SWITCH NEOPIXEL OFF

  /*
  extern Planner planner; // M92 Steps per mm
  CAN_host_send_gcode_2params('M', 92, 'X', planner.settings.axis_steps_per_mm[X_AXIS], 'Y', planner.settings.axis_steps_per_mm[Y_AXIS]);
  CAN_host_send_gcode_2params('M', 92, 'Z', planner.settings.axis_steps_per_mm[Z_AXIS], 'E', planner.settings.axis_steps_per_mm[E_AXIS]);

  // M200 Set filament diameter
  CAN_host_send_gcode_2params('M', 200, 'S', parser.volumetric_enabled, 'D', LINEAR_UNIT(planner.filament_size[0]));
  #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
     CAN_host_send_gcode_2params('M', 200, 'L', LINEAR_UNIT(planner.volumetric_extruder_limit[0])
  #endif

  // M201 Max acceleration
  CAN_host_send_gcode_2params('M', 201, 'X', planner.settings.max_acceleration_mm_per_s2[X_AXIS], 'Y', planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);
  CAN_host_send_gcode_2params('M', 201, 'Z', planner.settings.max_acceleration_mm_per_s2[Z_AXIS], 'E', planner.settings.max_acceleration_mm_per_s2[E_AXIS]);

  // M203 Max feedrate
  CAN_host_send_gcode_2params('M', 203, 'X', planner.settings.max_feedrate_mm_s[X_AXIS], 'Y', planner.settings.max_feedrate_mm_s[Y_AXIS]);
  CAN_host_send_gcode_2params('M', 203, 'Z', planner.settings.max_feedrate_mm_s[Z_AXIS], 'E', planner.settings.max_feedrate_mm_s[E_AXIS]);

  // M204 Accelerations in units/sec^2, ENABLED BECAUSE IT INFORMS THE TOOLHEAD THE CONFIGURATION WAS SENT
  CAN_host_send_gcode_2params('M', 204, 'P', planner.settings.acceleration, 'R', planner.settings.retract_acceleration);
  CAN_host_send_gcode_2params('M', 204, 'T', planner.settings.travel_acceleration, 0, 0);

  // M205
  #if ENABLED(CLASSIC_JERK)
    CAN_host_send_gcode_2params('M', 205,'S')) planner.settings.min_feedrate_mm_s, 'T')) planner.settings.min_travel_feedrate_mm_s);
    CAN_host_send_gcode_2params('M', 205, M205_MIN_SEG_TIME_PARAM, planner.settings.min_segment_time_us, 'J', planner.junction_deviation_mm);
    CAN_host_send_gcode_2params('M', 205, 'X', LINEAR_UNIT(planner.max_jerk.x), 'Y', LINEAR_UNIT(planner.max_jerk.y));
    CAN_host_send_gcode_2params('M', 205, 'Z', LINEAR_UNIT(planner.max_jerk.z), 'E', LINEAR_UNIT(planner.max_jerk.e));
    CAN_host_send_gcode_2params('M', 205, 'J', LINEAR_UNIT(planner.junction_deviation_mm), 0, 0);
  #endif

  // M206 Home offset
  #if DISABLED(NO_HOME_OFFSETS)
    CAN_host_send_gcode_2params('M', 206, 'X', LINEAR_UNIT(home_offset.x), 'Y', LINEAR_UNIT(home_offset.y));
    CAN_host_send_gcode_2params('M', 206, 'Z', LINEAR_UNIT(home_offset.z), 0, 0);
  #endif

  // M207 Set Firmware Retraction
  // M208 - Firmware Recover
  // M209 - Set Auto Retract

  // M220 Speed/feedrate
  CAN_host_send_gcode_2params('M', 220, 'S', feedrate_percentage, 0, 0);

  // M221 Flow percentage
  CAN_host_send_gcode_2params('M', 221, 'T', 0, 'S', planner.flow_percentage[0]);
  // CAN_host_send_gcode_2params('M', 221, 'T', 1, 'S', planner.flow_percentage[1]); // For 2nd extruder

  // M302 Cold extrude settings
  #if ENABLED(PREVENT_COLD_EXTRUSION)
    CAN_host_send_gcode_2params('M', 302, 'P', '0' + thermalManager.allow_cold_extrude, 'S', thermalManager.extrude_min_temp); // P0 enable cold extrusion checking, P1 = disabled, S=Minimum temperature
  #endif

  // M569 TMC Driver StealthChop/SpreadCycle
  CAN_host_send_gcode_2params('M', 569, 'S', stepperE0.get_stored_stealthChop(), 'E', 0); // M569 S[0/1] E

  // M592 Nonlinear Extrusion Control

  // M916 TMC Motor current
  CAN_host_send_gcode_2params('M', 906, 'E', stepperE0.getMilliamps(), 0, 0);

  // M919 TMC Chopper timing for E only
  CAN_host_send_gcode_2params('M', 919, 'O', off, 'P' , Hysteresis End);
  CAN_host_send_gcode_2params('M', 919, 'S', Hysteresis Start, 0, 0);
  */

  #if ALL(USE_CONTROLLER_FAN, CONTROLLER_FAN_EDITABLE)
    CAN_host_send_gcode_2params('M', 710, 'E', controllerFan.settings.extruder_auto_fan_speed, 'P', controllerFan.settings.probing_auto_fan_speed);
  #endif

  // Signal to the toolhead that the configuration is complete, use it as the last Gcode to send
  if (changeStatus)
    CAN_host_send_gcode_2params('M', CAN_HOST_CONFIGURATION_COMPLETE, 0, 0, 0, 0);
}

void CAN_host_idle() { // Tasks that can/should not be done in the ISR

  if (CAN_time_sync_request) { // Send time sync timestamps
     CAN_time_sync_request = false;
     CAN_host_send_timestamp();
  }

  if (string_message_complete) { // Received string message is complete, display the string
    BUZZ(1, SOUND_OK);
    SERIAL_ECHOPGM(">>> CAN toolhead MSG: ");

    string_message.echo(); // Show received string message, ends on '\n'

    string_message_complete = false; // Get ready for the next string
    string_message.clear();
  }

  // Report time sync results
  if ((hCAN1.ErrorCode || CAN_toolhead_error || HAL_FDCAN_error_code) && ELAPSED(millis(), CAN_next_error_message_time)) {

    BUZZ(1, SOUND_ERROR);

    if (CAN_toolhead_error) {
      SERIAL_ECHOLNPGM(">>> CAN Error reported by toolhead");
      CAN_toolhead_error = false; // Reset, but will be repeated by the toolhead
    }

    if (HAL_FDCAN_error_code)
      SERIAL_ECHOLNPGM(">>> HAL CAN Error reported: ", HAL_FDCAN_error_code);

    if (CAN_host_error_code)
      SERIAL_ECHOLNPGM(">>> HOST CAN Error Code: ", CAN_host_error_code);

    if (hCAN1.ErrorCode)
      SERIAL_ECHOLNPGM(">>> CAN Error Code = ", hCAN1.ErrorCode);

    CAN_next_error_message_time = millis() + CAN_HOST_ERROR_REPEAT_TIME;
  }

  if (ELAPSED(millis(), CAN_next_temp_report_time)) {

    CAN_next_temp_report_time = millis() + CAN_HOST_ERROR_REPEAT_TIME;
    if (first_E0_error) {   // Send error notification
      BUZZ(1, SOUND_ERROR); // Warn with sound
      SERIAL_ECHOLNPGM("Error: No CAN E0 temp updates");
    }
    else // Send only error message
      SERIAL_ECHOLNPGM(">>> CAN error: No E0 temp updates");

    first_E0_error = false; // Warn only once

    #if DISABLED(CAN_DEBUG) // Only kill if not debugging
      kill(F("CAN error: No E0 tempeature updates"));
    #endif

    if (CAN_toolhead_setup_request) // The toolhead requested the setup configuration
      CAN_host_send_setup(true);
  }
}

HAL_StatusTypeDef CAN_host_send_gcode() { // Forward a Marlin Gcode via CAN (uses parser.command_letter, Gcode_no, parser.value_float())
  // Send a Gcode to the toolhead with parameters and values
  // Gcode starts with extended frame which can send the Gcode with max 2 parameters and values.
  // Extended frames are send to complete all parameters and values (max 2 per extended message).
  // 1. Analyze Gcode command
  // 2. Ignore gcodes that do not need to be forwarded
  // 3. Send parameters and values
  // char s[] = "G0 X123.45678 Y124.45678 Z125.45678 E126.45678 F127.45678\n";

  HAL_StatusTypeDef status = HAL_OK;

  if (parser.command_letter != 'M') // Only forward Mxxx Gcode to toolhead
    return HAL_OK;

  uint32_t Gcode_type = CAN_ID_GCODE_TYPE_M; // M-code, fixed for now
  uint32_t Gcode_no = parser.codenum & CAN_ID_GCODE_NUMBER_MASK;

  if (Gcode_no == 109) // Convert M109(Hotend wait) to M104 (no wait) to keep the toolhead responsive
    Gcode_no = 104;

  if ((Gcode_no == 501) || (Gcode_no == 502)) // M501=Restore settings, M502=Factory defaults
    CAN_toolhead_setup_request = true; // Also update settings for the toolhead

  if ((Gcode_no != 104) &&  // Set hotend target temp
      (Gcode_no != 106) &&  // Set cooling fan speed
      (Gcode_no != 107) &&  // Cooling fan off
      (Gcode_no != 115) &&  // Firmware info (testing)
      (Gcode_no != 119) &&  // Endstop status (testing)
      (Gcode_no != 150) &&  // Set NeoPixel values
    //(Gcode_no != 108) &&  // Break and Continue
      (Gcode_no != 280) &&  // Servo position
      (Gcode_no != 306) &&  // MPC settings/tuning
      (Gcode_no != 710) &&  // Control fan PWM
      (Gcode_no != 997))    // Reboot
    return HAL_OK;          // Nothing to do

  uint32_t index;
  uint32_t parameter_counter = 0;
  char letters[] = "XYZEFPSABCHIJKLOQRTUVW"; // All possible parameters (22), defines scan order, no "D G M N", includes 'T' for autotune (M306 T)
  static uint32_t parameters[8] = { 0 }; // Store found parameters, send max 7 parameters (send in pairs, so reserve 8), CodeA=1 (ASCII65), CodeE=5, CodeF=6, CodeX=88-64=24, CodeY=89-64=25, CodeZ=90-64=26
  static float values[8]        = { 0 }; // Store found values, send max 7 parameters (send in pairs, so reserve 8)

  uint8_t CAN_tx_buffer[8];              // 8 bytes CAN data TX buffer

  /*
  switch (parser.command_letter) // Filter/adjust Gcodes
  {
    case 'G': Gcode_type = CAN_ID_GCODE_TYPE_G;
      switch (Gcode_no)
      {
        case 12: break; // No Nozzle cleaning support needed on toolhead
        case 29: case 34: return HAL_OK; // No bedleveling/Z-syncing on toolhead
        break;
      }
      break;

    case 'M': Gcode_type = CAN_ID_GCODE_TYPE_M;
      switch (Gcode_no)
      { // Save Prog mem: M112, M48, M85, M105, M114, M155, M500, M501, M502, M503, M226, M422
        case 109: Gcode_no = 104; break;   // Replace M109 with M104
        case 112: Gcode_no = 104; break;   // Don't shutdown board, should stop heating with "M104"

        case  20: case 21: case 22: case 23: case 24: case 25: case 26: case 524: case 540: case 928:
        case  27: case 28: case 29: case 30: case 32: case 33: case 34: // No SD file commands
        case  43:                     // No pin debug
        case  48:                     // No repeatability test
        case  85:                     // No inactivity shutdown
        case 100:                     // No show free memory support
        case 108:                     // Break and Continue
        case 105:                     // No temperature reporting
        case 114:                     // Don't report position
        case 117: case 118: case 119: // Don't send strings
        case 140: case 190:           // Ignore bed temp commands
        case 150:                     // Set NeoPixel values
        case 154:                     // No auto position reporting
        case 155:                     // No tempeature reporting
        case 226:                     // Wait for pin state
        case 240:                     // No camera support
        case 250:                     // No LCD contrast support
        case 260: case 261:           // No I2C on toolhead
        case 280:                     // Don't send servo angle, done via Servo.cpp already
        case 290:                     // No baby stepping
        case 300:                     // No tones
       // case 303:                     // No PID autotune (done on TOOLHEAD)
        case 304:                     // No bed PID settings
       // case 306:                     // MPC autotune (done on TOOLHEAD)
        case 350: case 351:           // No live microstepping adjustment
        case 380: case 381:           // No solenoid support
        case 401: case 402:           // No probe deploy/stow, done via M280 servo angles
        case 412:                     // Filament runout sensor done by MASTER
        case 420: case 421:           // No bed leveling state
        case 423:                     // No X Twist Compensation
        case 425:                     // No backlash compensation
        case 500: case 501: case 502: case 503: case 504: case 910: // No EEPROM on toolhead, remove M50x commands to save Prog mem
        case 510: case 511: case 512: // No locking of the machine
        case 605:                     // No IDEX commands
        case 810: case 811: case 812: case 813: case 814: case 815: case 816: case 817: case 818: case 819:
        case 851:                     //
        case 871:                     // No Probe temp config
        case 876:                     // No handle prompt response
        case 913:                     // No Set Hybrid Threshold Speed
        case 914:                     // No TMC Bump Sensitivity
        case 997:                     // No remote reset
        case 998:                     // No ESP3D reset
        return HAL_OK;                // NO CAN MESSAGE
      }
    break;

    case 'T': Gcode_type = CAN_ID_GCODE_TYPE_T;
      switch (Gcode_no)
      {
        case 0: case 1:
        break;
      }
    break;

    case 'D': Gcode_type = CAN_ID_GCODE_TYPE_D;
      switch (Gcode_no)
      {
        case 0: case 1:
        break;
      }
    break;
    default: return HAL_OK; // Invalid command, nothing to do
  }
  */

  #if ENABLED(CAN_DEBUG)
    SERIAL_ECHOPGM(">>> CAN Gcode to toolhead: ");
    SERIAL_CHAR(parser.command_letter);
    SERIAL_ECHO(Gcode_no);
  #endif

  if (strlen(parser.command_ptr) > 4) // "M107\0", Only scan for parameters if the string is long enough
  for (index = 0; index < sizeof(letters); index++) { // Scan parameters
    if (parser.seen(letters[index])) {
      parameters[parameter_counter] = (letters[index] - 64) & CAN_ID_PARAMETER_LETTER_MASK; // Store parameter letter, A=1, B=2...

      #if ENABLED(CAN_DEBUG)
        SERIAL_CHAR(' ', letters[index]);
      #endif

      if (parser.has_value()) { // Check if there is a value
        values[parameter_counter++] = parser.value_float();

        #if ENABLED(CAN_DEBUG)
          if (values[parameter_counter - 1] == int(values[parameter_counter - 1]))
            SERIAL_ECHO(i16tostr3left(values[parameter_counter - 1])); // Integer value
          else
            SERIAL_ECHO(p_float_t(values[parameter_counter - 1], 4));  // Float with 4 digits
        #endif
      }
      else // No value for parameter
        values[parameter_counter++] = NAN; // Not A Number, indicates no parameter value is present
    }

    if (parameter_counter == 8) { // Max is 7 parameters
      CAN_host_error_code |= CAN_ERROR_HOST_INVALID_GCODE;
      parameter_counter--;
      SERIAL_ECHOLNPGM("\nError: TOO MANY PARAMETERS (> 7): ", parser.command_ptr);
      BUZZ(1, SOUND_ERROR);
      break;
    }
  }

  #if ENABLED(CAN_DEBUG)
    SERIAL_EOL();
  #endif

  parameters[parameter_counter] = 0; // Set next parameter to 0 (0=no parameter), send in pairs
  index = 0;
  float * fp = (float *)CAN_tx_buffer; // Points to TX buffer

  TxHeader.IdType = FDCAN_EXTENDED_ID; // Start Gcode with Extended ID, then send Standard ID messages messages if there are more than 2 parameters

  uint32_t deadline = millis() + CAN_HOST_MAX_WAIT_TIME; // Record message send start time
  do {
    TxHeader.DataLength = MIN(8, (parameter_counter - index) << 2) << FDCAN_HOST_DATALENGTH_OFFSET; // 4 bytes --> only 1 parameter, 8 bytes --> 2 parameters

    if (TxHeader.IdType == FDCAN_EXTENDED_ID)
      TxHeader.Identifier = CAN_set_extended_id(Gcode_type, Gcode_no, parameters[index], parameters[index + 1], parameter_counter);
    else {
      CAN_host_FIFO_toggle_bit = !CAN_host_FIFO_toggle_bit;
      TxHeader.Identifier = (CAN_host_FIFO_toggle_bit ? STDID_FIFO_TOGGLE_BIT : 0) |      // Toggle bit
                       (parameters[index    ] << CAN_ID_PARAMETER1_BIT_POS) | // Parameter 1
                       (parameters[index + 1] << CAN_ID_PARAMETER2_BIT_POS);  // Parameter 2
    }

    *fp++ = values[index++]; // Copy first parameter value to data, move pointer to next 4 bytes
    *fp-- = values[index++]; // Copy 2nd parameter value to data, move pointer to beginning of data array for next round

    while ((HAL_FDCAN_GetTxFifoFreeLevel(&hCAN1) == 0) && PENDING(millis(), deadline)) { /* BLOCKING! Wait for empty TX Buffer */ }

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hCAN1))
      status = HAL_FDCAN_AddMessageToTxFifoQ(&hCAN1, &TxHeader, CAN_tx_buffer); // Queue CAN message
    else
      CAN_host_error_code |= CAN_ERROR_HOST_TX_MSG_DROPPED;

    if (status != HAL_OK) return status;

    TxHeader.IdType = FDCAN_STANDARD_ID; // All following messages have standard ID for parameter values, 11 bits identifier

  } while (index < parameter_counter);

  return status;

} // CAN_host_send_gcode

void CAN_host_send_position() { // Send the X, Y, Z and E position to the TOOLHEAD
  CAN_host_send_gcode_2params('G', 92, 'X', current_position.x, 'Y', current_position.y); // M92 X<pos> Y<pos>
  CAN_host_send_gcode_2params('G', 92, 'Z', current_position.z, 'E', current_position.e); // M92 E<pos> Z<pos>
}

// Enable a GPIO clock based on the GPIOx address for STM32H7
void gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t pos = ((uint32_t)regs - D3_AHB1PERIPH_BASE) >> 10;
    RCC->AHB4ENR |= (1 << pos);
    RCC->AHB4ENR;
}

// TODO: SETUP HARDWARE BASED ON CAN_RD_PIN, CAN_TD_PIN PINS
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* canHandle) { // Called by HAL_FDCAN_Init

   __HAL_RCC_FDCAN_CLK_ENABLE(); // Enable FDCAN1/2 clock

  // Use some macros to find the required setup info based on the provided CAN pins
  uint32_t _CAN_RD_pin = digitalPinToPinName(CAN_RD_PIN);
  uint32_t _CAN_TD_pin = digitalPinToPinName(CAN_TD_PIN);
  uint32_t _CAN_RD_function = pinmap_find_function(digitalPinToPinName(CAN_RD_PIN), PinMap_CAN_RD);
  uint32_t _CAN_TD_function = pinmap_find_function(digitalPinToPinName(CAN_TD_PIN), PinMap_CAN_TD);

  // Enable the GPIOx device related to the CAN_RD_pin
  gpio_clock_enable(get_GPIO_Port(STM_PORT(_CAN_RD_pin)));

  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull      = STM_PIN_PUPD(_CAN_RD_function);

  // SETUP CAN_RD_PIN
  GPIO_InitStruct.Pin       = STM_GPIO_PIN(STM_PIN(_CAN_RD_pin));
  GPIO_InitStruct.Alternate = STM_PIN_AFNUM(_CAN_RD_function);
  HAL_GPIO_Init(get_GPIO_Port(STM_PORT(_CAN_RD_pin)), &GPIO_InitStruct);

  // SETUP CAN_TD_PIN (Separated for flexibility, might not be needed)
  // Enable the GPIOx device related to the CAN_TD_pin
  gpio_clock_enable(get_GPIO_Port(STM_PORT(_CAN_TD_pin)));
  GPIO_InitStruct.Pin       = STM_GPIO_PIN(STM_PIN(_CAN_TD_pin));
  GPIO_InitStruct.Alternate = STM_PIN_AFNUM(_CAN_TD_function);
  HAL_GPIO_Init(get_GPIO_Port(STM_PORT(_CAN_TD_pin)), &GPIO_InitStruct);

  // Enabled the FDCAN interrupt handler for FDCAN1 or FDCAN2
  HAL_NVIC_SetPriority((canHandle->Instance == FDCAN1) ? FDCAN1_IT0_IRQn : FDCAN2_IT0_IRQn, 1, 1); // Set FDCAN interrupt priority
  HAL_NVIC_EnableIRQ(  (canHandle->Instance == FDCAN1) ? FDCAN1_IT0_IRQn : FDCAN2_IT0_IRQn);       // Enable FDCAN interrupt handler line 0 (default)
}

// Calculate the CAN sample timing, seg1 and seg2, sjw = 1 (no baudrate switching)
// seg1 range: 2-256, seg2 range: 1-128, SJW = 1, so minimum is 4 clocks per bit
int FDCAN_calculate_segments(uint32_t *seg1, uint32_t *seg2) {

  uint32_t CAN_clock =  HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

  float clocks_per_bit = CAN_clock / CAN_BAUDRATE; // Clocks per bit must be a whole number

  if ((clocks_per_bit != int(clocks_per_bit)) || (clocks_per_bit < 4)) // Minimal 4 clocks per bit (1+2+1)
    return -1; // Baudrate is not possible

 *seg2 = (clocks_per_bit / 8) + 0.5;  // Preferred sample point at 87.5% (7/8)
 *seg1 = uint32_t(clocks_per_bit) - *seg2 - 1; // SJW = 1;

  return HAL_OK;
}

HAL_StatusTypeDef CAN_host_stop() {
  return HAL_FDCAN_Stop(&hCAN1);
}

HAL_StatusTypeDef CAN_host_start() {

  HAL_StatusTypeDef status = HAL_OK;

// The FDCAN clock source must to be set early because the sample timing depends on it
    __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_HSE);  // 25MHz, select external crystal clock oscillator
//  __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PLL);  // 55MHz, select PLL
//  __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PLL2); // 80MHz, select PLL2

  // Initialize TxHeader with constant values
  TxHeader.FDFormat              = FDCAN_CLASSIC_CAN;   // FDCAN_CLASSIC_CAN / FDCAN_FD_CAN
  TxHeader.TxEventFifoControl    = FDCAN_NO_TX_EVENTS;  // FDCAN_NO_TX_EVENTS / FDCAN_STORE_TX_EVENTS
  TxHeader.MessageMarker         = 0;                   // 0-0xFF for tracking messages in FIFO buffer
  TxHeader.ErrorStateIndicator   = FDCAN_ESI_ACTIVE;    // FDCAN_ESI_PASSIVE / FDCAN_ESI_ACTIVE (ACTIVE will notify transmission errors)
  TxHeader.TxFrameType           = FDCAN_DATA_FRAME;    // FDCAN_DATA_FRAME / FDCAN_REMOTE_FRAME
  TxHeader.IdType                = FDCAN_STANDARD_ID;   // FDCAN_STANDARD_ID / FDCAN_EXTENDED_ID
  TxHeader.BitRateSwitch         = FDCAN_BRS_OFF;       // FDCAN_BRS_OFF / FDCAN_BRS_ON
  TxHeader.TxEventFifoControl    = FDCAN_NO_TX_EVENTS;  // FDCAN_NO_TX_EVENTS / FDCAN_STORE_TX_EVENTS (Do not store TX events)

  hCAN1.Instance                 = FDCAN1;              // The FDCAN device to use
  hCAN1.Init.FrameFormat         = FDCAN_FRAME_CLASSIC; // FDCAN_FRAME_CLASSIC / FDCAN_FRAME_FD_BRS / FDCAN_FRAME_FD_NO_BRS (Bit Rate Switching)
  hCAN1.Init.Mode                = FDCAN_MODE_NORMAL;   // FDCAN_MODE_NORMAL / FDCAN_MODE_EXTERNAL_LOOPBACK / FDCAN_MODE_INTERNAL_LOOPBACK / FDCAN_MODE_BUS_MONITORING / FDCAN_MODE_RESTRICTED_OPERATION
  hCAN1.Init.AutoRetransmission  = DISABLE;             // Auto Retransmission of message if error occured, warning: can cause lockup
  hCAN1.Init.TransmitPause       = DISABLE;             // Transmit Pause to allow for other device to send message
  hCAN1.Init.ProtocolException   = DISABLE;             // ProtocolException handling

  // FDCAN baud rate = FDCAN Clock / Prescaler / (SJW + TSG1 + TSG2)
  // Sample-Point from 50 to 90% (87.5 % is the preferred value used by CANopen and DeviceNet, 75% is the ARINC 825 default)
  // STM32H7xx @ 550MHz, FDCAN clock: HSE=25MHz, PLL=55Mhz, PLL2=80MHz
  // Baud rate = 25M / 1 / 1 / (1 + 21 + 3) = 1M baud (Sample point = 22/25 = 88%)
  // Baud rate = 25M / 1 / 2 / (1 + 21 + 3) = 500k baud
  // HSE  25MHz: ( 1  1  21   3) --> 1M baud; (  2  1 21  3) --> 500K baud; ( 4  1 21  3) --> 250K baud; 
  // PLL  55MHz: (11  1   3   1) --> 1M baud; ( 22  1  3  1) --> 500K baud; (44  1  3  1) --> 250K baud; 
  // PLL2 80MHz: ( 1  1  69  10) ( 2  1  34  5) ( 4  1  16 3) --> 1M baud
  // PLL2 80MHz: ( 2  1  69  10) ( 4  1  34  5) ( 8  1  16 3) --> 500K baud
  // PLL2 80MHz: ( 4  1  69  10) ( 8  1  34  5) (16  1  16 3) --> 250K baud

  uint32_t seg1, seg2;
  if (FDCAN_calculate_segments(&seg1, &seg2) != HAL_OK) {
    SERIAL_ECHOLNPGM("Impossible CAN baudrate, check CAN clock and baudrate");
    return HAL_ERROR;
  }

  hCAN1.Init.NominalPrescaler     =    1; // Arbitration/data clock prescaler/divider (1-512)
  hCAN1.Init.NominalSyncJumpWidth =    1; // Arbitration/data Sync Jump Width (1-128 SJW), should be 1
  hCAN1.Init.NominalTimeSeg1      = seg1; // Arbitration/data period 1 (2-256) // 21
  hCAN1.Init.NominalTimeSeg2      = seg2; // Arbitration/data period 2 (2-128)   //  3

/* No bitrate switching, not used:
  hCAN1.Init.DataPrescaler        =  1; // Arbitration/data clock prescaler/divider (1-32)
  hCAN1.Init.DataSyncJumpWidth    =  1; // Arbitration/data Sync Jump Width (1-16 SJW), should be 1
  hCAN1.Init.DataTimeSeg1         = 21; // Arbitration/data period 1 (2-32)
  hCAN1.Init.DataTimeSeg2         =  3; // Arbitration/data period 2 (1-16)
*/
  hCAN1.Init.MessageRAMOffset     =  0; // Only using 1 FDCAN device, so offset is 0. FDCAN_MESSAGE_RAM_SIZE: 2560 Words, 10KBytes
  hCAN1.Init.TxEventsNbr          =  0; // 0-32
  hCAN1.Init.TxBuffersNbr         =  0; // 0-32
  hCAN1.Init.TxFifoQueueElmtsNbr  =  FDCAN_HOST_TX_FIFO_DEPTH; // 0-32
  hCAN1.Init.TxElmtSize           =  FDCAN_DATA_BYTES_8;

  hCAN1.Init.RxBufferSize         =  FDCAN_DATA_BYTES_8; // FDCAN_data_field_size
  hCAN1.Init.RxBuffersNbr         =  0; // 0-64
  hCAN1.Init.RxFifo0ElmtsNbr      =  FDCAN_HOST_RX_FIFO_DEPTH; // 0-64
  hCAN1.Init.RxFifo0ElmtSize      =  FDCAN_DATA_BYTES_8;
  hCAN1.Init.RxFifo1ElmtsNbr      =  FDCAN_HOST_RX_FIFO_DEPTH; // 0-64
  hCAN1.Init.RxFifo1ElmtSize      =  FDCAN_DATA_BYTES_8;

  hCAN1.Init.StdFiltersNbr        =  2; // Number of standard frame filters
  hCAN1.Init.ExtFiltersNbr        =  2; // Number of extended frame filters
  hCAN1.Init.TxFifoQueueMode      =  FDCAN_TX_FIFO_OPERATION; // Queue mode: FDCAN_TX_FIFO_OPERATION / FDCAN_TX_QUEUE_OPERATION

  status = HAL_FDCAN_Init(&hCAN1); // Calls HAL_FDCAN_MspInit
  if (status != HAL_OK) return status;

  // FDCAN Filter configuration, toggle reception between FIFO0 and FIFO1 (not really needed for this FDCAN implementation)
  FDCAN_FilterTypeDef  sFilterConfig; // Configure RX message filter

  sFilterConfig.IdType       = FDCAN_EXTENDED_ID; // Filter extended ID messages to FIFO1 if higest ID bit is set
  sFilterConfig.FilterIndex  = 0;                 // Exteneded filter ID 0
  sFilterConfig.FilterType   = FDCAN_FILTER_MASK; // FDCAN_FILTER_MASK / FDCAN_FILTER_RANGE
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; // Filter to FIFO1
  sFilterConfig.FilterID1    = 0x10000000;        // Range: 0 - 0x1FFF FFFF, 0=don'tcare
  sFilterConfig.FilterID2    = 0x10000000;        // Range: 0 - 0x1FFF FFFF, 0=don'tcare
  status = HAL_FDCAN_ConfigFilter(&hCAN1, &sFilterConfig);
  if (status != HAL_OK) return status;
  
//sFilterConfig.IdType       = FDCAN_EXTENDED_ID; // Filter all remaining extended ID messages to FIFO0
  sFilterConfig.FilterIndex  = 1;                 // Exteneded filter ID 1
//sFilterConfig.FilterType   = FDCAN_FILTER_MASK; // FDCAN_FILTER_MASK / FDCAN_FILTER_RANGE
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Remaining messages go to FIFO0
  sFilterConfig.FilterID1    = 0;                 // Range: 0 - 0x1FFF FFFF, 0=don'tcare
  sFilterConfig.FilterID2    = 0;                 // Range: 0 - 0x1FFF FFFF, 0=don'tcare
  HAL_FDCAN_ConfigFilter(&hCAN1, &sFilterConfig);
  if (status != HAL_OK) return status;

  sFilterConfig.IdType       = FDCAN_STANDARD_ID; // Filter to FIFO1 if higest standard ID bit is set
  sFilterConfig.FilterIndex  = 0;                 // Standard filter ID 0
  sFilterConfig.FilterType   = FDCAN_FILTER_MASK; // FDCAN_FILTER_MASK / FDCAN_FILTER_RANGE
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1    = 0b10000000000;     // Range: 0 - 0x7FF, 0=don't care
  sFilterConfig.FilterID2    = 0b10000000000;     // Range: 0 - 0x7FF, 0=don't care
  status = HAL_FDCAN_ConfigFilter(&hCAN1, &sFilterConfig);
  if (status != HAL_OK) return status;

//sFilterConfig.IdType       = FDCAN_STANDARD_ID; // Filter all remaining standard ID messages to FIFO0
  sFilterConfig.FilterIndex  = 1;                 // Standard filter ID 1
//sFilterConfig.FilterType   = FDCAN_FILTER_MASK; // FDCAN_FILTER_MASK / FDCAN_FILTER_RANGE
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; // Remaining to FIFO0
  sFilterConfig.FilterID1    = 0;                 // Range: 0 - 0x7FF, 0=don't care
  sFilterConfig.FilterID2    = 0;                 // Range: 0 - 0x7FF, 0=don't care
  status = HAL_FDCAN_ConfigFilter(&hCAN1, &sFilterConfig);
  if (status != HAL_OK) return status;

  // status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  //if (status != HAL_OK) return status;

  status = HAL_FDCAN_ActivateNotification(&hCAN1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // Calls HAL_FDCAN_RxFifo0MsgPendingCallback
  if (status != HAL_OK) return status;

  status = HAL_FDCAN_ActivateNotification(&hCAN1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0); // Calls HAL_FDCAN_RxFifo1MsgPendingCallback
  if (status != HAL_OK) return status;

  HAL_FDCAN_ActivateNotification(&hCAN1, FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0); // Calls HAL_FDCAN_ErrorCallback
  if (status != HAL_OK) return status;

  status = HAL_FDCAN_ActivateNotification(&hCAN1, FDCAN_IT_RX_FIFO1_MESSAGE_LOST, 0); // Calls HAL_FDCAN_ErrorCallback
  if (status != HAL_OK) return status;

  status = HAL_FDCAN_ActivateNotification(&hCAN1, FDCAN_IT_ERROR_WARNING, 0); // Calls HAL_FDCAN_ErrorCallback
  if (status != HAL_OK) return status;

  status = HAL_FDCAN_Start(&hCAN1); // Start the FDCAN device
  if (status != HAL_OK) return status;

  #if ENABLED(CAN_DEBUG)
    SERIAL_ECHOLNPGM("Voltage Scaling: VOS", (PWR->CSR1 & PWR_CSR1_ACTVOS_Msk) >> PWR_CSR1_ACTVOS_Pos);
    SERIAL_ECHOLNPGM("FDCAN Peripheral Clock : ", HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) / 1000000, "MHz");
    SERIAL_ECHOLNPGM("FDCAN Timing. Prescaler: ", hCAN1.Init.NominalPrescaler, "  SJW: ", hCAN1.Init.NominalSyncJumpWidth, "  SEG1: ", hCAN1.Init.NominalTimeSeg1, "  SEG2: ", hCAN1.Init.NominalTimeSeg2);
    uint32_t baudrate = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) / hCAN1.Init.NominalPrescaler / (hCAN1.Init.NominalSyncJumpWidth + hCAN1.Init.NominalTimeSeg1 + hCAN1.Init.NominalTimeSeg2);
    SERIAL_ECHOLNPGM("FDCAN BAUDRATE: ", baudrate);

    if (baudrate != CAN_BAUDRATE) {
      SERIAL_ECHOLNPGM(">>> Host ", CAN_ERROR_MSG_INVALID_BAUDRATE, ": ", baudrate, "  CAN_BAUDRATE=", CAN_BAUDRATE);
      CAN_host_error_code |= CAN_ERROR_HOST_INVALID_BAUDRATE;
    }
  #endif

  #ifdef CAN_LED_PIN
    pinMode(CAN_LED_PIN, OUTPUT);
  #endif

  status = CAN_host_send_gcode_2params('M', 997, 0, 0, 0, 0); // M997, reset toolhead at host startup

  return status;
} // CAN_host_start()

void FDCAN_host_read_message(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo) { // ISR! FIFO 0/1 message interrupt handler

  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t CAN_RX_buffer_FIFO[8]; // CAN message buffer

  if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, RxFifo) &&
     (HAL_FDCAN_GetRxMessage(hfdcan, RxFifo, &RxHeader, CAN_RX_buffer_FIFO) == HAL_OK)) { // Get message from CAN_RX_FIFO0

    if ((RxHeader.Identifier & CAN_ID_IO_MASK) != CAN_io_state) { // First handle time critical virtual IO update
      CAN_io_state = (RxHeader.Identifier & CAN_ID_IO_MASK);
      endstops.update();
    }

    if (RxHeader.Identifier & CAN_ID_STRING_MESSAGE_BIT_MASK) { // Toolhead sent a string message
      char *CAN_RX_p = (char *)CAN_RX_buffer_FIFO;
      for (uint32_t i = 0; i < (RxHeader.DataLength >> FDCAN_HOST_DATALENGTH_OFFSET); i++)
        string_message.append(CAN_RX_p[i]); // Copy message to global buffer

      if (CAN_RX_p[(RxHeader.DataLength >> FDCAN_HOST_DATALENGTH_OFFSET) - 1] == '\n')
        string_message_complete = true; // String is complete, idle task can show the string
    }
    else if (RxHeader.DataLength == FDCAN_DLC_BYTES_4) { // Only 1 record, so it's a temperature update (DLC = Data Length Code == 4 bytes)
      float *fp = (float *)CAN_RX_buffer_FIFO;
      thermalManager.temp_hotend[0].celsius = *fp; // Set E0 hotend temperature from received message
      CAN_next_temp_report_time = millis() + CAN_HOST_E0_TEMP_UPDATE_WATCHDOG_TIME; // A temp update must be received within this window
      first_E0_error = true;                       // Reset error status
    }

    if (RxHeader.Identifier & CAN_ID_REQUEST_TIME_SYNC_BIT_MASK) { // Toolhead signals request for time stamp
      time_sync_request_time = micros(); // Record the time sync request receive time
      CAN_time_sync_request = true;
    }

    CAN_toolhead_setup_request = (RxHeader.Identifier & CAN_ID_REQUEST_SETUP_BIT_MASK) > 0; // Toolhead requests setup configuration

    CAN_toolhead_error = (RxHeader.Identifier & CAN_ID_ERROR_BIT_MASK) > 0; // Toolhead signals an error
  }
}

void FDCAN1_IT0_IRQHandler() { // ISR! FDCAN line 0 interrupt handler (overrides weak function)

  #ifdef FDCAN_LED_PIN
    pinMode(FDCAN_LED_PIN, OUTPUT);
    TOGGLE(FDCAN_LED_PIN);
  #endif 

  HAL_FDCAN_IRQHandler(&hCAN1); // Forward the interrupt call to the FDCAN interrupt handler for callbacks

}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) { // ISR! CAN FIFO0 new message interrupt handler
 
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) { // Check the error status first, it might be cleared by reading a message
    CAN_host_error_code |= CAN_ERROR_HOST_RX_FIFO_OVERFLOW;
    __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_IT_RX_FIFO0_MESSAGE_LOST); // Clear interrupt flag
  }

  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    FDCAN_host_read_message(hfdcan, FDCAN_RX_FIFO0); // Forward call

}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) { // ISR! CAN FIFO1 new message interrupt handler

  if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) { // Check the error status first, it might be cleared by reading a message
    CAN_host_error_code |= CAN_ERROR_HOST_RX_FIFO_OVERFLOW;
    __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_IT_RX_FIFO1_MESSAGE_LOST); // Clear interrupt flag
  }

  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE))
    FDCAN_host_read_message(hfdcan, FDCAN_RX_FIFO1); // Forward call
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) { // ISR!

  // FDCAN_IR_EP | FDCAN_IR_EW | FDCAN_IR_BO  ERROR PASSIVE, WARNINGS, BUS OFF
  HAL_FDCAN_error_code = hfdcan->ErrorCode; // Store the received FDCAN error code
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) { // ISR! FDCAN error interrupt handler
// FDCAN_IR_ELO | FDCAN_IR_WDI | FDCAN_IR_PEA | FDCAN_IR_PED | FDCAN_IR_ARA    LOGGING OVERFLOW, WATCHDOG INTERRUPT, PROTOCOL ERRORS, ACCESS RESERVED AREA
  HAL_FDCAN_error_code = hfdcan->ErrorCode; // Store the received FDCAN error code
}

#endif // CAN_HOST && STM32H7xx
