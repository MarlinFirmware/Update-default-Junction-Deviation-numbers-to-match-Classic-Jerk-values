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
 * NOTE 1: For MKS Monster 8 V1/V2 on Arduino use: Board "Generic STM32F4 series", Board part number "Generic F407VETx"
 * NOTE 2: Requires `HAL_CAN_MODULE_ENABLED`, e.g., with `-DHAL_CAN_MODULE_ENABLED`
 *         For Arduino IDE use "hal_conf_extra.h" with `#define HAL_CAN_MODULE_ENABLED`
 * NOTE 3: To accept all CAN messages, enable 1 filter (FilterBank = 0) in "FilterMode = CAN_FILTERMODE_IDMASK", mask and ID = 0 (0=don't care)
 * NOTE 4: Serial communication in ISR causes issues! Hangs etc. so avoid!
 * NOTE 5: A FIFO storage cell is called a "Mailbox" in STM32F4xx, FIFO0 and FIFO1 can (onlyd)hold 3 CAN messages each.
 * NOTE 6: The filter ID/mask numbers (LOW/HIGH) do not directly relate to the message ID numbers (See Figure 342 in RM0090)
 */

#include "../../inc/MarlinConfigPre.h"

#if ALL(CAN_HOST, STM32F4xx)

#include "../platforms.h"
#include "../../gcode/parser.h"
#include "../../module/temperature.h"
#include "../../module/motion.h"  // For current_position variable
#include "../../module/planner.h" // For steps/mm parameters variables
#include "../../feature/tmc_util.h"
#include "../../module/endstops.h"
#include "../../feature/controllerfan.h" // For controllerFan settings
#include "../../libs/numtostr.h"  // For float to string conversion

#include "../shared/CAN_host.h"

// Interrupt handlers controlled by the CAN_IER register
extern "C" void CAN1_RX0_IRQHandler(void);                                  // CAN1 FIFO0 interrupt handler (new message, full, overrun)
extern "C" void CAN1_RX1_IRQHandler(void);                                  // CAN1 FIFO1 interrupt handler (new message, full, overrun)
extern "C" void CAN1_SCE_IRQHandler(void);                                  // CAN1 status change interrupt handler

extern "C" void CAN2_RX0_IRQHandler(void);                                  // CAN2 FIFO0 interrupt handler (new message, full, overrun)
extern "C" void CAN2_RX1_IRQHandler(void);                                  // CAN2 FIFO1 interrupt handler (new message, full, overrun)
extern "C" void CAN2_SCE_IRQHandler(void);                                  // CAN2 status change error interrupt handler (See CAN_ESR/CAN_MSR registers)

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan); // CAN FIFO0 new message callback
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan); // CAN FIFO1 new message callback
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);             // CAN error interrupt callback

//#define CAN_LED_PIN   PC8

#ifndef CAN_BAUDRATE
  #define CAN_BAUDRATE 1000000
#endif

#if (CAN_BAUDRATE != 1000000) && (CAN_BAUDRATE != 500000) && (CAN_BAUDRATE != 250000) && (CAN_BAUDRATE != 125000)
  #error ERROR: Select a valid CAN_BAUDRATE: 1000000, 500000, 250000 or 125000 baud
#endif

#define STDID_FIFO_TOGGLE_BIT                 0b10000000000
#define EXTID_FIFO_TOGGLE_BIT                    0x10000000

#define CAN_HOST_FIFO_DEPTH                               3 // RX and TX FIFO0 and FIFO1 depth

CAN_HandleTypeDef hCAN1                            = { 0 }; // The global CAN handle
CAN_TxHeaderTypeDef TxHeader                       = { 0 }; // Header to send a CAN message
volatile uint32_t CAN_io_state                         = 0; // Virtual IO state variable
volatile bool CAN_toolhead_error                       = 0; // Register if an error was reported by the toolhead
volatile bool CAN_toolhead_setup_request           = false; // Signals the toolhead requested setup information
volatile bool CAN_time_sync_request                = false; // Signals the toolhead requested a time sync
volatile uint32_t time_sync_request_time               = 0; // Record the time the time sync request was received

volatile uint32_t HAL_CAN_error_code                   = 0; // Store the HAL CAN error code
volatile uint32_t CAN_host_error_code                  = 0; // Store the CAN host error code

volatile bool first_E0_error                        = true; // First CAN bus error, show warning only once
volatile bool string_message_complete              = false; // Signals a complete string message was received
volatile uint32_t string_message_index                 = 0; // Index into the CAN string that is being received
uint32_t CAN_next_temp_report_time                 = 12000; // Track when the next toolhead temperature report should have arrived
uint32_t CAN_next_error_message_time                   = 0; // Track when to display the next repeat of an error message
volatile bool CAN_host_FIFO_toggle_bit             = false; // FIFO toggle flag for receiver FIFO filtering
char string_message[CAN_HOST_MAX_STRING_MSG_LENGTH] = "\0"; // CAN string message buffer for incoming messages

uint32_t CAN_host_get_iostate() {
  return CAN_io_state;
}

uint32_t CAN_set_extended_id(int gcode_type, int gcode_no, int parameter1, int parameter2, int count) {

  CAN_host_FIFO_toggle_bit = !CAN_host_FIFO_toggle_bit;                                    // FIFO toggle bit

  return (CAN_host_FIFO_toggle_bit ? EXTID_FIFO_TOGGLE_BIT : 0)                      | // FIFO toggle bit
         (count << CAN_ID_PARAMETER_COUNT_BIT_POS)                                   | // Parameter count
         (gcode_type << CAN_ID_GCODE_TYPE_BIT_POS)                                   | // G/M/T/D-code
         ((gcode_no & CAN_ID_GCODE_NUMBER_MASK) << CAN_ID_GCODE_NUMBER_BIT_POS)      | // Gcode number
         ((parameter1 & CAN_ID_PARAMETER_LETTER_MASK) << CAN_ID_PARAMETER1_BIT_POS)  | // First parameter
         ((parameter2 & CAN_ID_PARAMETER_LETTER_MASK) << CAN_ID_PARAMETER2_BIT_POS);   // Second parameter

}

// Send time sync timestamp of arrival and response time
void CAN_host_send_timestamp() { // Request receive timestamp + request response timestamp

  uint32_t TxMailbox;  // Stores which Mailbox (0-2) was used to store the sent message

  TxHeader.IDE   = CAN_ID_EXT;
  TxHeader.DLC   = 8; // Send sync time t1(receive time, uint32_t) and t2(response time, uint32_t)
  TxHeader.ExtId = CAN_set_extended_id(CAN_ID_GCODE_TYPE_M, CAN_HOST_GCODE_TIME_SYNC_NO, 1, 2, 2);

  uint8_t CAN_tx_buffer[8];                       // 8 bytes CAN data TX buffer
  uint32_t * uint32p = (uint32_t *)CAN_tx_buffer; // Point to TX buffer
  *uint32p++ = time_sync_request_time;

  uint32_t deadline = millis() + CAN_HOST_MAX_WAIT_TIME;
  while ((HAL_CAN_GetTxMailboxesFreeLevel(&hCAN1) < CAN_HOST_FIFO_DEPTH) && PENDING(millis(), deadline)) { /* BLOCKING! Wait for empty TX buffer */ }

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hCAN1)) {
    *uint32p = micros(); // Only record the response time at the last possible moment
    HAL_CAN_AddTxMessage(&hCAN1, &TxHeader, CAN_tx_buffer, &TxMailbox); // Queue CAN message
  }
  else
    CAN_host_error_code |= CAN_ERROR_TX_MSG_DROPPED;

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
        SERIAL_ECHOPGM("; CAN TO TOOLHEAD: \"M", Gcode_no);
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

  TxHeader.IDE   = CAN_ID_EXT;
  TxHeader.DLC   = 4 * (!!parameter1 + !!parameter2); // Amount of bytes to send (4 or 8)

  TxHeader.ExtId = CAN_set_extended_id(Gcode_type, Gcode_no, parameter1, parameter2, TxHeader.DLC >> 2);

  uint8_t CAN_tx_buffer[8];            // 8 bytes CAN data TX buffer
  float * fp = (float *)CAN_tx_buffer; // Point to TX buffer
  *fp++ = value1;
  *fp   = value2;

  uint32_t TxMailbox;  // Stores which Mailbox (0-2) was used to store the sent message
  const uint32_t deadline = millis() + CAN_HOST_MAX_WAIT_TIME;
  while ((HAL_CAN_GetTxMailboxesFreeLevel(&hCAN1) == 0) && PENDING(millis(), deadline)) { /* BLOCKING! Wait for empty TX buffer */ }

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hCAN1))
    status = HAL_CAN_AddTxMessage(&hCAN1, &TxHeader, CAN_tx_buffer, &TxMailbox); // Queue CAN message
  else
    CAN_host_error_code |= CAN_ERROR_TX_MSG_DROPPED;

  return status;
}

void CAN_host_send_setup() { // Send setup to toolhead

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

  // Signals to the toolhead that the configuration is complete, use it as the last Gcode to send
  CAN_host_send_gcode_2params('M', CAN_HOST_CONFIGURATION_COMPLETE, 0, 0, 0, 0);
}

void CAN_host_idle() { // Tasks that can/should not be done in the ISR

  if (CAN_time_sync_request) { // Send time sync timestamps
     CAN_time_sync_request = false;
     CAN_host_send_timestamp();
  }

  if (string_message_complete) { // Received string message is complete, show the string
    BUZZ(1, SOUND_OK);
    SERIAL_ECHOPGM(">>> CAN toolhead MSG: ");

    for (uint32_t i = 0; i < string_message_index; i++)
      SERIAL_CHAR(string_message[i]); // Show received string message, ends on '\n'

    string_message_complete = false; // Get ready for the next string
    string_message_index    = 0;
  }

  // Report time sync results
  if ((hCAN1.ErrorCode || CAN_toolhead_error || HAL_CAN_error_code) && ELAPSED(millis(), CAN_next_error_message_time)) {

    BUZZ(1, SOUND_ERROR);

    if (CAN_toolhead_error) {
      SERIAL_ECHOLNPGM(">>> CAN Error reported by toolhead");
      CAN_toolhead_error = false; // Reset, but will be repeated by the toolhead
    }

    if (HAL_CAN_error_code)
      SERIAL_ECHOLNPGM(">>> HAL CAN Error reported: ", HAL_CAN_error_code);

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
      CAN_host_send_setup();
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
  uint32_t TxMailbox; // Stores which Mailbox (0-2) was used to store the sent message

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
      CAN_host_error_code |= CAN_ERROR_INVALID_GCODE;
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

  TxHeader.IDE = CAN_ID_EXT; // Start Gcode with Extended ID, then send Standard ID messages if there are more than 2 parameters

  uint32_t deadline = millis() + CAN_HOST_MAX_WAIT_TIME; // Record message send start time
  do {
    TxHeader.DLC = MIN(8, (parameter_counter - index) << 2); // Maximum 8 bytes, 4 bytes --> only 1 parameter, 8 bytes --> 2 parameters

    if (TxHeader.IDE == CAN_ID_EXT)
      TxHeader.ExtId = CAN_set_extended_id(Gcode_type, Gcode_no, parameters[index], parameters[index + 1], parameter_counter);
    else {
      CAN_host_FIFO_toggle_bit = !CAN_host_FIFO_toggle_bit;
      TxHeader.StdId = (CAN_host_FIFO_toggle_bit ? STDID_FIFO_TOGGLE_BIT : 0) | // Toggle bit
                       (parameters[index    ] << CAN_ID_PARAMETER1_BIT_POS)   | // Parameter 1
                       (parameters[index + 1] << CAN_ID_PARAMETER2_BIT_POS);    // Parameter 2
    }

    *fp++ = values[index++]; // Copy first parameter value to data, move pointer to next 4 bytes
    *fp-- = values[index++]; // Copy 2nd parameter value to data, move pointer to beginning of data array for next round

    while ((HAL_CAN_GetTxMailboxesFreeLevel(&hCAN1) == 0) && PENDING(millis(), deadline)) { /* BLOCKING! Wait for emtpy TX buffer */ }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hCAN1))
      status = HAL_CAN_AddTxMessage(&hCAN1, &TxHeader, CAN_tx_buffer, &TxMailbox); // Queue CAN message
    else
      CAN_host_error_code |= CAN_ERROR_TX_MSG_DROPPED;

    if (status != HAL_OK) return status;
    
    TxHeader.IDE = CAN_ID_STD; // All following messages have standard ID for parameter values, 11 bits identifier
  } while (index < parameter_counter);

  return status;

} // CAN_host_send_gcode

void CAN_host_send_position() { // Send the X, Y, Z and E position to the TOOLHEAD
  CAN_host_send_gcode_2params('G', 92, 'X', current_position.x, 'Y', current_position.y); // M92 X<pos> Y<pos>
  CAN_host_send_gcode_2params('G', 92, 'Z', current_position.z, 'E', current_position.e); // M92 E<pos> Z<pos>
}

// TODO: SETUP HARDWARE BASED ON CAN_RX, CAN_TX PINS
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle) { // Called by HAL_CAN_Init

  if (canHandle->Instance == CAN1) {
    if (__HAL_RCC_CAN1_IS_CLK_DISABLED())
      __HAL_RCC_CAN1_CLK_ENABLE();         // Enable CAN1 clock

    if (__HAL_RCC_GPIOB_IS_CLK_DISABLED())
      __HAL_RCC_GPIOB_CLK_ENABLE();        // Enable GPIO B clock
    // CAN1 GPIO Configuration
    // PB8     ------> CAN1_RX
    // PB9     ------> CAN1_TX

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9; // Pin PB8 and Pin PB9
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;           // Alternate function 9
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);              // Port B

    // Enable the CAN interrupt handlers
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);                   // Enable CAN1 FIFO0 interrupt handler

    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 1, 1);           // Set CAN interrupt priority
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);                   // Enable CAN1 FIFO1 interrupt handler
  }
}

HAL_StatusTypeDef CAN_host_stop() {
  return HAL_CAN_Stop(&hCAN1);
}

HAL_StatusTypeDef CAN_host_start() {

  HAL_StatusTypeDef status = HAL_OK;

  // Initialize TxHeader with constant values
  TxHeader.ExtId              = 0;
  TxHeader.StdId              = 0;
  TxHeader.RTR                = CAN_RTR_DATA; // Data transmission type: CAN_RTR_DATA / CAN_RTR_REMOTE
  TxHeader.TransmitGlobalTime = DISABLE;      // Put timestamp in Data[6-7], requires Time Triggered Communication Mode

  // CAN peripheral clock is 42MHz (168Mhz / 4)
  // CAN baud rate = clock frequency / clock divider / prescaler / (1 + TSG1 + TSG2)
  // Baud rate = 42M / 3 / 1 / (1 + 11 + 2) = 1M baud (Sample point = 12/14=86%)
  // Baud rate = 42M / 3 / 2 / (1 + 11 + 2) = 500k baud
  // Baud rate = 42M / 3 / 4 / (1 + 11 + 2) = 250k baud
  hCAN1.Instance                  = CAN1;
  hCAN1.Init.Prescaler            = 3;               // 1-1024, 42MHz peripheral clock / 3 --> 14MHz -> 1M baud. 6 --> 500K baud. 12 --> 250K baud.
  hCAN1.Init.AutoBusOff           = DISABLE;         // DISABLE: Software controlled Bus-off. ENABLE: Automatic hardware controlled (no send/receive)
  hCAN1.Init.AutoWakeUp           = ENABLE;          // ENABLE: Automatic hardware controlled bus wakeup. DISABLE: Software controlled bus wakeup.
  hCAN1.Init.AutoRetransmission   = ENABLE;          // DISABLE / ENABLE, resend if transmission failed, but locks up if communication fails/cable not connected!!!!!!!!!!!!!!!!!
  hCAN1.Init.SyncJumpWidth        = CAN_SJW_1TQ;     // CAN_SJW_1TQ  (1-4) Should be 1
  hCAN1.Init.TimeSeg1             = CAN_BS1_11TQ;    // CAN_BS1_11TQ (1-16)
  hCAN1.Init.TimeSeg2             = CAN_BS2_2TQ;     // CAN_BS2_2TQ  (1-8)
  hCAN1.Init.Mode                 = CAN_MODE_NORMAL; // CAN_MODE_NORMAL / CAN_MODE_SILENT / CAN_MODE_LOOPBACK / CAN_MODE_SILENT_LOOPBACK
  hCAN1.Init.TimeTriggeredMode    = DISABLE;         // TTCAN is used to assign timeslot to the devices for real time applications
  hCAN1.Init.ReceiveFifoLocked    = DISABLE;         // Handle RX FIFO overruns. DISABLE: Overwrite previous message with new one. ENABLE: Discard the new message.
  hCAN1.Init.TransmitFifoPriority = ENABLE;          // Handle TX FIFO send order. ENABLE: Chronologically. DISABLE: Transmit lower ID number first.

  status = HAL_CAN_Init(&hCAN1); // Calls HAL_CAN_MspInit
  if (status != HAL_OK) return status;

  CAN_FilterTypeDef  sFilterConfig;

  // Store CAN messags with highest bit of StdId set in FIFO0
  sFilterConfig.FilterBank           = 0; // This filter bank ID number (0-13 for single CAN instances)
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK; // Accept if "Received ID" & Mask = ID, CAN_FILTERMODE_IDMASK / CAN_FILTERMODE_IDLIST (See Figure 342 in RM0090)
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT; // CAN_FILTERSCALE_16BIT / CAN_FILTERSCALE_32BIT (See Figure 342 in RM0090)
  sFilterConfig.FilterIdHigh         = 0b1000000000000000;    // ID MSB:   (0-0xFFFF) (StdId[10-0] [ExtId17-13]) (See Figure 342 in RM0090)
  sFilterConfig.FilterIdLow          = 0;                     // ID LSB:   (0-0xFFFF) ([ExtId12-0][IDE][RTR]  0) (0="don't care")
  sFilterConfig.FilterMaskIdHigh     = 0b1000000000000000;    // Mask MSB: (0-0xFFFF) (StdId[10-0] [ExtId17-13]) (See Figure 342 in RM0090)
  sFilterConfig.FilterMaskIdLow      = 0;                     // Mask LSB: (0-0xFFFF) ([ExtId12-0][IDE][RTR]  0) (0="don't care")
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;      // Store message in FIFO1 (CAN_FILTER_FIFO0 / CAN_FILTER_FIFO1)
  sFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;     // CAN_FILTER_ENABLE / CAN_FILTER_DISABLE
  sFilterConfig.SlaveStartFilterBank = 0;                     // Start bank number for CAN slave instance (not used in single CAN setups)
  status = HAL_CAN_ConfigFilter(&hCAN1, &sFilterConfig);
  if (status != HAL_OK) return status;

  // Store all remaining CAN messages in FIFO1
  sFilterConfig.FilterBank           = 1; // This filter bank ID number (0-13 for single CAN instances)
//sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK; // CAN_FILTERMODE_IDMASK / CAN_FILTERMODE_IDLIST (See Figure 342 in RM0090)
//sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT; // CAN_FILTERSCALE_16BIT / CAN_FILTERSCALE_32BIT (See Figure 342 in RM0090)
  sFilterConfig.FilterIdHigh         = 0;                     // ID MSB:   (0-0xFFFF) (StdId[10-0] [ExtId17-13]) (See Figure 342 in RM0090)
//sFilterConfig.FilterIdLow          = 0;                     // ID LSB:   (0-0xFFFF) ([ExtId12-0][IDE][RTR]  0) (0="don't care")
  sFilterConfig.FilterMaskIdHigh     = 0;                     // Mask MSB: (0-0xFFFF) (StdId[10-0] [ExtId17-13]) (See Figure 342 in RM0090)
//sFilterConfig.FilterMaskIdLow      = 0;                     // Mask LSB: (0-0xFFFF) ([ExtId12-0][IDE][RTR]  0) (0="don't care")
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;      // Store message in FIFO0 (CAN_FILTER_FIFO0 / CAN_FILTER_FIFO1)
//sFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;     // CAN_FILTER_ENABLE / CAN_FILTER_DISABLE
//sFilterConfig.SlaveStartFilterBank = 0;                     // Start bank number for CAN slave instance (not used in single CAN setups)
  status = HAL_CAN_ConfigFilter(&hCAN1, &sFilterConfig);
  if (status != HAL_OK) return status;

  // Activate RX FIFO0/FIFO1 new message interrupt
  status = HAL_CAN_ActivateNotification(&hCAN1, CAN_IT_RX_FIFO0_MSG_PENDING); // Calls CAN1_RX0_IRQHandler / CAN2_RX0_IRQHandler
  if (status != HAL_OK) return status;

  status = HAL_CAN_ActivateNotification(&hCAN1, CAN_IT_RX_FIFO1_MSG_PENDING); // Calls CAN1_RX1_IRQHandler / CAN2_RX0_IRQHandler
  if (status != HAL_OK) return status;

  // Activate RX FIFO0/FIFO1 overrun interrupt
  status = HAL_CAN_ActivateNotification(&hCAN1, CAN_IT_RX_FIFO0_OVERRUN); // Calls CAN1_RX0_IRQHandler / CAN2_RX0_IRQHandler
  if (status != HAL_OK) return status;

  status = HAL_CAN_ActivateNotification(&hCAN1, CAN_IT_RX_FIFO1_OVERRUN); // Calls CAN1_RX1_IRQHandler / CAN2_RX0_IRQHandler
  if (status != HAL_OK) return status;

  status = HAL_CAN_ActivateNotification(&hCAN1, CAN_IT_ERROR); // Calls CAN1_RX0_IRQHandler / CAN2_RX0_IRQHandler
  if (status != HAL_OK) return status;

  status = HAL_CAN_Start(&hCAN1);   // Start the CAN module
  if (status != HAL_OK) return status;

  #ifdef CAN_LED_PIN
    pinMode(CAN_LED_PIN, OUTPUT);
  #endif

  #ifdef FDCAN_LED_PIN
    pinMode(FDCAN_LED_PIN, OUTPUT);
  #endif
  status = CAN_host_send_gcode_2params('M', 997, 0, 0, 0, 0); // M997, reset toolhead at host startup

  return status;
}// CAN_host_start()

void CAN_host_read_message(CAN_HandleTypeDef *hcan, uint32_t RxFifo) { // ISR! FIFO 0/1 CAN message interrupt handler

  CAN_RxHeaderTypeDef RxHeader;
  uint8_t CAN_RX_buffer_FIFO[8]; // CAN message buffer

  if (HAL_CAN_GetRxFifoFillLevel(hcan, RxFifo) &&
     (HAL_CAN_GetRxMessage(hcan, RxFifo, &RxHeader, CAN_RX_buffer_FIFO) == HAL_OK)) {

    if ((RxHeader.StdId & CAN_ID_IO_MASK) != CAN_io_state) { // First handle time critical virtual IO update
      CAN_io_state = (RxHeader.StdId & CAN_ID_IO_MASK);
      endstops.update();
    }

    if (RxHeader.StdId & CAN_ID_STRING_MESSAGE_MASK) { // Toolhead sends a string message
      char * CAN_RX_p = (char *)CAN_RX_buffer_FIFO;
      for (uint32_t i = 0; i < RxHeader.DLC; i++) {
        string_message[string_message_index++ % CAN_HOST_MAX_STRING_MSG_LENGTH] = CAN_RX_p[i]; // Copy message to global string buffer

        if (CAN_RX_p[i] == '\n') {
          string_message_complete = true; // String is complete, idle task can show the string
          string_message[string_message_index % CAN_HOST_MAX_STRING_MSG_LENGTH] = 0; // Close string with \0
        }
      }
    }
    else if (RxHeader.DLC == 4) { // Only 1 record, so it's a temperature update (DLC = Data Length Code is 4 bytes)
      float *fp = (float *)CAN_RX_buffer_FIFO;
      thermalManager.temp_hotend[0].celsius = *fp; // Set E0 hotend temperature from received message
      CAN_next_temp_report_time = millis() + CAN_HOST_E0_TEMP_UPDATE_WATCHDOG_TIME; // A temp update must be received within this window
      first_E0_error = true;                       // Reset error status
    }

    if (RxHeader.StdId & CAN_ID_REQUEST_TIME_SYNC_MASK) { // Toolhead signals request for time stamp
      time_sync_request_time = micros(); // Record the time sync request receive time
      CAN_time_sync_request = true;
    }

    CAN_toolhead_setup_request = (RxHeader.StdId & CAN_ID_REQUEST_SETUP_MASK) > 0; // Toolhead requests setup configuration

    CAN_toolhead_error = (RxHeader.StdId & CAN_ID_ERROR_MASK) > 0; // Toolhead signals an error
  }
}

void CAN1_RX0_IRQHandler() { // ISR! CAN FIFO0 interrupt handler (overrides weak function)

  HAL_CAN_IRQHandler(&hCAN1); // Forward call for callbacks --> HAL_CAN_RxFifo0MsgPendingCallback/HAL_CAN_ErrorCallback
  // OR
  //HAL_CAN_RxFifo0MsgPendingCallback(&hCAN1); // Call the required callback directly, faster but no error reporting
}

void CAN1_RX1_IRQHandler() { // ISR! CAN FIFO1 Interrupt handler (overrides weak function)

  HAL_CAN_IRQHandler(&hCAN1); // Forward call for callbacks --> HAL_CAN_RxFifo1MsgPendingCallback/HAL_CAN_ErrorCallback
  // OR
  //HAL_CAN_RxFifo1MsgPendingCallback(&hCAN1); // Call the required callback directly, faster but no error reporting
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) { // ISR! New FIFO0 message interrupt handler
  CAN_host_read_message(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) { // ISR! New FIFO1 message interrupt handler
  CAN_host_read_message(hcan, CAN_RX_FIFO1);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) { // ISR! Interrupt handler for any CAN error
  HAL_CAN_error_code = hcan->ErrorCode; // Store the received error code
}

#endif // CAN_HOST && STM32F4xx 
