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
 * ===============
 * In \Users\<username>\.platformio\packages\framework-arduinoststm32@4.20600.231001\libraries\SrcWrapper\src\HardwareTimer.cpp
 * Add "__weak" in front of "void TIM16_IRQHandler(void)" to be able to override the function and redirect the interrupt
 */

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(CAN_TOOLHEAD)
// TODO:
// 1. Convert into class
// 2. Pickup correct CAN device from CAN_RX_PIN/CAN_TX_PIN
// 3. Calculate baudrate and data sampling timing from clock frequencies defined by CAN_BAUDRATE

#include "../platforms.h"
#include "../../gcode/gcode.h"
#include "../../gcode/parser.h"
#include "../../gcode/queue.h"
#include "../../module/temperature.h"
#include "../../libs/numtostr.h"
#include "../../inc/MarlinConfig.h"
#include "../../feature/controllerfan.h"
#include "../../core/serial.h"

#include "../shared/CAN.h"

extern "C" void TIM16_IRQHandler(void); // Override weak functions
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);

#ifndef CAN_LED_PIN
  #define CAN_LED_PIN   LED_PIN
#endif

#define CAN_DATALENGTH_OFFSET    16 // Used for CAN TxHeader DataLength record

#ifndef CAN_BAUDRATE
  #define CAN_BAUDRATE 1000000LU
#endif

#ifndef CAN_RD_PIN
  #define CAN_RD_PIN PB0
#endif
#ifndef CAN_TD_PIN
  #define CAN_TD_PIN PB1
#endif

#define FDCAN_RX_FIFO0_MASK        (FDCAN_IR_RF0L | FDCAN_IR_RF0F | FDCAN_IR_RF0N) // FIFO0: Message lost | FIFO full | New message
#define FDCAN_RX_FIFO1_MASK        (FDCAN_IR_RF1L | FDCAN_IR_RF1F | FDCAN_IR_RF1N) // FIFO1: Message lost | FIFO full | New message
#define FDCAN_RX_FIFO_MASK         (FDCAN_RX_FIFO0_MASK | FDCAN_RX_FIFO1_MASK)     // FIFO : Message lost | FIFO full | New message
#define HAL_TIM_FLAG_ALL_MASK      (TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4 | TIM_FLAG_UPDATE | TIM_FLAG_BREAK | TIM_FLAG_BREAK2 | TIM_FLAG_TRIGGER | TIM_FLAG_COM)
#define M_GCODE(A)                 ((CAN_ID_GCODE_TYPE_M << (CAN_ID_GCODE_TYPE_BIT_POS - CAN_ID_GCODE_NUMBER_BIT_POS)) + A)

#define CAN_FIFO_DEPTH                         3 // 3 FIFO buffers per FIFO on STM32G0xx
#define CAN_MAX_WAIT_TIME                     25 // Amount of ms to wait for resources (TX buffers/Marlin command buffers)
#define CAN_TIMESTAMP_NO                    7777 // M7777 is used to handle timesync messages
#define CAN_TOOLHEAD_CONFIGURATION_COMPLETE 7778 // Signal the configuration is complete
#define CAN_TEMPERATURE_REPORT_INTERVAL      950 // Temperature report interval in ms (faster than once a second)
#define CAN_MAX_STRING_MESSAGE_LENGTH        128 // Max string length to send to the host
#define CAN_EXTENDED_ID_MARKER_MASK    (1 << 31) // Extended ID marker in CAN_BUFFER identifier
#define CAN_QUEUE_DEPTH                       16 // CAN queue depth

FDCAN_HandleTypeDef hCAN1;                       // The global CAN handle

struct CAN_MSG_BUFFER {                          // Struct to hold a CAN message
  uint32_t identifier;
  float data[2];
  uint32_t receive_time; // Used for time sync
};

SString<CAN_MAX_STRING_MESSAGE_LENGTH> CAN_string_buffer; // String buffer to hold outgoing string messages
char gcode_type[4]       = { 'D', 'G', 'M', 'T' }; // The 4 Gcode types
volatile bool CAN_toolhead_not_configured  = true; // Track if the host sent the required toolhead configuration
uint32_t CAN_toolhead_error_code              = 0; // Signals an CAN error occured, report to host
uint32_t CAN_previous_error_code              = 0; // Remember last error code so changes can be detected
uint32_t CAN_next_temp_report_time            = 0; // The next temperature report time, delay on startup
uint32_t CAN_send_next_string_part_time       = 0; // Send the next part of a string message
volatile bool CAN_FIFO_toggle_bit             = 0; // FIFO toggle bit for receiver filtering to FIFO0 and FIFO1
bool CAN_request_time_sync                    = false; // Request a timestamp for NTP time sync

int CAN_NTP_time_offset                       = 0; // Time offset in micro seconds in relation to the host time
volatile uint32_t NTP[5]                  = { 0 }; // NTP style timestamps for time sync
float CAN_NTP_clock_drift                     = 0; // Time sync calculated clock drift between host and toolhead

uint32_t CAN_next_led_flash_time              = 0; // Error LED flashing
uint32_t CAN_next_error_message_time          = 0; // Controls error message repeat frequency

CAN_MSG_BUFFER CAN_QUEUE[CAN_QUEUE_DEPTH] = { 0 }; // Circular CAN message queue 
volatile uint32_t CAN_queue_head              = 0; // Queue head index
volatile uint32_t CAN_queue_tail              = 0; // Queue tail index

// Function to calculate the CAN message ID holding various bits of information
uint32_t CAN_get_virtual_IO(bool tempUpdate) {

  CAN_FIFO_toggle_bit = !CAN_FIFO_toggle_bit; // Toggle FIFO bit for receiver filtering to FIFO0 and FIFO1

  bool configured = tempUpdate && CAN_toolhead_not_configured; // Only request setup during temp updates, not IO interrupts
  return (
  
    #ifdef Z_MIN_PROBE_PIN
      (READ(Z_MIN_PROBE_PIN) << CAN_ID_PROBE_BIT_POS) | // Report probe status
    #endif
    #ifdef X_MIN_PIN
      (READ(X_MIN_PIN) << CAN_ID_X_ENDSTOP_BIT_POS) | // Report X-min status
    #endif
    #ifdef Y_MIN_PIN
      (READ(Y_MIN_PIN) << CAN_ID_Y_ENDSTOP_BIT_POS) | // Report Y-min status
    #endif
    #ifdef Z_MIN_PIN
      (READ(Z_MIN_PIN) << CAN_ID_Z_ENDSTOP_BIT_POS) | // Report Z-min status
    #endif
    #ifdef FILAMENT_RUNOUT_PIN
      (READ(FILAMENT_RUNOUT_PIN) << CAN_ID_FILAMENT_BIT_POS) | // Report filament detector status
    #endif

    (CAN_FIFO_toggle_bit ? STDID_FIFO_TOGGLE_BIT : 0)   | // Add FIFO toggle bit
    (configured << CAN_ID_REQUEST_SETUP_BIT_POS)        | // Request toolhead setup configuration
    ((!!CAN_toolhead_error_code) << CAN_ID_ERROR_BIT_POS) // Report error (if any)
  );
}

// Process a received message (offline in the idle handler)
void process_can_queue() {
  static SString<MAX_CMD_SIZE> CAN_gcode_buffer;
  static uint32_t parameter_counter = 0;

  uint32_t identifier = CAN_QUEUE[CAN_queue_tail].identifier;
  bool enqueue = true;
  
  // Receiving new Gcode
  if (identifier & CAN_EXTENDED_ID_MARKER_MASK) {

    uint32_t gcode = (identifier >> CAN_ID_GCODE_NUMBER_BIT_POS) & CAN_ID_GCODE_MASK;

    // M997, restart the toolhead unconditionally
    if (gcode == M_GCODE(997)) {
      SERIAL_ECHOLNPGM("\n>>> Host requested a toolhead restart (M997)...");
      delay(25);
      flashFirmware(0);
      while (1);
    }

    // The host indicatds that the configuration is complete
    if (gcode == M_GCODE(CAN_TOOLHEAD_CONFIGURATION_COMPLETE)) {
      CAN_toolhead_not_configured = false;
      MString<60> buffer("FIRMWARE ", __DATE__, " ", __TIME__, "  Thermistor=", TEMP_SENSOR_0);
      CAN_toolhead_send_string(buffer);
      enqueue = false;
    }

    // M115 triggers the time sync process
    if (gcode == M_GCODE(115)) // TESTING...
      CAN_request_time_sync = true; // Send time sync request in idle task and wait for response

    // M<CAN_TIMESTAMP_NO>, time sync response message
    if (gcode == M_GCODE(CAN_TIMESTAMP_NO)) {
      // Data contains 2 timestamps in uint32_t format
      uint32_t *uint32p = (uint32_t*)CAN_QUEUE[CAN_queue_tail].data;
      NTP[1] = *uint32p++;
      NTP[2] = *uint32p;   
      NTP[3] = CAN_QUEUE[CAN_queue_tail].receive_time; // Record time sync response message receive time

      enqueue = false; // Timestamps were stored, the idle process will handle the rest
    }
    
    // New Gcode started, so previous Gcode must be complete (all parameters received)
    if (parameter_counter)
      CAN_toolhead_error_code |= CAN_ERROR_TOOLHEAD_INCOMPLETE_GCODE_RECEIVED;

    parameter_counter = (identifier >> CAN_ID_PARAMETER_COUNT_BIT_POS) & CAN_ID_PARAMETER_COUNT_MASK; // Get Gcode parameter count

    CAN_gcode_buffer = gcode_type[(identifier >> CAN_ID_GCODE_TYPE_BIT_POS) & CAN_ID_GCODE_TYPE_MASK]; // Add Gcode letter e.g., "G"
    CAN_gcode_buffer.append((identifier >> CAN_ID_GCODE_NUMBER_BIT_POS) & CAN_ID_GCODE_NUMBER_MASK);   // Add Gcode number e.g., G"92"
  }

  uint32_t backupLength = 0; // Backup string length in case we cannot enqueue the Gcode and have to try again
  // Add parameters (if present) to the received Gcode
  if (parameter_counter && ((identifier >> CAN_ID_PARAMETER1_BIT_POS) & CAN_ID_PARAMETER_LETTER_MASK)) { // Get 1st parameter, make sure it's not empty.
    backupLength = CAN_gcode_buffer.length();    // Point to place where parameters are added  
    CAN_gcode_buffer.append(char((identifier & CAN_ID_PARAMETER_LETTER_MASK) + 64)); // Add Gcode parameter letter, e.g., 'X'
    float value = CAN_QUEUE[CAN_queue_tail].data[0];
    if (!isnan(value)) { // No value for parameter if value is "Not A Number"
      if (value == int(value))  
        CAN_gcode_buffer.append(int(CAN_QUEUE[CAN_queue_tail].data[0])); // Integer value
      else
        CAN_gcode_buffer.append(p_float_t(CAN_QUEUE[CAN_queue_tail].data[0], 5));
    }
    parameter_counter--;

    // Add 2nd parameter if 2 values were provided
    if (parameter_counter && ((identifier >> CAN_ID_PARAMETER2_BIT_POS) & CAN_ID_PARAMETER_LETTER_MASK)) { // Get 2nd parameter, make sure it's not empty.
      CAN_gcode_buffer.append(char(((identifier >> CAN_ID_PARAMETER2_BIT_POS) & CAN_ID_PARAMETER_LETTER_MASK) + 64)); // Add Gcode parameter letter, e.g., 'X'
      if (!isnan(CAN_QUEUE[CAN_queue_tail].data[1])) { // No value for parameter if value is "Not A Number"
        if (value == int(value))  
          CAN_gcode_buffer.append(int(CAN_QUEUE[CAN_queue_tail].data[1])); // Integer value
        else
          CAN_gcode_buffer.append(p_float_t(CAN_QUEUE[CAN_queue_tail].data[1], 5));
      }
      parameter_counter--;
    }
  }

  if (!parameter_counter) { // Gcode is complete, including all parameters, process the Gcode
      if (enqueue) {
          // queue.enqueue_one returns TRUE if the command was queued, FALSE if the Marlin cmd buffer was full
          if (queue.enqueue_one(CAN_gcode_buffer)) { // Increase tail only when commands was enqueued
            CAN_queue_tail = (CAN_queue_tail + 1) % CAN_QUEUE_DEPTH;
            #ifdef CAN_DEBUG
              SERIAL_ECHOPGM(";", millis(), " "); CAN_gcode_buffer.echoln();
            #endif
          }
          else
            if (!(identifier & CAN_EXTENDED_ID_MARKER_MASK)) // Standard ID message, so parameters were added to the Gcode
              CAN_gcode_buffer.trunc(backupLength); // Cut off the part of the Gcode that was added, so we can process the CAN message again
      }
      else
        CAN_queue_tail = (CAN_queue_tail + 1) % CAN_QUEUE_DEPTH; // Always advance tail
  }
  else
    CAN_queue_tail = (CAN_queue_tail + 1) % CAN_QUEUE_DEPTH;

} // process_can_queue

HAL_StatusTypeDef CAN_receive_msg(uint32_t FIFO) { // ISR! Process received CAN message in interrupt handler

  if (((CAN_queue_head + 1) % CAN_QUEUE_DEPTH) != CAN_queue_tail) { // Check if a buffer is available

    FDCAN_RxHeaderTypeDef CAN_rx_header;
    CAN_QUEUE[CAN_queue_head].receive_time = micros(); // Save receiver timestamp for time sync
    HAL_FDCAN_GetRxMessage(&hCAN1, FIFO, &CAN_rx_header, (uint8_t*)CAN_QUEUE[CAN_queue_head].data);
    
    if (CAN_rx_header.IdType == FDCAN_EXTENDED_ID)                     // Mark extended ID message (new Gcode start)
      CAN_QUEUE[CAN_queue_head].identifier = CAN_EXTENDED_ID_MARKER_MASK + CAN_rx_header.Identifier;
    else
      CAN_QUEUE[CAN_queue_head].identifier = CAN_rx_header.Identifier; // Standard ID message (Gcode parameters)
      
    CAN_queue_head = (CAN_queue_head + 1) % CAN_QUEUE_DEPTH; 
  }
  else
    CAN_toolhead_error_code |= CAN_ERROR_TOOLHEAD_RX_FIFO_OVERFLOW;

  return HAL_OK;

} // CAN_receive_msg

// NOTE: TIM16 is also used for the stepper!
void TIM16_IRQHandler(void) { // ISR! Combined TIM16 and CAN interrupt handler (override weak function)

  // Check if the timer caused the interrupt
  if (TIM16->SR & HAL_TIM_FLAG_ALL_MASK)
//    HardwareTimer_Handle[TIMER16_INDEX]->handle.Instance->SR & HAL_TIM_FLAG_ALL_MASK)
    HAL_TIM_IRQHandler(&HardwareTimer_Handle[TIMER16_INDEX]->handle);
    // Forward call to timer interrupt handler
    //
    // OR
    //
    // Forward to Marlin stepper ISR directly?

  // Call the FDCAN interrupt handler
  //if (hCAN1.Instance->IR & (FDCAN_IR_RF0N | FDCAN_IR_RF0L | FDCAN_IR_RF1N | FDCAN_IR_RF1L))
  //  HAL_FDCAN_IRQHandler(&hCAN1); // Forward call to FDCAN interrupt handler, calls HAL_FDCAN_RxFifo0Callback and HAL_FDCAN_RxFifo1Callback
  //
  // OR
  //
  // Call the required callbacks directly, faster but limited error reporting
  // New FIFO0 CAN message
  if (hCAN1.Instance->IR & FDCAN_IR_RF0N) {
    __HAL_FDCAN_CLEAR_FLAG(&hCAN1, FDCAN_IR_RF0N);
    CAN_receive_msg(FDCAN_RX_FIFO0);
  }

 // New FIFO1 CAN message
  if (hCAN1.Instance->IR & FDCAN_IR_RF1N) {
    __HAL_FDCAN_CLEAR_FLAG(&hCAN1, FDCAN_IR_RF1N);
    CAN_receive_msg(FDCAN_RX_FIFO1);
  }

  // Check for lost CAN messages
  if (hCAN1.Instance->IR & (FDCAN_IR_RF0L | FDCAN_IR_RF1L)) {
    CAN_toolhead_error_code |= CAN_ERROR_TOOLHEAD_RX_FIFO_OVERFLOW;
    __HAL_FDCAN_CLEAR_FLAG(&hCAN1, FDCAN_IR_RF0L | FDCAN_IR_RF1L);
  }

} // TIM16_IRQHandler

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) { // ISR! Override "weak" function

  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST)
    CAN_toolhead_error_code |= CAN_ERROR_TOOLHEAD_RX_FIFO_OVERFLOW;

  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    CAN_receive_msg(FDCAN_RX_FIFO0);

}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) { // ISR! Override "weak" function

  if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST)
    __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_IT_RX_FIFO1_MESSAGE_LOST); // Clear FIFO1 message lost interrupt flag

  if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
    CAN_receive_msg(FDCAN_RX_FIFO1);

}

// Enable a GPIO clock based on the GPIOx address for STM32G0
void gpio_clock_enable(GPIO_TypeDef *regs)
{
  // Or use: STM_PORT(CAN_TD_pin) GPIOA=0, GPIOB=1 etc.
  uint32_t pos = ((uint32_t)regs - IOPORT_BASE) >> 10;
  RCC->IOPENR |= (1 << pos);
  RCC->IOPENR;
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle) { // Called automatically by FDCAN_init(), configure GPIO for CAN, enable interrupts

  // STM32G0B1 and STM32G0 C1 only

  // FDCAN2 GPIO Configuration
  // PB0 AF3 ------> FDCAN2_RD
  // PB1 AF3 ------> FDCAN2_TD

  // CAN1/2 clock enable (one clock for both CAN devices)
  __HAL_RCC_FDCAN_CLK_ENABLE();

  // Use some macros to find the required setup info based on the provided CAN pins
  uint32_t _CAN_RD_pin = digitalPinToPinName(CAN_RD_PIN);
  uint32_t _CAN_TD_pin = digitalPinToPinName(CAN_TD_PIN);
  uint32_t _CAN_RD_function = pinmap_find_function(digitalPinToPinName(CAN_RD_PIN), PinMap_CAN_RD);
  uint32_t _CAN_TD_function = pinmap_find_function(digitalPinToPinName(CAN_TD_PIN), PinMap_CAN_TD);
  
  // Enable the GPIOx device related to the CAN_RD_pin
  gpio_clock_enable(get_GPIO_Port(STM_PORT(_CAN_RD_pin))); // PB0

  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // High frequency device
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;      // Alternate function
  GPIO_InitStruct.Pin = STM_GPIO_PIN(STM_PIN(_CAN_RD_pin)) | STM_GPIO_PIN(STM_PIN(_CAN_TD_pin));
  GPIO_InitStruct.Pull = STM_PIN_PUPD(_CAN_RD_function);
  GPIO_InitStruct.Alternate = STM_PIN_AFNUM(_CAN_RD_function);
  HAL_GPIO_Init(get_GPIO_Port(STM_PORT(_CAN_RD_pin)), &GPIO_InitStruct);

  //NOTE: Separating the pin initialisation causes issues, why?
  //Enable the GPIOx device related to the CAN_TD_pin
  //gpio_clock_enable(get_GPIO_Port(STM_PORT(_CAN_TD_pin)));
  //GPIO_InitStruct.Pin = STM_GPIO_PIN(STM_PIN(_CAN_TD_pin));
  //GPIO_InitStruct.Alternate = STM_PIN_AFNUM(_CAN_TD_function);
  //HAL_GPIO_Init(get_GPIO_Port(STM_PORT(_CAN_TD_pin)), &GPIO_InitStruct);

  // Enable CAN interrupts
  HAL_NVIC_SetPriority(TIM16_FDCAN_IT0_IRQn, 1, 1);   // Set interrupt priority
  HAL_NVIC_EnableIRQ(TIM16_FDCAN_IT0_IRQn);           // Enable interrupt handler

} // HAL_FDCAN_MspInit

// Calculate the CAN sample timing, seg1 and seg2, sjw = 1 (no baudrate switching)
// seg1 range: 2-256, seg2 range: 2-128, SJW = 1, so minimum is 5 clocks per bit
int FDCAN_calculate_segments(uint32_t *seg1, uint32_t *seg2) {

  uint32_t CAN_clock =  HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
  float clocks_per_bit = CAN_clock / CAN_BAUDRATE; // Clocks per bit must be a whole number
  
  if ((clocks_per_bit != int(clocks_per_bit)) || (clocks_per_bit < 5)) // Minimal 5 clocks per bit (2+2+1)
    return -1; // Baudrate is not possible

 *seg2 = (clocks_per_bit / 8) + 0.5;  // Preferred sample point at 87.5% (7/8)
 *seg1 = uint32_t(clocks_per_bit) - *seg2 - 1; // sjw = 1;

  return HAL_OK;
}

HAL_StatusTypeDef CAN_toolhead_start() { // Start the CAN device

  // The CAN clock and clock source must to be set first because the sample timing depends on it
  RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.FdcanClockSelection  = RCC_FDCANCLKSOURCE_HSE;   //  8MHz external crystal
//PeriphClkInit.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL;   // 64MHz
//PeriphClkInit.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PCLK1; // 64MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit); // Select the FDCAN clock source

  HAL_StatusTypeDef status = HAL_OK;
  // CAN baud rate = Device Clock Frequency / Clock Divider / Prescaler / (SJW + TSG1 + TSG2)
  // Sample-Point from 50 to 90% (87.5 % is the preferred value used by CANopen and DeviceNet, 75% is the ARINC 825 default)
  // Baud rate = 64M / 1 /  4 / (1 + 13 + 2) = 1M    Bit sample point = (1 + 13) / (1 + 13 + 2) = 87.5%
  // Baud rate = 64M / 1 /  8 / (1 + 13 + 2) = 500K  Bit sample point = (1 + 13) / (1 + 13 + 2) = 87.5%
  // Baud rate = 64M / 1 / 16 / (1 + 13 + 2) = 250K  Bit sample point = (1 + 13) / (1 + 13 + 2) = 87.5%

  uint32_t seg1, seg2;
  if (FDCAN_calculate_segments(&seg1, &seg2) != HAL_OK) {
    SERIAL_ECHOLNPGM("Impossible CAN baudrate, check CAN clock and baudrate");
    return HAL_ERROR;
  }
  
  __HAL_RCC_FDCAN_CLK_DISABLE(); // Disable for startup, sample calculation were done

  hCAN1.Instance                  = FDCAN2;                  // The FDCAN device used
  hCAN1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;        // Clock divider 1 2 4 6 8 10 12 14 16 18 20 22 24 26 28 30
  hCAN1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;     // FDCAN_FRAME_CLASSIC / FDCAN_FRAME_FD_BRS / FDCAN_FRAME_FD_NO_BRS (Bit Rate Switching)
  hCAN1.Init.Mode                 = FDCAN_MODE_NORMAL;       // FDCAN_MODE_NORMAL / FDCAN_MODE_EXTERNAL_LOOPBACK / FDCAN_MODE_INTERNAL_LOOPBACK / FDCAN_MODE_BUS_MONITORING / FDCAN_MODE_RESTRICTED_OPERATION
  hCAN1.Init.AutoRetransmission   = DISABLE;                 // Auto Retransmission of message if error occured, can cause lockup
  hCAN1.Init.TransmitPause        = DISABLE;                 // Transmit Pause to allow for other device to send message
  hCAN1.Init.ProtocolException    = DISABLE;                 // ProtocolException handling

  hCAN1.Init.NominalPrescaler     =    1; // Arbitration/data clock prescaler (1-512)
  hCAN1.Init.NominalSyncJumpWidth =    1; // Arbitration/data Sync Jump Width (1-128)
  hCAN1.Init.NominalTimeSeg1      = seg1; // Arbitration/data period 1 (2-256)
  hCAN1.Init.NominalTimeSeg2      = seg2; // Arbitration/data period 2 (2-128)

  /* Not used, no bitrate switching
  hCAN1.Init.DataPrescaler        =  4; // Arbitration/data clock prescaler (1-32)
  hCAN1.Init.DataSyncJumpWidth    =  1; // Arbitration/data sync jump width (1-16)
  hCAN1.Init.DataTimeSeg1         = 13; // Arbitration/data period 1 (2-32)
  hCAN1.Init.DataTimeSeg2         =  2; // Arbitration/data period 2 (2-16) */

  hCAN1.Init.StdFiltersNbr        = 2;                       // Number of standard ID frame filters, 2 for FIFO toggling
  hCAN1.Init.ExtFiltersNbr        = 2;                       // Number of extended ID frame filters, 2 for FIFO toggling
  hCAN1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION; // Queue mode: FDCAN_TX_FIFO_OPERATION / FDCAN_TX_QUEUE_OPERATION
  // "FIFO OPERATION" means send in order of queueing, "QUEUE OPERATION" means send in order of ID priority

  status = HAL_FDCAN_Init(&hCAN1); // calls HAL_FDCAN_MspInit
  if (status != HAL_OK) return status;

  FDCAN_FilterTypeDef sFilterConfig;                    // Configure RX message filter
  sFilterConfig.IdType       = FDCAN_EXTENDED_ID;       // Filter to FIFO1 if higest ID bit is set
  sFilterConfig.FilterIndex  = 0;                       // Exteneded filter ID 0
  sFilterConfig.FilterType   = FDCAN_FILTER_MASK;       // FDCAN_FILTER_MASK / FDCAN_FILTER_RANGE
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; // Filter to FIFO1
  sFilterConfig.FilterID1    = 0x10000000;              // Range: 0 - 0x1FFF FFFF, 0=don'tcare
  sFilterConfig.FilterID2    = 0x10000000;              // Range: 0 - 0x1FFF FFFF, 0=don'tcare
  status = HAL_FDCAN_ConfigFilter(&hCAN1, &sFilterConfig);
  if (status != HAL_OK) return status;

//sFilterConfig.IdType       = FDCAN_EXTENDED_ID;       // Filter all remaining messages to FIFO0
  sFilterConfig.FilterIndex  = 1;                       // Exteneded filter ID 1
//sFilterConfig.FilterType   = FDCAN_FILTER_MASK;       // FDCAN_FILTER_MASK / FDCAN_FILTER_RANGE
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Remaining messages go to FIFO0
  sFilterConfig.FilterID1    = 0;                       // Range: 0 - 0x1FFF FFFF, 0=don'tcare
  sFilterConfig.FilterID2    = 0;                       // Range: 0 - 0x1FFF FFFF, 0=don'tcare
  status = HAL_FDCAN_ConfigFilter(&hCAN1, &sFilterConfig);
  if (status != HAL_OK) return status;

  sFilterConfig.IdType       = FDCAN_STANDARD_ID;       // Filter to FIFO1 if higest ID bit is set
  sFilterConfig.FilterIndex  = 0;                       // Standard filter ID 0
  sFilterConfig.FilterType   = FDCAN_FILTER_MASK;       // FDCAN_FILTER_MASK / FDCAN_FILTER_RANGE
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; // Filter to FIFO1
  sFilterConfig.FilterID1    = 0b10000000000;           // Range: 0 - 0x7FF, 0=don't care
  sFilterConfig.FilterID2    = 0b10000000000;           // Range: 0 - 0x7FF, 0=don't care
  status = HAL_FDCAN_ConfigFilter(&hCAN1, &sFilterConfig);
  if (status != HAL_OK) return status;

//sFilterConfig.IdType       = FDCAN_STANDARD_ID;       // Filter all remaining messages to FIFO0
  sFilterConfig.FilterIndex  = 1;                       // Standard filter ID 1
//sFilterConfig.FilterType   = FDCAN_FILTER_MASK;       // FDCAN_FILTER_MASK / FDCAN_FILTER_RANGE
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Remaining messages go to FIFO0
  sFilterConfig.FilterID1    = 0;                       // Range: 0 - 0x7FF, 0=don't care
  sFilterConfig.FilterID2    = 0;                       // Range: 0 - 0x7FF, 0=don't care
  status = HAL_FDCAN_ConfigFilter(&hCAN1, &sFilterConfig);
  if (status != HAL_OK) return status;

  status = HAL_FDCAN_Start(&hCAN1);
  if (status != HAL_OK) return status;

  // Activate RX FIFO0 new message interrupt
  status = HAL_FDCAN_ActivateNotification(&hCAN1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,  0); // Calls TIM16_IRQHandler (STM32G0xx)
  if (status != HAL_OK) return status;

  // Activate RX FIFO0 message lost interrupt
  status = HAL_FDCAN_ActivateNotification(&hCAN1, FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0); // Calls TIM16_IRQHandler (STM32G0xx)
  if (status != HAL_OK) return status;

  // Activate RX FIFO1 new message interrupt
  status = HAL_FDCAN_ActivateNotification(&hCAN1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,  0); // Calls TIM16_IRQHandler (STM32G0xx)
  if (status != HAL_OK) return status;

  // Activate RX FIFO1 message lost interrupt
  status = HAL_FDCAN_ActivateNotification(&hCAN1, FDCAN_IT_RX_FIFO1_MESSAGE_LOST, 0); // Calls TIM16_IRQHandler (STM32G0xx)
  if (status != HAL_OK) return status;

  #if ENABLED(CAN_DEBUG)

    SERIAL_ECHOLNPGM(">>> Voltage Scaling: VOS", (PWR->CR1 & PWR_CR1_VOS_Msk) >> PWR_CR1_VOS_Pos);
    SERIAL_ECHOLNPGM(">>> FDCAN Peripheral Clock: ", HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) / 1000000, "MHz");
    SERIAL_ECHOLNPGM(">>> FDCAN Timing. Prescaler: ", hCAN1.Init.NominalPrescaler, "   SJW: ", hCAN1.Init.NominalSyncJumpWidth, "   SEG1: ", hCAN1.Init.NominalTimeSeg1, "   SEG2: ", hCAN1.Init.NominalTimeSeg2);
    SERIAL_ECHOLNPGM(">>> FDCAN Bit sample point: ", 100 * (hCAN1.Init.NominalSyncJumpWidth +  hCAN1.Init.NominalTimeSeg1)/ (hCAN1.Init.NominalSyncJumpWidth +  hCAN1.Init.NominalTimeSeg1 + hCAN1.Init.NominalTimeSeg2), "%");
    uint32_t baudrate = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) / hCAN1.Init.NominalPrescaler / (hCAN1.Init.NominalSyncJumpWidth + hCAN1.Init.NominalTimeSeg1 + hCAN1.Init.NominalTimeSeg2);
    SERIAL_ECHOLNPGM(">>> FDCAN BAUDRATE: ", baudrate);

    if (baudrate != CAN_BAUDRATE) {
      SERIAL_ECHOLNPGM(">>> Toolhead ", CAN_ERROR_MSG_INVALID_BAUDRATE, ": ", baudrate, "  CAN_BAUDRATE=", CAN_BAUDRATE);
      CAN_toolhead_error_code |= CAN_ERROR_TOOLHEAD_INVALID_BAUDRATE;
    }

  #endif

  #ifdef CAN_LED_PIN
    pinMode(CAN_LED_PIN, OUTPUT);
    digitalWrite(CAN_LED_PIN, HIGH);
  #endif 

  return status;
} // CAN_toolhead_start

// Send an IO status update to the host, and the E0 temperature if requested
void CAN_toolhead_send_update(bool tempUpdate) { // Called from temperature ISR!
  // Send a IO/temp report from the toolhead to the host
  FDCAN_TxHeaderTypeDef CanTxHeader;
  uint8_t can_tx_buffer[8]; // Transmit FDCAN message buffer

  // Initialize standard TxHeader values
  CanTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;  // FDCAN_CLASSIC_CAN / FDCAN_FD_CAN
  CanTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // FDCAN_NO_TX_EVENTS / FDCAN_STORE_TX_EVENTS
  CanTxHeader.MessageMarker       = 0;                  // 0-0xFF for tracking messages in FIFO buffer
  CanTxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;  // FDCAN_ESI_PASSIVE / FDCAN_ESI_ACTIVE
  CanTxHeader.TxFrameType         = FDCAN_DATA_FRAME;   // FDCAN_DATA_FRAME / FDCAN_REMOTE_FRAME
  CanTxHeader.IdType              = FDCAN_STANDARD_ID;  // FDCAN_STANDARD_ID / FDCAN_EXTENDED_ID
  CanTxHeader.BitRateSwitch       = FDCAN_BRS_OFF;      // FDCAN_BRS_OFF / FDCAN_BRS_ON

  if (tempUpdate) {                             // Add temperature update to FDCAN message, 4 bytes
    float * fp = (float *)can_tx_buffer;        // Point to FDCAN TX buffer
    *fp = thermalManager.degHotend(0);          // Copy temp to can_tx_buffer
    CanTxHeader.DataLength = FDCAN_DLC_BYTES_4; // Hotend temp in payload only
  }
  else
    CanTxHeader.DataLength = FDCAN_DLC_BYTES_0; // 0 data byte FDCAN message, virtual IO is encoded in the message ID

  // Message might be important, don't wait, make room for the message
  if (!HAL_FDCAN_GetTxFifoFreeLevel(&hCAN1)) { // No TX slots available
    HAL_FDCAN_AbortTxRequest(&hCAN1, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2); // Flush all message buffers
    CAN_toolhead_error_code |= CAN_ERROR_TOOLHEAD_TX_FIFO_OVERFLOW;
  }

  CanTxHeader.Identifier = CAN_get_virtual_IO(tempUpdate);

  if (CAN_request_time_sync) {

    // For time sync request, wait for empty TX slot before requesting
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hCAN1) == CAN_FIFO_DEPTH) {
      CanTxHeader.Identifier |= (1 << CAN_ID_REQUEST_TIME_SYNC_BIT_POS); // Issue time sync request

      CAN_request_time_sync = false;
      NTP[0] = micros(); // Store time sync request message send time
    }
  }

  HAL_FDCAN_AddMessageToTxFifoQ(&hCAN1, &CanTxHeader, can_tx_buffer); 

} // CAN_toolhead_send_update

// Send a string message to the host
void CAN_toolhead_send_string(const char * message) {

  FDCAN_TxHeaderTypeDef CanTxHeader;
  uint8_t can_tx_buffer[8]; // Transmit CAN message buffer

  CAN_string_buffer.append(message);

  // Make sure there is a '\n' at the end of the string
  if (CAN_string_buffer[CAN_string_buffer.length() - 1] != '\n') {
    if (CAN_string_buffer.length() == CAN_MAX_STRING_MESSAGE_LENGTH)
      CAN_string_buffer.trunc(CAN_MAX_STRING_MESSAGE_LENGTH - 1);
    CAN_string_buffer.append('\n');
  }

  SERIAL_ECHOPGM(">>> STRING MSG TO HOST: ");
  CAN_string_buffer.echo();
}

void CAN_send_next_string_part() {

  uint32_t len = CAN_string_buffer.length();

  FDCAN_TxHeaderTypeDef CanTxHeader;
  uint8_t can_tx_buffer[8]; // Transmit CAN message buffer

  // Initialize standard TX header values
  CanTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;  // FDCAN_CLASSIC_CAN / FDCAN_FD_CAN
  CanTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // FDCAN_NO_TX_EVENTS / FDCAN_STORE_TX_EVENTS
  CanTxHeader.MessageMarker       = 0;                  // 0-0xFF for tracking messages in FIFO buffer
  CanTxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;  // FDCAN_ESI_PASSIVE / FDCAN_ESI_ACTIVE
  CanTxHeader.TxFrameType         = FDCAN_DATA_FRAME;   // FDCAN_DATA_FRAME / FDCAN_REMOTE_FRAME
  CanTxHeader.IdType              = FDCAN_STANDARD_ID;  // FDCAN_STANDARD_ID / FDCAN_EXTENDED_ID
  CanTxHeader.BitRateSwitch       = FDCAN_BRS_OFF;      // FDCAN_BRS_OFF / FDCAN_BRS_ON

  uint32_t c = MIN(8, len);
  CanTxHeader.DataLength = (c << CAN_DATALENGTH_OFFSET); // Max message length is 8 bytes (CAN), offset is 16 bits into the DataLength variable

  CanTxHeader.Identifier = CAN_get_virtual_IO(false) | CAN_ID_STRING_MESSAGE_BIT_MASK;

  // Low priority message, wait until TX FIFO is completely empty before sending the message
  if (HAL_FDCAN_GetTxFifoFreeLevel(&hCAN1) == CAN_FIFO_DEPTH) {
    HAL_FDCAN_AddMessageToTxFifoQ(&hCAN1, &CanTxHeader, (uint8_t *)&CAN_string_buffer); // Send message

    if (c < 8)
      CAN_string_buffer.clear();
    else
      strncpy(CAN_string_buffer, (char *)(&CAN_string_buffer + c), len - c + 1);
  }

} // CAN_send_next_string_part

void CAN_process_time_sync() {
  // NTP style time sync
  // NTP[0] = local time sync request time
  // NTP[1] = host time sync receive time
  // NTP[2] = host time sync reply time
  // NTP[3] = local time stamp receive time
  NTP[0] += CAN_NTP_time_offset; // Adjust local time stamps with CAN_NTP_time_offset
  NTP[3] += CAN_NTP_time_offset; // Adjust local time stamps with CAN_NTP_time_offset
  int CAN_local_time_adjustment = (NTP[1] - NTP[0] + NTP[2] - NTP[3]) >> 1;
  CAN_NTP_time_offset += CAN_local_time_adjustment;
  uint32_t CAN_Round_trip_delay = (NTP[3] - NTP[0] - NTP[2] + NTP[1]);
  SERIAL_ECHOLNPGM(">>> t0: ", NTP[0], " us (Local time sync request time)");
  SERIAL_ECHOLNPGM(">>> t1: ", NTP[1], " us (remote time sync arrival time)");
  SERIAL_ECHOLNPGM(">>> t2: ", NTP[2], " us (remote time sync response time)");
  SERIAL_ECHOLNPGM(">>> t3: ", NTP[3], " us (local time sync reply receive time)");
  
  if (CAN_NTP_clock_drift)
    SERIAL_ECHOLNPGM(">>> Predicted time adjustment: ", CAN_NTP_clock_drift * float(NTP[0] - NTP[4]) / 1000000.0 , " us");
    
  SERIAL_ECHOPGM(">>> Local time adjustment: ", CAN_local_time_adjustment, " us");
  if (NTP[4]) {
    SERIAL_ECHOLNPGM(" after ",  ftostr42_52(float(NTP[0] - NTP[4]) / 1000000.0), " seconds");
    CAN_NTP_clock_drift = CAN_local_time_adjustment / (float(NTP[0] - NTP[4]) / 1000000.0); // Calculate drift again
    SERIAL_ECHOLNPGM(">>> Clock drift: ", CAN_NTP_clock_drift, " us/s");
  }
  else
    SERIAL_EOL();

  SERIAL_ECHOLNPGM(">>> Time offset: ", CAN_NTP_time_offset, " us");
  SERIAL_ECHOLNPGM(">>> Round trip delay: ", CAN_Round_trip_delay, " us");
  SERIAL_ECHOLNPGM(">>> Host response time: ", (NTP[2] - NTP[1]), " us");

  NTP[4] = NTP[0] + CAN_local_time_adjustment; // Store previous time sync request time
  NTP[3] = 0;

}

void CAN_handle_errors() {

  uint32_t ms = millis();

  if (hCAN1.ErrorCode) {

    if (ELAPSED(ms, CAN_next_error_message_time)) { // 12 seconds repeat
      MString<40> tmp_string(F("Error: CAN Error Code = "), hCAN1.ErrorCode);
      CAN_toolhead_send_string(tmp_string);
      CAN_next_error_message_time += 12000; // Delay repeat of message
    }
  }

  if (CAN_toolhead_error_code) {

    if (ELAPSED(ms, CAN_next_led_flash_time)) { // Flash LED fast on error
      TOGGLE(LED_PIN);
      CAN_next_led_flash_time = ms + 100;
    }

    if (CAN_toolhead_error_code != CAN_previous_error_code) { // Show message on new error code

      #if ENABLED(CAN_DEBUG)

      if (CAN_toolhead_error_code & CAN_ERROR_TOOLHEAD_RX_FIFO_OVERFLOW) {
        CAN_toolhead_send_string(CAN_ERROR_MSG_RX_FIFO_OVERFLOW);
        SERIAL_ECHOLNPGM(">>> ", CAN_ERROR_MSG_RX_FIFO_OVERFLOW);
      }

      if (CAN_toolhead_error_code & CAN_ERROR_TOOLHEAD_TX_FIFO_OVERFLOW) {
        CAN_toolhead_send_string(CAN_ERROR_MSG_TX_FIFO_OVERFLOW);
        SERIAL_ECHOLNPGM(">>> ", CAN_ERROR_MSG_TX_FIFO_OVERFLOW);
      }

      if (CAN_toolhead_error_code & CAN_ERROR_TOOLHEAD_INCOMPLETE_GCODE_RECEIVED) {
        CAN_toolhead_send_string(CAN_ERROR_MSG_INCOMPLETE_GCODE);
        SERIAL_ECHOLNPGM(">>> ", CAN_ERROR_MSG_INCOMPLETE_GCODE);
      }

      if (CAN_toolhead_error_code & CAN_ERROR_TOOLHEAD_MARLIN_CMD_BUFFER_OVERFLOW) {
        CAN_toolhead_send_string(CAN_ERROR_MSG_INCOMPLETE_GCODE);
        SERIAL_ECHOLNPGM(">>> ", CAN_ERROR_MSG_INCOMPLETE_GCODE);
      }
      #else
        SString<40> string_buffer(F("Error: TOOLHEAD error code="), CAN_toolhead_error_code);
        CAN_toolhead_send_string(CAN_string_buffer);
        CAN_string_buffer.echoln();
      #endif
      CAN_previous_error_code = CAN_toolhead_error_code;
    }
  }
} // CAN_handle_errors

void CAN_toolhead_idle() { // Called from MarlinCore.cpp

  // Send temperature update to host
  if (ELAPSED(millis(), CAN_next_temp_report_time)) {
    CAN_toolhead_send_update(true);
    CAN_next_temp_report_time = millis() + CAN_TEMPERATURE_REPORT_INTERVAL;
  }

  // Process any new CAN messages
  if (CAN_queue_head != CAN_queue_tail)
    process_can_queue();

  // Send next part of a string message to the host
  if (CAN_string_buffer.length()) {
    if (ELAPSED(millis(), CAN_send_next_string_part_time)) {
      CAN_send_next_string_part();
      CAN_send_next_string_part_time = millis() + 3; // Delay a bit, don't overload the half-duplex CAN bus
    }
  }
  
  // NTP style time sync
  if (NTP[3]) // Indicates a host timestamp was received
    CAN_process_time_sync();

  CAN_handle_errors();

} // CAN_send_string

#endif // CAN_TOOLHEAD
