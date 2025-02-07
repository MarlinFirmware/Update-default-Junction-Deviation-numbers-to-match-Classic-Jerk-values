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

#ifdef ARDUINO_ARCH_HC32

#include "../../inc/MarlinConfig.h"
#include <drivers/panic/panic.h>

#if ANY(POSTMORTEM_DEBUGGING, PANIC_ENABLE)

#include <drivers/usart/usart_sync.h>

//
// Shared by both panic and PostMortem debugging
//
static void minserial_begin() {
  #if !WITHIN(SERIAL_PORT, 1, 4)
    #warning "MinSerial requires a physical UART port for output."
    #warning "Disabling MinSerial because the used serial port is not a HW port."
  #else

    // Prepare usart_sync configuration
    const stc_usart_uart_init_t usart_config = {
      .enClkMode = UsartIntClkCkNoOutput,
      .enClkDiv = UsartClkDiv_1,
      .enDataLength = UsartDataBits8,
      .enDirection = UsartDataLsbFirst,
      .enStopBit = UsartOneStopBit,
      .enParity = UsartParityNone,
      .enSampleMode = UsartSampleBit8,
      .enDetectMode = UsartStartBitFallEdge,
      .enHwFlow = UsartRtsEnable,
    };

    // Initializes usart_sync driver
    #define __USART_SYNC_INIT(port_no, baud, config) \
      usart_sync_init(M4_USART##port_no,             \
                      BOARD_USART##port_no##_TX_PIN, \
                      baud,                          \
                      config);
    #define USART_SYNC_INIT(port_no, baud, config) __USART_SYNC_INIT(port_no, baud, config)

    // This will reset the baudrate to what is defined in Configuration.h,
    // ignoring any changes made with e.g. M575.
    // keeping the dynamic baudrate would require re-calculating the baudrate
    // using the register values, which is a pain...

    // TODO: retain dynamic baudrate in MinSerial init
    // -> see USART_SetBaudrate(), needs to be inverted
    USART_SYNC_INIT(SERIAL_PORT, BAUDRATE, &usart_config);

    #undef USART_SYNC_INIT
    #undef __USART_SYNC_INIT
  #endif
}

static void minserial_putc(char c) {
  #if WITHIN(SERIAL_PORT, 1, 3)
    #define __USART_SYNC_PUTC(port_no, ch) usart_sync_putc(M4_USART##port_no, ch);
    #define USART_SYNC_PUTC(port_no, ch) __USART_SYNC_PUTC(port_no, ch)

    USART_SYNC_PUTC(SERIAL_PORT, c);

    #undef USART_SYNC_PUTC
    #undef __USART_SYNC_PUTC
  #endif
}

//
// Panic only
//
#ifdef PANIC_ENABLE

void panic_begin() {
  minserial_begin();
  panic_puts("\n\nPANIC:\n");
}

void panic_puts(const char *str) {
  while (*str) minserial_putc(*str++);
}

#endif // PANIC_ENABLE

//
// PostMortem debugging only
//
#if ENABLED(POSTMORTEM_DEBUGGING)

#include "../shared/MinSerial.h"
#include <drivers/panic/fault_handlers.h>

void fault_handlers_init() {
  // Enable cpu traps:
  // - Divide by zero
  // - Unaligned access
  SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk; //| SCB_CCR_UNALIGN_TRP_Msk;
}

void install_min_serial() {
  HAL_min_serial_init = &minserial_begin;
  HAL_min_serial_out = &minserial_putc;
}

extern "C" {
  __attribute__((naked)) void JumpHandler_ASM() {
    __asm__ __volatile__(
        "b CommonHandler_ASM\n");
  }
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) HardFault_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) BusFault_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) UsageFault_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) MemManage_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) NMI_Handler();
}

#endif // POSTMORTEM_DEBUGGING
#endif // POSTMORTEM_DEBUGGING || PANIC_ENABLE

//
// Panic_end is always required to print the '!!' to the host
//
void panic_end() {
  // Print '!!' to signal error to host
  // Do it 10x so it's not missed
  for (uint_fast8_t i = 10; i--;) panic_printf("\n!!\n");

  // Then, reset the board
  NVIC_SystemReset();
}

#endif // ARDUINO_ARCH_HC32
