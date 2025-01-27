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
#pragma once

#include <HardwareSerial.h>
#include <libmaple/usart.h>
#include <WString.h>

#include "../../inc/MarlinConfigPre.h"
#include "../../core/serial_hook.h"

#ifdef SERIAL_USB
  typedef ForwardSerial1Class< USBSerial > DefaultSerial1;
  extern DefaultSerial1 MSerial0;
  #if HAS_SD_HOST_DRIVE
    #define UsbSerial MarlinCompositeSerial
  #else
    #define UsbSerial MSerial0
  #endif
#endif

#define SERIAL_INDEX_MIN 1
#if ANY(STM32_HIGH_DENSITY, STM32_XL_DENSITY)
  #define SERIAL_INDEX_MAX 5
#else
  #define SERIAL_INDEX_MAX 3
#endif
#define USB_SERIAL_PORT(...) UsbSerial
#include "../shared/serial_ports.h"

#if defined(LCD_SERIAL_PORT) && ANY(HAS_DGUS_LCD, EXTENSIBLE_UI)
  #define LCD_SERIAL_TX_BUFFER_FREE() LCD_SERIAL.availableForWrite()
#endif

// Increase priority of serial interrupts, to reduce overflow errors
#define UART_IRQ_PRIO 1

struct MarlinSerial : public HardwareSerial {
  MarlinSerial(struct usart_dev *usart_device, uint8 tx_pin, uint8 rx_pin) : HardwareSerial(usart_device, tx_pin, rx_pin) { }

  #ifdef UART_IRQ_PRIO
    // Shadow the parent methods to set IRQ priority after begin()
    void begin(uint32 baud) {
      MarlinSerial::begin(baud, SERIAL_8N1);
    }

    void begin(uint32 baud, uint8_t config) {
      HardwareSerial::begin(baud, config);
      nvic_irq_set_priority(c_dev()->irq_num, UART_IRQ_PRIO);
    }
  #endif
};

typedef Serial1Class<MarlinSerial> MSerialT;

extern MSerialT MSerial1;
extern MSerialT MSerial2;
extern MSerialT MSerial3;
#if ANY(STM32_HIGH_DENSITY, STM32_XL_DENSITY)
  extern MSerialT MSerial4;
  extern MSerialT MSerial5;
#endif
