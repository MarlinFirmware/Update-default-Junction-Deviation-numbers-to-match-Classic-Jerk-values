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
#include "../inc/MarlinConfig.h"

#if HAS_RS485_SERIAL

#include "rs485.h"

HardwareSerialBusIO rs485BusIO(&RS485_SERIAL);
RS485Bus<RS485_BUS_BUFFER_SIZE> rs485Bus(rs485BusIO, RS485_RX_ENABLE_PIN, RS485_TX_ENABLE_PIN);

PhotonProtocol rs485Protocol;

Packetizer rs485Packetizer(rs485Bus, rs485Protocol);

uint8_t rs485Buffer[RS485_SEND_BUFFER_SIZE];

void rs485_init() { RS485_SERIAL.begin(57600); }

#endif // HAS_RS485_SERIAL
