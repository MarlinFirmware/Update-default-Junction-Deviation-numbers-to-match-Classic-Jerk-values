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

#include "../inc/MarlinConfigPre.h"

#include <rs485/rs485bus.hpp>
#include <rs485/bus_adapters/hardware_serial.h>

#include <rs485/protocols/photon.h>
#include <rs485/packetizer.h>

#define RS485_SEND_BUFFER_SIZE 32

extern HardwareSerialBusIO rs485BusIO;
extern RS485Bus<RS485_BUS_BUFFER_SIZE> rs485Bus;
extern PhotonProtocol rs485Protocol;
extern Packetizer rs485Packetizer;
extern uint8_t rs485Buffer[RS485_SEND_BUFFER_SIZE];

void rs485_init();
