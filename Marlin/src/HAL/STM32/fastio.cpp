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
#include "../platforms.h"

#ifdef HAL_STM32

#include "../../inc/MarlinConfig.h"

GPIO_TypeDef* FastIOPortMap[LastPort + 1] = { 0 };

void FastIO_init() {
  for (uint_fast8_t i = 0; i < NUM_DIGITAL_PINS; ++i)
    FastIOPortMap[STM_PORT(digitalPin[i])] = get_GPIO_Port(STM_PORT(digitalPin[i]));
}

#endif // HAL_STM32
