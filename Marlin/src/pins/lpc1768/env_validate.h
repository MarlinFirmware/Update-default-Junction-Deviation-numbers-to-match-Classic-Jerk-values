/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#ifndef ENV_VALIDATE_H
#define ENV_VALIDATE_H

#if NOT_TARGET(MCU_LPC1768, MCU_LPC1769)
  #if ENABLED(ALLOW_LPC1768_OR_9)
    #error "Oops! Make sure you have the LPC1768 or LPC1769 environment selected in your IDE."
  #elif ENABLED(REQUIRE_LPC1769)
    #error "Oops! Make sure you have the LPC1769 environment selected in your IDE."
  #else
    #error "Oops! Make sure you have the LPC1768 environment selected in your IDE."
  #endif
#endif

#undef ALLOW_LPC1768_OR_9
#undef REQUIRE_LPC1769

#endif // ENV_VALIDATE_H
