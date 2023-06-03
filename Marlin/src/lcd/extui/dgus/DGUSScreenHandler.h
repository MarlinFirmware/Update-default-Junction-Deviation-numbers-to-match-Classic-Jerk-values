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

/**
 * lcd/extui/dgus/DGUSScreenHandler.h
 */

#include "../../../inc/MarlinConfigPre.h"

#include "../ui_api.h"

#if ENABLED(DGUS_FILAMENT_LOADUNLOAD)

  typedef struct  {
    ExtUI::extruder_t extruder; // which extruder to operate
    uint8_t action; // load or unload
    bool heated; // heating done ?
    float purge_length; // the length to extrude before unload, prevent filament jam
  } filament_data_t;

  extern filament_data_t filament_data;

#endif

// endianness swap
#define BE16_P(V) ( ((uint8_t*)(V))[0] << 8U | ((uint8_t*)(V))[1] )
#define BE32_P(V) ( ((uint8_t*)(V))[0] << 24U | ((uint8_t*)(V))[1] << 16U | ((uint8_t*)(V))[2] << 8U | ((uint8_t*)(V))[3] )

#if DGUS_LCD_UI_ORIGIN
  #include "origin/DGUSScreenHandler.h"
#elif DGUS_LCD_UI_MKS
  #include "mks/DGUSScreenHandler.h"
#elif DGUS_LCD_UI_FYSETC
  #include "fysetc/DGUSScreenHandler.h"
#elif DGUS_LCD_UI_HIPRECY
  #include "hiprecy/DGUSScreenHandler.h"
#elif DGUS_LCD_UI_CREALITY_TOUCH
  #include "creality/DGUSScreenHandler.h"
#endif

extern DGUSScreenHandlerClass screen;

// Helper to define a DGUS_VP_Variable for common use-cases.
#define VPHELPER(VPADR, VPADRVAR, RXFPTR, TXFPTR) { \
  .VP = VPADR, \
  .memadr = VPADRVAR, \
  .size = sizeof(VPADRVAR), \
  .set_by_display_handler = RXFPTR, \
  .send_to_display_handler = TXFPTR \
}

// Helper to define a DGUS_VP_Variable when the size of the var cannot be determined automatically (e.g., a string)
#define VPHELPER_STR(VPADR, VPADRVAR, STRLEN, RXFPTR, TXFPTR) { \
  .VP = VPADR, \
  .memadr = VPADRVAR, \
  .size = STRLEN, \
  .set_by_display_handler = RXFPTR, \
  .send_to_display_handler = TXFPTR \
}
