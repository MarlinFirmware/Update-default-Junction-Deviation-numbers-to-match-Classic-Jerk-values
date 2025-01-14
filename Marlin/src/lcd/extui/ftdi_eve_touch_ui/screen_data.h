/*****************
 * screen_data.h *
 *****************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

#pragma once

#include "ftdi_eve_lib/ftdi_eve_lib.h"

// To save RAM, store state information related to a particular screen
// in a union. The values should be initialized in the onEntry method.

/**
 * The DECL_DATA_IF_INCLUDED macro:
 *
 * union screen_data_t {
 *   DECL_DATA_IF_INCLUDED(FTDI_EXAMPLE_SCREEN)
 * }
 *
 * Is a shorthand for:
 *
 * union screen_data_t {
 *   #ifdef FTDI_EXAMPLE_SCREEN
 *     struct ExampleScreenData ExampleScreen;
 *   #endif
 * }
 *
 */
#define __DECL_DATA_IF_INCLUDED(CLASS) struct CLASS ## Data CLASS ;
#define _DECL_DATA_IF_INCLUDED(CLASS) __DECL_DATA_IF_INCLUDED(CLASS)
#define DECL_DATA_IF_INCLUDED(HEADER) TERN_(HEADER, _DECL_DATA_IF_INCLUDED(HEADER ## _CLASS))

union screen_data_t {
  DECL_DATA_IF_INCLUDED(FTDI_INTERFACE_SETTINGS_SCREEN)
  DECL_DATA_IF_INCLUDED(FTDI_LOCK_SCREEN)
  DECL_DATA_IF_INCLUDED(FTDI_SPINNER_DIALOG_BOX)
  DECL_DATA_IF_INCLUDED(FTDI_CONFIRM_START_PRINT_DIALOG_BOX)
  DECL_DATA_IF_INCLUDED(FTDI_CHANGE_FILAMENT_SCREEN)
  DECL_DATA_IF_INCLUDED(FTDI_FILES_SCREEN)
  DECL_DATA_IF_INCLUDED(FTDI_MOVE_AXIS_SCREEN)
  DECL_DATA_IF_INCLUDED(FTDI_BED_MESH_VIEW_SCREEN)
  DECL_DATA_IF_INCLUDED(FTDI_BED_MESH_EDIT_SCREEN)
  DECL_DATA_IF_INCLUDED(FTDI_STRESS_TEST_SCREEN)
  DECL_DATA_IF_INCLUDED(FTDI_NUDGE_NOZZLE_SCREEN)
  DECL_DATA_IF_INCLUDED(FTDI_Z_OFFSET_SCREEN)
  DECL_DATA_IF_INCLUDED(FTDI_BASE_NUMERIC_ADJ_SCREEN)
  DECL_DATA_IF_INCLUDED(FTDI_ALERT_DIALOG_BOX)
  DECL_DATA_IF_INCLUDED(FTDI_FILAMENT_PROMPT_DIALOG_BOX)
  DECL_DATA_IF_INCLUDED(COCOA_STATUS_SCREEN)
  DECL_DATA_IF_INCLUDED(COCOA_PREHEAT_SCREEN)
  DECL_DATA_IF_INCLUDED(COCOA_LOAD_CHOCOLATE_SCREEN)
  DECL_DATA_IF_INCLUDED(COCOA_FILES_SCREEN)
  DECL_DATA_IF_INCLUDED(COCOA_Z_OFFSET_WIZARD)
};

extern screen_data_t screen_data;
