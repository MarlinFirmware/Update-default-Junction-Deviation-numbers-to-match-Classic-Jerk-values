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

/**
 * lcd/e3v2/jyersui/dwin.cpp
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(DWIN_CREALITY_LCD_JYERSUI)

#include "dwin.h"

#include "../../marlinui.h"
#include "../../../MarlinCore.h"

#include "../../../gcode/gcode.h"
#include "../../../module/temperature.h"
#include "../../../module/planner.h"
#include "../../../module/settings.h"
#include "../../../libs/buzzer.h"

//#define DEBUG_OUT 1
#include "../../../core/debug_out.h"

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "../../../feature/pause.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "../../../feature/runout.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../../../feature/host_actions.h"
#endif

#if ANY(BABYSTEPPING, HAS_BED_PROBE, HAS_WORKSPACE_OFFSET)
  #define HAS_ZOFFSET_ITEM 1
#endif

#if HAS_LEVELING
  #include "../../../feature/bedlevel/bedlevel.h"
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL)
  #include "../../../libs/least_squares_fit.h"
  #include "../../../libs/vector_3.h"
#endif

#if HAS_BED_PROBE
  #include "../../../module/probe.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

#if HAS_TRINAMIC_CONFIG
  #include "../../../module/stepper/trinamic.h"

  #define TMC_MIN_CURRENT 400
  #define TMC_MAX_CURRENT 1500
#endif

#define MACHINE_SIZE STRINGIFY(X_BED_SIZE) "x" STRINGIFY(Y_BED_SIZE) "x" STRINGIFY(Z_MAX_POS)

#define DWIN_FONT_MENU font8x16
#define DWIN_FONT_STAT font10x20
#define DWIN_FONT_HEAD font10x20

#define MENU_CHAR_LIMIT  24
#define STATUS_Y 352

#define MAX_PRINT_SPEED   999
#define MIN_PRINT_SPEED   10

#if HAS_FAN
  #define MAX_FAN_SPEED     255
  #define MIN_FAN_SPEED     0
#endif

#define MAX_XY_OFFSET 100

#if HAS_ZOFFSET_ITEM
  #define MAX_Z_OFFSET 9.99
  #if HAS_BED_PROBE
    #define MIN_Z_OFFSET -9.99
  #else
    #define MIN_Z_OFFSET -1
  #endif
#endif

#if HAS_HOTEND
  #define MAX_FLOW_RATE   299
  #define MIN_FLOW_RATE   10

  #define MAX_E_TEMP    thermalManager.hotend_max_target(0)
  #define MIN_E_TEMP    0
#endif

#if HAS_HEATED_BED
  #define MAX_BED_TEMP  BED_MAX_TARGET
  #define MIN_BED_TEMP  0
#endif

#define FEEDRATE_UNIT 1
#define ACCELERATION_UNIT 1
#define JERK_UNIT 10
#define STEPS_UNIT 10

//
// Custom menu items with JyersLCD
//
#if ENABLED(CUSTOM_MENU_CONFIG)
  #ifdef CONFIG_MENU_ITEM_5_DESC
    #define CUSTOM_MENU_COUNT 5
  #elif defined(CONFIG_MENU_ITEM_4_DESC)
    #define CUSTOM_MENU_COUNT 4
  #elif defined(CONFIG_MENU_ITEM_3_DESC)
    #define CUSTOM_MENU_COUNT 3
  #elif defined(CONFIG_MENU_ITEM_2_DESC)
    #define CUSTOM_MENU_COUNT 2
  #elif defined(CONFIG_MENU_ITEM_1_DESC)
    #define CUSTOM_MENU_COUNT 1
  #endif
  #if CUSTOM_MENU_COUNT
    #define HAS_CUSTOM_MENU 1
  #endif
#endif

constexpr uint16_t TROWS = 6, MROWS = TROWS - 1,
                   TITLE_HEIGHT = 30,
                   MLINE = 53,
                   LBLX = 60,
                   MENU_CHR_W = 8, MENU_CHR_H = 16, STAT_CHR_W = 10;

#define MBASE(L) (49 + MLINE * (L))

enum SelectItem : uint8_t {
  PAGE_PRINT = 0,
  PAGE_PREPARE,
  PAGE_CONTROL,
  PAGE_INFO_LEVELING,
  PAGE_COUNT,

  PRINT_SETUP = 0,
  PRINT_PAUSE_RESUME,
  PRINT_STOP,
  PRINT_COUNT
};

uint8_t active_menu = ID_MainMenu, last_menu = ID_MainMenu;
uint8_t selection = 0, last_selection = 0;
uint8_t scrollpos = 0;
uint8_t process = Proc_Main, last_process = Proc_Main;
PopupID popup, last_popup;

void (*funcpointer)() = nullptr;
void *valuepointer = nullptr;
float tempvalue;
float valuemin;
float valuemax;
uint8_t valueunit;
uint8_t valuetype;

char statusmsg[64];
char filename[LONG_FILENAME_LENGTH];
bool printing = false;
bool paused = false;
bool sdprint = false;

int16_t pausetemp, pausebed, pausefan;

bool livemove = false;
bool liveadjust = false;
uint8_t preheatmode = 0;
float zoffsetvalue = 0;
grid_count_t gridpoint;
float corner_avg;
float corner_pos;

bool probe_deployed = false;

JyersDWIN jyersDWIN;

template <unsigned N, unsigned S = N>
class TextScroller {
public:
  static const unsigned SIZE = N;
  static const unsigned SPACE = S;
  typedef char Buffer[SIZE + 1];

  inline TextScroller()
    : scrollpos(0)
  { }

  inline void reset() {
    scrollpos = 0;
  }

  const char* scroll(size_t& pos, Buffer &buf, const char * text, bool *updated = nullptr) {
    const size_t len = strlen(text);
    if (len > SIZE) {
      if (updated) *updated = true;
      if (scrollpos >= len + SPACE) scrollpos = 0;
      pos = 0;
      if (scrollpos < len) {
        const size_t n = min(len - scrollpos, SIZE);
        memcpy(buf, text + scrollpos, n);
        pos += n;
      }
      if (pos < SIZE) {
        const size_t n = min(len + SPACE - scrollpos, SIZE - pos);
        memset(buf + pos, ' ', n);
        pos += n;
      }
      if (pos < SIZE) {
        const size_t n = SIZE - pos;
        memcpy(buf + pos, text, n);
        pos += n;
      }
      buf[pos] = '\0';
      ++scrollpos;
      return buf;
    } else {
      pos = len;
      return text;
    }
  }

private:
  uint16_t scrollpos;
};

#if HAS_MESH

  struct {
    bool viewer_asymmetric_range = false;
    bool viewer_print_value = false;
    bool goto_mesh_value = false;
    bool drawing_mesh = false;
    uint8_t mesh_x = 0;
    uint8_t mesh_y = 0;

    #if ENABLED(AUTO_BED_LEVELING_UBL)
      uint8_t tilt_grid = 1;

      void manualValueUpdate(bool undefined=false) {
        gcode.process_subcommands_now(
          TS(F("M421I"), mesh_x, 'J', mesh_y, 'Z', p_float_t(current_position.z, 3), undefined ? "N" : "")
        );
        planner.synchronize();
      }

      bool createPlaneFromMesh() {
        struct linear_fit_data lsf_results;
        incremental_LSF_reset(&lsf_results);
        GRID_LOOP(x, y) {
          if (!isnan(bedlevel.z_values[x][y])) {
            xy_pos_t rpos = { bedlevel.get_mesh_x(x), bedlevel.get_mesh_y(y) };
            incremental_LSF(&lsf_results, rpos, bedlevel.z_values[x][y]);
          }
        }

        if (finish_incremental_LSF(&lsf_results)) {
          SERIAL_ECHOPGM("Could not complete LSF!");
          return true;
        }

        bedlevel.set_all_mesh_points_to_value(0);

        matrix_3x3 rotation = matrix_3x3::create_look_at(vector_3(lsf_results.A, lsf_results.B, 1));
        GRID_LOOP(i, j) {
          float mx = bedlevel.get_mesh_x(i), my = bedlevel.get_mesh_y(j), mz = bedlevel.z_values[i][j];

          if (DEBUGGING(LEVELING)) {
            DEBUG_ECHOLN(F("before rotation = ["), p_float_t(mx, 7), C(','), p_float_t(my, 7), C(','), p_float_t(mz, 7), F("]   ---> "));
            DEBUG_DELAY(20);
          }

          rotation.apply_rotation_xyz(mx, my, mz);

          if (DEBUGGING(LEVELING)) {
            DEBUG_ECHOLN(F("after rotation = ["), p_float_t(mx, 7), C(','), p_float_t(my, 7), C(','), p_float_t(mz, 7), C(']'));
            DEBUG_DELAY(20);
          }

          bedlevel.z_values[i][j] = mz - lsf_results.D;
        }
        return false;
      }

    #else

      void manualValueUpdate() {
        gcode.process_subcommands_now(
          TS(F("G29I"), mesh_x, 'J', mesh_y, 'Z', p_float_t(current_position.z, 3))
        );
        planner.synchronize();
      }

    #endif

    void manual_mesh_move(const bool zmove=false) {
      if (zmove) {
        planner.synchronize();
        current_position.z = goto_mesh_value ? bedlevel.z_values[mesh_x][mesh_y] : Z_CLEARANCE_BETWEEN_PROBES;
        planner.buffer_line(current_position, homing_feedrate(Z_AXIS), active_extruder);
        planner.synchronize();
      }
      else {
        jyersDWIN.popupHandler(Popup_MoveWait);
        gcode.process_subcommands_now(TS(F("G0F300Z"), p_float_t(current_position.z, 3)));
        gcode.process_subcommands_now(TS(F("G42 F4000 I"), mesh_x, 'J', mesh_y));
        planner.synchronize();
        current_position.z = goto_mesh_value ? bedlevel.z_values[mesh_x][mesh_y] : Z_CLEARANCE_BETWEEN_PROBES;
        planner.buffer_line(current_position, homing_feedrate(Z_AXIS), active_extruder);
        planner.synchronize();
        jyersDWIN.redrawMenu();
      }
    }

    float getMaxValue() {
      float max = -(__FLT_MAX__);
      GRID_LOOP(x, y) { const float z = bedlevel.z_values[x][y]; if (!isnan(z)) NOLESS(max, z); }
      return max;
    }

    float getMinValue() {
      float min = __FLT_MAX__;
      GRID_LOOP(x, y) { const float z = bedlevel.z_values[x][y]; if (!isnan(z)) NOMORE(min, z); }
      return min;
    }

    void drawBedMesh(const int16_t selected=-1, const uint8_t gridline_width=1, const uint16_t padding_x=8, const uint16_t padding_y_top=40 + 53 - 7) {
      drawing_mesh = true;
      const uint16_t total_width_px = DWIN_WIDTH - padding_x - padding_x,
                     cell_width_px  = total_width_px / (GRID_MAX_POINTS_X),
                     cell_height_px = total_width_px / (GRID_MAX_POINTS_Y);
      const float v_max = abs(getMaxValue()), v_min = abs(getMinValue()), rmax = _MAX(v_min, v_max);

      // Clear background from previous selection and select new square
      dwinDrawRectangle(1, COLOR_BG_BLACK, _MAX(0, padding_x - gridline_width), _MAX(0, padding_y_top - gridline_width), padding_x + total_width_px, padding_y_top + total_width_px);
      if (selected >= 0) {
        const auto selected_y = selected / (GRID_MAX_POINTS_X);
        const auto selected_x = selected - (GRID_MAX_POINTS_X) * selected_y;
        const auto start_y_px = padding_y_top + selected_y * cell_height_px;
        const auto start_x_px = padding_x + selected_x * cell_width_px;
        dwinDrawRectangle(1, COLOR_WHITE, _MAX(0, start_x_px - gridline_width), _MAX(0, start_y_px - gridline_width), start_x_px + cell_width_px, start_y_px + cell_height_px);
      }

      // Draw value square grid
      GRID_LOOP(x, y) {
        const auto start_x_px = padding_x + x * cell_width_px;
        const auto end_x_px   = start_x_px + cell_width_px - 1 - gridline_width;
        const auto start_y_px = padding_y_top + (GRID_MAX_POINTS_Y - y - 1) * cell_height_px;
        const auto end_y_px   = start_y_px + cell_height_px - 1 - gridline_width;
        dwinDrawRectangle(1,                                          // RGB565 colors: http://www.barth-dev.de/online/rgb565-color-picker/
          isnan(bedlevel.z_values[x][y]) ? COLOR_GREY : (             // gray if undefined
            (bedlevel.z_values[x][y] < 0 ?
              (uint16_t)round(0x1F * -bedlevel.z_values[x][y] / (!viewer_asymmetric_range ? rmax : v_min)) << 11 :  // red if mesh point value is negative
              (uint16_t)round(0x3F *  bedlevel.z_values[x][y] / (!viewer_asymmetric_range ? rmax : v_max)) << 5) |  // green if mesh point value is positive
                _MIN(0x1F, (((uint8_t)abs(bedlevel.z_values[x][y]) / 10) * 4))),                                    // + blue stepping for every mm
          start_x_px, start_y_px, end_x_px, end_y_px
        );

        safe_delay(10);
        LCD_SERIAL.flushTX();

        // Draw value text on
        if (viewer_print_value) {
          const int8_t offset_y = cell_height_px / 2 - 6;
          if (isnan(bedlevel.z_values[x][y])) {  // undefined
            dwinDrawString(false, font6x12, COLOR_WHITE, COLOR_BG_BLUE, start_x_px + cell_width_px / 2 - 5, start_y_px + offset_y, F("X"));
          }
          else {                          // has value
            MString<12> msg;
            if (GRID_MAX_POINTS_X < 10)
              msg.set(p_float_t(abs(bedlevel.z_values[x][y]), 2));
            else
              msg.setf(F("%02i"), uint16_t(abs(bedlevel.z_values[x][y] - int16_t(bedlevel.z_values[x][y])) * 100));
            const int8_t offset_x = cell_width_px / 2 - 3 * msg.length() - 2;
            if (GRID_MAX_POINTS_X >= 10)
              dwinDrawString(false, font6x12, COLOR_WHITE, COLOR_BG_BLUE, start_x_px - 2 + offset_x, start_y_px + offset_y /*+ square / 2 - 6*/, F("."));
            dwinDrawString(false, font6x12, COLOR_WHITE, COLOR_BG_BLUE, start_x_px + 1 + offset_x, start_y_px + offset_y /*+ square / 2 - 6*/, msg);
          }
          safe_delay(10);
          LCD_SERIAL.flushTX();
        }
      }
    }

    void setMeshViewerStatus() { // TODO: draw gradient with values as a legend instead
      float v1, v2,
            v_min = abs(getMinValue()),
            v_max = abs(getMaxValue());
      if (viewer_asymmetric_range) {
        if (v_min > 3e+10f) v_min = 0.0000001;
        if (v_max > 3e+10f) v_max = 0.0000001;
        v1 = -v_min;
        v2 =  v_max;
      }
      else {
        float rmax = _MAX(v_min, v_max), rmin = _MIN(v_min, v_max);
        if (rmax > 3e+10f) rmax = 0.0000001;
        if (rmin > 3e+10f) rmin = 0.0000001;
        v1 = -rmax;
        v2 =  rmin;
      }
      jyersDWIN.updateStatus(TS(GET_TEXT_F(MSG_COLORS_RED), ' ', p_float_t(v1, 3) , F("..0.."), p_float_t(v2, 3), ' ', GET_TEXT_F(MSG_COLORS_GREEN)));
      drawing_mesh = false;
    }

  } mesh_conf;

#endif // HAS_MESH

//
// General Display Functions
//

struct JyersDWIN::EEPROM_Settings JyersDWIN::eeprom_settings{0};
constexpr const char * const JyersDWIN::color_names[11];
constexpr const char * const JyersDWIN::preheat_modes[3];

// Clear a part of the screen
//  4=Entire screen
//  3=Title bar and Menu area (default)
//  2=Menu area
//  1=Title bar
void JyersDWIN::clearScreen(const uint8_t e/*=3*/) {
  if (e == 1 || e == 3 || e == 4) dwinDrawRectangle(1, getColor(eeprom_settings.menu_top_bg, COLOR_BG_BLUE, false), 0, 0, DWIN_WIDTH, TITLE_HEIGHT); // Clear Title Bar
  if (e == 2 || e == 3) dwinDrawRectangle(1, COLOR_BG_BLACK, 0, 31, DWIN_WIDTH, STATUS_Y); // Clear Menu Area
  if (e == 4) dwinDrawRectangle(1, COLOR_BG_BLACK, 0, 31, DWIN_WIDTH, DWIN_HEIGHT); // Clear Popup Area
}

void JyersDWIN::drawFloat(const_float_t value, const uint8_t row, const bool selected/*=false*/, const uint8_t minunit/*=10*/) {
  const uint8_t digits = (uint8_t)floor(log10(abs(value))) + log10(minunit) + (minunit > 1);
  const uint16_t bColor = selected ? COLOR_SELECT : COLOR_BG_BLACK;
  const uint16_t xpos = 240 - (digits * 8);
  dwinDrawRectangle(1, COLOR_BG_BLACK, 194, MBASE(row), 234 - (digits * 8), MBASE(row) + 16);
  if (isnan(value))
    dwinDrawString(true, DWIN_FONT_MENU, COLOR_WHITE, bColor, xpos - 8, MBASE(row), F(" NaN"));
  else {
    dwinDrawFloatValue(true, true, 0, DWIN_FONT_MENU, COLOR_WHITE, bColor, digits - log10(minunit) + 1, log10(minunit), xpos, MBASE(row), (value < 0 ? -value : value));
    dwinDrawString(true, DWIN_FONT_MENU, COLOR_WHITE, bColor, xpos - 8, MBASE(row), value < 0 ? F("-") : F(" "));
  }
}

void JyersDWIN::drawOption(const uint8_t value, const char * const * options, const uint8_t row, const bool selected/*=false*/, const bool color/*=false*/) {
  const uint16_t bColor = selected ? COLOR_SELECT : COLOR_BG_BLACK,
                 tColor = color ? getColor(value, COLOR_WHITE, false) : COLOR_WHITE;
  dwinDrawRectangle(1, bColor, 202, MBASE(row) + 14, 258, MBASE(row) - 2);
  dwinDrawString(false, DWIN_FONT_MENU, tColor, bColor, 202, MBASE(row) - 1, options[value]);
}

uint16_t JyersDWIN::getColor(const uint8_t color, const uint16_t original, const bool light/*=false*/) {
  switch (color) {
    case White:   return light ? COLOR_LIGHT_WHITE   : COLOR_WHITE;
    case Green:   return light ? COLOR_LIGHT_GREEN   : COLOR_GREEN;
    case Cyan:    return light ? COLOR_LIGHT_CYAN    : COLOR_CYAN;
    case Blue:    return light ? COLOR_LIGHT_BLUE    : COLOR_BLUE;
    case Magenta: return light ? COLOR_LIGHT_MAGENTA : COLOR_MAGENTA;
    case Red:     return light ? COLOR_LIGHT_RED     : COLOR_RED;
    case Orange:  return light ? COLOR_LIGHT_ORANGE  : COLOR_ORANGE;
    case Yellow:  return light ? COLOR_LIGHT_YELLOW  : COLOR_YELLOW;
    case Brown:   return light ? COLOR_LIGHT_BROWN   : COLOR_BROWN;
    case Black:   return COLOR_BLACK;
    case Default: return original;
  }
  return COLOR_WHITE;
}

void JyersDWIN::drawTitle(const char * const ctitle) {
  dwinDrawString(false, DWIN_FONT_HEAD, getColor(eeprom_settings.menu_top_txt, COLOR_WHITE, false), COLOR_BG_BLUE, (DWIN_WIDTH - strlen(ctitle) * STAT_CHR_W) / 2, 5, ctitle);
}
void JyersDWIN::drawTitle(FSTR_P const ftitle) {
  dwinDrawString(false, DWIN_FONT_HEAD, getColor(eeprom_settings.menu_top_txt, COLOR_WHITE, false), COLOR_BG_BLUE, (DWIN_WIDTH - strlen_P(FTOP(ftitle)) * STAT_CHR_W) / 2, 5, ftitle);
}

void _decorateMenuItem(uint8_t row, uint8_t icon, bool more) {
  if (icon) dwinIconShow(ICON, icon, 26, MBASE(row) - 3);       // Draw Menu Icon
  if (more) dwinIconShow(ICON, ICON_More, 226, MBASE(row) - 3); // Draw More Arrow
  dwinDrawLine(jyersDWIN.getColor(jyersDWIN.eeprom_settings.menu_split_line, COLOR_LINE, true), 16, MBASE(row) + 33, 256, MBASE(row) + 33); // Draw Menu Line
}

void JyersDWIN::drawMenuItem(const uint8_t row, const uint8_t icon/*=0*/, const char * const label1, const char * const label2, const bool more/*=false*/, const bool centered/*=false*/) {
  const uint8_t label_offset_y = label2 ? MENU_CHR_H * 3 / 5 : 0,
                label1_offset_x = !centered ? LBLX : LBLX * 4/5 + _MAX(LBLX * 1U/5, (DWIN_WIDTH - LBLX - (label1 ? strlen(label1) : 0) * MENU_CHR_W) / 2),
                label2_offset_x = !centered ? LBLX : LBLX * 4/5 + _MAX(LBLX * 1U/5, (DWIN_WIDTH - LBLX - (label2 ? strlen(label2) : 0) * MENU_CHR_W) / 2);
  if (label1) dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLACK, label1_offset_x, MBASE(row) - 1 - label_offset_y, label1); // Draw Label
  if (label2) dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLACK, label2_offset_x, MBASE(row) - 1 + label_offset_y, label2); // Draw Label
  _decorateMenuItem(row, icon, more);
}

void JyersDWIN::drawMenuItem(const uint8_t row, const uint8_t icon/*=0*/, FSTR_P const flabel1, FSTR_P const flabel2, const bool more/*=false*/, const bool centered/*=false*/) {
  const uint8_t label_offset_y = flabel2 ? MENU_CHR_H * 3 / 5 : 0,
                label1_offset_x = !centered ? LBLX : LBLX * 4/5 + _MAX(LBLX * 1U/5, (DWIN_WIDTH - LBLX - (flabel1 ? strlen_P(FTOP(flabel1)) : 0) * MENU_CHR_W) / 2),
                label2_offset_x = !centered ? LBLX : LBLX * 4/5 + _MAX(LBLX * 1U/5, (DWIN_WIDTH - LBLX - (flabel2 ? strlen_P(FTOP(flabel2)) : 0) * MENU_CHR_W) / 2);
  if (flabel1) dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLACK, label1_offset_x, MBASE(row) - 1 - label_offset_y, flabel1); // Draw Label
  if (flabel2) dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLACK, label2_offset_x, MBASE(row) - 1 + label_offset_y, flabel2); // Draw Label
  _decorateMenuItem(row, icon, more);
}

void JyersDWIN::drawCheckbox(const uint8_t row, const bool value) {
  #if ENABLED(DWIN_CREALITY_LCD_CUSTOM_ICONS)   // Draw appropriate checkbox icon
    dwinIconShow(ICON, (value ? ICON_Checkbox_T : ICON_Checkbox_F), 226, MBASE(row) - 3);
  #else                                         // Draw a basic checkbox using rectangles and lines
    dwinDrawRectangle(1, COLOR_BG_BLACK, 226, MBASE(row) - 3, 226 + 20, MBASE(row) - 3 + 20);
    dwinDrawRectangle(0, COLOR_WHITE, 226, MBASE(row) - 3, 226 + 20, MBASE(row) - 3 + 20);
    if (value) {
      dwinDrawLine(COLOR_CHECKBOX, 227, MBASE(row) - 3 + 11, 226 + 8, MBASE(row) - 3 + 17);
      dwinDrawLine(COLOR_CHECKBOX, 227 + 8, MBASE(row) - 3 + 17, 226 + 19, MBASE(row) - 3 + 1);
      dwinDrawLine(COLOR_CHECKBOX, 227, MBASE(row) - 3 + 12, 226 + 8, MBASE(row) - 3 + 18);
      dwinDrawLine(COLOR_CHECKBOX, 227 + 8, MBASE(row) - 3 + 18, 226 + 19, MBASE(row) - 3 + 2);
      dwinDrawLine(COLOR_CHECKBOX, 227, MBASE(row) - 3 + 13, 226 + 8, MBASE(row) - 3 + 19);
      dwinDrawLine(COLOR_CHECKBOX, 227 + 8, MBASE(row) - 3 + 19, 226 + 19, MBASE(row) - 3 + 3);
    }
  #endif
}

void JyersDWIN::drawMenu(const uint8_t menu, const uint8_t select/*=0*/, const uint8_t scroll/*=0*/) {
  if (active_menu != menu) {
    last_menu = active_menu;
    if (process == Proc_Menu) last_selection = selection;
  }
  selection = _MIN(select, getMenuSize(menu));
  scrollpos = scroll;
  if (selection - scrollpos > MROWS) scrollpos = selection - MROWS; // i.e., NOLESS(scrollpos, selection - MROWS);
  process = Proc_Menu;
  active_menu = menu;
  clearScreen();
  drawTitle(getMenuTitle(menu));
  for (uint8_t i = 0; i < TROWS; ++i) menuItemHandler(menu, i + scrollpos);
  dwinDrawRectangle(1, getColor(eeprom_settings.cursor_color, COLOR_RECTANGLE), 0, MBASE(selection - scrollpos) - 18, 14, MBASE(selection - scrollpos) + 33);
}

void JyersDWIN::redrawMenu(const bool lastproc/*=true*/, const bool lastsel/*=false*/, const bool lastmenu/*=false*/) {
  switch (lastproc ? last_process : process) {
    case Proc_Menu:
      drawMenu(lastmenu ? last_menu : active_menu, lastsel ? last_selection : selection, lastmenu ? 0 : scrollpos);
      break;
    case Proc_Main:  drawMainMenu(lastsel ? last_selection : selection); break;
    case Proc_Print: drawPrintScreen(); break;
    case Proc_File:  drawSDList(); break;
    default: break;
  }
}

void JyersDWIN::redrawScreen() {
  redrawMenu(false);
  drawStatusArea(true);
  updateStatusBar(true);
}

//
// Primary Menus and Screen Elements
//
void JyersDWIN::mainMenuIcons() {
  if (selection == 0) {
    dwinIconShow(ICON, ICON_Print_1, 17, 130);
    dwinDrawRectangle(0, getColor(eeprom_settings.highlight_box, COLOR_WHITE), 17, 130, 126, 229);
    dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 52, 200, GET_TEXT_F(MSG_BUTTON_PRINT));
  }
  else {
    dwinIconShow(ICON, ICON_Print_0, 17, 130);
    dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 52, 200, GET_TEXT_F(MSG_BUTTON_PRINT));
  }
  if (selection == 1) {
    dwinIconShow(ICON, ICON_Prepare_1, 145, 130);
    dwinDrawRectangle(0, getColor(eeprom_settings.highlight_box, COLOR_WHITE), 145, 130, 254, 229);
    dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 170, 200, GET_TEXT_F(MSG_PREPARE));
  }
  else {
    dwinIconShow(ICON, ICON_Prepare_0, 145, 130);
    dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 170, 200, GET_TEXT_F(MSG_PREPARE));
  }
  if (selection == 2) {
    dwinIconShow(ICON, ICON_Control_1, 17, 246);
    dwinDrawRectangle(0, getColor(eeprom_settings.highlight_box, COLOR_WHITE), 17, 246, 126, 345);
    dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 43, 317, GET_TEXT_F(MSG_CONTROL));
  }
  else {
    dwinIconShow(ICON, ICON_Control_0, 17, 246);
    dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 43, 317, GET_TEXT_F(MSG_CONTROL));
  }
  #if HAS_ABL_OR_UBL
    if (selection == 3) {
      dwinIconShow(ICON, ICON_Leveling_1, 145, 246);
      dwinDrawRectangle(0, getColor(eeprom_settings.highlight_box, COLOR_WHITE), 145, 246, 254, 345);
      dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 179, 317, GET_TEXT_F(MSG_BUTTON_LEVEL));
    }
    else {
      dwinIconShow(ICON, ICON_Leveling_0, 145, 246);
      dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 179, 317, GET_TEXT_F(MSG_BUTTON_LEVEL));
    }
  #else
    if (selection == 3) {
      dwinIconShow(ICON, ICON_Info_1, 145, 246);
      dwinDrawRectangle(0, getColor(eeprom_settings.highlight_box, COLOR_WHITE), 145, 246, 254, 345);
      dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 181, 317, GET_TEXT_F(MSG_BUTTON_INFO));
    }
    else {
      dwinIconShow(ICON, ICON_Info_0, 145, 246);
      dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 181, 317, GET_TEXT_F(MSG_BUTTON_INFO));
    }
  #endif
}

void JyersDWIN::drawMainMenu(const uint8_t select/*=0*/) {
  process = Proc_Main;
  active_menu = ID_MainMenu;
  selection = select;
  clearScreen();
  drawTitle(getMenuTitle(ID_MainMenu));
  SERIAL_ECHOPGM("\nDWIN handshake ");
  dwinIconShow(ICON, ICON_LOGO, 71, 72);
  mainMenuIcons();
}

void JyersDWIN::printScreenIcons() {
  if (selection == 0) {
    dwinIconShow(ICON, ICON_Setup_1, 8, 252);
    dwinDrawRectangle(0, getColor(eeprom_settings.highlight_box, COLOR_WHITE), 8, 252, 87, 351);
    dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 30, 322, GET_TEXT_F(MSG_TUNE));
  }
  else {
    dwinIconShow(ICON, ICON_Setup_0, 8, 252);
    dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 30, 322, GET_TEXT_F(MSG_TUNE));
  }
  if (selection == 2) {
    dwinIconShow(ICON, ICON_Stop_1, 184, 252);
    dwinDrawRectangle(0, getColor(eeprom_settings.highlight_box, COLOR_WHITE), 184, 252, 263, 351);
    dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 205, 322, GET_TEXT_F(MSG_BUTTON_STOP));
  }
  else {
    dwinIconShow(ICON, ICON_Stop_0, 184, 252);
    dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 205, 322, GET_TEXT_F(MSG_BUTTON_STOP));
  }
  if (paused) {
    if (selection == 1) {
      dwinIconShow(ICON, ICON_Continue_1, 96, 252);
      dwinDrawRectangle(0, getColor(eeprom_settings.highlight_box, COLOR_WHITE), 96, 252, 175, 351);
      dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 114, 322, GET_TEXT_F(MSG_BUTTON_PRINT));
    }
    else {
      dwinIconShow(ICON, ICON_Continue_0, 96, 252);
      dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 114, 322, GET_TEXT_F(MSG_BUTTON_PRINT));
    }
  }
  else {
    if (selection == 1) {
      dwinIconShow(ICON, ICON_Pause_1, 96, 252);
      dwinDrawRectangle(0, getColor(eeprom_settings.highlight_box, COLOR_WHITE), 96, 252, 175, 351);
      dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 114, 322, GET_TEXT_F(MSG_BUTTON_PAUSE));
    }
    else {
      dwinIconShow(ICON, ICON_Pause_0, 96, 252);
      dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLUE, 114, 322, GET_TEXT_F(MSG_BUTTON_PAUSE));
    }
  }
}

void JyersDWIN::drawPrintScreen() {
  process = Proc_Print;
  selection = 0;
  clearScreen();
  dwinDrawRectangle(1, COLOR_BG_BLACK, 8, 352, DWIN_WIDTH - 8, 376);
  drawTitle(GET_TEXT_F(MSG_PRINTING));
  printScreenIcons();
  dwinIconShow(ICON, ICON_PrintTime, 14, 171);
  dwinIconShow(ICON, ICON_RemainTime, 147, 169);
  dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLACK, 41, 163, F("Elapsed"));
  dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLACK, 176, 163, GET_TEXT_F(MSG_REMAINING_TIME));
  updateStatusBar(true);
  drawPrintProgressBar();
  drawPrintProgressElapsed();
  TERN_(SET_REMAINING_TIME, drawPrintProgressRemain());
  drawPrintFilename(true);
}

void JyersDWIN::drawPrintFilename(const bool reset/*=false*/) {
  typedef TextScroller<30> Scroller;
  static Scroller scroller;
  if (reset) scroller.reset();
  if (process == Proc_Print) {
    Scroller::Buffer buf;
    size_t outlen = 0;
    const char* outstr = scroller.scroll(outlen, buf, filename);
    dwinDrawRectangle(1, COLOR_BG_BLACK, 8, 50, DWIN_WIDTH - 8, 80);
    const int8_t npos = (DWIN_WIDTH - outlen * MENU_CHR_W) / 2;
    dwinDrawString(false, DWIN_FONT_MENU, COLOR_WHITE, COLOR_BG_BLACK, npos, 60, outstr);
  }
}

void JyersDWIN::drawPrintProgressBar() {
  uint8_t printpercent = sdprint ? card.percentDone() : (ui._get_progress() / 100);
  dwinIconShow(ICON, ICON_Bar, 15, 93);
  dwinDrawRectangle(1, COLOR_BARFILL, 16 + printpercent * 240 / 100, 93, 256, 113);
  dwinDrawIntValue(true, true, 0, DWIN_FONT_MENU, getColor(eeprom_settings.progress_percent, COLOR_PERCENT), COLOR_BG_BLACK, 3, 109, 133, printpercent);
  dwinDrawString(false, DWIN_FONT_MENU, getColor(eeprom_settings.progress_percent, COLOR_PERCENT), COLOR_BG_BLACK, 133, 133, F("%"));
}

#if ENABLED(SET_REMAINING_TIME)

  void JyersDWIN::drawPrintProgressRemain() {
    uint16_t remainingtime = ui.get_remaining_time();
    dwinDrawIntValue(true, true, 1, DWIN_FONT_MENU, getColor(eeprom_settings.progress_time, COLOR_WHITE), COLOR_BG_BLACK, 2, 176, 187, remainingtime / 3600);
    dwinDrawIntValue(true, true, 1, DWIN_FONT_MENU, getColor(eeprom_settings.progress_time, COLOR_WHITE), COLOR_BG_BLACK, 2, 200, 187, (remainingtime % 3600) / 60);
    if (eeprom_settings.time_format_textual) {
      dwinDrawString(false, DWIN_FONT_MENU, getColor(eeprom_settings.progress_time, COLOR_WHITE), COLOR_BG_BLACK, 192, 187, F("h"));
      dwinDrawString(false, DWIN_FONT_MENU, getColor(eeprom_settings.progress_time, COLOR_WHITE), COLOR_BG_BLACK, 216, 187, F("m"));
    }
    else
      dwinDrawString(false, DWIN_FONT_MENU, getColor(eeprom_settings.progress_time, COLOR_WHITE), COLOR_BG_BLACK, 192, 187, F(":"));
  }

#endif

void JyersDWIN::drawPrintProgressElapsed() {
  duration_t elapsed = print_job_timer.duration();
  dwinDrawIntValue(true, true, 1, DWIN_FONT_MENU, getColor(eeprom_settings.progress_time, COLOR_WHITE), COLOR_BG_BLACK, 2, 42, 187, elapsed.value / 3600);
  dwinDrawIntValue(true, true, 1, DWIN_FONT_MENU, getColor(eeprom_settings.progress_time, COLOR_WHITE), COLOR_BG_BLACK, 2, 66, 187, (elapsed.value % 3600) / 60);
  if (eeprom_settings.time_format_textual) {
    dwinDrawString(false, DWIN_FONT_MENU, getColor(eeprom_settings.progress_time, COLOR_WHITE), COLOR_BG_BLACK, 58, 187, F("h"));
    dwinDrawString(false, DWIN_FONT_MENU, getColor(eeprom_settings.progress_time, COLOR_WHITE), COLOR_BG_BLACK, 82, 187, F("m"));
  }
  else
    dwinDrawString(false, DWIN_FONT_MENU, getColor(eeprom_settings.progress_time, COLOR_WHITE), COLOR_BG_BLACK, 58, 187, F(":"));
}

void JyersDWIN::drawPrintConfirm() {
  drawPrintScreen();
  process = Proc_Confirm;
  popup = Popup_Complete;
  dwinDrawRectangle(1, COLOR_BG_BLACK, 8, 252, 263, 351);
  dwinIconShow(ICON, ICON_Confirm_E, 87, 283);
  dwinDrawRectangle(0, getColor(eeprom_settings.highlight_box, COLOR_WHITE), 86, 282, 187, 321);
  dwinDrawRectangle(0, getColor(eeprom_settings.highlight_box, COLOR_WHITE), 85, 281, 188, 322);
}

void JyersDWIN::drawSDItem(const uint8_t item, const uint8_t row) {
  if (item == 0)
    drawMenuItem(0, ICON_Back, card.flag.workDirIsRoot ? GET_TEXT_F(MSG_BACK) : F(".."));
  else {
    card.selectFileByIndexSorted(item - 1);
    char * const filename = card.longest_filename();
    size_t max = MENU_CHAR_LIMIT;
    size_t pos = strlen(filename), len = pos;
    if (!card.flag.filenameIsDir)
      while (pos && filename[pos] != '.') pos--;
    len = pos;
    if (len > max) len = max;
    char name[len + 1];
    for (uint8_t i = 0; i < len; ++i) name[i] = filename[i];
    if (pos > max)
      for (uint8_t i = len - 3; i < len; ++i) name[i] = '.';
    name[len] = '\0';
    drawMenuItem(row, card.flag.filenameIsDir ? ICON_More : ICON_File, name);
  }
}

void JyersDWIN::drawSDList(const bool removed/*=false*/) {
  clearScreen();
  drawTitle(GET_TEXT_F(MSG_MEDIA_MENU));
  selection = 0;
  scrollpos = 0;
  process = Proc_File;
  if (card.isMounted() && !removed) {
    for (uint8_t i = 0; i < _MIN(card.get_num_items() + 1, TROWS); ++i)
      drawSDItem(i, i);
  }
  else {
    drawMenuItem(0, ICON_Back, GET_TEXT_F(MSG_BACK));
    dwinDrawRectangle(1, COLOR_BG_RED, 10, MBASE(3) - 10, DWIN_WIDTH - 10, MBASE(4));
    dwinDrawString(false, font16x32, COLOR_YELLOW, COLOR_BG_RED, ((DWIN_WIDTH) - 8 * 16) / 2, MBASE(3), GET_TEXT_F(MSG_NO_MEDIA));
  }
  dwinDrawRectangle(1, getColor(eeprom_settings.cursor_color, COLOR_RECTANGLE), 0, MBASE(0) - 18, 14, MBASE(0) + 33);
}

void JyersDWIN::drawStatusArea(const bool icons/*=false*/) {

  if (icons) dwinDrawRectangle(1, COLOR_BG_BLACK, 0, STATUS_Y, DWIN_WIDTH, DWIN_HEIGHT - 1);

  #if HAS_HOTEND
    static float hotend = -1;
    static int16_t hotendtarget = -1, flow = -1;
    if (icons) {
      hotend = -1;
      hotendtarget = -1;
      dwinIconShow(ICON, ICON_HotendTemp, 10, 383);
      dwinDrawString(false, DWIN_FONT_STAT, getColor(eeprom_settings.status_area_text, COLOR_WHITE), COLOR_BG_BLACK, 25 + 3 * STAT_CHR_W + 5, 384, F("/"));
    }
    if (thermalManager.degHotend(0) != hotend) {
      hotend = thermalManager.degHotend(0);
      dwinDrawIntValue(true, true, 0, DWIN_FONT_STAT, getColor(eeprom_settings.status_area_text, COLOR_WHITE), COLOR_BG_BLACK, 3, 28, 384, hotend);
      dwinDrawDegreeSymbol(getColor(eeprom_settings.status_area_text, COLOR_WHITE), 25 + 3 * STAT_CHR_W + 5, 386);
    }
    if (thermalManager.wholeDegHotend(0) != hotendtarget) {
      hotendtarget = thermalManager.degTargetHotend(0);
      dwinDrawIntValue(true, true, 0, DWIN_FONT_STAT, getColor(eeprom_settings.status_area_text, COLOR_WHITE), COLOR_BG_BLACK, 3, 25 + 4 * STAT_CHR_W + 6, 384, hotendtarget);
      dwinDrawDegreeSymbol(getColor(eeprom_settings.status_area_text, COLOR_WHITE), 25 + 4 * STAT_CHR_W + 39, 386);
    }
    if (icons) {
      flow = -1;
      dwinIconShow(ICON, ICON_StepE, 112, 417);
      dwinDrawString(false, DWIN_FONT_STAT, getColor(eeprom_settings.status_area_text, COLOR_WHITE), COLOR_BG_BLACK, 116 + 5 * STAT_CHR_W + 2, 417, F("%"));
    }
    if (planner.flow_percentage[0] != flow) {
      flow = planner.flow_percentage[0];
      dwinDrawIntValue(true, true, 0, DWIN_FONT_STAT, getColor(eeprom_settings.status_area_text, COLOR_WHITE), COLOR_BG_BLACK, 3, 116 + 2 * STAT_CHR_W, 417, flow);
    }
  #endif

  #if HAS_HEATED_BED
    static float bed = -1;
    static int16_t bedtarget = -1;
    if (icons) {
      bed = -1;
      bedtarget = -1;
      dwinIconShow(ICON, ICON_BedTemp, 10, 416);
      dwinDrawString(false, DWIN_FONT_STAT, getColor(eeprom_settings.status_area_text, COLOR_WHITE), COLOR_BG_BLACK, 25 + 3 * STAT_CHR_W + 5, 417, F("/"));
    }
    if (thermalManager.degBed() != bed) {
      bed = thermalManager.degBed();
      dwinDrawIntValue(true, true, 0, DWIN_FONT_STAT, getColor(eeprom_settings.status_area_text, COLOR_WHITE), COLOR_BG_BLACK, 3, 28, 417, bed);
      dwinDrawDegreeSymbol(getColor(eeprom_settings.status_area_text, COLOR_WHITE), 25 + 3 * STAT_CHR_W + 5, 419);
    }
    if (thermalManager.degTargetBed() != bedtarget) {
      bedtarget = thermalManager.degTargetBed();
      dwinDrawIntValue(true, true, 0, DWIN_FONT_STAT, getColor(eeprom_settings.status_area_text, COLOR_WHITE), COLOR_BG_BLACK, 3, 25 + 4 * STAT_CHR_W + 6, 417, bedtarget);
      dwinDrawDegreeSymbol(getColor(eeprom_settings.status_area_text, COLOR_WHITE), 25 + 4 * STAT_CHR_W + 39, 419);
    }
  #endif

  #if HAS_FAN
    static uint8_t fan = -1;
    if (icons) {
      fan = -1;
      dwinIconShow(ICON, ICON_FanSpeed, 187, 383);
    }
    if (thermalManager.fan_speed[0] != fan) {
      fan = thermalManager.fan_speed[0];
      dwinDrawIntValue(true, true, 0, DWIN_FONT_STAT, getColor(eeprom_settings.status_area_text, COLOR_WHITE), COLOR_BG_BLACK, 3, 195 + 2 * STAT_CHR_W, 384, thermalManager.fan_speed[0]);
    }
  #endif

  #if HAS_ZOFFSET_ITEM
    static float offset = -1;

    if (icons) {
      offset = -1;
      dwinIconShow(ICON, ICON_Zoffset, 187, 416);
    }
    if (zoffsetvalue != offset) {
      offset = zoffsetvalue;
      dwinDrawFloatValue(true, true, 0, DWIN_FONT_STAT, getColor(eeprom_settings.status_area_text, COLOR_WHITE), COLOR_BG_BLACK, 2, 2, 207, 417, (zoffsetvalue < 0 ? -zoffsetvalue : zoffsetvalue));
      dwinDrawString(true, DWIN_FONT_MENU, getColor(eeprom_settings.status_area_text, COLOR_WHITE), COLOR_BG_BLACK, 205, 419, zoffsetvalue < 0 ? F("-") : F(" "));
    }
  #endif

  static int16_t feedrate = -1;
  if (icons) {
    feedrate = -1;
    dwinIconShow(ICON, ICON_Speed, 113, 383);
    dwinDrawString(false, DWIN_FONT_STAT, getColor(eeprom_settings.status_area_text, COLOR_WHITE), COLOR_BG_BLACK, 116 + 5 * STAT_CHR_W + 2, 384, F("%"));
  }
  if (feedrate_percentage != feedrate) {
    feedrate = feedrate_percentage;
    dwinDrawIntValue(true, true, 0, DWIN_FONT_STAT, getColor(eeprom_settings.status_area_text, COLOR_WHITE), COLOR_BG_BLACK, 3, 116 + 2 * STAT_CHR_W, 384, feedrate_percentage);
  }

  static float x = -1, y = -1, z = -1;
  static bool update_x = false, update_y = false, update_z = false;
  update_x = (current_position.x != x || axis_should_home(X_AXIS) || update_x);
  update_y = (current_position.y != y || axis_should_home(Y_AXIS) || update_y);
  update_z = (current_position.z != z || axis_should_home(Z_AXIS) || update_z);
  if (icons) {
    x = y = z = -1;
    dwinDrawLine(getColor(eeprom_settings.coordinates_split_line, COLOR_LINE, true), 16, 450, 256, 450);
    dwinIconShow(ICON, ICON_MaxSpeedX,  10, 456);
    dwinIconShow(ICON, ICON_MaxSpeedY,  95, 456);
    dwinIconShow(ICON, ICON_MaxSpeedZ, 180, 456);
  }
  if (update_x) {
    x = current_position.x;
    if ((update_x = axis_should_home(X_AXIS) && ui.get_blink()))
      dwinDrawString(true, DWIN_FONT_MENU, getColor(eeprom_settings.coordinates_text, COLOR_WHITE), COLOR_BG_BLACK, 35, 459, F("  -?-  "));
    else
      dwinDrawFloatValue(true, true, 0, DWIN_FONT_MENU, getColor(eeprom_settings.coordinates_text, COLOR_WHITE), COLOR_BG_BLACK, 3, 1, 35, 459, current_position.x);
  }
  if (update_y) {
    y = current_position.y;
    if ((update_y = axis_should_home(Y_AXIS) && ui.get_blink()))
      dwinDrawString(true, DWIN_FONT_MENU, getColor(eeprom_settings.coordinates_text, COLOR_WHITE), COLOR_BG_BLACK, 120, 459, F("  -?-  "));
    else
      dwinDrawFloatValue(true, true, 0, DWIN_FONT_MENU, getColor(eeprom_settings.coordinates_text, COLOR_WHITE), COLOR_BG_BLACK, 3, 1, 120, 459, current_position.y);
  }
  if (update_z) {
    z = current_position.z;
    if ((update_z = axis_should_home(Z_AXIS) && ui.get_blink()))
      dwinDrawString(true, DWIN_FONT_MENU, getColor(eeprom_settings.coordinates_text, COLOR_WHITE), COLOR_BG_BLACK, 205, 459, F("  -?-  "));
    else
      dwinDrawFloatValue(true, true, 0, DWIN_FONT_MENU, getColor(eeprom_settings.coordinates_text, COLOR_WHITE), COLOR_BG_BLACK, 3, 2, 205, 459, current_position.z >= 0 ? current_position.z : 0);
  }
  dwinUpdateLCD();
}

void JyersDWIN::drawPopup(FSTR_P const line1, FSTR_P const line2, FSTR_P const line3, uint8_t mode, uint8_t icon/*=0*/) {
  if (process != Proc_Confirm && process != Proc_Popup && process != Proc_Wait) last_process = process;
  if ((process == Proc_Menu || process == Proc_Wait) && mode == Proc_Popup) last_selection = selection;
  process = mode;
  clearScreen();
  dwinDrawRectangle(0, COLOR_WHITE, 13, 59, 259, 351);
  dwinDrawRectangle(1, COLOR_BG_WINDOW, 14, 60, 258, 350);
  const uint8_t ypos = (mode == Proc_Popup || mode == Proc_Confirm) ? 150 : 230;
  if (icon > 0) dwinIconShow(ICON, icon, 101, 105);
  dwinDrawString(true, DWIN_FONT_MENU, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * strlen_P(FTOP(line1))) / 2, ypos, line1);
  dwinDrawString(true, DWIN_FONT_MENU, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * strlen_P(FTOP(line2))) / 2, ypos + 30, line2);
  dwinDrawString(true, DWIN_FONT_MENU, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * strlen_P(FTOP(line3))) / 2, ypos + 60, line3);
  if (mode == Proc_Popup) {
    selection = 0;
    dwinDrawRectangle(1, COLOR_CONFIRM, 26, 280, 125, 317);
    dwinDrawRectangle(1, COLOR_CANCEL, 146, 280, 245, 317);
    dwinDrawString(false, DWIN_FONT_STAT, COLOR_WHITE, COLOR_BG_WINDOW, 39, 290, GET_TEXT_F(MSG_BUTTON_CONFIRM));
    dwinDrawString(false, DWIN_FONT_STAT, COLOR_WHITE, COLOR_BG_WINDOW, 165, 290, GET_TEXT_F(MSG_BUTTON_CANCEL));
    popupSelect();
  }
  else if (mode == Proc_Confirm) {
    dwinDrawRectangle(1, COLOR_CONFIRM, 87, 280, 186, 317);
    dwinDrawString(false, DWIN_FONT_STAT, COLOR_WHITE, COLOR_BG_WINDOW, 96, 290, GET_TEXT_F(MSG_BUTTON_CONTINUE));
  }
}

void MarlinUI::kill_screen(FSTR_P const error, FSTR_P const) {
  jyersDWIN.drawPopup(F("Printer Kill Reason:"), error, F("Restart Required"), Proc_Wait, ICON_BLTouch);
}

void JyersDWIN::popupSelect() {
  const uint16_t c1 = selection ? COLOR_BG_WINDOW : getColor(eeprom_settings.highlight_box, COLOR_WHITE),
                 c2 = selection ? getColor(eeprom_settings.highlight_box, COLOR_WHITE) : COLOR_BG_WINDOW;
  dwinDrawRectangle(0, c1, 25, 279, 126, 318);
  dwinDrawRectangle(0, c1, 24, 278, 127, 319);
  dwinDrawRectangle(0, c2, 145, 279, 246, 318);
  dwinDrawRectangle(0, c2, 144, 278, 247, 319);
}

void JyersDWIN::updateStatusBar(const bool refresh/*=false*/) {
  typedef TextScroller<30> Scroller;
  static bool new_msg;
  static Scroller scroller;
  static char lastmsg[64];
  if (strcmp(lastmsg, statusmsg) != 0 || refresh) {
    strcpy(lastmsg, statusmsg);
    scroller.reset();
    new_msg = true;
  }
  Scroller::Buffer buf;
  size_t len = 0;
  const char* dispmsg = scroller.scroll(len, buf, statusmsg, &new_msg);
  if (new_msg) {
    new_msg = false;
    if (process == Proc_Print) {
      dwinDrawRectangle(1, COLOR_GREY, 8, 214, DWIN_WIDTH - 8, 238);
      const int8_t npos = (DWIN_WIDTH - len * MENU_CHR_W) / 2;
      dwinDrawString(false, DWIN_FONT_MENU, getColor(eeprom_settings.status_bar_text, COLOR_WHITE), COLOR_BG_BLACK, npos, 219, dispmsg);
    }
    else {
      dwinDrawRectangle(1, COLOR_BG_BLACK, 8, 352, DWIN_WIDTH - 8, 376);
      const int8_t npos = (DWIN_WIDTH - len * MENU_CHR_W) / 2;
      dwinDrawString(false, DWIN_FONT_MENU, getColor(eeprom_settings.status_bar_text, COLOR_WHITE), COLOR_BG_BLACK, npos, 357, dispmsg);
    }
  }
}

//
// Menu Item Config
//

void JyersDWIN::menuItemHandler(const uint8_t menu, const uint8_t item, bool draw/*=true*/) {
  const uint8_t row = item - scrollpos;
  #if HAS_LEVELING
    static bool level_state;
  #endif

  #if HAS_PREHEAT

    #define PREHEAT_BACK 0
    #define PREHEAT_SUBMENU_HOTEND (PREHEAT_BACK + ENABLED(HAS_HOTEND))
    #define PREHEAT_SUBMENU_BED (PREHEAT_SUBMENU_HOTEND + ENABLED(HAS_HEATED_BED))
    #define PREHEAT_SUBMENU_FAN (PREHEAT_SUBMENU_BED + ENABLED(HAS_FAN))
    #define PREHEAT_SUBMENU_TOTAL PREHEAT_SUBMENU_FAN

    auto preheat_submenu = [&](const int index, const uint8_t item, const uint8_t sel) {
      switch (item) {
        case PREHEAT_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else
            drawMenu(ID_TempMenu, sel);
          break;
        #if HAS_HOTEND
          case PREHEAT_SUBMENU_HOTEND:
            if (draw) {
              drawMenuItem(row, ICON_SetEndTemp, GET_TEXT_F(MSG_NOZZLE));
              drawFloat(ui.material_preset[index].hotend_temp, row, false, 1);
            }
            else
              modifyValue(ui.material_preset[index].hotend_temp, MIN_E_TEMP, MAX_E_TEMP, 1);
            break;
        #endif
        #if HAS_HEATED_BED
          case PREHEAT_SUBMENU_BED:
            if (draw) {
              drawMenuItem(row, ICON_SetBedTemp, GET_TEXT_F(MSG_BED));
              drawFloat(ui.material_preset[index].bed_temp, row, false, 1);
            }
            else
              modifyValue(ui.material_preset[index].bed_temp, MIN_BED_TEMP, MAX_BED_TEMP, 1);
            break;
        #endif
        #if HAS_FAN
          case PREHEAT_SUBMENU_FAN:
            if (draw) {
              drawMenuItem(row, ICON_FanSpeed, GET_TEXT_F(MSG_FAN_SPEED));
              drawFloat(ui.material_preset[index].fan_speed, row, false, 1);
            }
            else
              modifyValue(ui.material_preset[index].fan_speed, MIN_FAN_SPEED, MAX_FAN_SPEED, 1);
            break;
        #endif
      }
    };

  #endif

  switch (menu) {
    case ID_Prepare:

      #define PREPARE_BACK 0
      #define PREPARE_MOVE (PREPARE_BACK + 1)
      #define PREPARE_DISABLE (PREPARE_MOVE + 1)
      #define PREPARE_HOME (PREPARE_DISABLE + 1)
      #define PREPARE_MANUALLEVEL (PREPARE_HOME + 1)
      #define PREPARE_ZOFFSET (PREPARE_MANUALLEVEL + ENABLED(HAS_ZOFFSET_ITEM))
      #define PREPARE_PREHEAT (PREPARE_ZOFFSET + ENABLED(HAS_PREHEAT))
      #define PREPARE_COOLDOWN (PREPARE_PREHEAT + ANY(HAS_HOTEND, HAS_HEATED_BED))
      #define PREPARE_CHANGEFIL (PREPARE_COOLDOWN + ENABLED(ADVANCED_PAUSE_FEATURE))
      #define PREPARE_CUSTOM_MENU (PREPARE_CHANGEFIL + ENABLED(HAS_CUSTOM_MENU))
      #define PREPARE_FWRETRACT (PREPARE_CUSTOM_MENU + ENABLED(FWRETRACT))
      #define PREPARE_TOTAL PREPARE_FWRETRACT

      switch (item) {
        case PREPARE_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else
            drawMainMenu(1);
          break;
        case PREPARE_MOVE:
          if (draw)
            drawMenuItem(row, ICON_Axis, GET_TEXT_F(MSG_MOVE_AXIS), nullptr, true);
          else
            drawMenu(ID_Move);
          break;
        case PREPARE_DISABLE:
          if (draw)
            drawMenuItem(row, ICON_CloseMotor, GET_TEXT_F(MSG_DISABLE_STEPPERS));
          else
            queue.inject(F("M84"));
          break;
        case PREPARE_HOME:
          if (draw)
            drawMenuItem(row, ICON_SetHome, GET_TEXT_F(MSG_HOMING), nullptr, true);
          else
            drawMenu(ID_HomeMenu);
          break;
        case PREPARE_MANUALLEVEL:
          if (draw)
            drawMenuItem(row, ICON_PrintSize, F("Manual Leveling"), nullptr, true);
          else {
            if (axes_should_home()) {
              popupHandler(Popup_Home);
              gcode.home_all_axes(true);
            }
            #if HAS_LEVELING
              level_state = planner.leveling_active;
              set_bed_leveling_enabled(false);
            #endif
            drawMenu(ID_ManualLevel);
          }
          break;

        #if HAS_ZOFFSET_ITEM
          case PREPARE_ZOFFSET:
            if (draw)
              drawMenuItem(row, ICON_Zoffset, F("Z-Offset"), nullptr, true);
            else {
              #if HAS_LEVELING
                level_state = planner.leveling_active;
                set_bed_leveling_enabled(false);
              #endif
              drawMenu(ID_ZOffset);
            }
            break;
        #endif

        #if HAS_PREHEAT
          case PREPARE_PREHEAT:
            if (draw)
              drawMenuItem(row, ICON_Temperature, F("Preheat"), nullptr, true);
            else
              drawMenu(ID_Preheat);
            break;
        #endif

        #if HAS_HOTEND || HAS_HEATED_BED
          case PREPARE_COOLDOWN:
            if (draw)
              drawMenuItem(row, ICON_Cool, GET_TEXT_F(MSG_COOLDOWN));
            else
              thermalManager.cooldown();
            break;
        #endif

        #if HAS_CUSTOM_MENU
          case PREPARE_CUSTOM_MENU:
            #ifndef CUSTOM_MENU_CONFIG_TITLE
              #define CUSTOM_MENU_CONFIG_TITLE "Custom Commands"
            #endif
            if (draw)
              drawMenuItem(row, ICON_Version, F(CUSTOM_MENU_CONFIG_TITLE));
            else
              drawMenu(ID_MenuCustom);
            break;
        #endif

        #if ENABLED(ADVANCED_PAUSE_FEATURE)
          case PREPARE_CHANGEFIL:
            if (draw) {
              drawMenuItem(row, ICON_ResetEEPROM, GET_TEXT_F(MSG_FILAMENTCHANGE)
                #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
                  , nullptr, true
                #endif
              );
            }
            else {
              #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
                drawMenu(ID_ChangeFilament);
              #else
                if (thermalManager.targetTooColdToExtrude(0))
                  popupHandler(Popup_ETemp);
                else {
                  if (thermalManager.temp_hotend[0].is_below_target(2)) {
                    popupHandler(Popup_Heating);
                    thermalManager.wait_for_hotend(0);
                  }
                  popupHandler(Popup_FilChange);
                  gcode.process_subcommands_now(TS(F("M600 B1 R"), thermalManager.degTargetHotend(0)));
                }
              #endif
            }
            break;
        #endif

        #if ENABLED(FWRETRACT)
          case PREPARE_FWRETRACT:
            if (draw)
              drawMenuItem(row, ICON_SetHome, GET_TEXT_F(MSG_FWRETRACT), nullptr, true);
            else
              drawMenu(ID_FWMenu);
            break;
        #endif
      }
      break;

    case ID_HomeMenu:

      #define HOME_BACK  0
      #define HOME_ALL   (HOME_BACK + 1)
      #define HOME_X     (HOME_ALL + 1)
      #define HOME_Y     (HOME_X + 1)
      #define HOME_Z     (HOME_Y + 1)
      #define HOME_SET   (HOME_Z + 1)
      #define HOME_TOTAL HOME_SET

      switch (item) {
        case HOME_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else
            drawMenu(ID_Prepare, PREPARE_HOME);
          break;
        case HOME_ALL:
          if (draw)
            drawMenuItem(row, ICON_Homing, GET_TEXT_F(MSG_AUTO_HOME));
          else {
            popupHandler(Popup_Home);
            gcode.home_all_axes(true);
            redrawMenu();
          }
          break;
        case HOME_X:
          if (draw)
            drawMenuItem(row, ICON_MoveX, GET_TEXT_F(MSG_AUTO_HOME_X));
          else {
            popupHandler(Popup_Home);
            gcode.process_subcommands_now(F("G28X"));
            planner.synchronize();
            redrawMenu();
          }
          break;
        case HOME_Y:
          if (draw)
            drawMenuItem(row, ICON_MoveY, GET_TEXT_F(MSG_AUTO_HOME_X));
          else {
            popupHandler(Popup_Home);
            gcode.process_subcommands_now(F("G28Y"));
            planner.synchronize();
            redrawMenu();
          }
          break;
        case HOME_Z:
          if (draw)
            drawMenuItem(row, ICON_MoveZ, GET_TEXT_F(MSG_AUTO_HOME_X));
          else {
            popupHandler(Popup_Home);
            gcode.process_subcommands_now(F("G28Z"));
            planner.synchronize();
            redrawMenu();
          }
          break;
        case HOME_SET:
          if (draw)
            drawMenuItem(row, ICON_SetHome, F("Set Home Here"));
          else {
            gcode.process_subcommands_now(F("G92X0Y0Z0"));
            audioFeedback();
          }
          break;
      }
      break;

    case ID_Move:

      #define MOVE_BACK 0
      #define MOVE_X (MOVE_BACK + 1)
      #define MOVE_Y (MOVE_X + 1)
      #define MOVE_Z (MOVE_Y + 1)
      #define MOVE_E (MOVE_Z + ENABLED(HAS_HOTEND))
      #define MOVE_P (MOVE_E + ENABLED(HAS_BED_PROBE))
      #define MOVE_LIVE (MOVE_P + 1)
      #define MOVE_TOTAL MOVE_LIVE

      switch (item) {
        case MOVE_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else {
            #if HAS_BED_PROBE
              probe_deployed = false;
              probe.set_deployed(probe_deployed);
            #endif
            drawMenu(ID_Prepare, PREPARE_MOVE);
          }
          break;
        case MOVE_X:
          if (draw) {
            drawMenuItem(row, ICON_MoveX, GET_TEXT_F(MSG_MOVE_X));
            drawFloat(current_position.x, row, false);
          }
          else
            modifyValue(current_position.x, X_MIN_POS, X_MAX_POS, 10);
          break;
        case MOVE_Y:
          if (draw) {
            drawMenuItem(row, ICON_MoveY, GET_TEXT_F(MSG_MOVE_Y));
            drawFloat(current_position.y, row);
          }
          else
            modifyValue(current_position.y, Y_MIN_POS, Y_MAX_POS, 10);
          break;
        case MOVE_Z:
          if (draw) {
            drawMenuItem(row, ICON_MoveZ, GET_TEXT_F(MSG_MOVE_Z));
            drawFloat(current_position.z, row);
          }
          else
            modifyValue(current_position.z, Z_MIN_POS, Z_MAX_POS, 10);
          break;

        #if HAS_HOTEND
          case MOVE_E:
            if (draw) {
              drawMenuItem(row, ICON_Extruder, GET_TEXT_F(MSG_MOVE_E));
              current_position.e = 0;
              sync_plan_position();
              drawFloat(current_position.e, row);
            }
            else {
              if (thermalManager.targetTooColdToExtrude(0)) {
                popupHandler(Popup_ETemp);
              }
              else {
                if (thermalManager.temp_hotend[0].is_below_target(2)) {
                  popupHandler(Popup_Heating);
                  thermalManager.wait_for_hotend(0);
                  redrawMenu();
                }
                current_position.e = 0;
                sync_plan_position();
                modifyValue(current_position.e, -500, 500, 10);
              }
            }
          break;
        #endif // HAS_HOTEND

        #if HAS_BED_PROBE
          case MOVE_P:
            if (draw) {
              drawMenuItem(row, ICON_StockConfiguration, F("Probe"));
              drawCheckbox(row, probe_deployed);
            }
            else {
              FLIP(probe_deployed);
              probe.set_deployed(probe_deployed);
              drawCheckbox(row, probe_deployed);
            }
            break;
        #endif

        case MOVE_LIVE:
          if (draw) {
            drawMenuItem(row, ICON_Axis, GET_TEXT_F(MSG_LIVE_MOVE));
            drawCheckbox(row, livemove);
          }
          else {
            FLIP(livemove);
            drawCheckbox(row, livemove);
          }
          break;
      }
      break;
    case ID_ManualLevel: {

      #define MLEVEL_BACK 0
      #define MLEVEL_PROBE (MLEVEL_BACK + ENABLED(HAS_BED_PROBE))
      #define MLEVEL_BL (MLEVEL_PROBE + 1)
      #define MLEVEL_TL (MLEVEL_BL + 1)
      #define MLEVEL_TR (MLEVEL_TL + 1)
      #define MLEVEL_BR (MLEVEL_TR + 1)
      #define MLEVEL_C (MLEVEL_BR + 1)
      #define MLEVEL_ZPOS (MLEVEL_C + 1)
      #define MLEVEL_TOTAL MLEVEL_ZPOS

      static float mlev_z_pos = 0;
      static bool use_probe = false;

      #if HAS_BED_PROBE
        const float probe_x_min = _MAX(0 + corner_pos, X_MIN_POS + probe.offset.x, X_MIN_POS + (PROBING_MARGIN_LEFT)) - probe.offset.x,
                    probe_x_max = _MIN((X_BED_SIZE + X_MIN_POS) - corner_pos, X_MAX_POS + probe.offset.x, X_MAX_POS - (PROBING_MARGIN_RIGHT)) - probe.offset.x,
                    probe_y_min = _MAX(0 + corner_pos, Y_MIN_POS + probe.offset.y, Y_MIN_POS + (PROBING_MARGIN_FRONT)) - probe.offset.y,
                    probe_y_max = _MIN((Y_BED_SIZE + Y_MIN_POS) - corner_pos, Y_MAX_POS + probe.offset.y, Y_MAX_POS - (PROBING_MARGIN_BACK)) - probe.offset.y;
      #endif

      switch (item) {
        case MLEVEL_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else {
            TERN_(HAS_LEVELING, set_bed_leveling_enabled(level_state));
            drawMenu(ID_Prepare, PREPARE_MANUALLEVEL);
          }
          break;

        #if HAS_BED_PROBE
          case MLEVEL_PROBE:
            if (draw) {
              drawMenuItem(row, ICON_Zoffset, F("Use Probe"));
              drawCheckbox(row, use_probe);
            }
            else {
              FLIP(use_probe);
              drawCheckbox(row, use_probe);
              if (use_probe) {
                popupHandler(Popup_Level);
                const struct { xy_pos_t p; ProbePtRaise r; } points[] = {
                  { { probe_x_min, probe_y_min }, PROBE_PT_RAISE },
                  { { probe_x_min, probe_y_max }, PROBE_PT_RAISE },
                  { { probe_x_max, probe_y_max }, PROBE_PT_RAISE },
                  { { probe_x_max, probe_y_min }, PROBE_PT_STOW }
                };
                corner_avg = 0;
                for (uint8_t i = 0; i < COUNT(points); i++) {
                  const float mz = probe.probe_at_point(points[i].p, points[i].r, 0, false);
                  if (isnan(mz)) { corner_avg = 0; break; }
                  corner_avg += mz;
                }
                corner_avg /= 4;
                redrawMenu();
              }
            }
            break;
        #endif

        case MLEVEL_BL:
          if (draw)
            drawMenuItem(row, ICON_AxisBL, GET_TEXT_F(MSG_BOTTOM_LEFT));
          else {
            popupHandler(Popup_MoveWait);
            if (use_probe) {
              #if HAS_BED_PROBE
                gcode.process_subcommands_now(
                  TS(F("G0F4000\nG0Z10\nG0X"), p_float_t(probe_x_min, 3), 'Y', p_float_t(probe_y_min, 3))
                );
                planner.synchronize();
                popupHandler(Popup_ManualProbing);
              #endif
            }
            else {
              gcode.process_subcommands_now(
                TS(F("G0F4000\nG0Z10\nG0X"), p_float_t(corner_pos, 3), 'Y', p_float_t(corner_pos, 3), F("\nG0F300Z"), p_float_t(mlev_z_pos, 3))
              );
              planner.synchronize();
              redrawMenu();
            }
          }
          break;
        case MLEVEL_TL:
          if (draw)
            drawMenuItem(row, ICON_AxisTL, GET_TEXT_F(MSG_TOP_LEFT));
          else {
            popupHandler(Popup_MoveWait);
            if (use_probe) {
              #if HAS_BED_PROBE
                gcode.process_subcommands_now(
                  TS(F("G0F4000\nG0Z10\nG0X"), p_float_t(probe_x_min, 3), 'Y', p_float_t(probe_y_max, 3))
                );
                planner.synchronize();
                popupHandler(Popup_ManualProbing);
              #endif
            }
            else {
              gcode.process_subcommands_now(
                TS(F("G0F4000\nG0Z10\nG0X"), p_float_t(corner_pos, 3), 'Y', p_float_t((Y_BED_SIZE + Y_MIN_POS) - corner_pos, 3), F("\nG0F300Z"), p_float_t(mlev_z_pos, 3))
              );
              planner.synchronize();
              redrawMenu();
            }
          }
          break;
        case MLEVEL_TR:
          if (draw)
            drawMenuItem(row, ICON_AxisTR, GET_TEXT_F(MSG_TOP_RIGHT));
          else {
            popupHandler(Popup_MoveWait);
            if (use_probe) {
              #if HAS_BED_PROBE
                gcode.process_subcommands_now(
                  TS(F("G0F4000\nG0Z10\nG0X"), p_float_t(probe_x_max, 3), 'Y', p_float_t(probe_y_max, 3), F("\nG0F300Z"), p_float_t(mlev_z_pos, 3))
                );
                planner.synchronize();
                popupHandler(Popup_ManualProbing);
              #endif
            }
            else {
              gcode.process_subcommands_now(
                TS(F("G0F4000\nG0Z10\nG0X"), p_float_t((X_BED_SIZE + X_MIN_POS) - corner_pos, 3), 'Y', p_float_t((Y_BED_SIZE + Y_MIN_POS) - corner_pos, 3), F("\nG0F300Z"), p_float_t(mlev_z_pos, 3))
              );
              planner.synchronize();
              redrawMenu();
            }
          }
          break;
        case MLEVEL_BR:
          if (draw)
            drawMenuItem(row, ICON_AxisBR, GET_TEXT_F(MSG_BOTTOM_RIGHT));
          else {
            popupHandler(Popup_MoveWait);
            if (use_probe) {
              #if HAS_BED_PROBE
                gcode.process_subcommands_now(
                  TS(F("G0F4000\nG0Z10\nG0X"), p_float_t(probe_x_max, 3), 'Y', p_float_t(probe_y_min, 3))
                );
                planner.synchronize();
                popupHandler(Popup_ManualProbing);
              #endif
            }
            else {
              gcode.process_subcommands_now(
                TS(F("G0F4000\nG0Z10\nG0X"), p_float_t((X_BED_SIZE + X_MIN_POS) - corner_pos, 3), 'Y', p_float_t(corner_pos, 3), F("\nG0F300Z"), p_float_t(mlev_z_pos, 3))
              );
              planner.synchronize();
              redrawMenu();
            }
          }
          break;
        case MLEVEL_C:
          if (draw)
            drawMenuItem(row, ICON_AxisC, GET_TEXT_F(MSG_TRAM_C));
          else {
            popupHandler(Popup_MoveWait);
            if (use_probe) {
              #if HAS_BED_PROBE
                gcode.process_subcommands_now(
                  TS(F("G0F4000\nG0Z10\nG0X"), p_float_t((X_MAX_POS) / 2.0f - probe.offset.x, 3), 'Y', p_float_t((Y_MAX_POS) / 2.0f - probe.offset.y, 3))
                );
                planner.synchronize();
                popupHandler(Popup_ManualProbing);
              #endif
            }
            else {
              gcode.process_subcommands_now(
                TS(F("G0F4000\nG0Z10\nG0X"), p_float_t((X_BED_SIZE + X_MIN_POS) - corner_pos, 3), 'Y', p_float_t((Y_BED_SIZE + Y_MIN_POS) / 2.0f, 3), F("\nG0F300Z"), p_float_t(mlev_z_pos, 3))
              );
              planner.synchronize();
              redrawMenu();
            }
          }
          break;
        case MLEVEL_ZPOS:
          if (draw) {
            drawMenuItem(row, ICON_SetZOffset, F("Z Position"));
            drawFloat(mlev_z_pos, row, false, 100);
          }
          else
            modifyValue(mlev_z_pos, 0, MAX_Z_OFFSET, 100);
          break;
      }

    } break;

    #if HAS_ZOFFSET_ITEM
      case ID_ZOffset:

        #define ZOFFSET_BACK 0
        #define ZOFFSET_HOME (ZOFFSET_BACK + 1)
        #define ZOFFSET_MODE (ZOFFSET_HOME + 1)
        #define ZOFFSET_OFFSET (ZOFFSET_MODE + 1)
        #define ZOFFSET_UP (ZOFFSET_OFFSET + 1)
        #define ZOFFSET_DOWN (ZOFFSET_UP + 1)
        #define ZOFFSET_SAVE (ZOFFSET_DOWN + ENABLED(EEPROM_SETTINGS))
        #define ZOFFSET_TOTAL ZOFFSET_SAVE

        switch (item) {
          case ZOFFSET_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else {
              liveadjust = false;
              TERN_(HAS_LEVELING, set_bed_leveling_enabled(level_state));
              drawMenu(ID_Prepare, PREPARE_ZOFFSET);
            }
            break;
          case ZOFFSET_HOME:
            if (draw)
              drawMenuItem(row, ICON_Homing, GET_TEXT_F(MSG_AUTO_HOME_Z));
            else {
              popupHandler(Popup_Home);
              gcode.process_subcommands_now(F("G28Z"));
              popupHandler(Popup_MoveWait);
              #if ENABLED(Z_SAFE_HOMING)
                planner.synchronize();
                gcode.process_subcommands_now(
                  TS(F("G0F4000X"), p_float_t(Z_SAFE_HOMING_X_POINT, 3), 'Y', p_float_t(Z_SAFE_HOMING_Y_POINT, 3))
                );
              #else
                gcode.process_subcommands_now(F("G0 F4000 X117.5 Y117.5"));
              #endif
              gcode.process_subcommands_now(F("G0F300Z0"));
              planner.synchronize();
              redrawMenu();
            }
            break;
          case ZOFFSET_MODE:
            if (draw) {
              drawMenuItem(row, ICON_Zoffset, F("Live Adjustment"));
              drawCheckbox(row, liveadjust);
            }
            else {
              if (!liveadjust) {
                if (axes_should_home()) {
                  popupHandler(Popup_Home);
                  gcode.home_all_axes(true);
                }
                popupHandler(Popup_MoveWait);
                #if ENABLED(Z_SAFE_HOMING)
                  planner.synchronize();
                  gcode.process_subcommands_now(
                    TS(F("G0F4000X"), p_float_t(Z_SAFE_HOMING_X_POINT, 3), 'Y', p_float_t(Z_SAFE_HOMING_Y_POINT, 3))
                  );
                #else
                  gcode.process_subcommands_now(F("G0 F4000 X117.5 Y117.5"));
                #endif
                gcode.process_subcommands_now(F("G0F300Z0"));
                planner.synchronize();
                redrawMenu();
              }
              FLIP(liveadjust);
              drawCheckbox(row, liveadjust);
            }
            break;
          case ZOFFSET_OFFSET:
            if (draw) {
              drawMenuItem(row, ICON_SetZOffset, F("Z Offset"));
              drawFloat(zoffsetvalue, row, false, 100);
            }
            else
              modifyValue(zoffsetvalue, MIN_Z_OFFSET, MAX_Z_OFFSET, 100);
            break;
          case ZOFFSET_UP:
            if (draw)
              drawMenuItem(row, ICON_Axis, F("+0.01mm Up"));
            else {
              if (zoffsetvalue < MAX_Z_OFFSET) {
                if (liveadjust) {
                  gcode.process_subcommands_now(F("M290 Z0.01"));
                  planner.synchronize();
                }
                zoffsetvalue += 0.01;
                drawFloat(zoffsetvalue, row - 1, false, 100);
              }
            }
            break;
          case ZOFFSET_DOWN:
            if (draw)
              drawMenuItem(row, ICON_AxisD, F("-0.01mm Down"));
            else {
              if (zoffsetvalue > MIN_Z_OFFSET) {
                if (liveadjust) {
                  gcode.process_subcommands_now(F("M290 Z-0.01"));
                  planner.synchronize();
                }
                zoffsetvalue -= 0.01;
                drawFloat(zoffsetvalue, row - 2, false, 100);
              }
            }
            break;
          #if ENABLED(EEPROM_SETTINGS)
            case ZOFFSET_SAVE:
              if (draw)
                drawMenuItem(row, ICON_WriteEEPROM, GET_TEXT_F(MSG_BUTTON_SAVE));
              else
                audioFeedback(settings.save());
              break;
          #endif
        }
        break;
    #endif

    #if HAS_PREHEAT
      case ID_Preheat: {
        #define PREHEAT_MODE (PREHEAT_BACK + 1)
        #define PREHEAT_1 (PREHEAT_MODE + 1)
        #define PREHEAT_2 (PREHEAT_1 + (PREHEAT_COUNT >= 2))
        #define PREHEAT_3 (PREHEAT_2 + (PREHEAT_COUNT >= 3))
        #define PREHEAT_4 (PREHEAT_3 + (PREHEAT_COUNT >= 4))
        #define PREHEAT_5 (PREHEAT_4 + (PREHEAT_COUNT >= 5))
        #define PREHEAT_TOTAL PREHEAT_5

        auto do_preheat = [](const uint8_t m) {
          thermalManager.cooldown();
          if (preheatmode == 0 || preheatmode == 1) { ui.preheat_hotend_and_fan(m); }
          if (preheatmode == 0 || preheatmode == 2) ui.preheat_bed(m);
        };

        switch (item) {
          case PREHEAT_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMenu(ID_Prepare, PREPARE_PREHEAT);
            break;
          case PREHEAT_MODE:
            if (draw) {
              drawMenuItem(row, ICON_Homing, F("Preheat Mode"));
              drawOption(preheatmode, preheat_modes, row);
            }
            else
              modifyOption(preheatmode, preheat_modes, 2);
            break;

          #define _PREHEAT_CASE(N) \
            case PREHEAT_##N: { \
              if (draw) drawMenuItem(row, ICON_Temperature, F(PREHEAT_## N ##_LABEL)); \
              else do_preheat(N - 1); \
            } break;

          REPEAT_1(PREHEAT_COUNT, _PREHEAT_CASE)
        }
      } break;
    #endif // HAS_PREHEAT

    #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
      case ID_ChangeFilament:

        #define CHANGEFIL_BACK 0
        #define CHANGEFIL_LOAD (CHANGEFIL_BACK + 1)
        #define CHANGEFIL_UNLOAD (CHANGEFIL_LOAD + 1)
        #define CHANGEFIL_CHANGE (CHANGEFIL_UNLOAD + 1)
        #define CHANGEFIL_TOTAL CHANGEFIL_CHANGE

        switch (item) {
          case CHANGEFIL_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMenu(ID_Prepare, PREPARE_CHANGEFIL);
            break;
          case CHANGEFIL_LOAD:
            if (draw)
              drawMenuItem(row, ICON_WriteEEPROM, GET_TEXT_F(MSG_FILAMENTLOAD));
            else {
              if (thermalManager.targetTooColdToExtrude(0))
                popupHandler(Popup_ETemp);
              else {
                if (thermalManager.temp_hotend[0].is_below_target(2)) {
                  popupHandler(Popup_Heating);
                  thermalManager.wait_for_hotend(0);
                }
                popupHandler(Popup_FilLoad);
                gcode.process_subcommands_now(F("M701"));
                planner.synchronize();
                redrawMenu();
              }
            }
            break;
          case CHANGEFIL_UNLOAD:
            if (draw)
              drawMenuItem(row, ICON_ReadEEPROM, GET_TEXT_F(MSG_FILAMENTUNLOAD));
            else {
              if (thermalManager.targetTooColdToExtrude(0)) {
                popupHandler(Popup_ETemp);
              }
              else {
                if (thermalManager.temp_hotend[0].is_below_target(2)) {
                  popupHandler(Popup_Heating);
                  thermalManager.wait_for_hotend(0);
                }
                popupHandler(Popup_FilLoad, true);
                gcode.process_subcommands_now(F("M702"));
                planner.synchronize();
                redrawMenu();
              }
            }
            break;
          case CHANGEFIL_CHANGE:
            if (draw)
              drawMenuItem(row, ICON_ResetEEPROM, GET_TEXT_F(MSG_FILAMENTCHANGE));
            else {
              if (thermalManager.targetTooColdToExtrude(0))
                popupHandler(Popup_ETemp);
              else {
                if (thermalManager.temp_hotend[0].is_below_target(2)) {
                  popupHandler(Popup_Heating);
                  thermalManager.wait_for_hotend(0);
                }
                popupHandler(Popup_FilChange);
                gcode.process_subcommands_now(TS(F("M600B1R"), thermalManager.degTargetHotend(0)));
              }
            }
            break;
        }
        break;
    #endif // FILAMENT_LOAD_UNLOAD_GCODES

    #if HAS_CUSTOM_MENU
      case ID_MenuCustom:

        #define CUSTOM_MENU_BACK 0
        #define CUSTOM_MENU_1 1
        #define CUSTOM_MENU_2 2
        #define CUSTOM_MENU_3 3
        #define CUSTOM_MENU_4 4
        #define CUSTOM_MENU_5 5
        #define CUSTOM_MENU_TOTAL CUSTOM_MENU_COUNT

        switch (item) {
          case CUSTOM_MENU_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMenu(ID_Prepare, PREPARE_CUSTOM_MENU);
            break;

          #if CUSTOM_MENU_COUNT >= 1
            case CUSTOM_MENU_1:
              if (draw)
                drawMenuItem(row, ICON_Info, F(CONFIG_MENU_ITEM_1_DESC));
              else {
                popupHandler(Popup_Custom);
                //queue.inject(F(CONFIG_MENU_ITEM_1_GCODE)); // Old code
                gcode.process_subcommands_now(F(CONFIG_MENU_ITEM_1_GCODE));
                planner.synchronize();
                redrawMenu();
                #if ENABLED(CUSTOM_MENU_CONFIG_SCRIPT_AUDIBLE_FEEDBACK)
                  audioFeedback();
                #endif
                #ifdef CUSTOM_MENU_CONFIG_SCRIPT_RETURN
                  queue.inject(F(CUSTOM_MENU_CONFIG_SCRIPT_DONE));
                #endif
              }
              break;
          #endif

          #if CUSTOM_MENU_COUNT >= 2
            case CUSTOM_MENU_2:
              if (draw)
                drawMenuItem(row, ICON_Info, F(CONFIG_MENU_ITEM_2_DESC));
              else {
                popupHandler(Popup_Custom);
                gcode.process_subcommands_now(F(CONFIG_MENU_ITEM_2_GCODE));
                planner.synchronize();
                redrawMenu();
                #if ENABLED(CUSTOM_MENU_CONFIG_SCRIPT_AUDIBLE_FEEDBACK)
                  audioFeedback();
                #endif
                #ifdef CUSTOM_MENU_CONFIG_SCRIPT_RETURN
                  queue.inject(F(CUSTOM_MENU_CONFIG_SCRIPT_DONE));
                #endif
              }
              break;
          #endif

          #if CUSTOM_MENU_COUNT >= 3
            case CUSTOM_MENU_3:
              if (draw)
                drawMenuItem(row, ICON_Info, F(CONFIG_MENU_ITEM_3_DESC));
              else {
                popupHandler(Popup_Custom);
                gcode.process_subcommands_now(F(CONFIG_MENU_ITEM_3_GCODE));
                planner.synchronize();
                redrawMenu();
                #if ENABLED(CUSTOM_MENU_CONFIG_SCRIPT_AUDIBLE_FEEDBACK)
                  audioFeedback();
                #endif
                #ifdef CUSTOM_MENU_CONFIG_SCRIPT_RETURN
                  queue.inject(F(CUSTOM_MENU_CONFIG_SCRIPT_DONE));
                #endif
              }
              break;
          #endif

          #if CUSTOM_MENU_COUNT >= 4
            case CUSTOM_MENU_4:
              if (draw)
                drawMenuItem(row, ICON_Info, F(CONFIG_MENU_ITEM_4_DESC));
              else {
                popupHandler(Popup_Custom);
                gcode.process_subcommands_now(F(CONFIG_MENU_ITEM_4_GCODE));
                planner.synchronize();
                redrawMenu();
                #if ENABLED(CUSTOM_MENU_CONFIG_SCRIPT_AUDIBLE_FEEDBACK)
                  audioFeedback();
                #endif
                #ifdef CUSTOM_MENU_CONFIG_SCRIPT_RETURN
                  queue.inject(F(CUSTOM_MENU_CONFIG_SCRIPT_DONE));
                #endif
              }
              break;
          #endif

          #if CUSTOM_MENU_COUNT >= 5
            case CUSTOM_MENU_5:
              if (draw)
                drawMenuItem(row, ICON_Info, F(CONFIG_MENU_ITEM_5_DESC));
              else {
                popupHandler(Popup_Custom);
                gcode.process_subcommands_now(F(CONFIG_MENU_ITEM_5_GCODE));
                planner.synchronize();
                redrawMenu();
                #if ENABLED(CUSTOM_MENU_CONFIG_SCRIPT_AUDIBLE_FEEDBACK)
                  audioFeedback();
                #endif
                #ifdef CUSTOM_MENU_CONFIG_SCRIPT_RETURN
                  queue.inject(F(CUSTOM_MENU_CONFIG_SCRIPT_DONE));
                #endif
              }
              break;
          #endif // Custom Menu
        }
        break;
    #endif // HAS_CUSTOM_MENU

    case ID_Control:

      #define CONTROL_BACK 0
      #define CONTROL_TEMP (CONTROL_BACK + 1)
      #define CONTROL_MOTION (CONTROL_TEMP + 1)
      #define CONTROL_VISUAL (CONTROL_MOTION + 1)
      #define CONTROL_ADVANCED (CONTROL_VISUAL + 1)
      #define CONTROL_SAVE (CONTROL_ADVANCED + ENABLED(EEPROM_SETTINGS))
      #define CONTROL_LOAD (CONTROL_SAVE + ENABLED(EEPROM_SETTINGS))
      #define CONTROL_RESET (CONTROL_LOAD + ENABLED(EEPROM_SETTINGS))
      #define CONTROL_INFO (CONTROL_RESET + 1)
      #define CONTROL_TOTAL CONTROL_INFO

      switch (item) {
        case CONTROL_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else
            drawMainMenu(2);
          break;
        case CONTROL_TEMP:
          if (draw)
            drawMenuItem(row, ICON_Temperature, GET_TEXT_F(MSG_TEMPERATURE), nullptr, true);
          else
            drawMenu(ID_TempMenu);
          break;
        case CONTROL_MOTION:
          if (draw)
            drawMenuItem(row, ICON_Motion, GET_TEXT_F(MSG_MOTION), nullptr, true);
          else
            drawMenu(ID_Motion);
          break;
        case CONTROL_VISUAL:
          if (draw)
            drawMenuItem(row, ICON_PrintSize, F("Visual"), nullptr, true);
          else
            drawMenu(ID_Visual);
          break;
        case CONTROL_ADVANCED:
          if (draw)
            drawMenuItem(row, ICON_Version, GET_TEXT_F(MSG_ADVANCED_SETTINGS), nullptr, true);
          else
            drawMenu(ID_Advanced);
          break;
        #if ENABLED(EEPROM_SETTINGS)
          case CONTROL_SAVE:
            if (draw)
              drawMenuItem(row, ICON_WriteEEPROM, GET_TEXT_F(MSG_STORE_EEPROM));
            else
              audioFeedback(settings.save());
            break;
          case CONTROL_LOAD:
            if (draw)
              drawMenuItem(row, ICON_ReadEEPROM, GET_TEXT_F(MSG_LOAD_EEPROM));
            else
              audioFeedback(settings.load());
            break;
          case CONTROL_RESET:
            if (draw)
              drawMenuItem(row, ICON_Temperature, GET_TEXT_F(MSG_RESTORE_DEFAULTS));
            else {
              settings.reset();
              audioFeedback();
            }
            break;
        #endif
        case CONTROL_INFO:
          if (draw)
            drawMenuItem(row, ICON_Info, GET_TEXT_F(MSG_INFO_SCREEN));
          else
            drawMenu(ID_Info);
          break;
      }
      break;

    case ID_TempMenu:

      #define TEMP_BACK 0
      #define TEMP_HOTEND (TEMP_BACK + ENABLED(HAS_HOTEND))
      #define TEMP_BED (TEMP_HOTEND + ENABLED(HAS_HEATED_BED))
      #define TEMP_FAN (TEMP_BED + ENABLED(HAS_FAN))
      #define TEMP_PID (TEMP_FAN + ANY(PIDTEMP, PIDTEMPBED))
      #define TEMP_MPC (TEMP_PID + ANY(MPC_EDIT_MENU, MPC_AUTOTUNE_MENU))
      #define TEMP_PREHEAT1 (TEMP_MPC + (PREHEAT_COUNT >= 1))
      #define TEMP_PREHEAT2 (TEMP_PREHEAT1 + (PREHEAT_COUNT >= 2))
      #define TEMP_PREHEAT3 (TEMP_PREHEAT2 + (PREHEAT_COUNT >= 3))
      #define TEMP_PREHEAT4 (TEMP_PREHEAT3 + (PREHEAT_COUNT >= 4))
      #define TEMP_PREHEAT5 (TEMP_PREHEAT4 + (PREHEAT_COUNT >= 5))
      #define TEMP_TOTAL TEMP_PREHEAT5

      switch (item) {
        case TEMP_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else
            drawMenu(ID_Control, CONTROL_TEMP);
          break;
        #if HAS_HOTEND
          case TEMP_HOTEND:
            if (draw) {
              drawMenuItem(row, ICON_SetEndTemp, GET_TEXT_F(MSG_NOZZLE));
              drawFloat(thermalManager.degTargetHotend(0), row, false, 1);
            }
            else
              modifyValue(thermalManager.temp_hotend[0].target, MIN_E_TEMP, MAX_E_TEMP, 1);
            break;
        #endif
        #if HAS_HEATED_BED
          case TEMP_BED:
            if (draw) {
              drawMenuItem(row, ICON_SetBedTemp, GET_TEXT_F(MSG_BED));
              drawFloat(thermalManager.degTargetBed(), row, false, 1);
            }
            else
              modifyValue(thermalManager.temp_bed.target, MIN_BED_TEMP, MAX_BED_TEMP, 1);
            break;
        #endif
        #if HAS_FAN
          case TEMP_FAN:
            if (draw) {
              drawMenuItem(row, ICON_FanSpeed, GET_TEXT_F(MSG_FAN_SPEED));
              drawFloat(thermalManager.fan_speed[0], row, false, 1);
            }
            else
              modifyValue(thermalManager.fan_speed[0], MIN_FAN_SPEED, MAX_FAN_SPEED, 1);
            break;
        #endif
        #if ANY(PIDTEMP, PIDTEMPBED)
          case TEMP_PID:
            if (draw)
              drawMenuItem(row, ICON_Step, F("PID"), nullptr, true);
            else
              drawMenu(ID_PID);
            break;
        #endif
        #if ANY(MPC_EDIT_MENU, MPC_AUTOTUNE_MENU)
          case TEMP_MPC:
            if (draw)
              drawMenuItem(row, ICON_Step, F("MPC"), nullptr, true);
            else
              drawMenu(ID_MPC);
            break;
        #endif

        #define _TEMP_PREHEAT_CASE(N) \
          case TEMP_PREHEAT##N: { \
            if (draw) drawMenuItem(row, ICON_Step, F(PREHEAT_## N ##_LABEL), nullptr, true); \
            else drawMenu(ID_Preheat##N); \
          } break;

        REPEAT_1(PREHEAT_COUNT, _TEMP_PREHEAT_CASE)
      }
      break;

    #if ANY(PIDTEMP, PIDTEMPBED)
      case ID_PID:

        #define PID_BACK 0
        #define PID_HOTEND (PID_BACK + ENABLED(PIDTEMP))
        #define PID_BED (PID_HOTEND + ENABLED(PIDTEMPBED))
        #define PID_CYCLES (PID_BED + 1)
        #define PID_TOTAL PID_CYCLES

        static uint8_t PID_cycles = 5;

        switch (item) {
          case PID_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMenu(ID_TempMenu, TEMP_PID);
            break;
          #if ENABLED(PIDTEMP)
            case PID_HOTEND:
              if (draw)
                drawMenuItem(row, ICON_HotendTemp, GET_TEXT_F(MSG_NOZZLE), nullptr, true);
              else
                drawMenu(ID_HotendPID);
              break;
          #endif
          #if ENABLED(PIDTEMPBED)
            case PID_BED:
              if (draw)
                drawMenuItem(row, ICON_BedTemp, GET_TEXT_F(MSG_BED), nullptr, true);
              else
                drawMenu(ID_BedPID);
              break;
          #endif
          case PID_CYCLES:
            if (draw) {
              drawMenuItem(row, ICON_FanSpeed, F("Cycles"));
              drawFloat(PID_cycles, row, false, 1);
            }
            else
              modifyValue(PID_cycles, 3, 50, 1);
            break;
        }
        break;
    #endif // PIDTEMP || PIDTEMPBED

    #if ENABLED(PIDTEMP)
      case ID_HotendPID:

        #define HOTENDPID_BACK 0
        #define HOTENDPID_TUNE (HOTENDPID_BACK + 1)
        #define HOTENDPID_TEMP (HOTENDPID_TUNE + 1)
        #define HOTENDPID_KP (HOTENDPID_TEMP + 1)
        #define HOTENDPID_KI (HOTENDPID_KP + 1)
        #define HOTENDPID_KD (HOTENDPID_KI + 1)
        #define HOTENDPID_TOTAL HOTENDPID_KD

        static uint16_t PID_e_temp = 180;

        switch (item) {
          case HOTENDPID_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMenu(ID_PID, PID_HOTEND);
            break;
          case HOTENDPID_TUNE:
            if (draw)
              drawMenuItem(row, ICON_HotendTemp, GET_TEXT_F(MSG_PID_AUTOTUNE));
            else {
              popupHandler(Popup_PIDWait);
              gcode.process_subcommands_now(TS(F("M303E0C"), PID_cycles, 'S', PID_e_temp, 'U'));
              planner.synchronize();
              redrawMenu();
            }
            break;
          case HOTENDPID_TEMP:
            if (draw) {
              drawMenuItem(row, ICON_Temperature, GET_TEXT_F(MSG_TEMPERATURE));
              drawFloat(PID_e_temp, row, false, 1);
            }
            else
              modifyValue(PID_e_temp, MIN_E_TEMP, MAX_E_TEMP, 1);
            break;
          case HOTENDPID_KP:
            if (draw) {
              drawMenuItem(row, ICON_Version, GET_TEXT_F(MSG_PID_P));
              drawFloat(thermalManager.temp_hotend[0].pid.p(), row, false, 100);
            }
            else
              modifyValue(thermalManager.temp_hotend[0].pid.Kp, 0, 5000, 100, thermalManager.updatePID);
            break;
          case HOTENDPID_KI:
            if (draw) {
              drawMenuItem(row, ICON_Version, GET_TEXT_F(MSG_PID_I));
              drawFloat(thermalManager.temp_hotend[0].pid.i(), row, false, 100);
            }
            else
              modifyValue(thermalManager.temp_hotend[0].pid.Ki, 0, 5000, 100, thermalManager.updatePID);
            break;
          case HOTENDPID_KD:
            if (draw) {
              drawMenuItem(row, ICON_Version, GET_TEXT_F(MSG_PID_D));
              drawFloat(thermalManager.temp_hotend[0].pid.d(), row, false, 100);
            }
            else
              modifyValue(thermalManager.temp_hotend[0].pid.Kd, 0, 5000, 100, thermalManager.updatePID);
            break;
        }
        break;
    #endif // PIDTEMP

    #if ENABLED(PIDTEMPBED)
      case ID_BedPID:

        #define BEDPID_BACK 0
        #define BEDPID_TUNE (BEDPID_BACK + 1)
        #define BEDPID_TEMP (BEDPID_TUNE + 1)
        #define BEDPID_KP (BEDPID_TEMP + 1)
        #define BEDPID_KI (BEDPID_KP + 1)
        #define BEDPID_KD (BEDPID_KI + 1)
        #define BEDPID_TOTAL BEDPID_KD

        static uint16_t PID_bed_temp = 60;

        switch (item) {
          case BEDPID_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMenu(ID_PID, PID_BED);
            break;
          case BEDPID_TUNE:
            if (draw)
              drawMenuItem(row, ICON_HotendTemp, GET_TEXT_F(MSG_PID_AUTOTUNE));
            else {
              popupHandler(Popup_PIDWait);
              gcode.process_subcommands_now(TS(F("M303E-1C"), PID_cycles, 'S', PID_bed_temp, 'U'));
              planner.synchronize();
              redrawMenu();
            }
            break;
          case BEDPID_TEMP:
            if (draw) {
              drawMenuItem(row, ICON_Temperature, GET_TEXT_F(MSG_TEMPERATURE));
              drawFloat(PID_bed_temp, row, false, 1);
            }
            else
              modifyValue(PID_bed_temp, MIN_BED_TEMP, MAX_BED_TEMP, 1);
            break;
          case BEDPID_KP:
            if (draw) {
              drawMenuItem(row, ICON_Version, GET_TEXT_F(MSG_PID_P));
              drawFloat(thermalManager.temp_bed.pid.p(), row, false, 100);
            }
            else
              modifyValue(thermalManager.temp_bed.pid.Kp, 0, 5000, 100, []{ thermalManager.temp_bed.pid.reset(); });
            break;
          case BEDPID_KI:
            if (draw) {
              drawMenuItem(row, ICON_Version, GET_TEXT_F(MSG_PID_I));
              drawFloat(thermalManager.temp_bed.pid.i(), row, false, 100);
            }
            else
              modifyValue(thermalManager.temp_bed.pid.Ki, 0, 5000, 100, []{ thermalManager.temp_bed.pid.reset(); });
            break;
          case BEDPID_KD:
            if (draw) {
              drawMenuItem(row, ICON_Version, GET_TEXT_F(MSG_PID_D));
              drawFloat(thermalManager.temp_bed.pid.d(), row, false, 100);
            }
            else
              modifyValue(thermalManager.temp_bed.pid.Kd, 0, 5000, 100, []{ thermalManager.temp_bed.pid.reset(); });
            break;
        }
        break;
    #endif // PIDTEMPBED

    #if ANY(MPC_EDIT_MENU, MPC_AUTOTUNE_MENU)
      case ID_MPC:

        #define MPCMENU_BACK 0
        #define MPCMENU_AUTOTUNE (MPCMENU_BACK + ENABLED(MPC_AUTOTUNE_MENU))
        #define MPCMENU_HEATER_POWER (MPCMENU_AUTOTUNE + ENABLED(MPC_EDIT_MENU))
        #define MPCMENU_BLOCK_HEAT_CAPACITY (MPCMENU_HEATER_POWER + ENABLED(MPC_EDIT_MENU))
        #define MPCMENU_SENSOR_RESPONSIVENESS (MPCMENU_BLOCK_HEAT_CAPACITY + ENABLED(MPC_EDIT_MENU))
        #define MPCMENU_AMBIENT_XFER_COEFF (MPCMENU_SENSOR_RESPONSIVENESS + ENABLED(MPC_EDIT_MENU))
        #define MPCMENU_AMBIENT_XFER_COEFF_FAN (MPCMENU_AMBIENT_XFER_COEFF + ALL(MPC_EDIT_MENU, MPC_INCLUDE_FAN))
        #define MPCMENU_TOTAL MPCMENU_AMBIENT_XFER_COEFF_FAN

        switch (item) {
          case MPCMENU_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMenu(ID_TempMenu, TEMP_MPC);
            break;

          #if ENABLED(MPC_AUTOTUNE_MENU)
            case MPCMENU_AUTOTUNE:
              if (draw)
                drawMenuItem(row, ICON_HotendTemp, GET_TEXT_F(MSG_PID_AUTOTUNE));
              else {
                popupHandler(Popup_MPCWait);
                thermalManager.MPC_autotune(active_extruder, Temperature::MPCTuningType::AUTO);
                redrawMenu();
              }
              break;
          #endif

          #if ENABLED(MPC_EDIT_MENU)
            case MPCMENU_HEATER_POWER:
              if (draw) {
                drawMenuItem(row, ICON_Version, GET_TEXT_F(MSG_MPC_POWER));
                drawFloat(thermalManager.temp_hotend[0].mpc.heater_power, row, false, 1);
              }
              else
                modifyValue(thermalManager.temp_hotend[0].mpc.heater_power, 1, 200, 1);
              break;

            case MPCMENU_BLOCK_HEAT_CAPACITY:
              if (draw) {
                drawMenuItem(row, ICON_Version, GET_TEXT_F(MSG_MPC_BLOCK_HEAT_CAPACITY));
                drawFloat(thermalManager.temp_hotend[0].mpc.block_heat_capacity, row, false, 100);
              }
              else
                modifyValue(thermalManager.temp_hotend[0].mpc.block_heat_capacity, 0, 40, 100);
              break;

            case MPCMENU_SENSOR_RESPONSIVENESS:
              if (draw) {
                drawMenuItem(row, ICON_Version, GET_TEXT_F(MSG_SENSOR_RESPONSIVENESS));
                drawFloat(thermalManager.temp_hotend[0].mpc.sensor_responsiveness, row, false, 10000);
              }
              else
                modifyValue(thermalManager.temp_hotend[0].mpc.sensor_responsiveness, 0, 1, 10000);
              break;

            case MPCMENU_AMBIENT_XFER_COEFF:
              if (draw) {
                drawMenuItem(row, ICON_Version, GET_TEXT_F(MSG_MPC_AMBIENT_XFER_COEFF));
                drawFloat(thermalManager.temp_hotend[0].mpc.ambient_xfer_coeff_fan0, row, false, 10000);
              }
              else
                modifyValue(thermalManager.temp_hotend[0].mpc.ambient_xfer_coeff_fan0, 0, 1, 10000);
              break;

            #if ENABLED(MPC_INCLUDE_FAN)
              case MPCMENU_AMBIENT_XFER_COEFF_FAN: {
                static float fan255_adjustment;
                if (draw) {
                  drawMenuItem(row, ICON_Version, GET_TEXT_F(MSG_MPC_AMBIENT_XFER_COEFF_FAN));
                  fan255_adjustment = thermalManager.temp_hotend[0].fanCoefficient();
                  drawFloat(fan255_adjustment, row, false, 10000);
                }
                else
                  modifyValue(fan255_adjustment, 0, 1, 10000, []{ thermalManager.temp_hotend[0].applyFanAdjustment(fan255_adjustment); });
              } break;
            #endif

          #endif // MPC_EDIT_MENU
        }
        break;

    #endif // MPC_EDIT_MENU || MPC_AUTOTUNE_MENU

    #if HAS_PREHEAT
      #define _PREHEAT_SUBMENU_CASE(N) case ID_Preheat##N: preheat_submenu((N) - 1, item, TEMP_PREHEAT##N); break;
      REPEAT_1(PREHEAT_COUNT, _PREHEAT_SUBMENU_CASE)
    #endif

    case ID_Motion:

      #define MOTION_BACK 0
      #define MOTION_HOMEOFFSETS (MOTION_BACK + 1)
      #define MOTION_SPEED (MOTION_HOMEOFFSETS + 1)
      #define MOTION_ACCEL (MOTION_SPEED + 1)
      #define MOTION_JERK (MOTION_ACCEL + ENABLED(CLASSIC_JERK))
      #define MOTION_STEPS (MOTION_JERK + 1)
      #define MOTION_FLOW (MOTION_STEPS + ENABLED(HAS_HOTEND))
      #define MOTION_LA (MOTION_FLOW + ENABLED(LIN_ADVANCE))
      #define MOTION_TOTAL MOTION_LA

      switch (item) {
        case MOTION_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else
            drawMenu(ID_Control, CONTROL_MOTION);
          break;
        case MOTION_HOMEOFFSETS:
          if (draw)
            drawMenuItem(row, ICON_SetHome, GET_TEXT_F(MSG_SET_HOME_OFFSETS), nullptr, true);
          else
            drawMenu(ID_HomeOffsets);
          break;
        case MOTION_SPEED:
          if (draw)
            drawMenuItem(row, ICON_MaxSpeed, GET_TEXT_F(MSG_MAX_SPEED), nullptr, true);
          else
            drawMenu(ID_MaxSpeed);
          break;
        case MOTION_ACCEL:
          if (draw)
            drawMenuItem(row, ICON_MaxAccelerated, GET_TEXT_F(MSG_ACCELERATION), nullptr, true);
          else
            drawMenu(ID_MaxAcceleration);
          break;

        #if ENABLED(CLASSIC_JERK)
          case MOTION_JERK:
            if (draw)
              drawMenuItem(row, ICON_MaxJerk, GET_TEXT_F(MSG_JERK), nullptr, true);
            else
              drawMenu(ID_MaxJerk);
            break;
        #endif

        #if ENABLED(EDITABLE_STEPS_PER_UNIT)
          case MOTION_STEPS:
            if (draw)
              drawMenuItem(row, ICON_Step, GET_TEXT_F(MSG_STEPS_PER_MM), nullptr, true);
            else
              drawMenu(ID_Steps);
            break;
        #endif

        #if HAS_HOTEND
          case MOTION_FLOW:
            if (draw) {
              drawMenuItem(row, ICON_Speed, GET_TEXT_F(MSG_FLOW));
              drawFloat(planner.flow_percentage[0], row, false, 1);
            }
            else
              modifyValue(planner.flow_percentage[0], MIN_FLOW_RATE, MAX_FLOW_RATE, 1, []{ planner.refresh_e_factor(0); });
            break;
        #endif

        #if ENABLED(LIN_ADVANCE)
          case MOTION_LA:
            if (draw) {
              drawMenuItem(row, ICON_MaxAccelerated, GET_TEXT_F(MSG_ADVANCE_K));
              drawFloat(planner.extruder_advance_K[0], row, false, 100);
            }
            else
              modifyValue(planner.extruder_advance_K[0], 0, 10, 100);
            break;
        #endif
      }
      break;

    case ID_HomeOffsets:

      #define HOMEOFFSETS_BACK 0
      #define HOMEOFFSETS_XOFFSET (HOMEOFFSETS_BACK + 1)
      #define HOMEOFFSETS_YOFFSET (HOMEOFFSETS_XOFFSET + 1)
      #define HOMEOFFSETS_TOTAL HOMEOFFSETS_YOFFSET

      switch (item) {
        case HOMEOFFSETS_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else
            drawMenu(ID_Motion, MOTION_HOMEOFFSETS);
          break;
        case HOMEOFFSETS_XOFFSET:
          if (draw) {
            drawMenuItem(row, ICON_StepX, GET_TEXT_F(MSG_HOME_OFFSET_X));
            drawFloat(home_offset.x, row, false, 100);
          }
          else
            modifyValue(home_offset.x, -MAX_XY_OFFSET, MAX_XY_OFFSET, 100);
          break;
        case HOMEOFFSETS_YOFFSET:
          if (draw) {
            drawMenuItem(row, ICON_StepY, GET_TEXT_F(MSG_HOME_OFFSET_Y));
            drawFloat(home_offset.y, row, false, 100);
          }
          else
            modifyValue(home_offset.y, -MAX_XY_OFFSET, MAX_XY_OFFSET, 100);
          break;
      }
      break;
    case ID_MaxSpeed:

      #define SPEED_BACK 0
      #define SPEED_X (SPEED_BACK + 1)
      #define SPEED_Y (SPEED_X + 1)
      #define SPEED_Z (SPEED_Y + 1)
      #define SPEED_E (SPEED_Z + ENABLED(HAS_HOTEND))
      #define SPEED_TOTAL SPEED_E

      switch (item) {
        case SPEED_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else
            drawMenu(ID_Motion, MOTION_SPEED);
          break;
        #if HAS_X_AXIS
          case SPEED_X:
            if (draw) {
              drawMenuItem(row, ICON_MaxSpeedX, GET_TEXT_F(MSG_VMAX_A));
              drawFloat(planner.settings.max_feedrate_mm_s[X_AXIS], row, false, FEEDRATE_UNIT);
            }
            else
              modifyValue(planner.settings.max_feedrate_mm_s[X_AXIS], min_feedrate_edit_values.x, max_feedrate_edit_values.x, FEEDRATE_UNIT);
            break;
        #endif

        #if HAS_Y_AXIS
          case SPEED_Y:
            if (draw) {
              drawMenuItem(row, ICON_MaxSpeedY, GET_TEXT_F(MSG_VMAX_B));
              drawFloat(planner.settings.max_feedrate_mm_s[Y_AXIS], row, false, FEEDRATE_UNIT);
            }
            else
              modifyValue(planner.settings.max_feedrate_mm_s[Y_AXIS], min_feedrate_edit_values.y, max_feedrate_edit_values.y, FEEDRATE_UNIT);
            break;
        #endif

        #if HAS_Z_AXIS
          case SPEED_Z:
            if (draw) {
              drawMenuItem(row, ICON_MaxSpeedZ, GET_TEXT_F(MSG_VMAX_C));
              drawFloat(planner.settings.max_feedrate_mm_s[Z_AXIS], row, false, FEEDRATE_UNIT);
            }
            else
              modifyValue(planner.settings.max_feedrate_mm_s[Z_AXIS], min_feedrate_edit_values.z, max_feedrate_edit_values.z, FEEDRATE_UNIT);
            break;
        #endif

        #if HAS_HOTEND
          case SPEED_E:
            if (draw) {
              drawMenuItem(row, ICON_MaxSpeedE, GET_TEXT_F(MSG_VMAX_E));
              drawFloat(planner.settings.max_feedrate_mm_s[E_AXIS], row, false, FEEDRATE_UNIT);
            }
            else
              modifyValue(planner.settings.max_feedrate_mm_s[E_AXIS], min_feedrate_edit_values.e, max_feedrate_edit_values.e, FEEDRATE_UNIT);
            break;
        #endif
      }
      break;

    case ID_MaxAcceleration:

      #define ACCEL_BACK 0
      #define ACCEL_X (ACCEL_BACK + ENABLED(HAS_X_AXIS))
      #define ACCEL_Y (ACCEL_X + ENABLED(HAS_Y_AXIS))
      #define ACCEL_Z (ACCEL_Y + ENABLED(HAS_Z_AXIS))
      #define ACCEL_E (ACCEL_Z + ENABLED(HAS_HOTEND))
      #define ACCEL_TOTAL ACCEL_E

      switch (item) {
        case ACCEL_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else
            drawMenu(ID_Motion, MOTION_ACCEL);
          break;
        #if HAS_X_AXIS
          case ACCEL_X:
            if (draw) {
              drawMenuItem(row, ICON_MaxAccX, GET_TEXT_F(MSG_AMAX_A));
              drawFloat(planner.settings.max_acceleration_mm_per_s2[X_AXIS], row, false, ACCELERATION_UNIT);
            }
            else
              modifyValue(planner.settings.max_acceleration_mm_per_s2[X_AXIS], min_acceleration_edit_values.x, max_acceleration_edit_values.x, ACCELERATION_UNIT);
            break;
        #endif
        #if HAS_Y_AXIS
          case ACCEL_Y:
            if (draw) {
              drawMenuItem(row, ICON_MaxAccY, GET_TEXT_F(MSG_AMAX_B));
              drawFloat(planner.settings.max_acceleration_mm_per_s2[Y_AXIS], row, false, ACCELERATION_UNIT);
            }
            else
              modifyValue(planner.settings.max_acceleration_mm_per_s2[Y_AXIS], min_acceleration_edit_values.y, max_acceleration_edit_values.y, ACCELERATION_UNIT);
            break;
        #endif
        #if HAS_Z_AXIS
          case ACCEL_Z:
            if (draw) {
              drawMenuItem(row, ICON_MaxAccZ, GET_TEXT_F(MSG_AMAX_C));
              drawFloat(planner.settings.max_acceleration_mm_per_s2[Z_AXIS], row, false, ACCELERATION_UNIT);
            }
            else
              modifyValue(planner.settings.max_acceleration_mm_per_s2[Z_AXIS], min_acceleration_edit_values.z, max_acceleration_edit_values.z, ACCELERATION_UNIT);
            break;
        #endif
        #if HAS_HOTEND
          case ACCEL_E:
            if (draw) {
              drawMenuItem(row, ICON_MaxAccE, GET_TEXT_F(MSG_AMAX_E));
              drawFloat(planner.settings.max_acceleration_mm_per_s2[E_AXIS], row, false, ACCELERATION_UNIT);
            }
            else
              modifyValue(planner.settings.max_acceleration_mm_per_s2[E_AXIS], min_acceleration_edit_values.e, max_acceleration_edit_values.e, ACCELERATION_UNIT);
            break;
        #endif
      }
      break;
    #if ENABLED(CLASSIC_JERK)
      case ID_MaxJerk:

        #define JERK_BACK 0
        #define JERK_X (JERK_BACK + ENABLED(HAS_X_AXIS))
        #define JERK_Y (JERK_X + ENABLED(HAS_Y_AXIS))
        #define JERK_Z (JERK_Y + ENABLED(HAS_Z_AXIS))
        #define JERK_E (JERK_Z + ENABLED(HAS_HOTEND))
        #define JERK_TOTAL JERK_E

        switch (item) {
          case JERK_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMenu(ID_Motion, MOTION_JERK);
            break;
          #if HAS_X_AXIS
            case JERK_X:
              if (draw) {
                drawMenuItem(row, ICON_MaxSpeedJerkX, GET_TEXT_F(MSG_VA_JERK));
                drawFloat(planner.max_jerk.x, row, false, JERK_UNIT);
              }
              else
                modifyValue(planner.max_jerk.x, min_jerk_edit_values.x, max_jerk_edit_values.x, JERK_UNIT);
              break;
          #endif
          #if HAS_Y_AXIS
            case JERK_Y:
              if (draw) {
                drawMenuItem(row, ICON_MaxSpeedJerkY, GET_TEXT_F(MSG_VB_JERK));
                drawFloat(planner.max_jerk.y, row, false, JERK_UNIT);
              }
              else
                modifyValue(planner.max_jerk.y, min_jerk_edit_values.y, max_jerk_edit_values.y, JERK_UNIT);
              break;
          #endif
          #if HAS_Z_AXIS
            case JERK_Z:
              if (draw) {
                drawMenuItem(row, ICON_MaxSpeedJerkZ, GET_TEXT_F(MSG_VC_JERK));
                drawFloat(planner.max_jerk.z, row, false, JERK_UNIT);
              }
              else
                modifyValue(planner.max_jerk.z, min_jerk_edit_values.z, max_jerk_edit_values.z, JERK_UNIT);
              break;
          #endif
          #if HAS_HOTEND
            case JERK_E:
              if (draw) {
                drawMenuItem(row, ICON_MaxSpeedJerkE, GET_TEXT_F(MSG_VE_JERK));
                drawFloat(planner.max_jerk.e, row, false, JERK_UNIT);
              }
              else
                modifyValue(planner.max_jerk.e, min_jerk_edit_values.e, max_jerk_edit_values.e, JERK_UNIT);
              break;
          #endif
        }
        break;
    #endif

    #if ENABLED(EDITABLE_STEPS_PER_UNIT)

      case ID_Steps:

        #define STEPS_BACK 0
        #define STEPS_X (STEPS_BACK + ENABLED(HAS_X_AXIS))
        #define STEPS_Y (STEPS_X + ENABLED(HAS_Y_AXIS))
        #define STEPS_Z (STEPS_Y + ENABLED(HAS_Z_AXIS))
        #define STEPS_E (STEPS_Z + ENABLED(HAS_HOTEND))
        #define STEPS_TOTAL STEPS_E

        switch (item) {
          case STEPS_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMenu(ID_Motion, MOTION_STEPS);
            break;
          #if HAS_X_AXIS
            case STEPS_X:
              if (draw) {
                drawMenuItem(row, ICON_StepX, GET_TEXT_F(MSG_A_STEPS));
                drawFloat(planner.settings.axis_steps_per_mm[X_AXIS], row, false, STEPS_UNIT);
              }
              else
                modifyValue(planner.settings.axis_steps_per_mm[X_AXIS], min_steps_edit_values.x, max_steps_edit_values.x, STEPS_UNIT);
              break;
          #endif
          #if HAS_Y_AXIS
            case STEPS_Y:
              if (draw) {
                drawMenuItem(row, ICON_StepY, GET_TEXT_F(MSG_B_STEPS));
                drawFloat(planner.settings.axis_steps_per_mm[Y_AXIS], row, false, STEPS_UNIT);
              }
              else
                modifyValue(planner.settings.axis_steps_per_mm[Y_AXIS], min_steps_edit_values.y, max_steps_edit_values.y, STEPS_UNIT);
              break;
          #endif
          #if HAS_Z_AXIS
            case STEPS_Z:
              if (draw) {
                drawMenuItem(row, ICON_StepZ, GET_TEXT_F(MSG_C_STEPS));
                drawFloat(planner.settings.axis_steps_per_mm[Z_AXIS], row, false, STEPS_UNIT);
              }
              else
                modifyValue(planner.settings.axis_steps_per_mm[Z_AXIS], min_steps_edit_values.z, max_steps_edit_values.z, STEPS_UNIT);
              break;
          #endif
          #if HAS_HOTEND
            case STEPS_E:
              if (draw) {
                drawMenuItem(row, ICON_StepE, GET_TEXT_F(MSG_E_STEPS));
                drawFloat(planner.settings.axis_steps_per_mm[E_AXIS], row, false, STEPS_UNIT);
              }
              else
                modifyValue(planner.settings.axis_steps_per_mm[E_AXIS], min_steps_edit_values.e, max_steps_edit_values.e, STEPS_UNIT);
              break;
          #endif
        }
        break;

    #endif // EDITABLE_STEPS_PER_UNIT

    case ID_Visual:

      #define VISUAL_BACK 0
      #define VISUAL_BACKLIGHT (VISUAL_BACK + 1)
      #define VISUAL_BRIGHTNESS (VISUAL_BACKLIGHT + 1)
      #define VISUAL_TIME_FORMAT (VISUAL_BRIGHTNESS + 1)
      #define VISUAL_COLOR_THEMES (VISUAL_TIME_FORMAT + 1)
      #define VISUAL_TOTAL VISUAL_COLOR_THEMES

      switch (item) {
        case VISUAL_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else
            drawMenu(ID_Control, CONTROL_VISUAL);
          break;
        case VISUAL_BACKLIGHT:
          if (draw)
            drawMenuItem(row, ICON_Brightness, GET_TEXT_F(MSG_BRIGHTNESS_OFF));
          else
            ui.set_brightness(0);
          break;
        case VISUAL_BRIGHTNESS:
          if (draw) {
            drawMenuItem(row, ICON_Brightness, GET_TEXT_F(MSG_BRIGHTNESS));
            drawFloat(ui.brightness, row, false, 1);
          }
          else
            modifyValue(ui.brightness, LCD_BRIGHTNESS_MIN, LCD_BRIGHTNESS_MAX, 1, ui.refresh_brightness);
          break;
        case VISUAL_TIME_FORMAT:
          if (draw) {
            drawMenuItem(row, ICON_PrintTime, F("Progress as __h__m"));
            drawCheckbox(row, eeprom_settings.time_format_textual);
          }
          else {
            FLIP(eeprom_settings.time_format_textual);
            drawCheckbox(row, eeprom_settings.time_format_textual);
          }
          break;
        case VISUAL_COLOR_THEMES:
          if (draw)
            drawMenuItem(row, ICON_MaxSpeed, F("UI Color Settings"), nullptr, true);
          else
            drawMenu(ID_ColorSettings);
        break;
      }
      break;

    case ID_ColorSettings:

      #define COLORSETTINGS_BACK 0
      #define COLORSETTINGS_CURSOR (COLORSETTINGS_BACK + 1)
      #define COLORSETTINGS_SPLIT_LINE (COLORSETTINGS_CURSOR + 1)
      #define COLORSETTINGS_MENU_TOP_TXT (COLORSETTINGS_SPLIT_LINE + 1)
      #define COLORSETTINGS_MENU_TOP_BG (COLORSETTINGS_MENU_TOP_TXT + 1)
      #define COLORSETTINGS_HIGHLIGHT_BORDER (COLORSETTINGS_MENU_TOP_BG + 1)
      #define COLORSETTINGS_PROGRESS_PERCENT (COLORSETTINGS_HIGHLIGHT_BORDER + 1)
      #define COLORSETTINGS_PROGRESS_TIME (COLORSETTINGS_PROGRESS_PERCENT + 1)
      #define COLORSETTINGS_PROGRESS_STATUS_BAR (COLORSETTINGS_PROGRESS_TIME + 1)
      #define COLORSETTINGS_PROGRESS_STATUS_AREA (COLORSETTINGS_PROGRESS_STATUS_BAR + 1)
      #define COLORSETTINGS_PROGRESS_COORDINATES (COLORSETTINGS_PROGRESS_STATUS_AREA + 1)
      #define COLORSETTINGS_PROGRESS_COORDINATES_LINE (COLORSETTINGS_PROGRESS_COORDINATES + 1)
      #define COLORSETTINGS_TOTAL COLORSETTINGS_PROGRESS_COORDINATES_LINE

      switch (item) {
        case COLORSETTINGS_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else
            drawMenu(ID_Visual, VISUAL_COLOR_THEMES);
          break;
        case COLORSETTINGS_CURSOR:
          if (draw) {
            drawMenuItem(row, ICON_MaxSpeed, F("Cursor"));
            drawOption(eeprom_settings.cursor_color, color_names, row, false, true);
          }
          else
            modifyOption(eeprom_settings.cursor_color, color_names, Custom_Colors);
          break;
        case COLORSETTINGS_SPLIT_LINE:
          if (draw) {
            drawMenuItem(row, ICON_MaxSpeed, F("Menu Split Line"));
            drawOption(eeprom_settings.menu_split_line, color_names, row, false, true);
          }
          else
            modifyOption(eeprom_settings.menu_split_line, color_names, Custom_Colors);
          break;
        case COLORSETTINGS_MENU_TOP_TXT:
          if (draw) {
            drawMenuItem(row, ICON_MaxSpeed, F("Menu Header Text"));
            drawOption(eeprom_settings.menu_top_txt, color_names, row, false, true);
          }
          else
            modifyOption(eeprom_settings.menu_top_txt, color_names, Custom_Colors);
          break;
        case COLORSETTINGS_MENU_TOP_BG:
          if (draw) {
            drawMenuItem(row, ICON_MaxSpeed, F("Menu Header Bg"));
            drawOption(eeprom_settings.menu_top_bg, color_names, row, false, true);
          }
          else
            modifyOption(eeprom_settings.menu_top_bg, color_names, Custom_Colors);
          break;
        case COLORSETTINGS_HIGHLIGHT_BORDER:
          if (draw) {
            drawMenuItem(row, ICON_MaxSpeed, F("Highlight Box"));
            drawOption(eeprom_settings.highlight_box, color_names, row, false, true);
          }
          else
            modifyOption(eeprom_settings.highlight_box, color_names, Custom_Colors);
          break;
        case COLORSETTINGS_PROGRESS_PERCENT:
          if (draw) {
            drawMenuItem(row, ICON_MaxSpeed, F("Progress Percent"));
            drawOption(eeprom_settings.progress_percent, color_names, row, false, true);
          }
          else
            modifyOption(eeprom_settings.progress_percent, color_names, Custom_Colors);
          break;
        case COLORSETTINGS_PROGRESS_TIME:
          if (draw) {
            drawMenuItem(row, ICON_MaxSpeed, F("Progress Time"));
            drawOption(eeprom_settings.progress_time, color_names, row, false, true);
          }
          else
            modifyOption(eeprom_settings.progress_time, color_names, Custom_Colors);
          break;
        case COLORSETTINGS_PROGRESS_STATUS_BAR:
          if (draw) {
            drawMenuItem(row, ICON_MaxSpeed, F("Status Bar Text"));
            drawOption(eeprom_settings.status_bar_text, color_names, row, false, true);
          }
          else
            modifyOption(eeprom_settings.status_bar_text, color_names, Custom_Colors);
          break;
        case COLORSETTINGS_PROGRESS_STATUS_AREA:
          if (draw) {
            drawMenuItem(row, ICON_MaxSpeed, F("Status Area Text"));
            drawOption(eeprom_settings.status_area_text, color_names, row, false, true);
          }
          else
            modifyOption(eeprom_settings.status_area_text, color_names, Custom_Colors);
          break;
        case COLORSETTINGS_PROGRESS_COORDINATES:
          if (draw) {
            drawMenuItem(row, ICON_MaxSpeed, F("Coordinates Text"));
            drawOption(eeprom_settings.coordinates_text, color_names, row, false, true);
          }
          else
            modifyOption(eeprom_settings.coordinates_text, color_names, Custom_Colors);
          break;
        case COLORSETTINGS_PROGRESS_COORDINATES_LINE:
          if (draw) {
            drawMenuItem(row, ICON_MaxSpeed, F("Coordinates Line"));
            drawOption(eeprom_settings.coordinates_split_line, color_names, row, false, true);
          }
          else
            modifyOption(eeprom_settings.coordinates_split_line, color_names, Custom_Colors);
          break;
      } // switch (item)
      break;

    case ID_Advanced:

      #define ADVANCED_BACK 0
      #define ADVANCED_BEEPER (ADVANCED_BACK + ENABLED(SOUND_MENU_ITEM))
      #define ADVANCED_PROBE (ADVANCED_BEEPER + ENABLED(HAS_BED_PROBE))
      #define ADVANCED_TMC (ADVANCED_PROBE + ENABLED(HAS_TRINAMIC_CONFIG))
      #define ADVANCED_CORNER (ADVANCED_TMC + 1)
      #define ADVANCED_LA (ADVANCED_CORNER + ENABLED(LIN_ADVANCE))
      #define ADVANCED_LOAD (ADVANCED_LA + ENABLED(ADVANCED_PAUSE_FEATURE))
      #define ADVANCED_UNLOAD (ADVANCED_LOAD + ENABLED(ADVANCED_PAUSE_FEATURE))
      #define ADVANCED_COLD_EXTRUDE  (ADVANCED_UNLOAD + ENABLED(PREVENT_COLD_EXTRUSION))
      #define ADVANCED_FILSENSORENABLED (ADVANCED_COLD_EXTRUDE + ENABLED(HAS_FILAMENT_SENSOR))
      #define ADVANCED_FILSENSORDISTANCE (ADVANCED_FILSENSORENABLED + ENABLED(HAS_FILAMENT_RUNOUT_DISTANCE))
      #define ADVANCED_POWER_LOSS (ADVANCED_FILSENSORDISTANCE + ENABLED(POWER_LOSS_RECOVERY))
      #define ADVANCED_TOTAL ADVANCED_POWER_LOSS

      switch (item) {
        case ADVANCED_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else
            drawMenu(ID_Control, CONTROL_ADVANCED);
          break;

        #if ENABLED(SOUND_MENU_ITEM)
          case ADVANCED_BEEPER:
            if (draw) {
              drawMenuItem(row, ICON_Version, GET_TEXT_F(MSG_SOUND));
              drawCheckbox(row, ui.sound_on);
            }
            else {
              FLIP(ui.sound_on);
              drawCheckbox(row, ui.sound_on);
            }
            break;
        #endif

        #if HAS_BED_PROBE
          case ADVANCED_PROBE:
            if (draw)
              drawMenuItem(row, ICON_StepX, F("Probe"), nullptr, true);
            else
              drawMenu(ID_ProbeMenu);
            break;
        #endif

        #if HAS_TRINAMIC_CONFIG
          case ADVANCED_TMC:
            if (draw)
              drawMenuItem(row, ICON_Motion, F("TMC Drivers"), nullptr, true);
            else
              drawMenu(ID_TMCMenu);
            break;
        #endif

        case ADVANCED_CORNER:
          if (draw) {
            drawMenuItem(row, ICON_MaxAccelerated, F("Bed Screw Inset"));
            drawFloat(corner_pos, row, false, 10);
          }
          else
            modifyValue(corner_pos, 1, 100, 10);
          break;

        #if ENABLED(LIN_ADVANCE)
          case ADVANCED_LA:
            if (draw) {
              drawMenuItem(row, ICON_MaxAccelerated, GET_TEXT_F(MSG_ADVANCE_K));
              drawFloat(planner.extruder_advance_K[0], row, false, 100);
            }
            else
              modifyValue(planner.extruder_advance_K[0], 0, 10, 100);
            break;
        #endif

        #if ENABLED(CONFIGURE_FILAMENT_CHANGE)
          case ADVANCED_LOAD:
            if (draw) {
              drawMenuItem(row, ICON_WriteEEPROM, F("Load Length"));
              drawFloat(fc_settings[0].load_length, row, false, 1);
            }
            else
              modifyValue(fc_settings[0].load_length, 0, EXTRUDE_MAXLENGTH, 1);
            break;
          case ADVANCED_UNLOAD:
            if (draw) {
              drawMenuItem(row, ICON_ReadEEPROM, F("Unload Length"));
              drawFloat(fc_settings[0].unload_length, row, false, 1);
            }
            else
              modifyValue(fc_settings[0].unload_length, 0, EXTRUDE_MAXLENGTH, 1);
            break;
        #endif // CONFIGURE_FILAMENT_CHANGE

        #if ENABLED(PREVENT_COLD_EXTRUSION)
          case ADVANCED_COLD_EXTRUDE:
            if (draw) {
              drawMenuItem(row, ICON_Cool, F("Min Extrusion T"));
              drawFloat(thermalManager.extrude_min_temp, row, false, 1);
            }
            else {
              modifyValue(thermalManager.extrude_min_temp, 0, MAX_E_TEMP, 1);
              thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0);
            }
            break;
        #endif

        #if HAS_FILAMENT_SENSOR
          case ADVANCED_FILSENSORENABLED:
            if (draw) {
              drawMenuItem(row, ICON_Extruder, GET_TEXT_F(MSG_RUNOUT_SENSOR));
              drawCheckbox(row, runout.enabled);
            }
            else {
              FLIP(runout.enabled);
              drawCheckbox(row, runout.enabled);
            }
            break;

          #if ENABLED(HAS_FILAMENT_RUNOUT_DISTANCE)
            case ADVANCED_FILSENSORDISTANCE:
              if (draw) {
                drawMenuItem(row, ICON_MaxAccE, GET_TEXT_F(MSG_RUNOUT_DISTANCE_MM));
                drawFloat(runout.runout_distance(), row, false, 10);
              }
              else
                modifyValue(runout.runout_distance(), 0, 999, 10);
              break;
          #endif
        #endif // HAS_FILAMENT_SENSOR

        #if ENABLED(POWER_LOSS_RECOVERY)
          case ADVANCED_POWER_LOSS:
            if (draw) {
              drawMenuItem(row, ICON_Motion, GET_TEXT_F(MSG_OUTAGE_RECOVERY));
              drawCheckbox(row, recovery.enabled);
            }
            else {
              recovery.enable(!recovery.enabled);
              drawCheckbox(row, recovery.enabled);
            }
            break;
        #endif
      }
      break;

    #if HAS_BED_PROBE
      case ID_ProbeMenu:

        #define PROBE_BACK 0
        #define PROBE_XOFFSET (PROBE_BACK + 1)
        #define PROBE_YOFFSET (PROBE_XOFFSET + 1)
        #define PROBE_TEST (PROBE_YOFFSET + 1)
        #define PROBE_TEST_COUNT (PROBE_TEST + 1)
        #define PROBE_TOTAL PROBE_TEST_COUNT

        static uint8_t testcount = 4;

        switch (item) {
          case PROBE_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMenu(ID_Advanced, ADVANCED_PROBE);
            break;

            case PROBE_XOFFSET:
              if (draw) {
                drawMenuItem(row, ICON_StepX, F("Probe X Offset"));
                drawFloat(probe.offset.x, row, false, 10);
              }
              else
                modifyValue(probe.offset.x, -MAX_XY_OFFSET, MAX_XY_OFFSET, 10);
              break;
            case PROBE_YOFFSET:
              if (draw) {
                drawMenuItem(row, ICON_StepY, F("Probe Y Offset"));
                drawFloat(probe.offset.y, row, false, 10);
              }
              else
                modifyValue(probe.offset.y, -MAX_XY_OFFSET, MAX_XY_OFFSET, 10);
              break;
            case PROBE_TEST:
              if (draw)
                drawMenuItem(row, ICON_StepY, F("M48 Probe Test"));
              else {
                gcode.process_subcommands_now(
                  TS(F("G28O\nM48X") , p_float_t((X_BED_SIZE + X_MIN_POS) / 2.0f, 3), 'Y', p_float_t((Y_BED_SIZE + Y_MIN_POS) / 2.0f, 3), 'P', testcount)
                );
              }
              break;
            case PROBE_TEST_COUNT:
              if (draw) {
                drawMenuItem(row, ICON_StepY, F("Probe Test Count"));
                drawFloat(testcount, row, false, 1);
              }
              else
                modifyValue(testcount, 4, 50, 1);
              break;
        }
        break;
    #endif  // HAS_PROBE_MENU

    #if HAS_TRINAMIC_CONFIG
      case ID_TMCMenu:

        #define TMC_BACK 0
        #define TMC_STEPPER_CURRENT_X (TMC_BACK + AXIS_IS_TMC(X))
        #define TMC_STEPPER_CURRENT_Y (TMC_STEPPER_CURRENT_X + AXIS_IS_TMC(Y))
        #define TMC_STEPPER_CURRENT_Z (TMC_STEPPER_CURRENT_Y + AXIS_IS_TMC(Z))
        #define TMC_STEPPER_CURRENT_E (TMC_STEPPER_CURRENT_Z + AXIS_IS_TMC(E0))
        #define TMC_TOTAL TMC_STEPPER_CURRENT_E

        switch (item) {

          case TMC_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMenu(ID_Advanced, ADVANCED_TMC);
            break;

          #if AXIS_IS_TMC(X)
            case TMC_STEPPER_CURRENT_X:

              static float stepper_current_x;

              if (draw) {
                drawMenuItem(row, ICON_StepX, GET_TEXT_F(MSG_TMC_ACURRENT));
                stepper_current_x = stepperX.getMilliamps();
                drawFloat(stepper_current_x, row, false, 1);
              }
              else {
                modifyValue(stepper_current_x, TMC_MIN_CURRENT, TMC_MAX_CURRENT, 1, []{ stepperX.rms_current(stepper_current_x); });
              }
              break;
          #endif

          #if AXIS_IS_TMC(Y)
            case TMC_STEPPER_CURRENT_Y:
              static float stepper_current_y;
              if (draw) {
                drawMenuItem(row, ICON_StepY, GET_TEXT_F(MSG_TMC_BCURRENT));
                stepper_current_y = stepperY.getMilliamps();
                drawFloat(stepper_current_y, row, false, 1);
              }
              else {
                modifyValue(stepper_current_y, TMC_MIN_CURRENT, TMC_MAX_CURRENT, 1, []{ stepperY.rms_current(stepper_current_y); });
              }
              break;
          #endif

          #if AXIS_IS_TMC(Z)
            case TMC_STEPPER_CURRENT_Z:
              static float stepper_current_z;
              if (draw) {
                drawMenuItem(row, ICON_StepZ, GET_TEXT_F(MSG_TMC_CCURRENT));
                stepper_current_z = stepperZ.getMilliamps();
                drawFloat(stepper_current_z, row, false, 1);
              }
              else {
                modifyValue(stepper_current_z, TMC_MIN_CURRENT, TMC_MAX_CURRENT, 1, []{ stepperZ.rms_current(stepper_current_z); });
              }
              break;
          #endif

          #if AXIS_IS_TMC(E0)
            case TMC_STEPPER_CURRENT_E:
              static float stepper_current_e;
              if (draw) {
                drawMenuItem(row, ICON_StepE, GET_TEXT_F(MSG_TMC_ECURRENT));
                stepper_current_e = stepperE0.getMilliamps();
                drawFloat(stepper_current_e, row, false, 1);
              }
              else
                modifyValue(stepper_current_e, TMC_MIN_CURRENT, TMC_MAX_CURRENT, 1, []{ stepperE0.rms_current(stepper_current_e); });
              break;
          #endif
        };
        break;
    #endif // HAS_TRINAMIC_CONFIG

    case ID_InfoMain:
    case ID_Info:

      #define INFO_BACK 0
      #define INFO_PRINTCOUNT (INFO_BACK + ENABLED(PRINTCOUNTER))
      #define INFO_PRINTTIME (INFO_PRINTCOUNT + ENABLED(PRINTCOUNTER))
      #define INFO_SIZE (INFO_PRINTTIME + 1)
      #define INFO_VERSION (INFO_SIZE + 1)
      #define INFO_CONTACT (INFO_VERSION + 1)
      #define INFO_TOTAL INFO_BACK

      switch (item) {
        case INFO_BACK:
          if (draw) {
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));

            #if ENABLED(PRINTCOUNTER)

              printStatistics ps = print_job_timer.getStats();
              drawMenuItem(INFO_PRINTCOUNT, ICON_HotendTemp,
                TS(ps.totalPrints, F(" prints, "), ps.finishedPrints, F(" finished")),
                TS(p_float_t(ps.filamentUsed / 1000, 2), F(" m filament used")),
                false, true
              );

              char buf[32];
              drawMenuItem(INFO_PRINTTIME, ICON_PrintTime,
                MString<50>(F("Printed: "), duration_t(print_job_timer.getStats().printTime).toString(buf)),
                MString<50>(F("Longest: "), duration_t(print_job_timer.getStats().longestPrint).toString(buf)),
                false, true
              );

            #endif

            drawMenuItem(INFO_SIZE, ICON_PrintSize, F(MACHINE_SIZE), nullptr, false, true);
            drawMenuItem(INFO_VERSION, ICON_Version, F(SHORT_BUILD_VERSION), nullptr, false, true);
            drawMenuItem(INFO_CONTACT, ICON_Contact, F(CORP_WEBSITE), nullptr, false, true);
          }
          else {
            if (menu == ID_Info)
              drawMenu(ID_Control, CONTROL_INFO);
            else
              drawMainMenu(3);
          }
          break;
      }
      break;

    #if HAS_MESH
      case ID_Leveling:

        #define LEVELING_BACK 0
        #define LEVELING_ACTIVE (LEVELING_BACK + 1)
        #define LEVELING_GET_TILT (LEVELING_ACTIVE + ALL(HAS_BED_PROBE, AUTO_BED_LEVELING_UBL))
        #define LEVELING_GET_MESH (LEVELING_GET_TILT + 1)
        #define LEVELING_MANUAL (LEVELING_GET_MESH + 1)
        #define LEVELING_VIEW (LEVELING_MANUAL + 1)
        #define LEVELING_SETTINGS (LEVELING_VIEW + 1)
        #define LEVELING_SLOT (LEVELING_SETTINGS + ENABLED(AUTO_BED_LEVELING_UBL))
        #define LEVELING_LOAD (LEVELING_SLOT + ENABLED(AUTO_BED_LEVELING_UBL))
        #define LEVELING_SAVE (LEVELING_LOAD + ENABLED(AUTO_BED_LEVELING_UBL))
        #define LEVELING_TOTAL LEVELING_SAVE

        switch (item) {
          case LEVELING_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMainMenu(3);
            break;
          case LEVELING_ACTIVE:
            if (draw) {
              drawMenuItem(row, ICON_StockConfiguration, GET_TEXT_F(MSG_BED_LEVELING));
              drawCheckbox(row, planner.leveling_active);
            }
            else {
              if (!planner.leveling_active) {
                set_bed_leveling_enabled(!planner.leveling_active);
                if (!planner.leveling_active) {
                  confirmHandler(Popup_LevelError);
                  break;
                }
              }
              else
                set_bed_leveling_enabled(!planner.leveling_active);
              drawCheckbox(row, planner.leveling_active);
            }
            break;
          #if ALL(HAS_BED_PROBE, AUTO_BED_LEVELING_UBL)
            case LEVELING_GET_TILT:
              if (draw)
                drawMenuItem(row, ICON_Tilt, GET_TEXT_F(MSG_UBL_TILT_MESH));
              else {
                if (bedlevel.storage_slot < 0) { popupHandler(Popup_MeshSlot); break; }
                popupHandler(Popup_Home);
                gcode.home_all_axes(true);
                popupHandler(Popup_Level);
                if (mesh_conf.tilt_grid > 1)
                  gcode.process_subcommands_now(TS(F("G29J"), mesh_conf.tilt_grid));
                else
                  gcode.process_subcommands_now(F("G29J"));
                planner.synchronize();
                redrawMenu();
              }
              break;
          #endif
          case LEVELING_GET_MESH:
            if (draw)
              drawMenuItem(row, ICON_Mesh, F("Create New Mesh"));
            else {
              popupHandler(Popup_Home);
              gcode.home_all_axes(true);
              #if ENABLED(AUTO_BED_LEVELING_UBL)
                #if ENABLED(PREHEAT_BEFORE_LEVELING)
                  popupHandler(Popup_Heating);
                  probe.preheat_for_probing(LEVELING_NOZZLE_TEMP, LEVELING_BED_TEMP);
                #endif
                #if HAS_BED_PROBE
                  popupHandler(Popup_Level);
                  gcode.process_subcommands_now(F("G29P0\nG29P1"));
                  gcode.process_subcommands_now(F("G29P3\nG29P3\nG29P3\nG29P3\nG29P3\nG29P3\nG29P3\nG29P3\nG29P3\nG29P3\nG29P3\nG29P3\nG29P3\nG29P3\nG29P3\nM420S1"));
                  planner.synchronize();
                  updateStatus("Probed all reachable points");
                  popupHandler(Popup_SaveLevel);
                #else
                  level_state = planner.leveling_active;
                  set_bed_leveling_enabled(false);
                  mesh_conf.goto_mesh_value = true;
                  mesh_conf.mesh_x = mesh_conf.mesh_y = 0;
                  popupHandler(Popup_MoveWait);
                  mesh_conf.manual_mesh_move();
                  drawMenu(ID_UBLMesh);
                #endif
              #elif HAS_BED_PROBE
                popupHandler(Popup_Level);
                gcode.process_subcommands_now(F("G29"));
                planner.synchronize();
                popupHandler(Popup_SaveLevel);
              #else
                level_state = planner.leveling_active;
                set_bed_leveling_enabled(false);
                gridpoint = 1;
                popupHandler(Popup_MoveWait);
                gcode.process_subcommands_now(F("G29"));
                planner.synchronize();
                drawMenu(ID_ManualMesh);
              #endif
            }
            break;
          case LEVELING_MANUAL:
            if (draw)
              drawMenuItem(row, ICON_Mesh, F("Manual Tuning"), nullptr, true);
            else {
              #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
                if (!leveling_is_valid()) {
                  confirmHandler(Popup_InvalidMesh);
                  break;
                }
              #endif
              #if ENABLED(AUTO_BED_LEVELING_UBL)
                if (bedlevel.storage_slot < 0) {
                  popupHandler(Popup_MeshSlot);
                  break;
                }
              #endif
              if (axes_should_home()) {
                popupHandler(Popup_Home);
                gcode.home_all_axes(true);
              }
              level_state = planner.leveling_active;
              set_bed_leveling_enabled(false);
              mesh_conf.goto_mesh_value = false;
              #if ENABLED(PREHEAT_BEFORE_LEVELING)
                popupHandler(Popup_Heating);
                #if HAS_HOTEND
                  if (thermalManager.degTargetHotend(0) < LEVELING_NOZZLE_TEMP)
                    thermalManager.setTargetHotend(LEVELING_NOZZLE_TEMP, 0);
                #endif
                #if HAS_HEATED_BED
                  if (thermalManager.degTargetBed() < LEVELING_BED_TEMP)
                    thermalManager.setTargetBed(LEVELING_BED_TEMP);
                #endif
                TERN_(HAS_HOTEND, thermalManager.wait_for_hotend(0));
                TERN_(HAS_HEATED_BED, thermalManager.wait_for_bed_heating());
              #endif
              popupHandler(Popup_MoveWait);
              mesh_conf.manual_mesh_move();
              drawMenu(ID_LevelManual);
            }
            break;
          case LEVELING_VIEW:
            if (draw)
              drawMenuItem(row, ICON_Mesh, GET_TEXT_F(MSG_MESH_VIEW), nullptr, true);
            else {
              #if ENABLED(AUTO_BED_LEVELING_UBL)
                if (bedlevel.storage_slot < 0) {
                  popupHandler(Popup_MeshSlot);
                  break;
                }
              #endif
              drawMenu(ID_LevelView);
            }
            break;
          case LEVELING_SETTINGS:
            if (draw)
              drawMenuItem(row, ICON_Step, F("Leveling Settings"), nullptr, true);
            else
              drawMenu(ID_LevelSettings);
            break;
          #if ENABLED(AUTO_BED_LEVELING_UBL)
            case LEVELING_SLOT:
              if (draw) {
                drawMenuItem(row, ICON_PrintSize, GET_TEXT_F(MSG_UBL_STORAGE_SLOT));
                drawFloat(bedlevel.storage_slot, row, false, 1);
              }
              else
                modifyValue(bedlevel.storage_slot, 0, settings.calc_num_meshes() - 1, 1);
              break;
            case LEVELING_LOAD:
              if (draw)
                drawMenuItem(row, ICON_ReadEEPROM, GET_TEXT_F(MSG_UBL_LOAD_MESH));
              else {
                if (bedlevel.storage_slot < 0) {
                  popupHandler(Popup_MeshSlot);
                  break;
                }
                gcode.process_subcommands_now(F("G29 L"));
                planner.synchronize();
                audioFeedback(true);
              }
              break;
            case LEVELING_SAVE:
              if (draw)
                drawMenuItem(row, ICON_WriteEEPROM, GET_TEXT_F(MSG_UBL_SAVE_MESH));
              else {
                if (bedlevel.storage_slot < 0) {
                  popupHandler(Popup_MeshSlot);
                  break;
                }
                gcode.process_subcommands_now(F("G29 S"));
                planner.synchronize();
                audioFeedback(true);
              }
              break;
          #endif // AUTO_BED_LEVELING_UBL
        }
        break;

      case ID_LevelView:

        #define LEVELING_VIEW_BACK 0
        #define LEVELING_VIEW_MESH (LEVELING_VIEW_BACK + 1)
        #define LEVELING_VIEW_TEXT (LEVELING_VIEW_MESH + 1)
        #define LEVELING_VIEW_ASYMMETRIC (LEVELING_VIEW_TEXT + 1)
        #define LEVELING_VIEW_TOTAL LEVELING_VIEW_ASYMMETRIC

        switch (item) {
          case LEVELING_VIEW_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMenu(ID_Leveling, LEVELING_VIEW);
            break;
          case LEVELING_VIEW_MESH:
            if (draw)
              drawMenuItem(row, ICON_PrintSize, GET_TEXT_F(MSG_MESH_VIEW), nullptr, true);
            else
              drawMenu(ID_MeshViewer);
            break;
          case LEVELING_VIEW_TEXT:
            if (draw) {
              drawMenuItem(row, ICON_Contact, F("Viewer Show Values"));
              drawCheckbox(row, mesh_conf.viewer_print_value);
            }
            else {
              FLIP(mesh_conf.viewer_print_value);
              drawCheckbox(row, mesh_conf.viewer_print_value);
            }
            break;
          case LEVELING_VIEW_ASYMMETRIC:
            if (draw) {
              drawMenuItem(row, ICON_Axis, F("Viewer Asymmetric"));
              drawCheckbox(row, mesh_conf.viewer_asymmetric_range);
            }
            else {
              FLIP(mesh_conf.viewer_asymmetric_range);
              drawCheckbox(row, mesh_conf.viewer_asymmetric_range);
            }
            break;
        }
        break;

      case ID_LevelSettings:

        #define LEVELING_SETTINGS_BACK 0
        #define LEVELING_SETTINGS_FADE (LEVELING_SETTINGS_BACK + 1)
        #define LEVELING_SETTINGS_TILT (LEVELING_SETTINGS_FADE + ENABLED(AUTO_BED_LEVELING_UBL))
        #define LEVELING_SETTINGS_PLANE (LEVELING_SETTINGS_TILT + ENABLED(AUTO_BED_LEVELING_UBL))
        #define LEVELING_SETTINGS_ZERO (LEVELING_SETTINGS_PLANE + ENABLED(AUTO_BED_LEVELING_UBL))
        #define LEVELING_SETTINGS_UNDEF (LEVELING_SETTINGS_ZERO + ENABLED(AUTO_BED_LEVELING_UBL))
        #define LEVELING_SETTINGS_TOTAL LEVELING_SETTINGS_UNDEF

        switch (item) {
          case LEVELING_SETTINGS_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else
              drawMenu(ID_Leveling, LEVELING_SETTINGS);
            break;
          case LEVELING_SETTINGS_FADE:
              if (draw) {
                drawMenuItem(row, ICON_Fade, GET_TEXT_F(MSG_Z_FADE_HEIGHT));
                drawFloat(planner.z_fade_height, row, false, 1);
              }
              else {
                modifyValue(planner.z_fade_height, 0, Z_MAX_POS, 1);
                planner.z_fade_height = -1;
                set_z_fade_height(planner.z_fade_height);
              }
              break;

          #if ENABLED(AUTO_BED_LEVELING_UBL)
            case LEVELING_SETTINGS_TILT:
              if (draw) {
                drawMenuItem(row, ICON_Tilt, F("Tilting Grid Size"));
                drawFloat(mesh_conf.tilt_grid, row, false, 1);
              }
              else
                modifyValue(mesh_conf.tilt_grid, 1, 8, 1);
              break;
            case LEVELING_SETTINGS_PLANE:
              if (draw)
                drawMenuItem(row, ICON_ResetEEPROM, F("Convert Mesh to Plane"));
              else {
                if (mesh_conf.createPlaneFromMesh()) break;
                gcode.process_subcommands_now(F("M420 S1"));
                planner.synchronize();
                audioFeedback(true);
              }
              break;
            case LEVELING_SETTINGS_ZERO:
              if (draw)
                drawMenuItem(row, ICON_Mesh, F("Mesh Zero"));
              else
                ZERO(bedlevel.z_values);
              break;
            case LEVELING_SETTINGS_UNDEF:
              if (draw)
                drawMenuItem(row, ICON_Mesh, GET_TEXT_F(MSG_MESH_RESET));
              else
                bedlevel.invalidate();
              break;
          #endif // AUTO_BED_LEVELING_UBL
        }
        break;

      case ID_MeshViewer:
        #define MESHVIEW_BACK 0
        #define MESHVIEW_TOTAL MESHVIEW_BACK

        if (item == MESHVIEW_BACK) {
          if (draw) {
            drawMenuItem(0, ICON_Back, GET_TEXT_F(MSG_BACK));
            mesh_conf.drawBedMesh();
            mesh_conf.setMeshViewerStatus();
          }
          else if (!mesh_conf.drawing_mesh) {
            drawMenu(ID_LevelView, LEVELING_VIEW_MESH);
            updateStatus("");
          }
        }
        break;

      case ID_LevelManual:

        #define LEVELING_M_BACK 0
        #define LEVELING_M_X (LEVELING_M_BACK + 1)
        #define LEVELING_M_Y (LEVELING_M_X + 1)
        #define LEVELING_M_NEXT (LEVELING_M_Y + 1)
        #define LEVELING_M_OFFSET (LEVELING_M_NEXT + 1)
        #define LEVELING_M_UP (LEVELING_M_OFFSET + 1)
        #define LEVELING_M_DOWN (LEVELING_M_UP + 1)
        #define LEVELING_M_GOTO_VALUE (LEVELING_M_DOWN + 1)
        #define LEVELING_M_UNDEF (LEVELING_M_GOTO_VALUE + ENABLED(AUTO_BED_LEVELING_UBL))
        #define LEVELING_M_TOTAL LEVELING_M_UNDEF

        switch (item) {
          case LEVELING_M_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else {
              set_bed_leveling_enabled(level_state);
              TERN_(AUTO_BED_LEVELING_BILINEAR, bedlevel.refresh_bed_level());
              drawMenu(ID_Leveling, LEVELING_MANUAL);
            }
            break;
          case LEVELING_M_X:
            if (draw) {
              drawMenuItem(row, ICON_MoveX, GET_TEXT_F(MSG_MESH_X));
              drawFloat(mesh_conf.mesh_x, row, 0, 1);
            }
            else
              modifyValue(mesh_conf.mesh_x, 0, GRID_MAX_POINTS_X - 1, 1);
            break;
          case LEVELING_M_Y:
            if (draw) {
              drawMenuItem(row, ICON_MoveY, GET_TEXT_F(MSG_MESH_Y));
              drawFloat(mesh_conf.mesh_y, row, 0, 1);
            }
            else
              modifyValue(mesh_conf.mesh_y, 0, GRID_MAX_POINTS_Y - 1, 1);
            break;
          case LEVELING_M_NEXT:
            if (draw)
              drawMenuItem(row, ICON_More, GET_TEXT_F(MSG_LEVEL_BED_NEXT_POINT));
            else {
              if (mesh_conf.mesh_x != (GRID_MAX_POINTS_X - 1) || mesh_conf.mesh_y != (GRID_MAX_POINTS_Y - 1)) {
                if ((mesh_conf.mesh_x == (GRID_MAX_POINTS_X - 1) && mesh_conf.mesh_y % 2 == 0) || (mesh_conf.mesh_x == 0 && mesh_conf.mesh_y % 2 == 1))
                  mesh_conf.mesh_y++;
                else if (mesh_conf.mesh_y % 2 == 0)
                  mesh_conf.mesh_x++;
                else
                  mesh_conf.mesh_x--;
                mesh_conf.manual_mesh_move();
              }
            }
            break;
          case LEVELING_M_OFFSET:
            if (draw) {
              drawMenuItem(row, ICON_SetZOffset, GET_TEXT_F(MSG_MESH_EDIT_Z));
              drawFloat(bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y], row, false, 100);
            }
            else {
              if (isnan(bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y]))
                bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y] = 0;
              modifyValue(bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y], MIN_Z_OFFSET, MAX_Z_OFFSET, 100);
            }
            break;
          case LEVELING_M_UP:
            if (draw)
              drawMenuItem(row, ICON_Axis, F("+0.01mm Up"));
            else if (bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y] < MAX_Z_OFFSET) {
              bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y] += 0.01;
              gcode.process_subcommands_now(F("M290 Z0.01"));
              planner.synchronize();
              current_position.z += 0.01f;
              sync_plan_position();
              drawFloat(bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y], row - 1, false, 100);
            }
            break;
          case LEVELING_M_DOWN:
            if (draw)
              drawMenuItem(row, ICON_AxisD, F("-0.01mm Down"));
            else if (bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y] > MIN_Z_OFFSET) {
              bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y] -= 0.01;
              gcode.process_subcommands_now(F("M290 Z-0.01"));
              planner.synchronize();
              current_position.z -= 0.01f;
              sync_plan_position();
              drawFloat(bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y], row - 2, false, 100);
            }
            break;
          case LEVELING_M_GOTO_VALUE:
            if (draw) {
              drawMenuItem(row, ICON_StockConfiguration, F("Go to Mesh Z Value"));
              drawCheckbox(row, mesh_conf.goto_mesh_value);
            }
            else {
              FLIP(mesh_conf.goto_mesh_value);
              current_position.z = 0;
              mesh_conf.manual_mesh_move(true);
              drawCheckbox(row, mesh_conf.goto_mesh_value);
            }
            break;
          #if ENABLED(AUTO_BED_LEVELING_UBL)
            case LEVELING_M_UNDEF:
              if (draw)
                drawMenuItem(row, ICON_ResetEEPROM, F("Clear Point Value"));
              else {
                mesh_conf.manualValueUpdate(true);
                redrawMenu(false);
              }
              break;
          #endif
        }
        break;
    #endif // HAS_MESH

    #if ENABLED(AUTO_BED_LEVELING_UBL) && !HAS_BED_PROBE
      case ID_UBLMesh:

        #define UBL_M_BACK 0
        #define UBL_M_NEXT (UBL_M_BACK + 1)
        #define UBL_M_PREV (UBL_M_NEXT + 1)
        #define UBL_M_OFFSET (UBL_M_PREV + 1)
        #define UBL_M_UP (UBL_M_OFFSET + 1)
        #define UBL_M_DOWN (UBL_M_UP + 1)
        #define UBL_M_TOTAL UBL_M_DOWN

        switch (item) {
          case UBL_M_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
            else {
              set_bed_leveling_enabled(level_state);
              drawMenu(ID_Leveling, LEVELING_GET_MESH);
            }
            break;
          case UBL_M_NEXT:
            if (draw) {
              if (mesh_conf.mesh_x != (GRID_MAX_POINTS_X - 1) || mesh_conf.mesh_y != (GRID_MAX_POINTS_Y - 1))
                drawMenuItem(row, ICON_More, GET_TEXT_F(MSG_LEVEL_BED_NEXT_POINT));
              else
                drawMenuItem(row, ICON_More, GET_TEXT_F(MSG_UBL_SAVE_MESH));
            }
            else {
              if (mesh_conf.mesh_x != (GRID_MAX_POINTS_X - 1) || mesh_conf.mesh_y != (GRID_MAX_POINTS_Y - 1)) {
                if ((mesh_conf.mesh_x == (GRID_MAX_POINTS_X - 1) && mesh_conf.mesh_y % 2 == 0) || (mesh_conf.mesh_x == 0 && mesh_conf.mesh_y % 2 == 1))
                  mesh_conf.mesh_y++;
                else if (mesh_conf.mesh_y % 2 == 0)
                  mesh_conf.mesh_x++;
                else
                  mesh_conf.mesh_x--;
                mesh_conf.manual_mesh_move();
              }
              else {
                gcode.process_subcommands_now(F("G29 S"));
                planner.synchronize();
                audioFeedback(true);
                drawMenu(ID_Leveling, LEVELING_GET_MESH);
              }
            }
            break;
          case UBL_M_PREV:
            if (draw)
              drawMenuItem(row, ICON_More, F("Previous Point"));
            else {
              if (mesh_conf.mesh_x != 0 || mesh_conf.mesh_y != 0) {
                if ((mesh_conf.mesh_x == (GRID_MAX_POINTS_X - 1) && mesh_conf.mesh_y % 2 == 1) || (mesh_conf.mesh_x == 0 && mesh_conf.mesh_y % 2 == 0))
                  mesh_conf.mesh_y--;
                else if (mesh_conf.mesh_y % 2 == 0)
                  mesh_conf.mesh_x--;
                else
                  mesh_conf.mesh_x++;
                mesh_conf.manual_mesh_move();
              }
            }
            break;
          case UBL_M_OFFSET:
            if (draw) {
              drawMenuItem(row, ICON_SetZOffset, GET_TEXT_F(MSG_MESH_EDIT_Z));
              drawFloat(bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y], row, false, 100);
            }
            else {
              if (isnan(bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y]))
                bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y] = 0;
              modifyValue(bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y], MIN_Z_OFFSET, MAX_Z_OFFSET, 100);
            }
            break;
          case UBL_M_UP:
            if (draw)
              drawMenuItem(row, ICON_Axis, F("+0.01mm Up"));
            else if (bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y] < MAX_Z_OFFSET) {
              bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y] += 0.01;
              gcode.process_subcommands_now(F("M290 Z0.01"));
              planner.synchronize();
              current_position.z += 0.01f;
              sync_plan_position();
              drawFloat(bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y], row - 1, false, 100);
            }
            break;
          case UBL_M_DOWN:
            if (draw)
              drawMenuItem(row, ICON_Axis, F("-0.01mm Down"));
            else if (bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y] > MIN_Z_OFFSET) {
              bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y] -= 0.01;
              gcode.process_subcommands_now(F("M290 Z-0.01"));
              planner.synchronize();
              current_position.z -= 0.01f;
              sync_plan_position();
              drawFloat(bedlevel.z_values[mesh_conf.mesh_x][mesh_conf.mesh_y], row - 2, false, 100);
            }
            break;
        }
        break;
    #endif // AUTO_BED_LEVELING_UBL && !HAS_BED_PROBE

    #if ENABLED(PROBE_MANUALLY)
      case ID_ManualMesh:

        #define MMESH_BACK 0
        #define MMESH_NEXT (MMESH_BACK + 1)
        #define MMESH_OFFSET (MMESH_NEXT + 1)
        #define MMESH_UP (MMESH_OFFSET + 1)
        #define MMESH_DOWN (MMESH_UP + 1)
        #define MMESH_OLD (MMESH_DOWN + 1)
        #define MMESH_TOTAL MMESH_OLD

        switch (item) {
          case MMESH_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BUTTON_CANCEL));
            else {
              gcode.process_subcommands_now(F("G29 A"));
              planner.synchronize();
              set_bed_leveling_enabled(level_state);
              drawMenu(ID_Leveling, LEVELING_GET_MESH);
            }
            break;
          case MMESH_NEXT:
            if (draw) {
              if (gridpoint < GRID_MAX_POINTS)
                drawMenuItem(row, ICON_More, GET_TEXT_F(MSG_LEVEL_BED_NEXT_POINT));
              else
                drawMenuItem(row, ICON_More, GET_TEXT_F(MSG_UBL_SAVE_MESH));
            }
            else if (gridpoint < GRID_MAX_POINTS) {
              popupHandler(Popup_MoveWait);
              gcode.process_subcommands_now(F("G29"));
              planner.synchronize();
              gridpoint++;
              redrawMenu();
            }
            else {
              gcode.process_subcommands_now(F("G29"));
              planner.synchronize();
              audioFeedback(settings.save());
              drawMenu(ID_Leveling, LEVELING_GET_MESH);
            }
            break;
          case MMESH_OFFSET:
            if (draw) {
              drawMenuItem(row, ICON_SetZOffset, F("Z Position"));
              current_position.z = MANUAL_PROBE_START_Z;
              drawFloat(current_position.z, row, false, 100);
            }
            else
              modifyValue(current_position.z, MIN_Z_OFFSET, MAX_Z_OFFSET, 100);
            break;
          case MMESH_UP:
            if (draw)
              drawMenuItem(row, ICON_Axis, F("+0.01mm Up"));
            else if (current_position.z < MAX_Z_OFFSET) {
              gcode.process_subcommands_now(F("M290 Z0.01"));
              planner.synchronize();
              current_position.z += 0.01f;
              sync_plan_position();
              drawFloat(current_position.z, row - 1, false, 100);
            }
            break;
          case MMESH_DOWN:
            if (draw)
              drawMenuItem(row, ICON_AxisD, F("-0.01mm Down"));
            else if (current_position.z > MIN_Z_OFFSET) {
              gcode.process_subcommands_now(F("M290 Z-0.01"));
              planner.synchronize();
              current_position.z -= 0.01f;
              sync_plan_position();
              drawFloat(current_position.z, row - 2, false, 100);
            }
            break;
          case MMESH_OLD:
            uint8_t mesh_x, mesh_y;
            // 0,0 -> 1,0 -> 2,0 -> 2,1 -> 1,1 -> 0,1 -> 0,2 -> 1,2 -> 2,2
            mesh_y = (gridpoint - 1) / (GRID_MAX_POINTS_Y);
            mesh_x = (gridpoint - 1) % (GRID_MAX_POINTS_X);

            if (mesh_y % 2 == 1)
              mesh_x = (GRID_MAX_POINTS_X) - mesh_x - 1;

            const float currval = bedlevel.z_values[mesh_x][mesh_y];

            if (draw) {
              drawMenuItem(row, ICON_Zoffset, F("Goto Mesh Value"));
              drawFloat(currval, row, false, 100);
            }
            else if (!isnan(currval)) {
              current_position.z = currval;
              planner.synchronize();
              planner.buffer_line(current_position, homing_feedrate(Z_AXIS), active_extruder);
              planner.synchronize();
              drawFloat(current_position.z, row - 3, false, 100);
            }
            break;
        }
        break;
    #endif // PROBE_MANUALLY

    case ID_Tune:

      #define TUNE_BACK 0
      #define TUNE_SPEED (TUNE_BACK + 1)
      #define TUNE_FLOW (TUNE_SPEED + ENABLED(HAS_HOTEND))
      #define TUNE_HOTEND (TUNE_FLOW + ENABLED(HAS_HOTEND))
      #define TUNE_BED (TUNE_HOTEND + ENABLED(HAS_HEATED_BED))
      #define TUNE_FAN (TUNE_BED + ENABLED(HAS_FAN))
      #define TUNE_ZOFFSET (TUNE_FAN + ENABLED(HAS_ZOFFSET_ITEM))
      #define TUNE_ZUP (TUNE_ZOFFSET + ENABLED(HAS_ZOFFSET_ITEM))
      #define TUNE_ZDOWN (TUNE_ZUP + ENABLED(HAS_ZOFFSET_ITEM))
      #define TUNE_LA (TUNE_ZDOWN + ENABLED(LIN_ADVANCE))
      #define TUNE_CHANGEFIL (TUNE_LA + ENABLED(FILAMENT_LOAD_UNLOAD_GCODES))
      #define TUNE_FWRETRACT (TUNE_CHANGEFIL + ENABLED(FWRETRACT))
      #define TUNE_FILSENSORENABLED (TUNE_FWRETRACT + ENABLED(HAS_FILAMENT_SENSOR))
      #define TUNE_BACKLIGHT_OFF (TUNE_FILSENSORENABLED + 1)
      #define TUNE_BACKLIGHT (TUNE_BACKLIGHT_OFF + 1)
      #define TUNE_TOTAL TUNE_BACKLIGHT

      switch (item) {
        case TUNE_BACK:
          if (draw)
            drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BACK));
          else
            drawPrintScreen();
          break;
        case TUNE_SPEED:
          if (draw) {
            drawMenuItem(row, ICON_Speed, GET_TEXT_F(MSG_SPEED));
            drawFloat(feedrate_percentage, row, false, 1);
          }
          else
            modifyValue(feedrate_percentage, MIN_PRINT_SPEED, MAX_PRINT_SPEED, 1);
          break;

        #if HAS_HOTEND
          case TUNE_FLOW:
            if (draw) {
              drawMenuItem(row, ICON_Speed, GET_TEXT_F(MSG_FLOW));
              drawFloat(planner.flow_percentage[0], row, false, 1);
            }
            else
              modifyValue(planner.flow_percentage[0], MIN_FLOW_RATE, MAX_FLOW_RATE, 1, []{ planner.refresh_e_factor(0); });
            break;
          case TUNE_HOTEND:
            if (draw) {
              drawMenuItem(row, ICON_SetEndTemp, GET_TEXT_F(MSG_NOZZLE));
              drawFloat(thermalManager.degTargetHotend(0), row, false, 1);
            }
            else
              modifyValue(thermalManager.temp_hotend[0].target, MIN_E_TEMP, MAX_E_TEMP, 1);
            break;
        #endif

        #if HAS_HEATED_BED
          case TUNE_BED:
            if (draw) {
              drawMenuItem(row, ICON_SetBedTemp, GET_TEXT_F(MSG_BED));
              drawFloat(thermalManager.degTargetBed(), row, false, 1);
            }
            else
              modifyValue(thermalManager.temp_bed.target, MIN_BED_TEMP, MAX_BED_TEMP, 1);
            break;
        #endif

        #if HAS_FAN
          case TUNE_FAN:
            if (draw) {
              drawMenuItem(row, ICON_FanSpeed, GET_TEXT_F(MSG_FAN_SPEED));
              drawFloat(thermalManager.fan_speed[0], row, false, 1);
            }
            else
              modifyValue(thermalManager.fan_speed[0], MIN_FAN_SPEED, MAX_FAN_SPEED, 1);
            break;
        #endif

        #if HAS_ZOFFSET_ITEM
          case TUNE_ZOFFSET:
            if (draw) {
              drawMenuItem(row, ICON_FanSpeed, F("Z-Offset"));
              drawFloat(zoffsetvalue, row, false, 100);
            }
            else
              modifyValue(zoffsetvalue, MIN_Z_OFFSET, MAX_Z_OFFSET, 100);
            break;
          case TUNE_ZUP:
            if (draw)
              drawMenuItem(row, ICON_Axis, F("Z-Offset +0.01mm Up"));
            else if (zoffsetvalue < MAX_Z_OFFSET) {
              gcode.process_subcommands_now(F("M290 Z0.01"));
              zoffsetvalue += 0.01;
              drawFloat(zoffsetvalue, row - 1, false, 100);
            }
            break;
          case TUNE_ZDOWN:
            if (draw)
              drawMenuItem(row, ICON_AxisD, F("Z-Offset -0.01mm Down"));
            else if (zoffsetvalue > MIN_Z_OFFSET) {
              gcode.process_subcommands_now(F("M290 Z-0.01"));
              zoffsetvalue -= 0.01;
              drawFloat(zoffsetvalue, row - 2, false, 100);
            }
            break;
        #endif

        #if ENABLED(LIN_ADVANCE)
          case TUNE_LA:
            if (draw) {
              drawMenuItem(row, ICON_MaxAccelerated, GET_TEXT_F(MSG_ADVANCE_K));
              drawFloat(planner.extruder_advance_K[0], row, false, 100);
            }
            else
              modifyValue(planner.extruder_advance_K[0], 0, 10, 100);
            break;
        #endif

        #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
          case TUNE_CHANGEFIL:
            if (draw)
              drawMenuItem(row, ICON_ResetEEPROM, GET_TEXT_F(MSG_FILAMENTCHANGE));
            else
              popupHandler(Popup_ConfFilChange);
            break;
        #endif

        #if ENABLED(FWRETRACT)
          case TUNE_FWRETRACT:
            if (draw)
              drawMenuItem(row, ICON_SetHome, GET_TEXT_F(MSG_FWRETRACT), nullptr, true);
            else
              drawMenu(ID_FWMenu);
            break;
        #endif

        #if HAS_FILAMENT_SENSOR
          case TUNE_FILSENSORENABLED:
            if (draw) {
              drawMenuItem(row, ICON_Extruder, GET_TEXT_F(MSG_RUNOUT_SENSOR));
              drawCheckbox(row, runout.enabled);
            }
            else {
              FLIP(runout.enabled);
              drawCheckbox(row, runout.enabled);
            }
            break;
        #endif

        case TUNE_BACKLIGHT_OFF:
          if (draw)
            drawMenuItem(row, ICON_Brightness, GET_TEXT_F(MSG_BRIGHTNESS_OFF));
          else
            ui.set_brightness(0);
          break;
        case TUNE_BACKLIGHT:
          if (draw) {
            drawMenuItem(row, ICON_Brightness, GET_TEXT_F(MSG_BRIGHTNESS));
            drawFloat(ui.brightness, row, false, 1);
          }
          else
            modifyValue(ui.brightness, LCD_BRIGHTNESS_MIN, LCD_BRIGHTNESS_MAX, 1, ui.refresh_brightness);
          break;
      }
      break;

    #if HAS_PREHEAT && HAS_HOTEND

      case ID_PreheatHotend:

        #define PREHEATHOTEND_BACK 0
        #define PREHEATHOTEND_CONTINUE (PREHEATHOTEND_BACK + 1)
        #define PREHEATHOTEND_1 (PREHEATHOTEND_CONTINUE + (PREHEAT_COUNT >= 1))
        #define PREHEATHOTEND_2 (PREHEATHOTEND_1 + (PREHEAT_COUNT >= 2))
        #define PREHEATHOTEND_3 (PREHEATHOTEND_2 + (PREHEAT_COUNT >= 3))
        #define PREHEATHOTEND_4 (PREHEATHOTEND_3 + (PREHEAT_COUNT >= 4))
        #define PREHEATHOTEND_5 (PREHEATHOTEND_4 + (PREHEAT_COUNT >= 5))
        #define PREHEATHOTEND_CUSTOM (PREHEATHOTEND_5 + 1)
        #define PREHEATHOTEND_TOTAL PREHEATHOTEND_CUSTOM

        switch (item) {
          case PREHEATHOTEND_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BUTTON_CANCEL));
            else {
              thermalManager.setTargetHotend(0, 0);
              TERN_(HAS_FAN, thermalManager.set_fan_speed(0, 0));
              redrawMenu(false, true, true);
            }
            break;
          case PREHEATHOTEND_CONTINUE:
            if (draw)
              drawMenuItem(row, ICON_SetEndTemp, GET_TEXT_F(MSG_BUTTON_CONTINUE));
            else {
              popupHandler(Popup_Heating);
              thermalManager.wait_for_hotend(0);
              switch (last_menu) {
                case ID_Prepare:
                  popupHandler(Popup_FilChange);
                  gcode.process_subcommands_now(TS(F("M600 B1 R"), thermalManager.degTargetHotend(0)));
                  break;
                #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
                  case ID_ChangeFilament:
                    switch (last_selection) {
                      case CHANGEFIL_LOAD:
                        popupHandler(Popup_FilLoad);
                        gcode.process_subcommands_now(F("M701"));
                        planner.synchronize();
                        redrawMenu(true, true, true);
                        break;
                      case CHANGEFIL_UNLOAD:
                        popupHandler(Popup_FilLoad, true);
                        gcode.process_subcommands_now(F("M702"));
                        planner.synchronize();
                        redrawMenu(true, true, true);
                        break;
                      case CHANGEFIL_CHANGE:
                        popupHandler(Popup_FilChange);
                        gcode.process_subcommands_now(TS(F("M600 B1 R"), thermalManager.degTargetHotend(0)));
                        break;
                    }
                    break;
                #endif
                default:
                  redrawMenu(true, true, true);
                  break;
              }
            }
            break;

          #define _PREHEAT_HOTEND_CASE(N) \
            case PREHEATHOTEND_##N: \
              if (draw) drawMenuItem(row, ICON_Temperature, F(PREHEAT_## N ##_LABEL)); \
              else ui.preheat_hotend_and_fan((N) - 1); \
              break;

          REPEAT_1(PREHEAT_COUNT, _PREHEAT_HOTEND_CASE)

          case PREHEATHOTEND_CUSTOM:
            if (draw) {
              drawMenuItem(row, ICON_Temperature, F("Custom"));
              drawFloat(thermalManager.degTargetHotend(0), row, false, 1);
            }
            else
              modifyValue(thermalManager.temp_hotend[0].target, EXTRUDE_MINTEMP, MAX_E_TEMP, 1);
            break;
        }
        break;

    #endif // HAS_PREHEAT && HAS_HOTEND

    #if ENABLED(FWRETRACT)
      case ID_FWMenu:
        #define FWRETRACT_BACK 0
        #define FWRETRACT_RETLEN (FWRETRACT_BACK + 1)
        #define FWRETRACT_RETSPD (FWRETRACT_RETLEN + 1)
        #define FWRETRACT_RETZHOP (FWRETRACT_RETSPD + 1)
        #define FWRETRACT_RECSPD (FWRETRACT_RETZHOP + 1)
        #define FWRETRACT_RECLEN (FWRETRACT_RECSPD + 1)
        #define FWRETRACT_TOTAL (FWRETRACT_RECLEN + 1)

        switch (item) {
          case FWRETRACT_BACK:
            if (draw)
              drawMenuItem(row, ICON_Back, GET_TEXT_F(MSG_BUTTON_BACK));
            else {
              if (last_menu == ID_Prepare)
                drawMenu(ID_Prepare, PREPARE_FWRETRACT);
              else if (last_menu == ID_Tune)
                drawMenu(ID_Tune, TUNE_FWRETRACT);
            }
            break;
          case FWRETRACT_RETLEN:
            if (draw) {
              drawMenuItem(row, ICON_FWRetLength, GET_TEXT_F(MSG_CONTROL_RETRACT));
              drawFloat(fwretract.settings.retract_length, row, false, 10);
            }
            else
              modifyValue(fwretract.settings.retract_length, 0, 10, 10);
            break;
          case FWRETRACT_RETSPD:
            if (draw) {
              drawMenuItem(row, ICON_FWRetLength, GET_TEXT_F(MSG_SINGLENOZZLE_RETRACT_SPEED));
              drawFloat(fwretract.settings.retract_feedrate_mm_s, row, false, 1);
            }
            else
              modifyValue(fwretract.settings.retract_feedrate_mm_s, 1, 90, 1);
            break;
          case FWRETRACT_RETZHOP:
            if (draw) {
              drawMenuItem(row, ICON_FWRetLength, GET_TEXT_F(MSG_CONTROL_RETRACT_ZHOP));
              drawFloat(fwretract.settings.retract_zraise, row, false, 100);
            }
            else
              modifyValue(fwretract.settings.retract_zraise, 0, 2, 100);
            break;
          case FWRETRACT_RECSPD:
            if (draw) {
              drawMenuItem(row, ICON_FWRetLength, GET_TEXT_F(MSG_SINGLENOZZLE_UNRETRACT_SPEED));
              drawFloat(fwretract.settings.retract_recover_feedrate_mm_s, row, false, 1);
            }
            else
              modifyValue(fwretract.settings.retract_recover_feedrate_mm_s, 1, 90, 1);
            break;
          case FWRETRACT_RECLEN:
            if (draw) {
              drawMenuItem(row, ICON_FWRetLength, GET_TEXT_F(MSG_CONTROL_RETRACT_RECOVER));
              drawFloat(fwretract.settings.retract_recover_extra, row, false, 10);
            }
            else
              modifyValue(fwretract.settings.retract_recover_extra, -5, 5, 10);
            break;
        }
        break;
    #endif
  }
}

FSTR_P JyersDWIN::getMenuTitle(const uint8_t menu) {
  switch (menu) {
    case ID_MainMenu:       return GET_TEXT_F(MSG_MAIN_MENU);
    case ID_Prepare:        return GET_TEXT_F(MSG_PREPARE);
    case ID_HomeMenu:       return F("Homing Menu");
    case ID_Move:           return GET_TEXT_F(MSG_MOVE_AXIS);
    case ID_ManualLevel:    return GET_TEXT_F(MSG_BED_TRAMMING_MANUAL);
    #if HAS_ZOFFSET_ITEM
      case ID_ZOffset:      return liveadjust ? GET_TEXT_F(MSG_BABYSTEP_PROBE_Z) : GET_TEXT_F(MSG_ZPROBE_ZOFFSET);
    #endif
    #if HAS_PREHEAT
      case ID_Preheat:      return F("Preheat");
    #endif
    #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
      case ID_ChangeFilament: return GET_TEXT_F(MSG_FILAMENTCHANGE);
    #endif
    #if HAS_CUSTOM_MENU
      case ID_MenuCustom:
        #ifdef CUSTOM_MENU_CONFIG_TITLE
          return F(CUSTOM_MENU_CONFIG_TITLE);
        #else
          return GET_TEXT_F(MSG_CUSTOM_COMMANDS);
        #endif
    #endif
    #if ENABLED(FWRETRACT)
      case ID_FWMenu:       return GET_TEXT_F(MSG_FWRETRACT);
    #endif
    case ID_Control:        return GET_TEXT_F(MSG_CONTROL);
    case ID_TempMenu:       return GET_TEXT_F(MSG_TEMPERATURE);
    #if ANY(PIDTEMP, PIDTEMPBED)
      case ID_PID:          return F("PID Menu");
    #endif
    #if ENABLED(PIDTEMP)
      case ID_HotendPID:    return F("Hotend PID Settings");
    #endif
    #if ENABLED(PIDTEMPBED)
      case ID_BedPID:       return F("Bed PID Settings");
    #endif
    #if ANY(MPC_EDIT_MENU, MPC_AUTOTUNE_MENU)
      case ID_MPC:          return F("MPC Menu");
    #endif
    #if HAS_PREHEAT
      #define _PREHEAT_TITLE_CASE(N) case ID_Preheat##N: return F(PREHEAT_## N ##_LABEL " Settings");
      REPEAT_1(PREHEAT_COUNT, _PREHEAT_TITLE_CASE)
    #endif
    case ID_Motion:         return GET_TEXT_F(MSG_MOTION);
    case ID_HomeOffsets:    return GET_TEXT_F(MSG_SET_HOME_OFFSETS);
    case ID_MaxSpeed:       return GET_TEXT_F(MSG_MAX_SPEED);
    case ID_MaxAcceleration: return F("Max Acceleration");
    #if ENABLED(CLASSIC_JERK)
      case ID_MaxJerk:      return F("Max Jerk");
    #endif
    #if ENABLED(EDITABLE_STEPS_PER_UNIT)
      case ID_Steps:        return GET_TEXT_F(MSG_STEPS_PER_MM);
    #endif
    case ID_Visual:         return F("Visual Settings");
    case ID_Advanced:       return GET_TEXT_F(MSG_ADVANCED_SETTINGS);
    #if HAS_BED_PROBE
      case ID_ProbeMenu:    return F("Bed Probe");
    #endif
    #if HAS_TRINAMIC_CONFIG
      case ID_TMCMenu:      return GET_TEXT_F(MSG_TMC_DRIVERS);
    #endif
    case ID_ColorSettings:  return F("UI Color Settings");
    case ID_Info:           return GET_TEXT_F(MSG_BUTTON_INFO);
    case ID_InfoMain:       return GET_TEXT_F(MSG_BUTTON_INFO);
    #if HAS_MESH
      case ID_Leveling:     return GET_TEXT_F(MSG_BED_LEVELING);
      case ID_LevelView:    return GET_TEXT_F(MSG_MESH_VIEW);
      case ID_LevelSettings: return F("Leveling Settings");
      case ID_MeshViewer:   return GET_TEXT_F(MSG_MESH_VIEW);
      case ID_LevelManual:  return F("Manual Tuning");
    #endif
    #if ENABLED(AUTO_BED_LEVELING_UBL) && !HAS_BED_PROBE
      case ID_UBLMesh:      return F("UBL Bed Leveling");
    #endif
    #if ENABLED(PROBE_MANUALLY)
      case ID_ManualMesh:   return GET_TEXT_F(MSG_MANUAL_LEVELING);
    #endif
    case ID_Tune:           return GET_TEXT_F(MSG_TUNE);
    case ID_PreheatHotend:  return GET_TEXT_F(MSG_PREHEAT_HOTEND);
  }
  return F("");
}

uint8_t JyersDWIN::getMenuSize(const uint8_t menu) {
  switch (menu) {
    case ID_Prepare:        return PREPARE_TOTAL;
    case ID_HomeMenu:       return HOME_TOTAL;
    case ID_Move:           return MOVE_TOTAL;
    case ID_ManualLevel:    return MLEVEL_TOTAL;
    #if HAS_ZOFFSET_ITEM
      case ID_ZOffset:      return ZOFFSET_TOTAL;
    #endif
    #if HAS_PREHEAT
      case ID_Preheat:      return PREHEAT_TOTAL;
    #endif
    #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
      case ID_ChangeFilament: return CHANGEFIL_TOTAL;
    #endif
    #if HAS_CUSTOM_MENU
      case ID_MenuCustom:   return CUSTOM_MENU_TOTAL;
    #endif
    #if ENABLED(FWRETRACT)
      case ID_FWMenu:       return FWRETRACT_TOTAL;
    #endif
    case ID_Control:        return CONTROL_TOTAL;
    case ID_TempMenu:       return TEMP_TOTAL;
    #if ANY(PIDTEMP, PIDTEMPBED)
      case ID_PID:          return PID_TOTAL;
    #endif
    #if ENABLED(PIDTEMP)
      case ID_HotendPID:    return HOTENDPID_TOTAL;
    #endif
    #if ENABLED(PIDTEMPBED)
      case ID_BedPID:       return BEDPID_TOTAL;
    #endif
    #if ANY(MPC_EDIT_MENU, MPC_AUTOTUNE_MENU)
      case ID_MPC:          return MPCMENU_TOTAL;
    #endif
    #if HAS_PREHEAT
      case ID_Preheat1 ... CAT(ID_Preheat, PREHEAT_COUNT):
                            return PREHEAT_SUBMENU_TOTAL;
    #endif
    case ID_Motion:         return MOTION_TOTAL;
    case ID_HomeOffsets:    return HOMEOFFSETS_TOTAL;
    case ID_MaxSpeed:       return SPEED_TOTAL;
    case ID_MaxAcceleration: return ACCEL_TOTAL;
    #if ENABLED(CLASSIC_JERK)
      case ID_MaxJerk:      return JERK_TOTAL;
    #endif
    #if ENABLED(EDITABLE_STEPS_PER_UNIT)
      case ID_Steps:        return STEPS_TOTAL;
    #endif
    case ID_Visual:         return VISUAL_TOTAL;
    case ID_Advanced:       return ADVANCED_TOTAL;
    #if HAS_BED_PROBE
      case ID_ProbeMenu:    return PROBE_TOTAL;
    #endif
    #if HAS_TRINAMIC_CONFIG
      case ID_TMCMenu:      return TMC_TOTAL;
    #endif
    case ID_Info:           return INFO_TOTAL;
    case ID_InfoMain:       return INFO_TOTAL;
    #if ENABLED(AUTO_BED_LEVELING_UBL) && !HAS_BED_PROBE
      case ID_UBLMesh:      return UBL_M_TOTAL;
    #endif
    #if ENABLED(PROBE_MANUALLY)
      case ID_ManualMesh:   return MMESH_TOTAL;
    #endif
    #if HAS_MESH
      case ID_Leveling:     return LEVELING_TOTAL;
      case ID_LevelView:    return LEVELING_VIEW_TOTAL;
      case ID_LevelSettings: return LEVELING_SETTINGS_TOTAL;
      case ID_MeshViewer:   return MESHVIEW_TOTAL;
      case ID_LevelManual:  return LEVELING_M_TOTAL;
    #endif
    case ID_Tune:           return TUNE_TOTAL;

    #if HAS_PREHEAT && HAS_HOTEND
      case ID_PreheatHotend: return PREHEATHOTEND_TOTAL;
    #endif

    case ID_ColorSettings:  return COLORSETTINGS_TOTAL;
  }
  return 0;
}

//
// Popup Config
//

void JyersDWIN::popupHandler(const PopupID popupid, const bool option/*=false*/) {
  popup = last_popup = popupid;
  FSTR_P const PWID = F("Please wait until done.");
  switch (popupid) {
    case Popup_Pause:         drawPopup(GET_TEXT_F(MSG_PAUSE_PRINT), F(""), F(""), Proc_Popup); break;
    case Popup_Stop:          drawPopup(GET_TEXT_F(MSG_STOP_PRINT), F(""), F(""), Proc_Popup); break;
    case Popup_Resume:        drawPopup(F("Resume Print?"), F("Looks Like the last"), F("print was interrupted."), Proc_Popup); break;
    case Popup_ConfFilChange: drawPopup(F("Confirm Filament Change"), F(""), F(""), Proc_Popup); break;
    case Popup_PurgeMore:     drawPopup(F("Purge more filament?"), F("(Cancel to finish process)"), F(""), Proc_Popup); break;
    #if ENABLED(AUTO_BED_LEVELING_UBL)
      case Popup_SaveLevel:   drawPopup(GET_TEXT_F(MSG_LEVEL_BED_DONE), F("Save to EEPROM?"), F(""), Proc_Popup); break;
      case Popup_MeshSlot:    drawPopup(F("Mesh slot not selected"), F("(Confirm to select slot 0)"), F(""), Proc_Popup); break;
    #endif
    case Popup_ETemp:         drawPopup(GET_TEXT_F(MSG_HOTEND_TOO_COLD), F("Open Preheat Menu?"), F(""), Proc_Popup); break;
    case Popup_ManualProbing: drawPopup(F("Manual Probing"), F("(Confirm to probe)"), F("(cancel to exit)"), Proc_Popup); break;
    case Popup_Level:         drawPopup(GET_TEXT_F(MSG_BED_LEVELING), PWID, F(""), Proc_Wait, ICON_AutoLeveling); break;
    case Popup_Home:          drawPopup(option ? F("Parking") : GET_TEXT_F(MSG_HOMING), PWID, F(""), Proc_Wait, ICON_BLTouch); break;
    case Popup_MoveWait:      drawPopup(GET_TEXT_F(MSG_UBL_MOVING_TO_NEXT), PWID, F(""), Proc_Wait, ICON_BLTouch); break;
    case Popup_Heating:       drawPopup(GET_TEXT_F(MSG_HEATING), PWID, F(""), Proc_Wait, ICON_BLTouch); break;
    case Popup_FilLoad:       drawPopup(option ? F("Unloading Filament") : F("Loading Filament"), PWID, F(""), Proc_Wait, ICON_BLTouch); break;
    case Popup_FilChange:     drawPopup(F("Filament Change"), F("Please wait for prompt."), F(""), Proc_Wait, ICON_BLTouch); break;
    case Popup_TempWarn:      drawPopup(option ? F("Nozzle temp too low!") : F("Nozzle temp too high!"), F(""), F(""), Proc_Wait, option ? ICON_TempTooLow : ICON_TempTooHigh); break;
    #if HAS_FILAMENT_SENSOR
      case Popup_Runout:      drawPopup(F("Filament Runout"), F(""), F(""), Proc_Wait, ICON_BLTouch); break;
    #endif
    #if ANY(PIDTEMP, PIDTEMPBED)
      case Popup_PIDWait:     drawPopup(GET_TEXT_F(MSG_PID_AUTOTUNE), F("in progress"), PWID, Proc_Wait, ICON_BLTouch); break;
    #endif
    #if ENABLED(MPC_AUTOTUNE_MENU)
      case Popup_MPCWait:     drawPopup(GET_TEXT_F(MSG_MPC_AUTOTUNE), F("in progress"), PWID, Proc_Wait, ICON_BLTouch); break;
    #endif
    case Popup_Resuming:      drawPopup(F("Resuming Print"), PWID, F(""), Proc_Wait, ICON_BLTouch); break;
    case Popup_Custom:        drawPopup(F("Running Custom G-Code"), PWID, F(""), Proc_Wait, ICON_BLTouch); break;
    default: break;
  }
}

void JyersDWIN::confirmHandler(PopupID popupid) {
  popup = popupid;
  switch (popupid) {
    case Popup_FilInsert:   drawPopup(F("Insert Filament"), F("Press to Continue"), F(""), Proc_Confirm); break;
    case Popup_HeaterTime:  drawPopup(F("Heater Timed Out"), F("Press to Reheat"), F(""), Proc_Confirm); break;
    case Popup_UserInput:   drawPopup(F("Waiting for Input"), F("Press to Continue"), F(""), Proc_Confirm); break;
    case Popup_LevelError:  drawPopup(F("Couldn't enable Leveling"), F("(Valid mesh must exist)"), F(""), Proc_Confirm); break;
    case Popup_InvalidMesh: drawPopup(F("Valid mesh must exist"), F("before tuning can be"), F("performed"), Proc_Confirm); break;
    default: break;
  }
}

//
// Navigation and Control
//

void JyersDWIN::mainMenuControl() {
  EncoderState encoder_diffState = encoderReceiveAnalyze();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (encoder_diffState == ENCODER_DIFF_CW && selection < PAGE_COUNT - 1) {
    selection++; // Select Down
    mainMenuIcons();
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW && selection > 0) {
    selection--; // Select Up
    mainMenuIcons();
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER)
    switch (selection) {
      case PAGE_PRINT: card.mount(); drawSDList(); break;
      case PAGE_PREPARE: drawMenu(ID_Prepare); break;
      case PAGE_CONTROL: drawMenu(ID_Control); break;
      case PAGE_INFO_LEVELING: drawMenu(TERN(HAS_MESH, ID_Leveling, ID_InfoMain)); break;
    }
  dwinUpdateLCD();
}

void JyersDWIN::menuControl() {
  EncoderState encoder_diffState = encoderReceiveAnalyze();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (encoder_diffState == ENCODER_DIFF_CW && selection < getMenuSize(active_menu)) {
    dwinDrawRectangle(1, COLOR_BG_BLACK, 0, MBASE(selection - scrollpos) - 18, 14, MBASE(selection - scrollpos) + 33);
    selection++; // Select Down
    if (selection > scrollpos + MROWS) {
      scrollpos++;
      dwinFrameAreaMove(1, 2, MLINE, COLOR_BG_BLACK, 0, 31, DWIN_WIDTH, 349);
      menuItemHandler(active_menu, selection);
    }
    dwinDrawRectangle(1, getColor(eeprom_settings.cursor_color, COLOR_RECTANGLE), 0, MBASE(selection - scrollpos) - 18, 14, MBASE(selection - scrollpos) + 33);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW && selection > 0) {
    dwinDrawRectangle(1, COLOR_BG_BLACK, 0, MBASE(selection - scrollpos) - 18, 14, MBASE(selection - scrollpos) + 33);
    selection--; // Select Up
    if (selection < scrollpos) {
      scrollpos--;
      dwinFrameAreaMove(1, 3, MLINE, COLOR_BG_BLACK, 0, 31, DWIN_WIDTH, 349);
      menuItemHandler(active_menu, selection);
    }
    dwinDrawRectangle(1, getColor(eeprom_settings.cursor_color, COLOR_RECTANGLE), 0, MBASE(selection - scrollpos) - 18, 14, MBASE(selection - scrollpos) + 33);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER)
    menuItemHandler(active_menu, selection, false);
  dwinUpdateLCD();
}

void JyersDWIN::valueControl() {
  EncoderState encoder_diffState = encoderReceiveAnalyze();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (encoder_diffState == ENCODER_DIFF_CW)
    tempvalue += encoderRate.encoderMoveValue;
  else if (encoder_diffState == ENCODER_DIFF_CCW)
    tempvalue -= encoderRate.encoderMoveValue;
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    process = Proc_Menu;
    encoderRate.enabled = false;
    drawFloat(tempvalue / valueunit, selection - scrollpos, false, valueunit);
    dwinUpdateLCD();
    if (active_menu == ID_ZOffset && liveadjust) {
      planner.synchronize();
      current_position.z += (tempvalue / valueunit - zoffsetvalue);
      planner.buffer_line(current_position, homing_feedrate(Z_AXIS), active_extruder);
      current_position.z = 0;
      sync_plan_position();
    }
    else if (active_menu == ID_Tune && selection == TUNE_ZOFFSET) {
      gcode.process_subcommands_now(TS(F("M290Z"), p_float_t((tempvalue / valueunit - zoffsetvalue), 3)));
    }
    if (TERN0(PIDTEMP, valuepointer == &thermalManager.temp_hotend[0].pid.Ki) || TERN0(PIDTEMPBED, valuepointer == &thermalManager.temp_bed.pid.Ki))
      tempvalue = scalePID_i(tempvalue);
    if (TERN0(PIDTEMP, valuepointer == &thermalManager.temp_hotend[0].pid.Kd) || TERN0(PIDTEMPBED, valuepointer == &thermalManager.temp_bed.pid.Kd))
      tempvalue = scalePID_d(tempvalue);
    switch (valuetype) {
      case 0: *(float*)valuepointer = tempvalue / valueunit; break;
      case 1: *(uint8_t*)valuepointer = tempvalue / valueunit; break;
      case 2: *(uint16_t*)valuepointer = tempvalue / valueunit; break;
      case 3: *(int16_t*)valuepointer = tempvalue / valueunit; break;
      case 4: *(uint32_t*)valuepointer = tempvalue / valueunit; break;
      case 5: *(int8_t*)valuepointer = tempvalue / valueunit; break;
    }
    switch (active_menu) {
      case ID_Move:
        planner.synchronize();
        planner.buffer_line(current_position, manual_feedrate_mm_s[selection - 1], active_extruder);
        break;
      #if HAS_MESH
        case ID_ManualMesh:
          planner.synchronize();
          planner.buffer_line(current_position, homing_feedrate(Z_AXIS), active_extruder);
          planner.synchronize();
          break;
        case ID_UBLMesh: mesh_conf.manual_mesh_move(true); break;
        case ID_LevelManual: mesh_conf.manual_mesh_move(selection == LEVELING_M_OFFSET); break;
      #endif
    }
    if (funcpointer) funcpointer();
    return;
  }
  LIMIT(tempvalue, valuemin * valueunit, valuemax * valueunit);
  drawFloat(tempvalue / valueunit, selection - scrollpos, true, valueunit);
  dwinUpdateLCD();
  if (active_menu == ID_Move && livemove) {
    *(float*)valuepointer = tempvalue / valueunit;
    planner.buffer_line(current_position, manual_feedrate_mm_s[selection - 1], active_extruder);
  }
}

void JyersDWIN::optionControl() {
  EncoderState encoder_diffState = encoderReceiveAnalyze();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (encoder_diffState == ENCODER_DIFF_CW)
    tempvalue += encoderRate.encoderMoveValue;
  else if (encoder_diffState == ENCODER_DIFF_CCW)
    tempvalue -= encoderRate.encoderMoveValue;
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    process = Proc_Menu;
    encoderRate.enabled = false;
    if (valuepointer == &color_names) {
      switch (selection) {
        case COLORSETTINGS_CURSOR: eeprom_settings.cursor_color = tempvalue; break;
        case COLORSETTINGS_SPLIT_LINE: eeprom_settings.menu_split_line = tempvalue; break;
        case COLORSETTINGS_MENU_TOP_BG: eeprom_settings.menu_top_bg = tempvalue; break;
        case COLORSETTINGS_MENU_TOP_TXT: eeprom_settings.menu_top_txt = tempvalue; break;
        case COLORSETTINGS_HIGHLIGHT_BORDER: eeprom_settings.highlight_box = tempvalue; break;
        case COLORSETTINGS_PROGRESS_PERCENT: eeprom_settings.progress_percent = tempvalue; break;
        case COLORSETTINGS_PROGRESS_TIME: eeprom_settings.progress_time = tempvalue; break;
        case COLORSETTINGS_PROGRESS_STATUS_BAR: eeprom_settings.status_bar_text = tempvalue; break;
        case COLORSETTINGS_PROGRESS_STATUS_AREA: eeprom_settings.status_area_text = tempvalue; break;
        case COLORSETTINGS_PROGRESS_COORDINATES: eeprom_settings.coordinates_text = tempvalue; break;
        case COLORSETTINGS_PROGRESS_COORDINATES_LINE: eeprom_settings.coordinates_split_line = tempvalue; break;
      }
      redrawScreen();
    }
    else if (valuepointer == &preheat_modes)
      preheatmode = tempvalue;

    drawOption(tempvalue, static_cast<const char * const *>(valuepointer), selection - scrollpos, false, (valuepointer == &color_names));
    dwinUpdateLCD();
    return;
  }
  LIMIT(tempvalue, valuemin, valuemax);
  drawOption(tempvalue, static_cast<const char * const *>(valuepointer), selection - scrollpos, true);
  dwinUpdateLCD();
}

void JyersDWIN::fileControl() {
  typedef TextScroller<MENU_CHAR_LIMIT> Scroller;
  static Scroller scroller;
  EncoderState encoder_diffState = encoderReceiveAnalyze();
  if (encoder_diffState == ENCODER_DIFF_NO) {
    if (selection > 0) {
      card.selectFileByIndexSorted(selection - 1);
      char * const filename = card.longest_filename();
      size_t len = strlen(filename), pos = len;
      if (!card.flag.filenameIsDir)
        while (pos && filename[pos] != '.') pos--;
      if (pos > MENU_CHAR_LIMIT) {
        static millis_t time = 0;
        if (PENDING(millis(), time)) return;
        time = millis() + 200;
        Scroller::Buffer buf;
        const char* const name = scroller.scroll(pos, buf, filename);
        dwinDrawRectangle(1, COLOR_BG_BLACK, LBLX, MBASE(selection - scrollpos) - 14, 271, MBASE(selection - scrollpos) + 28);
        drawMenuItem(selection - scrollpos, card.flag.filenameIsDir ? ICON_More : ICON_File, name);
        dwinUpdateLCD();
      }
    }
    return;
  }
  if (encoder_diffState == ENCODER_DIFF_CW && selection < card.get_num_items()) {
    dwinDrawRectangle(1, COLOR_BG_BLACK, 0, MBASE(selection - scrollpos) - 18, 14, MBASE(selection - scrollpos) + 33);
    if (selection > 0) {
      dwinDrawRectangle(1, COLOR_BG_BLACK, LBLX, MBASE(selection - scrollpos) - 14, 271, MBASE(selection - scrollpos) + 28);
      drawSDItem(selection, selection - scrollpos);
    }
    scroller.reset();
    selection++; // Select Down
    if (selection > scrollpos + MROWS) {
      scrollpos++;
      dwinFrameAreaMove(1, 2, MLINE, COLOR_BG_BLACK, 0, 31, DWIN_WIDTH, 349);
      drawSDItem(selection, selection - scrollpos);
    }
    dwinDrawRectangle(1, getColor(eeprom_settings.cursor_color, COLOR_RECTANGLE), 0, MBASE(selection - scrollpos) - 18, 14, MBASE(selection - scrollpos) + 33);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW && selection > 0) {
    dwinDrawRectangle(1, COLOR_BG_BLACK, 0, MBASE(selection - scrollpos) - 18, 14, MBASE(selection - scrollpos) + 33);
    dwinDrawRectangle(1, COLOR_BG_BLACK, LBLX, MBASE(selection - scrollpos) - 14, 271, MBASE(selection - scrollpos) + 28);
    drawSDItem(selection, selection - scrollpos);
    scroller.reset();
    selection--; // Select Up
    if (selection < scrollpos) {
      scrollpos--;
      dwinFrameAreaMove(1, 3, MLINE, COLOR_BG_BLACK, 0, 31, DWIN_WIDTH, 349);
      drawSDItem(selection, selection - scrollpos);
    }
    dwinDrawRectangle(1, getColor(eeprom_settings.cursor_color, COLOR_RECTANGLE), 0, MBASE(selection - scrollpos) - 18, 14, MBASE(selection - scrollpos) + 33);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (selection == 0) {
      if (card.flag.workDirIsRoot) {
        process = Proc_Main;
        drawMainMenu();
      }
      else {
        card.cdup();
        drawSDList();
      }
    }
    else {
      card.selectFileByIndexSorted(selection - 1);
      if (card.flag.filenameIsDir) {
        card.cd(card.filename);
        drawSDList();
      }
      else
        card.openAndPrintFile(card.filename);
    }
  }
  dwinUpdateLCD();
}

void JyersDWIN::printScreenControl() {
  EncoderState encoder_diffState = encoderReceiveAnalyze();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (encoder_diffState == ENCODER_DIFF_CW && selection < PRINT_COUNT - 1) {
    selection++; // Select Down
    printScreenIcons();
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW && selection > 0) {
    selection--; // Select Up
    printScreenIcons();
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (selection) {
      case PRINT_SETUP:
        drawMenu(ID_Tune);
        updateStatusBar(true);
        break;
      case PRINT_PAUSE_RESUME:
        if (paused) {
          if (sdprint) {
            wait_for_user = false;
            #if ENABLED(PARK_HEAD_ON_PAUSE)
              card.startOrResumeFilePrinting();
              TERN_(POWER_LOSS_RECOVERY, recovery.prepare());
            #else
              #if HAS_HEATED_BED
                gcode.process_subcommands_now(TS(F("M140 S"), pausebed));
              #endif
              #if HAS_EXTRUDERS
                gcode.process_subcommands_now(TS(F("M109 S"), pausetemp));
              #endif
              TERN_(HAS_FAN, thermalManager.fan_speed[0] = pausefan);
              planner.synchronize();
              TERN_(HAS_MEDIA, queue.inject(FPSTR(M24_STR)));
            #endif
          }
          else {
            TERN_(HOST_ACTION_COMMANDS, hostui.resume());
          }
          drawPrintScreen();
        }
        else
          popupHandler(Popup_Pause);
        break;
      case PRINT_STOP: popupHandler(Popup_Stop); break;
    }
  }
  dwinUpdateLCD();
}

void JyersDWIN::popupControl() {
  EncoderState encoder_diffState = encoderReceiveAnalyze();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (encoder_diffState == ENCODER_DIFF_CW && selection < 1) {
    selection++;
    popupSelect();
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW && selection > 0) {
    selection--;
    popupSelect();
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (popup) {
      case Popup_Pause:
        if (selection == 0) {
          if (sdprint) {
            #if ENABLED(POWER_LOSS_RECOVERY)
              if (recovery.enabled) recovery.save(true);
            #endif
            #if ENABLED(PARK_HEAD_ON_PAUSE)
              popupHandler(Popup_Home, true);
              #if HAS_MEDIA
                if (IS_SD_PRINTING()) card.pauseSDPrint();
              #endif
              planner.synchronize();
              queue.inject(F("M125"));
              planner.synchronize();
            #else
              queue.inject(F("M25"));
              TERN_(HAS_HOTEND, pausetemp = thermalManager.degTargetHotend(0));
              TERN_(HAS_HEATED_BED, pausebed = thermalManager.degTargetBed());
              TERN_(HAS_FAN, pausefan = thermalManager.fan_speed[0]);
              thermalManager.cooldown();
            #endif
          }
          else {
            TERN_(HOST_ACTION_COMMANDS, hostui.pause());
          }
        }
        drawPrintScreen();
        break;
      case Popup_Stop:
        if (selection == 0) {
          if (sdprint) {
            ui.abort_print();
            thermalManager.cooldown();
          }
          else {
            TERN_(HOST_ACTION_COMMANDS, hostui.cancel());
          }
        }
        else
          drawPrintScreen();
        break;
      case Popup_Resume:
        if (selection == 0)
          queue.inject(F("M1000"));
        else {
          queue.inject(F("M1000 C"));
          drawMainMenu();
        }
        break;

      #if HAS_HOTEND
        case Popup_ETemp:
          if (selection == 0) {
            thermalManager.setTargetHotend(EXTRUDE_MINTEMP, 0);
            TERN_(HAS_FAN, thermalManager.set_fan_speed(0, MAX_FAN_SPEED));
            drawMenu(ID_PreheatHotend);
          }
          else
            redrawMenu(true, true, false);
          break;
      #endif

      #if HAS_BED_PROBE
        case Popup_ManualProbing:
          if (selection == 0) {
            const float dif = probe.probe_at_point(current_position.x, current_position.y, PROBE_PT_STOW, 0, false) - corner_avg;
            updateStatus(TS(F("Corner is "), p_float_t(abs(dif), 3), "mm ", dif > 0 ? F("high") : F("low")));
          }
          else {
            redrawMenu(true, true, false);
            updateStatus("");
          }
          break;
      #endif

      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        case Popup_ConfFilChange:
          if (selection == 0) {
            if (thermalManager.targetTooColdToExtrude(0))
              popupHandler(Popup_ETemp);
            else {
              if (thermalManager.temp_hotend[0].is_below_target(2)) {
                popupHandler(Popup_Heating);
                thermalManager.wait_for_hotend(0);
              }
              popupHandler(Popup_FilChange);
              gcode.process_subcommands_now(TS(F("M600B1R"), thermalManager.degTargetHotend(0)));
            }
          }
          else
            redrawMenu(true, true, false);
          break;
        case Popup_PurgeMore:
          if (selection == 0) {
            pause_menu_response = PAUSE_RESPONSE_EXTRUDE_MORE;
            popupHandler(Popup_FilChange);
          }
          else {
            pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
            if (printing) popupHandler(Popup_Resuming);
            else redrawMenu(true, true, active_menu == ID_PreheatHotend);
          }
          break;
      #endif // ADVANCED_PAUSE_FEATURE

      #if HAS_MESH
        case Popup_SaveLevel:
          if (selection == 0) {
            #if ENABLED(AUTO_BED_LEVELING_UBL)
              gcode.process_subcommands_now(F("G29 S"));
              planner.synchronize();
              audioFeedback(true);
            #else
              audioFeedback(settings.save());
            #endif
          }
          drawMenu(ID_Leveling, LEVELING_GET_MESH);
          break;
      #endif

      #if ENABLED(AUTO_BED_LEVELING_UBL)
        case Popup_MeshSlot:
          if (selection == 0) bedlevel.storage_slot = 0;
          redrawMenu(true, true);
          break;
      #endif
      default: break;
    }
  }
  dwinUpdateLCD();
}

void JyersDWIN::confirmControl() {
  EncoderState encoder_diffState = encoderReceiveAnalyze();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (popup) {
      case Popup_Complete:
        drawMainMenu();
        break;
      case Popup_FilInsert:
        popupHandler(Popup_FilChange);
        wait_for_user = false;
        break;
      case Popup_HeaterTime:
        popupHandler(Popup_Heating);
        wait_for_user = false;
        break;
      default:
        redrawMenu(true, true, false);
        wait_for_user = false;
        break;
    }
  }
  dwinUpdateLCD();
}

//
// In-Menu Value Modification
//

void JyersDWIN::setupValue(const_float_t value, const_float_t min, const_float_t max, const_float_t unit, const uint8_t type) {
  if (TERN0(PIDTEMP, valuepointer == &thermalManager.temp_hotend[0].pid.Ki) || TERN0(PIDTEMPBED, valuepointer == &thermalManager.temp_bed.pid.Ki))
    tempvalue = unscalePID_i(value) * unit;
  else if (TERN0(PIDTEMP, valuepointer == &thermalManager.temp_hotend[0].pid.Kd) || TERN0(PIDTEMPBED, valuepointer == &thermalManager.temp_bed.pid.Kd))
    tempvalue = unscalePID_d(value) * unit;
  else
    tempvalue = value * unit;
  valuemin = min;
  valuemax = max;
  valueunit = unit;
  valuetype = type;
  process = Proc_Value;
  encoderRate.enabled = true;
  drawFloat(tempvalue / unit, selection - scrollpos, true, valueunit);
}

void JyersDWIN::modifyValue(float &value, const_float_t min, const_float_t max, const_float_t unit, void (*f)()/*=nullptr*/) {
  valuepointer = &value;
  funcpointer = f;
  setupValue((float)value, min, max, unit, 0);
}
void JyersDWIN::modifyValue(uint8_t &value, const_float_t min, const_float_t max, const_float_t unit, void (*f)()/*=nullptr*/) {
  valuepointer = &value;
  funcpointer = f;
  setupValue((float)value, min, max, unit, 1);
}
void JyersDWIN::modifyValue(uint16_t &value, const_float_t min, const_float_t max, const_float_t unit, void (*f)()/*=nullptr*/) {
  valuepointer = &value;
  funcpointer = f;
  setupValue((float)value, min, max, unit, 2);
}
void JyersDWIN::modifyValue(int16_t &value, const_float_t min, const_float_t max, const_float_t unit, void (*f)()/*=nullptr*/) {
  valuepointer = &value;
  funcpointer = f;
  setupValue((float)value, min, max, unit, 3);
}
void JyersDWIN::modifyValue(uint32_t &value, const_float_t min, const_float_t max, const_float_t unit, void (*f)()/*=nullptr*/) {
  valuepointer = &value;
  funcpointer = f;
  setupValue((float)value, min, max, unit, 4);
}
void JyersDWIN::modifyValue(int8_t &value, const_float_t min, const_float_t max, const_float_t unit, void (*f)()/*=nullptr*/) {
  valuepointer = &value;
  funcpointer = f;
  setupValue((float)value, min, max, unit, 5);
}

void JyersDWIN::modifyOption(const uint8_t value, const char * const * options, const uint8_t max) {
  tempvalue = value;
  valuepointer = const_cast<const char * *>(options);
  valuemin = 0;
  valuemax = max;
  process = Proc_Option;
  encoderRate.enabled = true;
  drawOption(value, options, selection - scrollpos, true);
}

//
// Main Functions
//

void JyersDWIN::updateStatus(const char * const text) {
  if (strncmp_P(text, PSTR("<F>"), 3) == 0) {
    for (uint8_t i = 0; i < _MIN((size_t)LONG_FILENAME_LENGTH, strlen(text)); ++i) filename[i] = text[i + 3];
    filename[_MIN((size_t)LONG_FILENAME_LENGTH - 1, strlen(text))] = '\0';
    drawPrintFilename(true);
  }
  else {
    for (uint8_t i = 0; i < _MIN((size_t)64, strlen(text)); ++i) statusmsg[i] = text[i];
    statusmsg[_MIN((size_t)64, strlen(text))] = '\0';
  }
}

void JyersDWIN::startPrint(const bool sd) {
  sdprint = sd;
  if (!printing) {
    printing = true;
    statusmsg[0] = '\0';
    if (sd) {
      #if ENABLED(POWER_LOSS_RECOVERY)
        if (recovery.valid()) {
          MediaFile *diveDir = nullptr;
          const char * const fname = card.diveToFile(true, diveDir, recovery.info.sd_filename);
          card.selectFileByName(fname);
        }
      #endif
      strcpy(filename, card.longest_filename());
    }
    else
      strcpy_P(filename, PSTR("Host Print"));
    TERN_(SET_PROGRESS_PERCENT, ui.set_progress(0));
    TERN_(SET_REMAINING_TIME, ui.set_remaining_time(0));
    drawPrintScreen();
  }
}

void JyersDWIN::stopPrint() {
  printing = false;
  sdprint = false;
  thermalManager.cooldown();
  TERN_(SET_PROGRESS_PERCENT, ui.set_progress(100 * (PROGRESS_SCALE)));
  TERN_(SET_REMAINING_TIME, ui.set_remaining_time(0));
  drawPrintConfirm();
}

void JyersDWIN::update() {
  stateUpdate();
  screenUpdate();
  switch (process) {
    case Proc_Main:    mainMenuControl();     break;
    case Proc_Menu:    menuControl();         break;
    case Proc_Value:   valueControl();        break;
    case Proc_Option:  optionControl();       break;
    case Proc_File:    fileControl();         break;
    case Proc_Print:   printScreenControl();  break;
    case Proc_Popup:   popupControl();        break;
    case Proc_Confirm: confirmControl();      break;
  }
}

void MarlinUI::update() { jyersDWIN.update(); }

#if HAS_LCD_BRIGHTNESS
  void MarlinUI::_set_brightness() { dwinLCDBrightness(backlight ? brightness : 0); }
#endif

void JyersDWIN::stateUpdate() {
  if ((print_job_timer.isRunning() || print_job_timer.isPaused()) != printing) {
    if (!printing) startPrint(card.isFileOpen() || TERN0(POWER_LOSS_RECOVERY, recovery.valid()));
    else stopPrint();
  }
  if (print_job_timer.isPaused() != paused) {
    paused = print_job_timer.isPaused();
    if (process == Proc_Print) printScreenIcons();
    if (process == Proc_Wait && !paused) redrawMenu(true, true);
  }
  if (wait_for_user && !(process == Proc_Confirm) && !print_job_timer.isPaused())
    confirmHandler(Popup_UserInput);
  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    if (process == Proc_Popup && popup == Popup_PurgeMore) {
      if (pause_menu_response == PAUSE_RESPONSE_EXTRUDE_MORE)
        popupHandler(Popup_FilChange);
      else if (pause_menu_response == PAUSE_RESPONSE_RESUME_PRINT) {
        if (printing) popupHandler(Popup_Resuming);
        else redrawMenu(true, true, active_menu == ID_PreheatHotend);
      }
    }
  #endif
  #if HAS_FILAMENT_SENSOR
    static bool ranout = false;
    if (runout.filament_ran_out != ranout) {
      ranout = runout.filament_ran_out;
      if (ranout) popupHandler(Popup_Runout);
    }
  #endif
}

void JyersDWIN::screenUpdate() {
  const millis_t ms = millis();
  static millis_t scrltime = 0;
  if (ELAPSED(ms, scrltime)) {
    scrltime = ms + 200;
    updateStatusBar();
    if (process == Proc_Print) drawPrintFilename();
  }

  static millis_t statustime = 0;
  if (ELAPSED(ms, statustime)) {
    statustime = ms + 500;
    drawStatusArea();
  }

  static millis_t printtime = 0;
  if (ELAPSED(ms, printtime)) {
    printtime = ms + 1000;
    if (process == Proc_Print) {
      drawPrintProgressBar();
      drawPrintProgressElapsed();
      TERN_(SET_REMAINING_TIME, drawPrintProgressRemain());
    }
  }

  static bool mounted = card.isMounted();
  if (mounted != card.isMounted()) {
    mounted = card.isMounted();
    if (process == Proc_File)
      drawSDList();
  }

  #if HAS_HOTEND
    static int16_t hotendtarget = -1;
  #endif
  #if HAS_HEATED_BED
    static int16_t bedtarget = -1;
  #endif
  #if HAS_FAN
    static int16_t fanspeed = -1;
  #endif

  #if HAS_ZOFFSET_ITEM
    static float lastzoffset = zoffsetvalue;
    if (zoffsetvalue != lastzoffset) {
      lastzoffset = zoffsetvalue;
      #if HAS_BED_PROBE
        probe.offset.z = zoffsetvalue;
      #else
        set_home_offset(Z_AXIS, -zoffsetvalue);
      #endif
    }

    #if HAS_BED_PROBE
      if (probe.offset.z != lastzoffset)
        zoffsetvalue = lastzoffset = probe.offset.z;
    #else
      if (-home_offset.z != lastzoffset)
        zoffsetvalue = lastzoffset = -home_offset.z;
    #endif
  #endif // HAS_ZOFFSET_ITEM

  if (process == Proc_Menu || process == Proc_Value) {
    switch (active_menu) {
      case ID_TempMenu:
        #if HAS_HOTEND
          if (thermalManager.degTargetHotend(0) != hotendtarget) {
            hotendtarget = thermalManager.degTargetHotend(0);
            if (scrollpos <= TEMP_HOTEND && TEMP_HOTEND <= scrollpos + MROWS) {
              if (process != Proc_Value || selection != TEMP_HOTEND - scrollpos)
                drawFloat(hotendtarget, TEMP_HOTEND - scrollpos, false, 1);
            }
          }
        #endif
        #if HAS_HEATED_BED
          if (thermalManager.degTargetBed() != bedtarget) {
            bedtarget = thermalManager.degTargetBed();
            if (scrollpos <= TEMP_BED && TEMP_BED <= scrollpos + MROWS) {
              if (process != Proc_Value || selection != TEMP_HOTEND - scrollpos)
                drawFloat(bedtarget, TEMP_BED - scrollpos, false, 1);
            }
          }
        #endif
        #if HAS_FAN
          if (thermalManager.fan_speed[0] != fanspeed) {
            fanspeed = thermalManager.fan_speed[0];
            if (scrollpos <= TEMP_FAN && TEMP_FAN <= scrollpos + MROWS) {
              if (process != Proc_Value || selection != TEMP_HOTEND - scrollpos)
                drawFloat(fanspeed, TEMP_FAN - scrollpos, false, 1);
            }
          }
        #endif
        break;
      case ID_Tune:
        #if HAS_HOTEND
          if (thermalManager.degTargetHotend(0) != hotendtarget) {
            hotendtarget = thermalManager.degTargetHotend(0);
            if (scrollpos <= TUNE_HOTEND && TUNE_HOTEND <= scrollpos + MROWS) {
              if (process != Proc_Value || selection != TEMP_HOTEND - scrollpos)
                drawFloat(hotendtarget, TUNE_HOTEND - scrollpos, false, 1);
            }
          }
        #endif
        #if HAS_HEATED_BED
          if (thermalManager.degTargetBed() != bedtarget) {
            bedtarget = thermalManager.degTargetBed();
            if (scrollpos <= TUNE_BED && TUNE_BED <= scrollpos + MROWS) {
              if (process != Proc_Value || selection != TEMP_HOTEND - scrollpos)
                drawFloat(bedtarget, TUNE_BED - scrollpos, false, 1);
            }
          }
        #endif
        #if HAS_FAN
          if (thermalManager.fan_speed[0] != fanspeed) {
            fanspeed = thermalManager.fan_speed[0];
            if (scrollpos <= TUNE_FAN && TUNE_FAN <= scrollpos + MROWS) {
              if (process != Proc_Value || selection != TEMP_HOTEND - scrollpos)
                drawFloat(fanspeed, TUNE_FAN - scrollpos, false, 1);
            }
          }
        #endif
        break;
    }
  }
}

void JyersDWIN::audioFeedback(const bool success/*=true*/) {
  if (ui.sound_on)
    DONE_BUZZ(success);
  else
    updateStatus(success ? "Success" : "Failed");
}

void JyersDWIN::saveSettings(char * const buff) {
  TERN_(AUTO_BED_LEVELING_UBL, eeprom_settings.tilt_grid_size = mesh_conf.tilt_grid - 1);
  eeprom_settings.corner_pos = corner_pos * 10;
  memcpy(buff, &eeprom_settings, _MIN(sizeof(eeprom_settings), eeprom_data_size));
}

void JyersDWIN::loadSettings(const char * const buff) {
  memcpy(&eeprom_settings, buff, _MIN(sizeof(eeprom_settings), eeprom_data_size));
  TERN_(AUTO_BED_LEVELING_UBL, mesh_conf.tilt_grid = eeprom_settings.tilt_grid_size + 1);
  if (eeprom_settings.corner_pos == 0) eeprom_settings.corner_pos = 325;
  corner_pos = eeprom_settings.corner_pos / 10.0f;
  redrawScreen();
  #if ENABLED(POWER_LOSS_RECOVERY)
    static bool init = true;
    if (init) {
      init = false;
      queue.inject(F("M1000 S"));
    }
  #endif
}

void JyersDWIN::resetSettings() {
  eeprom_settings.time_format_textual = false;
  TERN_(AUTO_BED_LEVELING_UBL, eeprom_settings.tilt_grid_size = 0);
  eeprom_settings.corner_pos = 325;
  eeprom_settings.cursor_color = 0;
  eeprom_settings.menu_split_line = 0;
  eeprom_settings.menu_top_bg = 0;
  eeprom_settings.menu_top_txt = 0;
  eeprom_settings.highlight_box = 0;
  eeprom_settings.progress_percent = 0;
  eeprom_settings.progress_time = 0;
  eeprom_settings.status_bar_text = 0;
  eeprom_settings.status_area_text = 0;
  eeprom_settings.coordinates_text = 0;
  eeprom_settings.coordinates_split_line = 0;
  TERN_(AUTO_BED_LEVELING_UBL, mesh_conf.tilt_grid = eeprom_settings.tilt_grid_size + 1);
  corner_pos = eeprom_settings.corner_pos / 10.0f;
  TERN_(SOUND_MENU_ITEM, ui.sound_on = ENABLED(SOUND_ON_DEFAULT));
  redrawScreen();
}

void MarlinUI::init_lcd() {
  delay(800);
  SERIAL_ECHOPGM("\nDWIN handshake ");
  if (dwinHandshake()) SERIAL_ECHOLNPGM("ok."); else SERIAL_ECHOLNPGM("error.");
  dwinFrameSetDir(1); // Orientation 90°
  dwinUpdateLCD();     // Show bootscreen (first image)
  for (uint16_t t = 0; t <= 100; t += 2) {
    dwinIconShow(ICON, ICON_Bar, 15, 260);
    dwinDrawRectangle(1, COLOR_BG_BLACK, 15 + t * 242 / 100, 260, 257, 280);
    dwinUpdateLCD();
    delay(20);
  }

  dwinJPGShowAndCache(3);
  dwinJPGCacheTo1(Language_English);
  jyersDWIN.redrawScreen();
}

void MarlinUI::clear_lcd() {}

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  void MarlinUI::pause_show_message(const PauseMessage message, const PauseMode mode/*=PAUSE_MODE_SAME*/, const uint8_t extruder/*=active_extruder*/) {
    if (mode != PAUSE_MODE_SAME) pause_mode = mode;
    switch (message) {
      case PAUSE_MESSAGE_INSERT:  jyersDWIN.confirmHandler(Popup_FilInsert);  break;
      case PAUSE_MESSAGE_PURGE:
      case PAUSE_MESSAGE_OPTION:  jyersDWIN.popupHandler(Popup_PurgeMore);  break;
      case PAUSE_MESSAGE_HEAT:    jyersDWIN.confirmHandler(Popup_HeaterTime); break;
      case PAUSE_MESSAGE_WAITING: jyersDWIN.drawPrintScreen();          break;
      default: break;
    }
  }
#endif

#endif // DWIN_CREALITY_LCD_JYERSUI
