/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../../../inc/MarlinConfig.h"

#if ENABLED(BD_SENSOR)

#include "../../../MarlinCore.h"
#include "../../../gcode/gcode.h"
#include "../../../module/settings.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../module/stepper.h"
#include "../../../module/probe.h"
#include "../../../module/temperature.h"
#include "../../../module/endstops.h"
#include "../../babystep.h"
#include "../../../lcd/marlinui.h"

// I2C software Master library for segment bed heating and bed distance sensor
#include <Panda_segmentBed_I2C.h>

#include "bdl.h"
BDS_Leveling bdl;

//#define DEBUG_OUT_BD
#define DEBUG_OUT ENABLED(DEBUG_OUT_BD)
#include "../../../core/debug_out.h"

// M102 S-5   Read raw Calibrate data
// M102 S-6   Start Calibrate
// M102 S4    Set the adjustable Z height value (e.g., 'M102 S4' means it will do adjusting while the Z height <= 0.4mm , disable with 'M102 S0'.)
// M102 S-1   Read sensor information

#define MAX_BD_HEIGHT                 4.0f
#define CMD_READ_DATA                 1015
#define CMD_READ_VERSION              1016
#define CMD_START_READ_CALIBRATE_DATA 1017
#define CMD_END_READ_CALIBRATE_DATA   1018
#define CMD_START_CALIBRATE           1019
#define CMD_DISTANCE_RAWDATA_TYPE     1020
#define CMD_END_CALIBRATE             1021
#define CMD_REBOOT_SENSOR             1022
#define CMD_SWITCH_MODE               1023
#define DATA_ERROR                    1024

#define BD_SENSOR_I2C_ADDR            0x3C

I2C_SegmentBED BD_I2C_SENSOR;
float BDS_Leveling::pos_zero_offset;
int BDS_Leveling::config_state;
int BDS_Leveling::sw_mode;
bool BDS_Leveling::bd_triggered;
#define move_waiting()  {while(abs(planner.get_axis_position_mm(Z_AXIS)-current_position.z)>0.005) safe_delay(100);}
void BDS_Leveling::init(uint8_t _sda, uint8_t _scl, uint16_t delay_s) {
  config_state = BDS_IDLE;
  sw_mode = -1;
  const int ret = BD_I2C_SENSOR.i2c_init(_sda, _scl, BD_SENSOR_I2C_ADDR, delay_s);
  if (ret != 1) SERIAL_ECHOLNPGM("BD Sensor Init Fail (", ret, ")");
  sync_plan_position();
  pos_zero_offset = planner.get_axis_position_mm(Z_AXIS) - current_position.z;
  SERIAL_ECHOLNPGM("BD Sensor Zero Offset:", pos_zero_offset);
}

bool BDS_Leveling::check(const uint16_t data, const bool raw_data/*=false*/, const bool hicheck/*=false*/) {
  if (BD_I2C_SENSOR.BD_Check_OddEven(data) == 0) {
    SERIAL_ECHOLNPGM("Read Error.");
    return true; // error
  }
  if (raw_data == true) {
    if (hicheck && (data & 0x3FF) > 400)
      SERIAL_ECHOLNPGM("Bad BD Sensor height! Recommended distance 0.5-2.0mm");
    else if (!good_data(data))
      SERIAL_ECHOLNPGM("Invalid data, please calibrate.");
    else
      return false;
  }
  else {
    if ((data & 0x3FF) >= (MAX_BD_HEIGHT) * 100 - 10)
      SERIAL_ECHOLNPGM("Out of Range.");
    else
      return false;
  }
  return true; // error
}

float BDS_Leveling::interpret(const uint16_t data) {
  return (data & 0x3FF) / 100.0f;
}

float BDS_Leveling::bd_read() {
  const uint16_t data = BD_I2C_SENSOR.BD_i2c_read();
  return check(data) ? NAN : interpret(data);
}

float BDS_Leveling::adjust_probe_down(float down_steps){
  int intr, raw_d;
  char tmp_1[21];
  float z_limit = 1.0;
  move_waiting();
  intr = raw_d = BD_I2C_SENSOR.BD_i2c_read() & 0x3FF;
  while (z_limit>0) {
      z_limit -=  down_steps;
       sprintf_P(tmp_1, PSTR("G1Z%d.%02d F100"), int(z_limit), int(z_limit * 100) % 100);
      gcode.process_subcommands_now(tmp_1);
      //gcode.process_subcommands_now(TS(F("G1Z"), p_float_t(z_limit, 3)));
      move_waiting();
      raw_d = BD_I2C_SENSOR.BD_i2c_read() & 0x3FF;
      SERIAL_ECHOLNPGM("down ",intr,"  ", raw_d);
      if (((intr - raw_d) < down_steps*100) && (raw_d < 500)){
          break;
      }
      intr = raw_d;
  }
  return z_limit;
}

void BDS_Leveling::adjust_probe_up(float up_steps){
  int intr0,intr1,intr2;
  //char tmp_1[21];
  float z0;
  BD_I2C_SENSOR.BD_i2c_write(CMD_END_READ_CALIBRATE_DATA);
  BD_I2C_SENSOR.BD_i2c_write(CMD_DISTANCE_RAWDATA_TYPE);
  move_waiting();
 // gcode.process_subcommands_now("G1 Z1");
  z0 = Z_CLEARANCE_BETWEEN_PROBES;
  if(Z_CLEARANCE_BETWEEN_PROBES>=2.5){
      kill(F("Z_CLEARANCE_BETWEEN_PROBES must < 2.5"));
  }
  gcode.process_subcommands_now(TS(F("G1Z"), p_float_t(z0, 3)));
  intr0 = BD_I2C_SENSOR.BD_i2c_read() & 0x3FF;
  move_waiting();
  if (intr0 > 720)
    SERIAL_ECHOLNPGM("warning: triggered in air", intr0);
  if(adjust_probe_down(0.1)<=0.1){
    SERIAL_ECHOLNPGM("adjust failed0, homing again");
    gcode.process_subcommands_now("G1Z4");
    gcode.process_subcommands_now("G28Z0");
    return;
  }

  intr0 = BD_I2C_SENSOR.BD_i2c_read() & 0x3FF;//
  gcode.process_subcommands_now("G92Z0");
  z0 = 0;
  z0 = z0 + 0.4;
  move_waiting();
  gcode.process_subcommands_now(TS(F("G1Z"), p_float_t(z0, 3)));
  planner.synchronize();
  move_waiting();
  safe_delay(500);
  intr1 = BD_I2C_SENSOR.BD_i2c_read() & 0x3FF;//
  if((intr1-10)<intr0){
    //config_state = BDS_HOME_END;
    SERIAL_ECHOLNPGM("adjust failed, homing again");
    gcode.process_subcommands_now("G28Z0");
    return;
  }
  z0 = z0 + 0.1;
  gcode.process_subcommands_now(TS(F("G1Z"), p_float_t(z0, 3)));
  planner.synchronize();
  move_waiting();
  safe_delay(500);
  intr2 = BD_I2C_SENSOR.BD_i2c_read() & 0x3FF;//

  z0 = z0 - (0.5-((intr1 - intr0)*0.1 / ((intr2 - intr1)))) ;
  SERIAL_ECHOLNPGM("z:", intr0,",", intr1,",", intr2,"; ",z0);
  //gcode.process_subcommands_now(F("G1Z0.2"));
  //safe_delay(500);
  gcode.process_subcommands_now(TS(F("G92Z"), p_float_t(z0, 3)));
  //planner.synchronize();
  //safe_delay(100);
  //intr2 = BD_I2C_SENSOR.BD_i2c_read() & 0x3FF;
  //SERIAL_ECHOLNPGM("Zend: ", intr2,", z0:", z0);
  //current_position.z = 4;// z0;
  //gcode.process_subcommands_now(TS(F("G92Z0")));
  //sprintf_P(tmp_1, PSTR("G92Z%d.%02d"), int(z0), int(z0 * 100) % 100);
  z0 = Z_CLEARANCE_BETWEEN_PROBES;//raise after homing
  gcode.process_subcommands_now(TS(F("G1Z"), p_float_t(z0, 3)));
  //gcode.process_subcommands_now("G1Z1");//
  BD_I2C_SENSOR.BD_i2c_write(CMD_END_READ_CALIBRATE_DATA);

}

void BDS_Leveling::prepare_homing() {
  config_state = BDS_HOMING_Z;
  if(sw_mode ==-1){
    read_version(6);
  }
  if(sw_mode == 1){
    BD_I2C_SENSOR.BD_i2c_write(CMD_SWITCH_MODE);
   // BD_I2C_SENSOR.BD_i2c_write(BD_SENSOR_HOME_Z_POSITION*100);
    #ifdef BD_SENSOR_CONTACT_PROBE
      BD_I2C_SENSOR.BD_i2c_write(1);
    #else
      BD_I2C_SENSOR.BD_i2c_write((int)(BD_SENSOR_HOME_Z_POSITION*100));
    #endif
    pinMode(I2C_BD_SDA_PIN, INPUT);
    safe_delay(50);
  }
}

void BDS_Leveling::end_homing() {
  BD_I2C_SENSOR.BD_i2c_write(CMD_END_READ_CALIBRATE_DATA);
  safe_delay(100);         
  if(sw_mode == 1){
    #ifdef BD_SENSOR_CONTACT_PROBE
      current_position.z = Z_CLEARANCE_BETWEEN_PROBES ;
    #else
      current_position.z = bd_read();
    #endif
    // bdl.end_homing();
  }
  else
    current_position.z = bd_read();
  config_state = BDS_HOME_END;
}

bool BDS_Leveling::read_endstop() {
  if(sw_mode == 1){
    bd_triggered=digitalRead(I2C_BD_SDA_PIN);
   // SERIAL_ECHOLNPGM("r", bd_triggered);
  }
  return bd_triggered;
}

void BDS_Leveling::read_version(int len_s) {
  char tmp_1[21];
  BD_I2C_SENSOR.BD_i2c_write(CMD_READ_VERSION);
  safe_delay(50);
  for (int i = 0; i < len_s; i++) {
    tmp_1[i] = BD_I2C_SENSOR.BD_i2c_read() & 0xFF;
    safe_delay(50);
  }
  BD_I2C_SENSOR.BD_i2c_write(CMD_END_READ_CALIBRATE_DATA);
  BD_I2C_SENSOR.BD_i2c_write(CMD_END_READ_CALIBRATE_DATA);
  //SERIAL_ECHOLNPGM("BD Sensor version:", tmp_1);
  if (tmp_1[0] != 'V') {
    SERIAL_ECHOLNPGM("Read Error. Check connection and delay.");
    sw_mode = -1;
  }
  else if (tmp_1[4] != ' '){
    sw_mode = 1;
  }
  else
    sw_mode = 0;
  SERIAL_ECHOLNPGM("BD Sensor version:", tmp_1, ",sw_mode:", sw_mode);
  safe_delay(50);
}

void BDS_Leveling::process() {
  if (config_state == BDS_IDLE && printingIsActive()) return;
  static millis_t next_check_ms = 0; // starting at T=0
  static float zpos = 0.0f;
  const millis_t ms = millis();
  if(config_state == BDS_HOMING_Z && sw_mode == 1)
    return;
  if(config_state == BDS_HOME_END){
    config_state = BDS_IDLE;
#ifdef BD_SENSOR_CONTACT_PROBE
    if(sw_mode == 1){
      adjust_probe_up(0);
    }
#endif    
  }
  if (ELAPSED(ms, next_check_ms)) { // timed out (or first run)
    // Check at 1KHz, 5Hz, or 20Hz
    next_check_ms = ms + (config_state == BDS_HOMING_Z ? 1 : (config_state < BDS_IDLE ? 100 : 50));

    uint16_t tmp = 0;
    const float cur_z = planner.get_axis_position_mm(Z_AXIS) - pos_zero_offset;
    static float old_cur_z = cur_z, old_buf_z = current_position.z;
    tmp = BD_I2C_SENSOR.BD_i2c_read();
    if (BD_I2C_SENSOR.BD_Check_OddEven(tmp) && good_data(tmp)) {
      const float z_sensor = interpret(tmp);
      #if ENABLED(BABYSTEPPING)
        if (config_state > 0) {
          if (cur_z < config_state * 0.1f
            && old_cur_z == cur_z
            && old_buf_z == current_position.z
            && z_sensor < (MAX_BD_HEIGHT) - 0.1f
          ) {
            babystep.set_mm(Z_AXIS, cur_z - z_sensor);
            DEBUG_ECHOLNPGM("BD:", z_sensor, ", Z:", cur_z, "|", current_position.z);
          }
          else
            babystep.set_mm(Z_AXIS, 0);
        }
      #endif

      old_cur_z = cur_z;
      old_buf_z = current_position.z;
      if(sw_mode <= 0){
        if(z_sensor <= BD_SENSOR_HOME_Z_POSITION)
          bd_triggered = true;
        else
          bd_triggered = false;
      }
      //endstops.bdp_state_update(z_sensor <= BD_SENSOR_HOME_Z_POSITION);

      #if HAS_STATUS_MESSAGE
        static float old_z_sensor = 0;
        if (old_z_sensor != z_sensor) {
          old_z_sensor = z_sensor;
          char tmp_1[32];
          sprintf_P(tmp_1, PSTR("BD:%d.%02dmm"), int(z_sensor), int(z_sensor * 100) % 100);
          //SERIAL_ECHOLNPGM("Bed Dis:", z_sensor, "mm");
          ui.set_status(tmp_1, true);
        }
      #endif
    }
    else if (config_state == BDS_HOMING_Z) {
      SERIAL_ECHOLNPGM("Read:", tmp);
      kill(F("BDsensor connect Err!"));
    }

    DEBUG_ECHOLNPGM("BD:", tmp & 0x3FF, " Z:", cur_z, "|", current_position.z);
    if (TERN0(DEBUG_OUT_BD, BD_I2C_SENSOR.BD_Check_OddEven(tmp) == 0)) DEBUG_ECHOLNPGM("CRC error");

    if (!good_data(tmp)) {
      BD_I2C_SENSOR.BD_i2c_stop();
      safe_delay(10);
    }

    // Read version. Usually used as a connection check
    if (config_state == BDS_VERSION) {
      config_state = BDS_IDLE;
      read_version(19);
    }
    // read raw calibrate data
    else if (config_state == BDS_READ_RAW) {
      BD_I2C_SENSOR.BD_i2c_write(CMD_START_READ_CALIBRATE_DATA);
      safe_delay(100);

      for (int i = 0; i < MAX_BD_HEIGHT * 10; i++) {
        tmp = BD_I2C_SENSOR.BD_i2c_read();
        SERIAL_ECHOLNPGM("Calibrate data:", i, ",", tmp & 0x3FF);
        (void)check(tmp, true, i == 0);
        safe_delay(50);
      }
      BD_I2C_SENSOR.BD_i2c_write(CMD_END_READ_CALIBRATE_DATA);
      safe_delay(50);
      BD_I2C_SENSOR.BD_i2c_write(CMD_END_READ_CALIBRATE_DATA);
      config_state = BDS_IDLE;
    }
    else if (config_state <= BDS_CALIBRATE_START) {   // Start Calibrate
      safe_delay(10);
      if (config_state == BDS_CALIBRATE_START) {
        config_state = BDS_CALIBRATING;
        REMEMBER(gsit, gcode.stepper_inactive_time, MIN_TO_MS(5));
        SERIAL_ECHOLNPGM("c_z0:", planner.get_axis_position_mm(Z_AXIS), "-", pos_zero_offset);

        // Move the z axis instead of enabling the Z axis with M17
        // TODO: Use do_blocking_move_to_z for synchronized move.
      //  current_position.z = 0;
      //  sync_plan_position();
        gcode.process_subcommands_now(F("G1Z0.05"));
        safe_delay(300);
        gcode.process_subcommands_now(F("G1Z0.00"));
        safe_delay(300);
       // current_position.z = 0;
        sync_plan_position();
        planner.synchronize();
        //safe_delay(1000);
/*
        while ((planner.get_axis_position_mm(Z_AXIS) - pos_zero_offset) > 0.00001f) {
          safe_delay(200);
          SERIAL_ECHOLNPGM("waiting cur_z:", planner.get_axis_position_mm(Z_AXIS));
        }*/
        zpos = 0.00001f;
        safe_delay(100);
        BD_I2C_SENSOR.BD_i2c_write(CMD_START_CALIBRATE); // Begin calibrate
        SERIAL_ECHOLNPGM("BD Sensor Calibrating...");
        safe_delay(200);
      }
      else if ((planner.get_axis_position_mm(Z_AXIS) - pos_zero_offset) < 10.0f) {
        if (zpos >= MAX_BD_HEIGHT) {
          config_state = BDS_IDLE;
          BD_I2C_SENSOR.BD_i2c_write(CMD_END_CALIBRATE); // End calibrate
          SERIAL_ECHOLNPGM("BD Sensor calibrated.");
          zpos = 7.0f;
          safe_delay(500);
        }
        else {
          char tmp_1[32];
          // TODO: Use prepare_internal_move_to_destination to guarantee machine space
          sprintf_P(tmp_1, PSTR("G1Z%d.%d"), int(zpos), int(zpos * 10) % 10);
          gcode.process_subcommands_now(tmp_1);
          SERIAL_ECHO(tmp_1); SERIAL_ECHOLNPGM(", Z:", current_position.z);
          uint16_t failcount = 300;
          for (float tmp_k = 0; abs(zpos - tmp_k) > 0.006f && failcount--;) {
            tmp_k = planner.get_axis_position_mm(Z_AXIS) - pos_zero_offset;
            safe_delay(10);
            if (!failcount--) break;
          }
          safe_delay(600);
          tmp = uint16_t((zpos + 0.00001f) * 10);
          BD_I2C_SENSOR.BD_i2c_write(tmp);
          SERIAL_ECHOLNPGM("w:", tmp, ", Z:", zpos);
          zpos += 0.1001f;
        }
      }
    }
  }
}

#endif // BD_SENSOR
