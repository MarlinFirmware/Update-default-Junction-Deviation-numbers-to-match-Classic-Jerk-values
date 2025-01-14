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
 * -----------------
 * G-Codes in Marlin
 * -----------------
 *
 * Helpful G-code references:
 *  - https://marlinfw.org/meta/gcode
 *  - https://reprap.org/wiki/G-code
 *  - https://linuxcnc.org/docs/html/gcode.html
 *
 * Help to document Marlin's G-codes online:
 *  - https://github.com/MarlinFirmware/MarlinDocumentation
 *
 * -----------------
 *
 * "G" Codes
 *
 * G0   -> G1
 * G1   - Coordinated Movement X Y Z E
 * G2   - CW ARC
 * G3   - CCW ARC
 * G4   - Dwell S<seconds> or P<milliseconds>
 * G5   - Cubic B-spline with XYZE destination and IJPQ offsets
 * G10  - Retract filament according to settings of M207 (Requires FWRETRACT)
 * G11  - Retract recover filament according to settings of M208 (Requires FWRETRACT)
 * G12  - Clean tool (Requires NOZZLE_CLEAN_FEATURE)
 * G17  - Select Plane XY (Requires CNC_WORKSPACE_PLANES)
 * G18  - Select Plane ZX (Requires CNC_WORKSPACE_PLANES)
 * G19  - Select Plane YZ (Requires CNC_WORKSPACE_PLANES)
 * G20  - Set input units to inches (Requires INCH_MODE_SUPPORT)
 * G21  - Set input units to millimeters (Requires INCH_MODE_SUPPORT)
 * G26  - Mesh Validation Pattern (Requires G26_MESH_VALIDATION)
 * G27  - Park Nozzle (Requires NOZZLE_PARK_FEATURE)
 * G28  - Home one or more axes
 * G29  - Start or continue the bed leveling probe procedure (Requires bed leveling)
 * G30  - Single Z probe, probes bed at X Y location (defaults to current XY location)
 * G31  - Dock sled (Z_PROBE_SLED only)
 * G32  - Undock sled (Z_PROBE_SLED only)
 * G33  - Delta Auto-Calibration (Requires DELTA_AUTO_CALIBRATION)
 * G34  - Z Stepper automatic alignment using probe: I<iterations> T<accuracy> A<amplification> (Requires Z_STEPPER_AUTO_ALIGN)
 * G35  - Read bed corners to help adjust bed screws: T<screw_thread> (Requires ASSISTED_TRAMMING)
 * G38  - Probe in any direction using the Z_MIN_PROBE (Requires G38_PROBE_TARGET)
 * G42  - Coordinated move to a mesh point (Requires MESH_BED_LEVELING, AUTO_BED_LEVELING_BLINEAR, or AUTO_BED_LEVELING_UBL)
 * G60  - Save current position. (Requires SAVED_POSITIONS)
 * G61  - Apply/restore saved coordinates. (Requires SAVED_POSITIONS)
 * G76  - Calibrate first layer temperature offsets. (Requires PTC_PROBE and PTC_BED)
 * G80  - Cancel current motion mode (Requires GCODE_MOTION_MODES)
 * G90  - Use Absolute Coordinates
 * G91  - Use Relative Coordinates
 * G92  - Set current position to coordinates given
 *
 * "M" Codes
 *
 * M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
 * M1   -> M0
 * M3   - Turn ON Laser | Spindle (clockwise), set Power | Speed. (Requires SPINDLE_FEATURE or LASER_FEATURE)
 * M4   - Turn ON Laser | Spindle (counter-clockwise), set Power | Speed. (Requires SPINDLE_FEATURE or LASER_FEATURE)
 * M5   - Turn OFF Laser | Spindle. (Requires SPINDLE_FEATURE or LASER_FEATURE)
 * M7   - Turn mist coolant ON. (Requires COOLANT_CONTROL)
 * M8   - Turn flood coolant ON. (Requires COOLANT_CONTROL)
 * M9   - Turn coolant OFF. (Requires COOLANT_CONTROL)
 * M10  - Turn Vacuum or Blower motor ON (Requires AIR_EVACUATION)
 * M11  - Turn Vacuum or Blower motor OFF (Requires AIR_EVACUATION)
 * M12  - Set up closed loop control system. (Requires EXTERNAL_CLOSED_LOOP_CONTROLLER)
 * M16  - Expected printer check. (Requires EXPECTED_PRINTER_CHECK)
 * M17  - Enable/Power all stepper motors
 * M18  - Disable all stepper motors; same as M84
 *
 *** Print from Media (SDSUPPORT) ***
 * M20  - List SD card. (Requires SDSUPPORT)
 * M21  - Init SD card. (Requires SDSUPPORT) With MULTI_VOLUME select a drive with `M21 Pn` / 'M21 S' / 'M21 U'.
 * M22  - Release SD card. (Requires SDSUPPORT)
 * M23  - Select SD file: "M23 /path/file.gco". (Requires SDSUPPORT)
 * M24  - Start/resume SD print. (Requires SDSUPPORT)
 * M25  - Pause SD print. (Requires SDSUPPORT)
 * M26  - Set SD position in bytes: "M26 S12345". (Requires SDSUPPORT)
 * M27  - Report SD print status. (Requires SDSUPPORT)
 *        OR, with 'S<seconds>' set the SD status auto-report interval. (Requires AUTO_REPORT_SD_STATUS)
 *        OR, with 'C' get the current filename.
 * M28  - Start SD write: "M28 /path/file.gco". (Requires SDSUPPORT)
 * M29  - Stop SD write. (Requires SDSUPPORT)
 * M30  - Delete file from SD: "M30 /path/file.gco" (Requires SDSUPPORT)
 * M31  - Report time since last M109 or SD card start to serial.
 * M32  - Select file and start SD print: "M32 [S<bytepos>] !/path/file.gco#". (Requires SDSUPPORT)
 *        Use P to run other files as sub-programs: "M32 P !filename#"
 *        The '#' is necessary when calling from within sd files, as it stops buffer prereading
 * M33  - Get the longname version of a path. (Requires LONG_FILENAME_HOST_SUPPORT)
 * M34  - Set SD Card sorting options. (Requires SDCARD_SORT_ALPHA)
 *
 * M42  - Change pin status via G-code: M42 P<pin> S<value>. LED pin assumed if P is omitted. (Requires DIRECT_PIN_CONTROL)
 * M43  - Display pin status, watch pins for changes, watch endstops & toggle LED, Z servo probe test, toggle pins (Requires PINS_DEBUGGING)
 * M48  - Measure Z Probe repeatability: M48 P<points> X<pos> Y<pos> V<level> E<engage> L<legs> S<chizoid>. (Requires Z_MIN_PROBE_REPEATABILITY_TEST)
 *
 * M73  - Set the progress percentage. (Requires SET_PROGRESS_MANUALLY)
 * M75  - Start the print job timer.
 * M76  - Pause the print job timer.
 * M77  - Stop the print job timer.
 * M78  - Show statistical information about the print jobs. (Requires PRINTCOUNTER)
 *
 * M80  - Turn on Power Supply. (Requires PSU_CONTROL)
 * M81  - Turn off Power Supply. (Requires PSU_CONTROL)
 *
 * M82  - Set E codes absolute (default).
 * M83  - Set E codes relative while in Absolute (G90) mode.
 * M84  - Disable steppers until next move, or use S<seconds> to specify an idle
 *        duration after which steppers should turn off. S0 disables the timeout.
 * M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 * M86  - Set / Report Hotend Idle Timeout. (Requires HOTEND_IDLE_TIMEOUT)
 * M87  - Cancel Hotend Idle Timeout (by setting the timeout period to 0). (Requires HOTEND_IDLE_TIMEOUT)
 * M92  - Set planner.settings.axis_steps_per_mm for one or more axes. (Requires EDITABLE_STEPS_PER_UNIT)
 *
 * M100 - Watch Free Memory (for debugging) (Requires M100_FREE_MEMORY_WATCHER)
 *
 * M102 - Configure Bed Distance Sensor. (Requires BD_SENSOR)
 *
 * M104 - Set extruder target temp.
 * M105 - Report current temperatures.
 * M106 - Set print fan speed.
 * M107 - Print fan off.
 * M108 - Break out of heating loops (M109, M190, M303). With no controller, breaks out of M0/M1. (Requires EMERGENCY_PARSER)
 * M109 - S<temp> Wait for extruder current temp to reach target temp. ** Wait only when heating! **
 *        R<temp> Wait for extruder current temp to reach target temp. ** Wait for heating or cooling. **
 *        If AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
 *
 * M110 - Get or set the current line number. (Used by host printing)
 * M111 - Set debug flags: "M111 S<flagbits>". See flag bits defined in enum.h.
 * M112 - Full Shutdown.
 *
 * M113 - Get or set the timeout interval for Host Keepalive "busy" messages. (Requires HOST_KEEPALIVE_FEATURE)
 * M114 - Report current position.
 * M115 - Report capabilities. (Requires CAPABILITIES_REPORT)
 * M117 - Display a message on the controller screen. (Requires an LCD)
 * M118 - Display a message in the host console.
 *
 * M119 - Report endstops status.
 * M120 - Enable endstops detection.
 * M121 - Disable endstops detection.
 *
 * M122 - Debug stepper (Requires at least one _DRIVER_TYPE defined as TMC2130/2160/5130/5160/2208/2209/2660)
 * M123 - Report fan tachometers. (Requires En_FAN_TACHO_PIN) Optionally set auto-report interval. (Requires AUTO_REPORT_FANS)
 * M125 - Save current position and move to filament change position. (Requires PARK_HEAD_ON_PAUSE)
 *
 * M126 - Solenoid Air Valve Open. (Requires BARICUDA)
 * M127 - Solenoid Air Valve Closed. (Requires BARICUDA)
 * M128 - EtoP Open. (Requires BARICUDA)
 * M129 - EtoP Closed. (Requires BARICUDA)
 *
 * M140 - Set bed target temp. S<temp>
 * M141 - Set heated chamber target temp. S<temp> (Requires a chamber heater)
 * M143 - Set cooler target temp. S<temp> (Requires a laser cooling device)
 * M145 - Set heatup values for materials on the LCD. H<hotend> B<bed> F<fan speed> for S<material> (0=PLA, 1=ABS)
 * M149 - Set temperature units. (Requires TEMPERATURE_UNITS_SUPPORT)
 * M150 - Set Status LED Color as R<red> U<green> B<blue> W<white> P<bright>. Values 0-255. (Requires BLINKM, RGB_LED, RGBW_LED, NEOPIXEL_LED, PCA9533, or PCA9632).
 * M154 - Auto-report position with interval of S<seconds>. (Requires AUTO_REPORT_POSITION)
 * M155 - Auto-report temperatures with interval of S<seconds>. (Requires AUTO_REPORT_TEMPERATURES)
 * M163 - Set a single proportion for a mixing extruder. (Requires MIXING_EXTRUDER)
 * M164 - Commit the mix and save to a virtual tool (current, or as specified by 'S'). (Requires MIXING_EXTRUDER)
 * M165 - Set the mix for the mixing extruder (and current virtual tool) with parameters ABCDHI. (Requires MIXING_EXTRUDER and DIRECT_MIXING_IN_G1)
 * M166 - Set the Gradient Mix for the mixing extruder. (Requires GRADIENT_MIX)
 * M190 - Set bed target temperature and wait. R<temp> Set target temperature and wait. S<temp> Set, but only wait when heating. (Requires TEMP_SENSOR_BED)
 * M192 - Wait for probe to reach target temperature. (Requires TEMP_SENSOR_PROBE)
 * M193 - R<temp> Wait for cooler to reach target temp. ** Wait for cooling. **
 * M200 - Set filament diameter, D<diameter>, setting E axis units to cubic. (Use S0 to revert to linear units.)
 * M201 - Set max acceleration in units/s^2 for print moves: "M201 X<accel> Y<accel> Z<accel> E<accel>"
 * M202 - Set max acceleration in units/s^2 for travel moves: "M202 X<accel> Y<accel> Z<accel> E<accel>" ** UNUSED IN MARLIN! **
 * M203 - Set maximum feedrate: "M203 X<fr> Y<fr> Z<fr> E<fr>" in units/sec.
 * M204 - Set default acceleration in units/sec^2: P<printing> R<extruder_only> T<travel>
 * M205 - Set advanced settings. Current units apply:
            S<print> T<travel> minimum speeds
            B<minimum segment time>
            X<max X jerk>, Y<max Y jerk>, Z<max Z jerk>, E<max E jerk>
 * M206 - Set additional homing offset. (Disabled by NO_WORKSPACE_OFFSETS or DELTA)
 * M207 - Set Retract Length: S<length>, Feedrate: F<units/min>, and Z lift: Z<distance>. (Requires FWRETRACT)
 * M208 - Set Recover (unretract) Additional (!) Length: S<length> and Feedrate: F<units/min>. (Requires FWRETRACT)
 * M209 - Turn Automatic Retract Detection on/off: S<0|1> (For slicers that don't support G10/11). (Requires FWRETRACT_AUTORETRACT)
          Every normal extrude-only move will be classified as retract depending on the direction.
 * M210 - Set or Report the homing feedrate (Requires EDITABLE_HOMING_FEEDRATE)
 * M211 - Enable, Disable, and/or Report software endstops: S<0|1> (Requires MIN_SOFTWARE_ENDSTOPS or MAX_SOFTWARE_ENDSTOPS)
 * M217 - Set filament swap parameters: "M217 S<length> P<feedrate> R<feedrate>". (Requires SINGLENOZZLE)
 * M218 - Set/get a tool offset: "M218 T<index> X<offset> Y<offset>". (Requires 2 or more extruders)
 * M220 - Set Feedrate Percentage: "M220 S<percent>" (i.e., "FR" on the LCD)
 *        Use "M220 B" to back up the Feedrate Percentage and "M220 R" to restore it. (Requires an MMU_MODEL version 2 or 2S)
 * M221 - Set Flow Percentage: "M221 S<percent>" (Requires an extruder)
 * M226 - Wait until a pin is in a given state: "M226 P<pin> S<state>" (Requires DIRECT_PIN_CONTROL)
 * M240 - Trigger a camera to take a photograph. (Requires PHOTO_GCODE)
 * M250 - Set LCD contrast: "M250 C<contrast>" (0-63). (Requires LCD support)
 * M255 - Set LCD sleep time: "M255 S<minutes>" (0-99). (Requires an LCD with brightness or sleep/wake)
 * M256 - Set LCD brightness: "M256 B<brightness>" (0-255). (Requires an LCD with brightness control)
 * M260 - i2c Send Data (Requires EXPERIMENTAL_I2CBUS)
 * M261 - i2c Request Data (Requires EXPERIMENTAL_I2CBUS)
 * M280 - Set servo position absolute: "M280 P<index> S<angle|µs>". (Requires servos)
 * M281 - Set servo min|max position: "M281 P<index> L<min> U<max>". (Requires EDITABLE_SERVO_ANGLES)
 * M282 - Detach servo: "M282 P<index>". (Requires SERVO_DETACH_GCODE)
 * M290 - Babystepping (Requires BABYSTEPPING)
 * M293 - Babystep Z UP (Requires EP_BABYSTEPPING)
 * M294 - Babystep Z DOWN (Requires EP_BABYSTEPPING)
 * M300 - Play beep sound S<frequency Hz> P<duration ms>
 * M301 - Set PID parameters P I and D. (Requires PIDTEMP)
 * M302 - Allow cold extrudes, or set the minimum extrude S<temperature>. (Requires PREVENT_COLD_EXTRUSION)
 * M303 - PID relay autotune S<temperature> sets the target temperature. Default 150C. (Requires PIDTEMP)
 * M304 - Set bed PID parameters P I and D. (Requires PIDTEMPBED)
 * M305 - Set user thermistor parameters R T and P. (Requires TEMP_SENSOR_x 1000)
 * M306 - MPC autotune. (Requires MPCTEMP)
 * M309 - Set chamber PID parameters P I and D. (Requires PIDTEMPCHAMBER)
 * M350 - Set microstepping mode. (Requires digital microstepping pins.)
 * M351 - Toggle MS1 MS2 pins directly. (Requires digital microstepping pins.)
 * M355 - Set Case Light on/off and set brightness. (Requires CASE_LIGHT_PIN)
 * M380 - Activate solenoid on active tool (Requires EXT_SOLENOID) or the tool specified by 'S' (Requires MANUAL_SOLENOID_CONTROL).
 * M381 - Disable solenoids on all tools (Requires EXT_SOLENOID) or the tool specified by 'S' (Requires MANUAL_SOLENOID_CONTROL).
 * M400 - Finish all moves.
 * M401 - Deploy and activate Z probe. (Requires a probe)
 * M402 - Deactivate and stow Z probe. (Requires a probe)
 * M403 - Set filament type for PRUSA MMU2
 * M404 - Display or set the Nominal Filament Width: "W<diameter>". (Requires FILAMENT_WIDTH_SENSOR)
 * M405 - Enable Filament Sensor flow control. "M405 D<delay_cm>". (Requires FILAMENT_WIDTH_SENSOR)
 * M406 - Disable Filament Sensor flow control. (Requires FILAMENT_WIDTH_SENSOR)
 * M407 - Display measured filament diameter in millimeters. (Requires FILAMENT_WIDTH_SENSOR)
 * M410 - Quickstop. Abort all planned moves.
 * M412 - Enable / Disable Filament Runout Detection. (Requires FILAMENT_RUNOUT_SENSOR)
 * M413 - Enable / Disable Power-Loss Recovery. (Requires POWER_LOSS_RECOVERY)
 * M414 - Set language by index. (Requires LCD_LANGUAGE_2...)
 * M420 - Enable/Disable Leveling (with current values) S1=enable S0=disable (Requires MESH_BED_LEVELING or ABL)
 * M421 - Set a single Z coordinate in the Mesh Leveling grid. X<units> Y<units> Z<units> (Requires MESH_BED_LEVELING, AUTO_BED_LEVELING_BILINEAR, or AUTO_BED_LEVELING_UBL)
 * M422 - Set Z Stepper automatic alignment position using probe. X<units> Y<units> A<axis> (Requires Z_STEPPER_AUTO_ALIGN)
 * M425 - Enable/Disable and tune backlash correction. (Requires BACKLASH_COMPENSATION and BACKLASH_GCODE)
 * M428 - Set the home_offset based on the current_position. Nearest edge applies. (Disabled by NO_WORKSPACE_OFFSETS or DELTA)
 * M430 - Read the system current, voltage, and power (Requires POWER_MONITOR_CURRENT, POWER_MONITOR_VOLTAGE, or POWER_MONITOR_FIXED_VOLTAGE)
 * M485 - Send RS485 packets (Requires RS485_SERIAL_PORT)
 * M486 - Identify and cancel objects. (Requires CANCEL_OBJECTS)
 * M493 - Get or set input FT Motion / Shaping parameters. (Requires FT_MOTION)
 * M500 - Store parameters in EEPROM. (Requires EEPROM_SETTINGS)
 * M501 - Restore parameters from EEPROM. (Requires EEPROM_SETTINGS)
 * M502 - Revert to the default "factory settings". ** Does not write them to EEPROM! **
 * M503 - Print the current settings (in memory): "M503 S<verbose>". S0 specifies compact output.
 * M504 - Validate EEPROM contents. (Requires EEPROM_SETTINGS)
 * M510 - Lock Printer (Requires PASSWORD_FEATURE)
 * M511 - Unlock Printer (Requires PASSWORD_UNLOCK_GCODE)
 * M512 - Set/Change/Remove Password (Requires PASSWORD_CHANGE_GCODE)
 * M524 - Abort the current SD print job started with M24. (Requires SDSUPPORT)
 * M540 - Enable/disable SD card abort on endstop hit: "M540 S<state>". (Requires SD_ABORT_ON_ENDSTOP_HIT)
 * M552 - Get or set IP address. Enable/disable network interface. (Requires enabled Ethernet port)
 * M553 - Get or set IP netmask. (Requires enabled Ethernet port)
 * M554 - Get or set IP gateway. (Requires enabled Ethernet port)
 * M569 - Enable stealthChop on an axis. (Requires at least one _DRIVER_TYPE to be TMC2130/2160/2208/2209/5130/5160)
 * M575 - Change the serial baud rate. (Requires BAUD_RATE_GCODE)
 * M592 - Get or set Nonlinear Extrusion parameters. (Requires NONLINEAR_EXTRUSION)
 * M593 - Get or set input shaping parameters. (Requires INPUT_SHAPING_[XY])
 * M600 - Pause for filament change: "M600 X<pos> Y<pos> Z<raise> E<first_retract> L<later_retract>". (Requires ADVANCED_PAUSE_FEATURE)
 * M603 - Configure filament change: "M603 T<tool> U<unload_length> L<load_length>". (Requires ADVANCED_PAUSE_FEATURE)
 * M605 - Set Dual X-Carriage movement mode: "M605 S<mode> [X<x_offset>] [R<temp_offset>]". (Requires DUAL_X_CARRIAGE)
 * M665 - Set delta configurations: "M665 H<delta height> L<diagonal rod> R<delta radius> S<segments/s> B<calibration radius> X<Alpha angle trim> Y<Beta angle trim> Z<Gamma angle trim> (Requires DELTA)
 *        Set SCARA configurations: "M665 S<segments-per-second> P<theta-psi-offset> T<theta-offset> Z<z-offset> (Requires MORGAN_SCARA or MP_SCARA)
 *        Set Polargraph draw area and belt length: "M665 S<segments-per-second> L<draw-area-left> R<draw-area-right> T<draw-area-top> B<draw-area-bottom> H<max-belt-length>"
 * M666 - Set/get offsets for delta (Requires DELTA) or dual endstops. (Requires [XYZ]_DUAL_ENDSTOPS)
 * M672 - Set/Reset Duet Smart Effector's sensitivity. (Requires DUET_SMART_EFFECTOR and SMART_EFFECTOR_MOD_PIN)
 * M701 - Load filament (Requires FILAMENT_LOAD_UNLOAD_GCODES)
 * M702 - Unload filament (Requires FILAMENT_LOAD_UNLOAD_GCODES)
 *
 *** PRUSA_MMU3 ***
 * M704 - Preload to MMU
 * M705 - Eject filament
 * M706 - Cut filament
 * M707 - Read from MMU register
 * M708 - Write to MMU register
 * M709 - MMU power & reset
 *
 * M808 - Set or Goto a Repeat Marker (Requires GCODE_REPEAT_MARKERS)
 * M810-M819 - Define/execute a G-code macro (Requires GCODE_MACROS)
 * M820 - Report all defined M810-M819 G-code macros (Requires GCODE_MACROS)
 * M851 - Set Z probe's XYZ offsets in current units. (Negative values: X=left, Y=front, Z=below)
 * M852 - Set skew factors: "M852 [I<xy>] [J<xz>] [K<yz>]". (Requires SKEW_CORRECTION_GCODE, plus SKEW_CORRECTION_FOR_Z for IJ)
 *
 *** I2C_POSITION_ENCODERS ***
 * M860 - Report the position of position encoder modules.
 * M861 - Report the status of position encoder modules.
 * M862 - Perform an axis continuity test for position encoder modules.
 * M863 - Perform steps-per-mm calibration for position encoder modules.
 * M864 - Change position encoder module I2C address.
 * M865 - Check position encoder module firmware version.
 * M866 - Report or reset position encoder module error count.
 * M867 - Enable/disable or toggle error correction for position encoder modules.
 * M868 - Report or set position encoder module error correction threshold.
 * M869 - Report position encoder module error.
 *
 * M871 - Print/reset/clear first layer temperature offset values. (Requires PTC_PROBE, PTC_BED, or PTC_HOTEND)
 * M876 - Handle Prompt Response. (Requires HOST_PROMPT_SUPPORT and not EMERGENCY_PARSER)
 * M900 - Get or Set Linear Advance K-factor. (Requires LIN_ADVANCE)
 * M906 - Set or get motor current in milliamps using axis codes XYZE, etc. Report values if no axis codes given. (Requires at least one _DRIVER_TYPE defined as TMC2130/2160/5130/5160/2208/2209/2660)
 * M907 - Set digital trimpot motor current using axis codes. (Requires a board with digital trimpots)
 * M908 - Control digital trimpot directly. (Requires HAS_MOTOR_CURRENT_DAC or DIGIPOTSS_PIN)
 * M909 - Print digipot/DAC current value. (Requires HAS_MOTOR_CURRENT_DAC)
 * M910 - Commit digipot/DAC value to external EEPROM via I2C. (Requires HAS_MOTOR_CURRENT_DAC)
 * M911 - Report stepper driver overtemperature pre-warn condition. (Requires at least one _DRIVER_TYPE defined as TMC2130/2160/5130/5160/2208/2209/2660)
 * M912 - Clear stepper driver overtemperature pre-warn condition flag. (Requires at least one _DRIVER_TYPE defined as TMC2130/2160/5130/5160/2208/2209/2660)
 * M913 - Set HYBRID_THRESHOLD speed. (Requires HYBRID_THRESHOLD)
 * M914 - Set StallGuard sensitivity. (Requires SENSORLESS_HOMING or SENSORLESS_PROBING)
 * M919 - Get or Set motor Chopper Times (time_off, hysteresis_end, hysteresis_start) using axis codes XYZE, etc. If no parameters are given, report. (Requires at least one _DRIVER_TYPE defined as TMC2130/2160/5130/5160/2208/2209/2660)
 * M936 - OTA update firmware. (Requires OTA_FIRMWARE_UPDATE)
 * M951 - Set Magnetic Parking Extruder parameters. (Requires MAGNETIC_PARKING_EXTRUDER)
 * M3426 - Read MCP3426 ADC over I2C. (Requires HAS_MCP3426_ADC)
 * M7219 - Control Max7219 Matrix LEDs. (Requires MAX7219_GCODE)
 *
 *** SCARA ***
 * M360 - SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
 * M361 - SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
 * M362 - SCARA calibration: Move to cal-position PsiA (0 deg calibration)
 * M363 - SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
 * M364 - SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
 *
 *** Custom codes (can be changed to suit future G-code standards) ***
 * G425 - Calibrate using a conductive object. (Requires CALIBRATION_GCODE)
 * M928 - Start SD logging: "M928 filename.gco". Stop with M29. (Requires SDSUPPORT)
 * M993 - Backup SPI Flash to SD
 * M994 - Load a Backup from SD to SPI Flash
 * M995 - Touch screen calibration for TFT display
 * M997 - Perform in-application firmware update
 * M999 - Restart after being stopped by error
 *
 * D... - Custom Development G-code. Add hooks to 'gcode_D.cpp' for developers to test features. (Requires MARLIN_DEV_MODE)
 *        D576 - Set buffer monitoring options. (Requires BUFFER_MONITORING)
 *
 *** "T" Codes ***
 *
 * T0-T3 - Select an extruder (tool) by index: "T<n> F<units/min>"
 */

#include "../inc/MarlinConfig.h"
#include "parser.h"

#if ENABLED(I2C_POSITION_ENCODERS)
  #include "../feature/encoder_i2c.h"
#endif

#if ANY(IS_SCARA, POLAR) || defined(G0_FEEDRATE)
  #define HAS_FAST_MOVES 1
#endif

#if ENABLED(MARLIN_SMALL_BUILD)
  #define GCODE_ERR_MSG(V...) "?"
#else
  #define GCODE_ERR_MSG(V...) "?" V
#endif

enum AxisRelative : uint8_t {
  LOGICAL_AXIS_LIST(REL_E, REL_X, REL_Y, REL_Z, REL_I, REL_J, REL_K, REL_U, REL_V, REL_W)
  #if HAS_EXTRUDERS
    , E_MODE_ABS, E_MODE_REL
  #endif
  , NUM_REL_MODES
};
typedef bits_t(NUM_REL_MODES) relative_t;

extern const char G28_STR[];

class GcodeSuite {
public:

  static relative_t axis_relative;

  GcodeSuite() { // Relative motion mode for each logical axis
    axis_relative = AxisBits(AXIS_RELATIVE_MODES).bits;
  }

  static bool axis_is_relative(const AxisEnum a) {
    #if HAS_EXTRUDERS
      if (a == E_AXIS) {
        if (TEST(axis_relative, E_MODE_REL)) return true;
        if (TEST(axis_relative, E_MODE_ABS)) return false;
      }
    #endif
    return TEST(axis_relative, a);
  }
  static void set_relative_mode(const bool rel) {
    axis_relative = rel ? (0 LOGICAL_AXIS_GANG(
      | _BV(REL_E),
      | _BV(REL_X), | _BV(REL_Y), | _BV(REL_Z),
      | _BV(REL_I), | _BV(REL_J), | _BV(REL_K),
      | _BV(REL_U), | _BV(REL_V), | _BV(REL_W)
    )) : 0;
  }
  #if HAS_EXTRUDERS
    static void set_e_relative() {
      CBI(axis_relative, E_MODE_ABS);
      SBI(axis_relative, E_MODE_REL);
    }
    static void set_e_absolute() {
      CBI(axis_relative, E_MODE_REL);
      SBI(axis_relative, E_MODE_ABS);
    }
  #endif

  #if ENABLED(CNC_WORKSPACE_PLANES)
    /**
     * Workspace planes only apply to G2/G3 moves
     * (and "canned cycles" - not a current feature)
     */
    enum WorkspacePlane : char { PLANE_XY, PLANE_ZX, PLANE_YZ };
    static WorkspacePlane workspace_plane;
  #endif

  #define MAX_COORDINATE_SYSTEMS 9
  #if ENABLED(CNC_COORDINATE_SYSTEMS)
    static int8_t active_coordinate_system;
    static xyz_pos_t coordinate_system[MAX_COORDINATE_SYSTEMS];
    static bool select_coordinate_system(const int8_t _new);
  #endif

  static millis_t previous_move_ms, max_inactive_time;
  FORCE_INLINE static bool stepper_max_timed_out(const millis_t ms=millis()) {
    return max_inactive_time && ELAPSED(ms, previous_move_ms + max_inactive_time);
  }
  FORCE_INLINE static void reset_stepper_timeout(const millis_t ms=millis()) { previous_move_ms = ms; }

  #if HAS_DISABLE_IDLE_AXES
    static millis_t stepper_inactive_time;
    FORCE_INLINE static bool stepper_inactive_timeout(const millis_t ms=millis()) {
      return ELAPSED(ms, previous_move_ms + stepper_inactive_time);
    }
  #else
    static bool stepper_inactive_timeout(const millis_t) { return false; }
  #endif

  static void report_echo_start(const bool forReplay);
  static void report_heading(const bool forReplay, FSTR_P const fstr, const bool eol=true);
  static void report_heading_etc(const bool forReplay, FSTR_P const fstr, const bool eol=true) {
    report_heading(forReplay, fstr, eol);
    report_echo_start(forReplay);
  }
  static void say_units();

  static int8_t get_target_extruder_from_command();
  static int8_t get_target_e_stepper_from_command(const int8_t dval=-1);
  static void get_destination_from_command();

  static void process_parsed_command(const bool no_ok=false);
  static void process_next_command();

  // Execute G-code in-place, preserving current G-code parameters
  static void process_subcommands_now(FSTR_P fgcode);
  static void process_subcommands_now(char * gcode);

  static void home_all_axes(const bool keep_leveling=false) {
    process_subcommands_now(keep_leveling ? FPSTR(G28_STR) : TERN(CAN_SET_LEVELING_AFTER_G28, F("G28L0"), FPSTR(G28_STR)));
  }

  #if ANY(HAS_AUTO_REPORTING, HOST_KEEPALIVE_FEATURE)
    static bool autoreport_paused;
    static bool set_autoreport_paused(const bool p) {
      const bool was = autoreport_paused;
      autoreport_paused = p;
      return was;
    }
  #else
    static constexpr bool autoreport_paused = false;
    static bool set_autoreport_paused(const bool) { return false; }
  #endif

  #if ENABLED(HOST_KEEPALIVE_FEATURE)
    /**
     * States for managing Marlin and host communication
     * Marlin sends messages if blocked or busy
     */
    enum MarlinBusyState : char {
      NOT_BUSY,           // Not in a handler
      IN_HANDLER,         // Processing a G-Code
      IN_PROCESS,         // Known to be blocking command input (as in G29)
      PAUSED_FOR_USER,    // Blocking pending any input
      PAUSED_FOR_INPUT    // Blocking pending text input (concept)
    };

    static MarlinBusyState busy_state;
    static uint8_t host_keepalive_interval;

    static void host_keepalive();
    static bool host_keepalive_is_paused() { return busy_state >= PAUSED_FOR_USER; }

    #define KEEPALIVE_STATE(N) REMEMBER(_KA_, gcode.busy_state, gcode.N)
  #else
    #define KEEPALIVE_STATE(N) NOOP
  #endif

  static void dwell(millis_t time);

private:

  friend class MarlinSettings;
  #if ENABLED(ARC_SUPPORT)
    friend void plan_arc(const xyze_pos_t&, const ab_float_t&, const bool, const uint8_t);
  #endif

  #if ENABLED(MARLIN_DEV_MODE)
    static void D(const int16_t dcode);
  #endif

  static void G0_G1(TERN_(HAS_FAST_MOVES, const bool fast_move=false));

  #if ENABLED(ARC_SUPPORT)
    static void G2_G3(const bool clockwise);
  #endif

  static void G4();

  #if ENABLED(BEZIER_CURVE_SUPPORT)
    static void G5();
  #endif

  #if ENABLED(DIRECT_STEPPING)
    static void G6();
  #endif

  #if ENABLED(FWRETRACT)
    static void G10();
    static void G11();
  #endif

  #if ENABLED(NOZZLE_CLEAN_FEATURE)
    static void G12();
  #endif

  #if ENABLED(CNC_WORKSPACE_PLANES)
    static void G17();
    static void G18();
    static void G19();
  #endif

  #if ENABLED(INCH_MODE_SUPPORT)
    static void G20();
    static void G21();
  #endif

  #if ENABLED(G26_MESH_VALIDATION)
    static void G26();
  #endif

  #if ENABLED(NOZZLE_PARK_FEATURE)
    static void G27();
  #endif

  static void G28();

  #if HAS_LEVELING
    #if ENABLED(G29_RETRY_AND_RECOVER)
      static void event_probe_failure();
      static void event_probe_recover();
      static void G29_with_retry();
      #define G29_TYPE bool
    #else
      #define G29_TYPE void
    #endif
    static G29_TYPE G29();
  #endif

  #if HAS_BED_PROBE
    static void G30();
    #if ENABLED(Z_PROBE_SLED)
      static void G31();
      static void G32();
    #endif
  #endif

  #if ENABLED(DELTA_AUTO_CALIBRATION)
    static void G33();
  #endif

  #if ANY(Z_MULTI_ENDSTOPS, Z_STEPPER_AUTO_ALIGN, MECHANICAL_GANTRY_CALIBRATION)
    static void G34();
  #endif

  #if ENABLED(Z_STEPPER_AUTO_ALIGN)
    static void M422();
    static void M422_report(const bool forReplay=true);
  #endif

  #if ENABLED(ASSISTED_TRAMMING)
    static void G35();
  #endif

  #if ENABLED(G38_PROBE_TARGET)
    static void G38(const int8_t subcode);
  #endif

  #if HAS_MESH
    static void G42();
  #endif

  #if ENABLED(CNC_COORDINATE_SYSTEMS)
    static void G53();
    static void G54();
    static void G55();
    static void G56();
    static void G57();
    static void G58();
    static void G59();
  #endif

  #if ALL(PTC_PROBE, PTC_BED)
    static void G76();
  #endif

  #if SAVED_POSITIONS
    static void G60();
    static void G61(int8_t slot=-1);
  #endif

  #if ENABLED(GCODE_MOTION_MODES)
    static void G80();
  #endif

  static void G92();

  #if ENABLED(CALIBRATION_GCODE)
    static void G425();
  #endif

  #if HAS_RESUME_CONTINUE
    static void M0_M1();
  #endif

  #if HAS_CUTTER
    static void M3_M4(const bool is_M4);
    static void M5();
  #endif

  #if ENABLED(COOLANT_MIST)
    static void M7();
  #endif

  #if ANY(AIR_ASSIST, COOLANT_FLOOD)
    static void M8();
  #endif

  #if ANY(AIR_ASSIST, COOLANT_CONTROL)
    static void M9();
  #endif

  #if ENABLED(AIR_EVACUATION)
    static void M10();
    static void M11();
  #endif

  #if ENABLED(EXTERNAL_CLOSED_LOOP_CONTROLLER)
    static void M12();
  #endif

  #if ENABLED(EXPECTED_PRINTER_CHECK)
    static void M16();
  #endif

  static void M17();

  static void M18_M84();

  #if HAS_MEDIA
    static void M20();
    static void M21();
    static void M22();
    static void M23();
    static void M24();
    static void M25();
    static void M26();
    static void M27();
    static void M28();
    static void M29();
    static void M30();
  #endif

  static void M31();

  #if HAS_MEDIA
    #if HAS_MEDIA_SUBCALLS
      static void M32();
    #endif
    #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
      static void M33();
    #endif
    #if ALL(SDCARD_SORT_ALPHA, SDSORT_GCODE)
      static void M34();
    #endif
  #endif

  #if ENABLED(DIRECT_PIN_CONTROL)
    static void M42();
  #endif
  #if ENABLED(PINS_DEBUGGING)
    static void M43();
  #endif

  #if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
    static void M48();
  #endif

  #if ENABLED(SET_PROGRESS_MANUALLY)
    static void M73();
  #endif

  static void M75();
  static void M76();
  static void M77();

  #if ENABLED(PRINTCOUNTER)
    static void M78();
  #endif

  #if ENABLED(PSU_CONTROL)
    static void M80();
  #endif
  static void M81();

  #if HAS_EXTRUDERS
    static void M82();
    static void M83();
  #endif

  static void M85();

  #if ENABLED(HOTEND_IDLE_TIMEOUT)
    static void M86();
    static void M86_report(const bool forReplay=true);
    static void M87();
  #endif

  #if ENABLED(EDITABLE_STEPS_PER_UNIT)
    static void M92();
    static void M92_report(const bool forReplay=true, const int8_t e=-1);
  #endif

  #if ENABLED(M100_FREE_MEMORY_WATCHER)
    static void M100();
  #endif

  #if ENABLED(BD_SENSOR)
    static void M102();
  #endif

  #if HAS_HOTEND
    static void M104_M109(const bool isM109);
    FORCE_INLINE static void M104() { M104_M109(false); }
    FORCE_INLINE static void M109() { M104_M109(true); }
  #endif

  static void M105();

  #if HAS_FAN
    static void M106();
    static void M107();
  #endif

  #if DISABLED(EMERGENCY_PARSER)
    static void M108();
    static void M112();
    static void M410();
    #if ENABLED(HOST_PROMPT_SUPPORT)
      static void M876();
    #endif
  #endif

  static void M110();
  static void M111();

  #if ENABLED(HOST_KEEPALIVE_FEATURE)
    static void M113();
  #endif

  static void M114();

  #if ENABLED(CAPABILITIES_REPORT)
    static void M115();
  #endif

  #if HAS_STATUS_MESSAGE
    static void M117();
  #endif

  static void M118();
  static void M119();
  static void M120();
  static void M121();

  #if HAS_FANCHECK
    static void M123();
  #endif

  #if ENABLED(PARK_HEAD_ON_PAUSE)
    static void M125();
  #endif

  #if ENABLED(BARICUDA)
    #if HAS_HEATER_1
      static void M126();
      static void M127();
    #endif
    #if HAS_HEATER_2
      static void M128();
      static void M129();
    #endif
  #endif

  #if HAS_HEATED_BED
    static void M140_M190(const bool isM190);
    FORCE_INLINE static void M140() { M140_M190(false); }
    FORCE_INLINE static void M190() { M140_M190(true); }
  #endif

  #if HAS_HEATED_CHAMBER
    static void M141();
    static void M191();
  #endif

  #if HAS_TEMP_PROBE
    static void M192();
  #endif

  #if HAS_COOLER
    static void M143();
    static void M193();
  #endif

  #if HAS_PREHEAT
    static void M145();
    static void M145_report(const bool forReplay=true);
  #endif

  #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
    static void M149();
    static void M149_report(const bool forReplay=true);
  #endif

  #if HAS_COLOR_LEDS
    static void M150();
  #endif

  #if ENABLED(AUTO_REPORT_POSITION)
    static void M154();
  #endif

  #if ALL(AUTO_REPORT_TEMPERATURES, HAS_TEMP_SENSOR)
    static void M155();
  #endif

  #if ENABLED(MIXING_EXTRUDER)
    static void M163();
    static void M164();
    #if ENABLED(DIRECT_MIXING_IN_G1)
      static void M165();
    #endif
    #if ENABLED(GRADIENT_MIX)
      static void M166();
    #endif
  #endif

  #if DISABLED(NO_VOLUMETRICS)
    static void M200();
    static void M200_report(const bool forReplay=true);
  #endif
  static void M201();
  static void M201_report(const bool forReplay=true);

  #if 0
    static void M202(); // Not used for Sprinter/grbl gen6
  #endif

  static void M203();
  static void M203_report(const bool forReplay=true);
  static void M204();
  static void M204_report(const bool forReplay=true);
  static void M205();
  static void M205_report(const bool forReplay=true);

  #if HAS_HOME_OFFSET
    static void M206();
    static void M206_report(const bool forReplay=true);
  #endif

  #if ENABLED(FWRETRACT)
    static void M207();
    static void M207_report(const bool forReplay=true);
    static void M208();
    static void M208_report(const bool forReplay=true);
    #if ENABLED(FWRETRACT_AUTORETRACT)
      static void M209();
      static void M209_report(const bool forReplay=true);
    #endif
  #endif

  #if ENABLED(EDITABLE_HOMING_FEEDRATE)
    static void M210();
    static void M210_report(const bool forReplay=true);
  #endif

  #if HAS_SOFTWARE_ENDSTOPS
    static void M211();
    static void M211_report(const bool forReplay=true);
  #endif

  #if HAS_MULTI_EXTRUDER
    static void M217();
    static void M217_report(const bool forReplay=true);
  #endif

  #if HAS_HOTEND_OFFSET
    static void M218();
    static void M218_report(const bool forReplay=true);
  #endif

  static void M220();

  #if HAS_EXTRUDERS
    static void M221();
  #endif

  #if ENABLED(DIRECT_PIN_CONTROL)
    static void M226();
  #endif

  #if ENABLED(PHOTO_GCODE)
    static void M240();
  #endif

  #if HAS_LCD_CONTRAST
    static void M250();
    static void M250_report(const bool forReplay=true);
  #endif

  #if ENABLED(EDITABLE_DISPLAY_TIMEOUT)
    static void M255();
    static void M255_report(const bool forReplay=true);
  #endif

  #if HAS_LCD_BRIGHTNESS
    static void M256();
    static void M256_report(const bool forReplay=true);
  #endif

  #if ENABLED(EXPERIMENTAL_I2CBUS)
    static void M260();
    static void M261();
  #endif

  #if HAS_SERVOS
    static void M280();
    #if ENABLED(EDITABLE_SERVO_ANGLES)
      static void M281();
      static void M281_report(const bool forReplay=true);
    #endif
    #if ENABLED(SERVO_DETACH_GCODE)
      static void M282();
    #endif
  #endif

  #if ENABLED(BABYSTEPPING)
    static void M290();
    #if ENABLED(EP_BABYSTEPPING)
      static void M293();
      static void M294();
    #endif
  #endif

  #if HAS_SOUND
    static void M300();
  #endif

  #if ENABLED(PIDTEMP)
    static void M301();
    static void M301_report(const bool forReplay=true E_OPTARG(const int8_t eindex=-1));
  #endif

  #if ENABLED(PREVENT_COLD_EXTRUSION)
    static void M302();
  #endif

  #if HAS_PID_HEATING
    static void M303();
  #endif

  #if ENABLED(PIDTEMPBED)
    static void M304();
    static void M304_report(const bool forReplay=true);
  #endif

  #if HAS_USER_THERMISTORS
    static void M305();
  #endif

  #if ENABLED(MPCTEMP)
    static void M306();
    static void M306_report(const bool forReplay=true);
  #endif

  #if ENABLED(PIDTEMPCHAMBER)
    static void M309();
    static void M309_report(const bool forReplay=true);
  #endif

  #if HAS_MICROSTEPS
    static void M350();
    static void M351();
  #endif

  #if ENABLED(CASE_LIGHT_ENABLE)
    static void M355();
  #endif

  #if ENABLED(REPETIER_GCODE_M360)
    static void M360();
  #endif

  #if ENABLED(MORGAN_SCARA)
    static bool M360();
    static bool M361();
    static bool M362();
    static bool M363();
    static bool M364();
  #endif

  #if ANY(EXT_SOLENOID, MANUAL_SOLENOID_CONTROL)
    static void M380();
    static void M381();
  #endif

  static void M400();

  #if HAS_BED_PROBE
    static void M401();
    static void M402();
  #endif

  #if HAS_PRUSA_MMU2 || HAS_PRUSA_MMU3
    static void M403();
  #endif

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    static void M404();
    static void M405();
    static void M406();
    static void M407();
  #endif

  #if HAS_FILAMENT_SENSOR
    static void M412();
    static void M412_report(const bool forReplay=true);
  #endif

  #if HAS_MULTI_LANGUAGE
    static void M414();
    static void M414_report(const bool forReplay=true);
  #endif

  #if HAS_LEVELING
    static void M420();
    static void M420_report(const bool forReplay=true);
    static void M421();
  #endif

  #if ENABLED(BACKLASH_GCODE)
    static void M425();
    static void M425_report(const bool forReplay=true);
  #endif

  #if HAS_HOME_OFFSET
    static void M428();
  #endif

  #if HAS_POWER_MONITOR
    static void M430();
  #endif

  #if HAS_RS485_SERIAL
    static void M485();
  #endif

  #if ENABLED(CANCEL_OBJECTS)
    static void M486();
  #endif

  #if ENABLED(FT_MOTION)
    static void M493();
    static void M493_report(const bool forReplay=true);
  #endif

  static void M500();
  static void M501();
  static void M502();
  #if DISABLED(DISABLE_M503)
    static void M503();
  #endif
  #if ENABLED(EEPROM_SETTINGS)
    static void M504();
  #endif

  #if ENABLED(PASSWORD_FEATURE)
    static void M510();
    #if ENABLED(PASSWORD_UNLOCK_GCODE)
      static void M511();
    #endif
    #if ENABLED(PASSWORD_CHANGE_GCODE)
      static void M512();
    #endif
  #endif

  #if HAS_MEDIA
    static void M524();
  #endif

  #if ENABLED(SD_ABORT_ON_ENDSTOP_HIT)
    static void M540();
  #endif

  #if HAS_ETHERNET
    static void M552();
    static void M552_report();
    static void M553();
    static void M553_report();
    static void M554();
    static void M554_report();
  #endif

  #if HAS_STEALTHCHOP
    static void M569();
    static void M569_report(const bool forReplay=true);
  #endif

  #if ENABLED(BAUD_RATE_GCODE)
    static void M575();
  #endif

  #if ENABLED(NONLINEAR_EXTRUSION)
    static void M592();
    static void M592_report(const bool forReplay=true);
  #endif

  #if HAS_ZV_SHAPING
    static void M593();
    static void M593_report(const bool forReplay=true);
  #endif

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    static void M600();
    static void M603();
    static void M603_report(const bool forReplay=true);
  #endif

  #if HAS_DUPLICATION_MODE
    static void M605();
  #endif

  #if IS_KINEMATIC
    static void M665();
    static void M665_report(const bool forReplay=true);
  #endif

  #if ANY(DELTA, HAS_EXTRA_ENDSTOPS)
    static void M666();
    static void M666_report(const bool forReplay=true);
  #endif

  #if ENABLED(DUET_SMART_EFFECTOR) && PIN_EXISTS(SMART_EFFECTOR_MOD)
    static void M672();
  #endif

  #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
    static void M701();
    static void M702();
  #endif

  #if HAS_PRUSA_MMU3
    static void M704();
    static void M705();
    static void M706();
    static void M707();
    static void M708();
    static void M709();
    static void MMU3_report(const bool forReplay=true);
  #endif

  #if ENABLED(GCODE_REPEAT_MARKERS)
    static void M808();
  #endif

  #if ENABLED(GCODE_MACROS)
    static void M810_819();
    static void M820();
  #endif

  #if HAS_BED_PROBE
    static void M851();
    static void M851_report(const bool forReplay=true);
  #endif

  #if ENABLED(SKEW_CORRECTION_GCODE)
    static void M852();
    static void M852_report(const bool forReplay=true);
  #endif

  #if ENABLED(I2C_POSITION_ENCODERS)
    FORCE_INLINE static void M860() { I2CPEM.M860(); }
    FORCE_INLINE static void M861() { I2CPEM.M861(); }
    FORCE_INLINE static void M862() { I2CPEM.M862(); }
    FORCE_INLINE static void M863() { I2CPEM.M863(); }
    FORCE_INLINE static void M864() { I2CPEM.M864(); }
    FORCE_INLINE static void M865() { I2CPEM.M865(); }
    FORCE_INLINE static void M866() { I2CPEM.M866(); }
    FORCE_INLINE static void M867() { I2CPEM.M867(); }
    FORCE_INLINE static void M868() { I2CPEM.M868(); }
    FORCE_INLINE static void M869() { I2CPEM.M869(); }
  #endif

  #if HAS_PTC
    static void M871();
  #endif

  #if ENABLED(LIN_ADVANCE)
    static void M900();
    static void M900_report(const bool forReplay=true);
  #endif

  #if HAS_TRINAMIC_CONFIG
    static void M122();
    static void M906();
    static void M906_report(const bool forReplay=true);
    #if ENABLED(MONITOR_DRIVER_STATUS)
      static void M911();
      static void M912();
    #endif
    #if ENABLED(HYBRID_THRESHOLD)
      static void M913();
      static void M913_report(const bool forReplay=true);
    #endif
    #if USE_SENSORLESS
      static void M914();
      static void M914_report(const bool forReplay=true);
    #endif
    static void M919();
  #endif

  #if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM || HAS_MOTOR_CURRENT_I2C || HAS_MOTOR_CURRENT_DAC
    static void M907();
    #if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM
      static void M907_report(const bool forReplay=true);
    #endif
  #endif
  #if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_DAC
    static void M908();
  #endif
  #if HAS_MOTOR_CURRENT_DAC
    static void M909();
    static void M910();
  #endif

  #if HAS_MEDIA
    static void M928();
  #endif

  #if ENABLED(OTA_FIRMWARE_UPDATE)
    static void M936();
  #endif

  #if ENABLED(MAGNETIC_PARKING_EXTRUDER)
    static void M951();
  #endif

  #if ENABLED(TOUCH_SCREEN_CALIBRATION)
    static void M995();
  #endif

  #if SPI_FLASH_BACKUP
    static void M993();
    static void M994();
  #endif

  #if ENABLED(PLATFORM_M997_SUPPORT)
    static void M997();
  #endif

  static void M999();

  #if ENABLED(POWER_LOSS_RECOVERY)
    static void M413();
    static void M413_report(const bool forReplay=true);
    static void M1000();
  #endif

  #if ENABLED(X_AXIS_TWIST_COMPENSATION)
    static void M423();
    static void M423_report(const bool forReplay=true);
  #endif

  #if HAS_MEDIA
    static void M1001();
  #endif

  #if DGUS_LCD_UI_MKS
    static void M1002();
  #endif

  #if ENABLED(ONE_CLICK_PRINT)
    static void M1003();
  #endif

  #if ENABLED(UBL_MESH_WIZARD)
    static void M1004();
  #endif

  #if ENABLED(HAS_MCP3426_ADC)
    static void M3426();
  #endif

  #if ENABLED(MAX7219_GCODE)
    static void M7219();
  #endif

  #if ENABLED(CONTROLLER_FAN_EDITABLE)
    static void M710();
    static void M710_report(const bool forReplay=true);
  #endif

  static void T(const int8_t tool_index) IF_DISABLED(HAS_TOOLCHANGE, { UNUSED(tool_index); });

};

extern GcodeSuite gcode;
