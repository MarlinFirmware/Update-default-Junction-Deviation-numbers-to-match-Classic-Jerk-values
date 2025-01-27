#!/usr/bin/env python3
#
# signature.py
#
import schema, subprocess, re, json, hashlib
from datetime import datetime
from pathlib import Path
from functools import reduce

def enabled_defines(filepath):
    '''
    Return all enabled #define items from a given C header file in a dictionary.
    A "#define" in a multi-line comment could produce a false positive if it's not
    preceded by a non-space character (like * in a multi-line comment).

    Output:
    Each entry is a dictionary with a 'name' and a 'section' key. We end up with:
        { MOTHERBOARD: { name: "MOTHERBOARD", section: "hardware" }, ... }

    TODO: Drop the 'name' key as redundant. For now it's useful for debugging.

    This list is only used to filter config-defined options from those defined elsewhere.

    Because the option names are the keys, only the last occurrence is retained.
    This means the actual used value might not be reflected by this function.
    The Schema class does more complete parsing for a more accurate list of options.

    While the Schema class parses the configurations on its own, this script will
    get the preprocessor output and get the intersection of the enabled options from
    our crude scraping method and the actual compiler output.
    We end up with the actual configured state,
    better than what the config files say. You can then use the
    resulting config.ini to produce more exact configuration files.
    '''
    outdict = {}
    section = "user"
    spatt = re.compile(r".*@section +([-a-zA-Z0-9_\s]+)$") # @section ...

    f = open(filepath, encoding="utf8").read().split("\n")

    incomment = False
    for line in f:
        sline = line.strip()

        m = re.match(spatt, sline) # @section ...
        if m: section = m.group(1).strip() ; continue

        if incomment:
            if '*/' in sline:
                incomment = False
            continue
        else:
            mpos, spos = sline.find('/*'), sline.find('//')
            if mpos >= 0 and (spos < 0 or spos > mpos):
                incomment = True
                continue

        if sline[:7] == "#define":
            # Extract the key here (we don't care about the value)
            kv = sline[8:].strip().split()
            outdict[kv[0]] = { 'name':kv[0], 'section': section }
    return outdict

# Compute the SHA256 hash of a file
def get_file_sha256sum(filepath):
    sha256_hash = hashlib.sha256()
    with open(filepath,"rb") as f:
        # Read and update hash string value in blocks of 4K
        for byte_block in iter(lambda: f.read(4096),b""):
            sha256_hash.update(byte_block)
    return sha256_hash.hexdigest()

#
# Compress a JSON file into a zip file
#
import zipfile
def compress_file(filepath, storedname, outpath):
    with zipfile.ZipFile(outpath, 'w', compression=zipfile.ZIP_DEFLATED, allowZip64=False, compresslevel=9) as zipf:
        zipf.write(filepath, arcname=storedname)

ignore = ('CONFIGURATION_H_VERSION', 'CONFIGURATION_ADV_H_VERSION', 'CONFIG_EXAMPLES_DIR', 'CONFIG_EXPORT')

#
# Compute a build signature and/or export the configuration
#
def compute_build_signature(env):
    '''
    Compute the build signature by extracting all configuration settings and
    building a unique reversible signature that can be included in the binary.
    The signature can be reversed to get a 1:1 equivalent configuration file.
    Used by common-dependencies.py after filtering build files by feature.
    '''
    if 'BUILD_SIGNATURE' in env: return
    env.Append(BUILD_SIGNATURE=1)

    build_path = Path(env['PROJECT_BUILD_DIR'], env['PIOENV'])
    json_name = 'marlin_config.json'
    marlin_json = build_path / json_name
    marlin_zip = build_path / 'mc.zip'

    # ANSI colors
    green = "\u001b[32m"
    yellow = "\u001b[33m"
    red = "\u001b[31m"

    # Definitions from these files will be kept
    header_paths = ('Marlin/Configuration.h', 'Marlin/Configuration_adv.h')

    # Check if we can skip processing
    hashes = ''
    for header in header_paths:
        hashes += get_file_sha256sum(header)[0:10]

    # Read a previously exported JSON file
    # Same configuration, skip recomputing the build signature
    same_hash = False
    try:
        with marlin_json.open() as infile:
            conf = json.load(infile)
            same_hash = conf['__INITIAL_HASH'] == hashes
            if same_hash:
                compress_file(marlin_json, json_name, marlin_zip)
    except:
        pass

    # Extract "enabled" #define lines by scraping the configuration files.
    # This data also contains the @section for each option.
    conf_defines = {}
    conf_names = []
    for hpath in header_paths:
        # Get defines in the form of { name: { name:..., section:... }, ... }
        defines = enabled_defines(hpath)
        # Get all unique define names into a flat array
        conf_names += defines.keys()
        # Remember which file these defines came from
        conf_defines[hpath.split('/')[-1]] = defines

    # Get enabled config options based on running GCC to preprocess the config files.
    # The result is a list of line strings, each starting with '#define'.
    from preprocessor import run_preprocessor
    build_output = run_preprocessor(env)

    # Dumb regex to filter out some dumb macros
    r = re.compile(r"\(+(\s*-*\s*_.*)\)+")

    # Extract all the #define lines in the build output as key/value pairs
    build_defines = {}
    for line in build_output:
        # Split the define from the value.
        key_val = line[8:].strip().decode().split(' ')
        key, value = key_val[0], ' '.join(key_val[1:])
        # Ignore values starting with two underscore, since it's low level
        if len(key) > 2 and key[0:2] == "__": continue
        # Ignore values containing parentheses (likely a function macro)
        if '(' in key and ')' in key: continue
        # Then filter dumb values
        if r.match(value): continue

        build_defines[key] = value if len(value) else ""

    #
    # Continue to gather data for CONFIGURATION_EMBEDDING or CONFIG_EXPORT
    #
    is_embed = 'CONFIGURATION_EMBEDDING' in build_defines
    if not (is_embed or 'CONFIG_EXPORT' in build_defines):
        return

    # Filter out useless macros from the output
    cleaned_build_defines = {}
    for key in build_defines:
        # Remove all boards now
        if key.startswith("BOARD_") and key != "BOARD_INFO_NAME": continue
        # Remove all keys ending by "_T_DECLARED" as it's a copy of extraneous system stuff
        if key.endswith("_T_DECLARED"): continue
        # Remove keys that are not in the #define list in the Configuration list
        if key not in conf_names + [ 'DETAILED_BUILD_VERSION', 'STRING_DISTRIBUTION_DATE' ]: continue
        # Add to a new dictionary for simplicity
        cleaned_build_defines[key] = build_defines[key]

    # And we only care about defines that (most likely) came from the config files
    # Build a dictionary of dictionaries with keys: 'name', 'section', 'value'
    # { 'file1': { 'option': { 'name':'option', 'section':..., 'value':... }, ... }, 'file2': { ... } }
    real_config = {}
    for header in conf_defines:
        real_config[header] = {}
        for key in cleaned_build_defines:
            if key in conf_defines[header]:
                if key[0:2] == '__': continue
                val = cleaned_build_defines[key]
                real_config[header][key] = { 'file':header, 'name': key, 'value': val, 'section': conf_defines[header][key]['section']}

    def tryint(key):
        try: return int(build_defines[key])
        except: return 0

    # Get the CONFIG_EXPORT value and do an extended dump if > 100
    # For example, CONFIG_EXPORT 102 will make a 'config.ini' with a [config:] group for each schema @section
    config_dump = tryint('CONFIG_EXPORT')
    extended_dump = config_dump > 100
    if extended_dump: config_dump -= 100

    # Get the schema class for exports that require it
    if config_dump in (3, 4) or (extended_dump and config_dump in (2, 5)):
        try:
            conf_schema = schema.extract()
        except Exception as exc:
            print(red + "Error: " + str(exc))
            conf_schema = None

    optorder = ('MOTHERBOARD','SERIAL_PORT','BAUDRATE','USE_WATCHDOG','THERMAL_PROTECTION_HOTENDS','THERMAL_PROTECTION_HYSTERESIS','THERMAL_PROTECTION_PERIOD','BUFSIZE','BLOCK_BUFFER_SIZE','MAX_CMD_SIZE','EXTRUDERS','TEMP_SENSOR_0','TEMP_HYSTERESIS','HEATER_0_MINTEMP','HEATER_0_MAXTEMP','PREHEAT_1_TEMP_HOTEND','BANG_MAX','PIDTEMP','PID_K1','PID_MAX','PID_FUNCTIONAL_RANGE','DEFAULT_KP','DEFAULT_KI','DEFAULT_KD','X_DRIVER_TYPE','Y_DRIVER_TYPE','Z_DRIVER_TYPE','E0_DRIVER_TYPE','X_BED_SIZE','X_MIN_POS','X_MAX_POS','Y_BED_SIZE','Y_MIN_POS','Y_MAX_POS','Z_MIN_POS','Z_MAX_POS','X_HOME_DIR','Y_HOME_DIR','Z_HOME_DIR','X_MIN_ENDSTOP_HIT_STATE','Y_MIN_ENDSTOP_HIT_STATE','Z_MIN_ENDSTOP_HIT_STATE','DEFAULT_AXIS_STEPS_PER_UNIT','AXIS_RELATIVE_MODES','DEFAULT_MAX_FEEDRATE','DEFAULT_MAX_ACCELERATION','HOMING_FEEDRATE_MM_M','HOMING_BUMP_DIVISOR','X_ENABLE_ON','Y_ENABLE_ON','Z_ENABLE_ON','E_ENABLE_ON','INVERT_X_DIR','INVERT_Y_DIR','INVERT_Z_DIR','INVERT_E0_DIR','STEP_STATE_E','STEP_STATE_X','STEP_STATE_Y','STEP_STATE_Z','DISABLE_X','DISABLE_Y','DISABLE_Z','DISABLE_E','PROPORTIONAL_FONT_RATIO','DEFAULT_NOMINAL_FILAMENT_DIA','JUNCTION_DEVIATION_MM','DEFAULT_ACCELERATION','DEFAULT_TRAVEL_ACCELERATION','DEFAULT_RETRACT_ACCELERATION','DEFAULT_MINIMUMFEEDRATE','DEFAULT_MINTRAVELFEEDRATE','MINIMUM_PLANNER_SPEED','MIN_STEPS_PER_SEGMENT','DEFAULT_MINSEGMENTTIME','BED_OVERSHOOT','BUSY_WHILE_HEATING','DEFAULT_EJERK','DEFAULT_KEEPALIVE_INTERVAL','DEFAULT_LEVELING_FADE_HEIGHT','DISABLE_OTHER_EXTRUDERS','DISPLAY_CHARSET_HD44780','EEPROM_BOOT_SILENT','EEPROM_CHITCHAT','ENDSTOPPULLUPS','EXTRUDE_MAXLENGTH','EXTRUDE_MINTEMP','HOST_KEEPALIVE_FEATURE','HOTEND_OVERSHOOT','JD_HANDLE_SMALL_SEGMENTS','LCD_INFO_SCREEN_STYLE','LCD_LANGUAGE','MAX_BED_POWER','MESH_INSET','MIN_SOFTWARE_ENDSTOPS','MAX_SOFTWARE_ENDSTOPS','MIN_SOFTWARE_ENDSTOP_X','MIN_SOFTWARE_ENDSTOP_Y','MIN_SOFTWARE_ENDSTOP_Z','MAX_SOFTWARE_ENDSTOP_X','MAX_SOFTWARE_ENDSTOP_Y','MAX_SOFTWARE_ENDSTOP_Z','PREHEAT_1_FAN_SPEED','PREHEAT_1_LABEL','PREHEAT_1_TEMP_BED','PREVENT_COLD_EXTRUSION','PREVENT_LENGTHY_EXTRUDE','PRINTJOB_TIMER_AUTOSTART','PROBING_MARGIN','SHOW_BOOTSCREEN','SOFT_PWM_SCALE','STRING_CONFIG_H_AUTHOR','TEMP_BED_HYSTERESIS','TEMP_BED_RESIDENCY_TIME','TEMP_BED_WINDOW','TEMP_RESIDENCY_TIME','TEMP_WINDOW','VALIDATE_HOMING_ENDSTOPS','XY_PROBE_FEEDRATE','Z_CLEARANCE_BETWEEN_PROBES','Z_CLEARANCE_DEPLOY_PROBE','Z_CLEARANCE_MULTI_PROBE','ARC_SUPPORT','AUTO_REPORT_TEMPERATURES','AUTOTEMP','AUTOTEMP_OLDWEIGHT','BED_CHECK_INTERVAL','DEFAULT_STEPPER_TIMEOUT_SEC','DEFAULT_VOLUMETRIC_EXTRUDER_LIMIT','DISABLE_IDLE_X','DISABLE_IDLE_Y','DISABLE_IDLE_Z','DISABLE_IDLE_E','E0_AUTO_FAN_PIN','ENCODER_100X_STEPS_PER_SEC','ENCODER_10X_STEPS_PER_SEC','ENCODER_RATE_MULTIPLIER','EXTENDED_CAPABILITIES_REPORT','EXTRUDER_AUTO_FAN_SPEED','EXTRUDER_AUTO_FAN_TEMPERATURE','FANMUX0_PIN','FANMUX1_PIN','FANMUX2_PIN','FASTER_GCODE_PARSER','HOMING_BUMP_MM','MAX_ARC_SEGMENT_MM','MIN_ARC_SEGMENT_MM','MIN_CIRCLE_SEGMENTS','N_ARC_CORRECTION','SERIAL_OVERRUN_PROTECTION','SLOWDOWN','SLOWDOWN_DIVISOR','TEMP_SENSOR_BED','THERMAL_PROTECTION_BED_HYSTERESIS','THERMOCOUPLE_MAX_ERRORS','TX_BUFFER_SIZE','WATCH_BED_TEMP_INCREASE','WATCH_BED_TEMP_PERIOD','WATCH_TEMP_INCREASE','WATCH_TEMP_PERIOD')

    def optsort(x, optorder):
        return optorder.index(x) if x in optorder else float('inf')

    #
    # CONFIG_EXPORT 102 = config.ini, 105 = Config.h
    # Get sections using the schema class
    #
    if extended_dump and config_dump in (2, 5):
        if not conf_schema: exit(1)

        # Start with a preferred @section ordering
        preorder = ('test','custom','info','machine','eeprom','stepper drivers','multi stepper','idex','extruder','geometry','homing','kinematics','motion','motion control','endstops','filament runout sensors','probe type','probes','bltouch','leveling','temperature','hotend temp','mpctemp','pid temp','mpc temp','bed temp','chamber temp','fans','tool change','advanced pause','calibrate','calibration','media','lcd','lights','caselight','interface','custom main menu','custom config menu','custom buttons','develop','debug matrix','delta','scara','tpara','polar','polargraph','cnc','nozzle park','nozzle clean','gcode','serial','host','filament width','i2c encoders','i2cbus','joystick','multi-material','nanodlp','network','photo','power','psu control','reporting','safety','security','servos','stats','tmc/config','tmc/hybrid','tmc/serial','tmc/smart','tmc/spi','tmc/stallguard','tmc/status','tmc/stealthchop','tmc/tmc26x','units','volumetrics','extras')

        sections = { key:{} for key in preorder }

        # Group options by schema @section
        for header in real_config:
            for name in real_config[header]:
                #print(f"  name: {name}")
                if name in ignore: continue
                ddict = real_config[header][name]
                #print(f"   real_config[{header}][{name}]:", ddict)
                sect = ddict['section']
                if sect not in sections: sections[sect] = {}
                sections[sect][name] = ddict

    #
    # CONFIG_EXPORT 2 or 102 = config.ini
    #
    if config_dump == 2:
        print(yellow + "Generating config.ini ...")

        ini_fmt = '{0:40} = {1}'
        ext_fmt = '{0:40}   {1}'

        if extended_dump:
            # Extended export will dump config options by section

            # We'll use Schema class to get the sections
            if not conf_schema: exit(1)

            # Then group options by schema @section
            sections = {}
            for header in real_config:
                for name in real_config[header]:
                    #print(f"  name: {name}")
                    if name not in ignore:
                        ddict = real_config[header][name]
                        #print(f"   real_config[{header}][{name}]:", ddict)
                        sect = ddict['section']
                        if sect not in sections: sections[sect] = {}
                        sections[sect][name] = ddict

            # Get all sections as a list of strings, with spaces and dashes replaced by underscores
            long_list = [ re.sub(r'[- ]+', '_', x).lower() for x in sections.keys() ]
            # Make comma-separated lists of sections with 64 characters or less
            sec_lines = []
            while len(long_list):
                line = long_list.pop(0) + ', '
                while len(long_list) and len(line) + len(long_list[0]) < 64 - 1:
                    line += long_list.pop(0) + ', '
                sec_lines.append(line.strip())
            sec_lines[-1] = sec_lines[-1][:-1] # Remove the last comma

        else:
            sec_lines = ['all']

        # Build the ini_use_config item
        sec_list = ini_fmt.format('ini_use_config', sec_lines[0])
        for line in sec_lines[1:]: sec_list += '\n' + ext_fmt.format('', line)

        config_ini = build_path / 'config.ini'
        with config_ini.open('w') as outfile:
            filegrp = { 'Configuration.h':'config:basic', 'Configuration_adv.h':'config:advanced' }
            vers = build_defines["CONFIGURATION_H_VERSION"]
            dt_string = datetime.now().strftime("%Y-%m-%d at %H:%M:%S")

            outfile.write(
f'''#
# Marlin Firmware
# config.ini - Options to apply before the build
#
# Generated by Marlin build on {dt_string}
#
[config:base]
#
# ini_use_config - A comma-separated list of actions to apply to the Configuration files.
#                  The actions will be applied in the listed order.
#  - none
#    Ignore this file and don't apply any configuration options
#
#  - base
#    Just apply the options in config:base to the configuration
#
#  - minimal
#    Just apply the options in config:minimal to the configuration
#
#  - all
#    Apply all 'config:*' sections in this file to the configuration
#
#  - another.ini
#    Load another INI file with a path relative to this config.ini file (i.e., within Marlin/)
#
#  - https://me.myserver.com/path/to/configs
#    Fetch configurations from any URL.
#
#  - example/Creality/Ender-5 Plus @ bugfix-2.1.x
#    Fetch example configuration files from the MarlinFirmware/Configurations repository
#    https://raw.githubusercontent.com/MarlinFirmware/Configurations/bugfix-2.1.x/config/examples/Creality/Ender-5%20Plus/
#
#  - example/default @ release-2.0.9.7
#    Fetch default configuration files from the MarlinFirmware/Configurations repository
#    https://raw.githubusercontent.com/MarlinFirmware/Configurations/release-2.0.9.7/config/default/
#
#  - [disable]
#    Comment out all #defines in both Configuration.h and Configuration_adv.h. This is useful
#    to start with a clean slate before applying any config: options, so only the options explicitly
#    set in config.ini will be enabled in the configuration.
#
#  - [flatten] (Not yet implemented)
#    Produce a flattened set of Configuration.h and Configuration_adv.h files with only the enabled
#    #defines and no comments. A clean look, but context-free.
#
{sec_list}
{ini_fmt.format('ini_config_vers', vers)}
'''         )

            if extended_dump:

                # Loop through the sections
                for skey in sorted(sections):
                    #print(f"  skey: {skey}")
                    sani = re.sub(r'[- ]+', '_', skey).lower()
                    outfile.write(f"\n[config:{sani}]\n")
                    opts = sections[skey]
                    opts_keys = sorted(opts.keys(), key=lambda x: optsort(x, optorder))
                    for name in opts_keys:
                        if name in ignore: continue
                        val = opts[name]['value']
                        if val == '': val = 'on'
                        #print(f"  {name} = {val}")
                        outfile.write(ini_fmt.format(name.lower(), val) + '\n')

            else:

                # Standard export just dumps config:basic and config:advanced sections
                for header in real_config:
                    outfile.write(f'\n[{filegrp[header]}]\n')
                    opts = real_config[header]
                    opts_keys = sorted(opts.keys(), key=lambda x: optsort(x, optorder))
                    for name in opts_keys:
                        if name in ignore: continue
                        val = opts[name]['value']
                        if val == '': val = 'on'
                        outfile.write(ini_fmt.format(name.lower(), val) + '\n')

    #
    # CONFIG_EXPORT 5 or 105 = Config.h
    #
    if config_dump == 5:
        print(yellow + "Generating Config-export.h ...")

        config_h = Path('Marlin', 'Config-export.h')
        with config_h.open('w') as outfile:
            filegrp = { 'Configuration.h':'config:basic', 'Configuration_adv.h':'config:advanced' }
            vers = build_defines["CONFIGURATION_H_VERSION"]
            dt_string = datetime.utcnow().strftime("%Y-%m-%d at %H:%M:%S")

            out_text = f'''/**
 * Config.h - Marlin Firmware distilled configuration
 * Usage: Place this file in the 'Marlin' folder with the name 'Config.h'.
 *
 * Exported by Marlin build on {dt_string}.
 */
'''

            subs = (('Bltouch','BLTouch'),('hchop','hChop'),('Eeprom','EEPROM'),('Gcode','G-code'),('lguard','lGuard'),('Idex','IDEX'),('Lcd','LCD'),('Mpc','MPC'),('Pid','PID'),('Psu','PSU'),('Scara','SCARA'),('Spi','SPI'),('Tmc','TMC'),('Tpara','TPARA'))
            define_fmt = '#define {0:40} {1}'
            if extended_dump:
                # Loop through the sections
                for skey in sections:
                    #print(f"  skey: {skey}")
                    opts = sections[skey]
                    headed = False
                    opts_keys = sorted(opts.keys(), key=lambda x: optsort(x, optorder))
                    for name in opts_keys:
                        if name in ignore: continue
                        val = opts[name]['value']
                        if not headed:
                            head = reduce(lambda s, r: s.replace(*r), subs, skey.title())
                            out_text += f"\n//\n// {head}\n//\n"
                            headed = True
                        out_text += define_fmt.format(name, val).strip() + '\n'

            else:
                # Dump config options in just two sections, by file
                for header in real_config:
                    out_text += f'\n/**\n * Overrides for {header}\n */\n'
                    opts = real_config[header]
                    opts_keys = sorted(opts.keys(), key=lambda x: optsort(x, optorder))
                    for name in opts_keys:
                        if name in ignore: continue
                        val = opts[name]['value']
                        out_text += define_fmt.format(name, val).strip() + '\n'

            outfile.write(out_text)

    #
    # CONFIG_EXPORT 3 = schema.json, 13 = schema_grouped.json, 4 = schema.yml
    #
    if config_dump in (3, 4, 13):

        if conf_schema:
            #
            # 3 = schema.json
            #
            if config_dump in (3, 13):
                print(yellow + "Generating schema.json ...")
                schema.dump_json(conf_schema, build_path / 'schema.json')
                if config_dump == 13:
                    schema.group_options(conf_schema)
                    schema.dump_json(conf_schema, build_path / 'schema_grouped.json')

            #
            # 4 = schema.yml
            #
            elif config_dump == 4:
                print(yellow + "Generating schema.yml ...")
                try:
                    import yaml
                except ImportError:
                    env.Execute(env.VerboseAction(
                        '$PYTHONEXE -m pip install "pyyaml"',
                        "Installing YAML for schema.yml export",
                    ))
                    import yaml
                schema.dump_yaml(conf_schema, build_path / 'schema.yml')

    #
    # Produce a JSON file for CONFIGURATION_EMBEDDING or CONFIG_EXPORT == 1 or 101
    # Skip if an identical JSON file was already present.
    #
    if not same_hash and (config_dump == 1 or is_embed):
        with marlin_json.open('w') as outfile:

            json_data = {}
            if extended_dump:
                print(yellow + "Extended dump ...")
                for header in real_config:
                    confs = real_config[header]
                    json_data[header] = {}
                    for name in confs:
                        if name in ignore: continue
                        c = confs[name]
                        s = c['section']
                        if s not in json_data[header]: json_data[header][s] = {}
                        json_data[header][s][name] = c['value']
            else:
                for header in real_config:
                    json_data[header] = {}
                    conf = real_config[header]
                    #print(f"real_config[{header}]", conf)
                    for name in conf:
                        if name in ignore: continue
                        json_data[header][name] = conf[name]['value']

            json_data['__INITIAL_HASH'] = hashes

            # Append the source code version and date
            json_data['VERSION'] = {
                'DETAILED_BUILD_VERSION': cleaned_build_defines['DETAILED_BUILD_VERSION'],
                'STRING_DISTRIBUTION_DATE': cleaned_build_defines['STRING_DISTRIBUTION_DATE']
            }
            try:
                curver = subprocess.check_output(["git", "describe", "--match=NeVeRmAtCh", "--always"]).strip()
                json_data['VERSION']['GIT_REF'] = curver.decode()
            except:
                pass

            json.dump(json_data, outfile, separators=(',', ':'))

    #
    # The rest only applies to CONFIGURATION_EMBEDDING
    #
    if not is_embed:
        (build_path / 'mc.zip').unlink(missing_ok=True)
        return

    # Compress the JSON file as much as we can
    if not same_hash:
        compress_file(marlin_json, json_name, marlin_zip)

    # Generate a C source file containing the entire ZIP file as an array
    with open('Marlin/src/mczip.h','wb') as result_file:
        result_file.write(
              b'#ifndef NO_CONFIGURATION_EMBEDDING_WARNING\n'
            + b'  #warning "Generated file \'mc.zip\' is embedded (Define NO_CONFIGURATION_EMBEDDING_WARNING to suppress this warning.)"\n'
            + b'#endif\n'
            + b'const unsigned char mc_zip[] PROGMEM = {\n '
        )
        count = 0
        for b in (build_path / 'mc.zip').open('rb').read():
            result_file.write(b' 0x%02X,' % b)
            count += 1
            if count % 16 == 0: result_file.write(b'\n ')
        if count % 16: result_file.write(b'\n')
        result_file.write(b'};\n')

if __name__ == "__main__":
    # Build required. From command line just explain usage.
    print("*** THIS SCRIPT USED BY common-dependencies.py ***\n\n"
        + "Current options for config and schema export:\n"
        + " - marlin_config.json  : Build Marlin with CONFIG_EXPORT 1 or 101. (Use CONFIGURATION_EMBEDDING for 'mc.zip')\n"
        + " - config.ini          : Build Marlin with CONFIG_EXPORT 2 or 102.\n"
        + " - schema.json         : Run 'schema.py json' (CONFIG_EXPORT 3).\n"
        + " - schema_grouped.json : Run 'schema.py group' (CONFIG_EXPORT 13).\n"
        + " - schema.yml          : Run 'schema.py yml' (CONFIG_EXPORT 4).\n"
        + " - Config-export.h     : Build Marlin with CONFIG_EXPORT 5 or 105.\n"
    )
