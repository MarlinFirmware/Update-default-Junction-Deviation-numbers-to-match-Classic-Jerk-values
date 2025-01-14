#!/usr/bin/env python3
#
# configuration.py
# Apply options from config.ini to the existing Configuration headers
#
import re, shutil, configparser, datetime
from pathlib import Path

verbose = 0
def blab(str,level=1):
    if verbose >= level: print(f"[config] {str}")

def config_path(cpath):
    return Path("Marlin", cpath)

# Apply a single name = on/off ; name = value ; etc.
# TODO: Limit to the given (optional) configuration
def apply_opt(name, val, conf=None):
    if name == "lcd": name, val = val, "on"

    # Create a regex to match the option and capture parts of the line
    # 1: Indentation
    # 2: Comment
    # 3: #define and whitespace
    # 4: Option name
    # 5: First space after name
    # 6: Remaining spaces between name and value
    # 7: Option value
    # 8: Whitespace after value
    # 9: End comment
    regex = re.compile(rf'^(\s*)(//\s*)?(#define\s+)({name}\b)(\s?)(\s*)(.*?)(\s*)(//.*)?$', re.IGNORECASE)

    # Find and enable and/or update all matches
    for file in ("Configuration.h", "Configuration_adv.h"):
        fullpath = config_path(file)
        lines = fullpath.read_text(encoding='utf-8').split('\n')
        found = False
        for i in range(len(lines)):
            line = lines[i]
            match = regex.match(line)
            if match and match[4].upper() == name.upper():
                found = True
                # For boolean options un/comment the define
                if val in ("on", "", None):
                    newline = re.sub(r'^(\s*)//+\s*(#define)(\s{1,3})?(\s*)', r'\1\2 \4', line)
                elif val == "off":
                    # TODO: Comment more lines in a multi-line define with \ continuation
                    newline = re.sub(r'^(\s*)(#define)(\s{1,3})?(\s*)', r'\1//\2 \4', line)
                else:
                    # For options with values, enable and set the value
                    addsp = '' if match[5] else ' '
                    newline = match[1] + match[3] + match[4] + match[5] + addsp + val + match[6]
                    if match[9]:
                        sp = match[8] if match[8] else ' '
                        newline += sp + match[9]
                lines[i] = newline
                blab(f"Set {name} to {val}")

        # If the option was found, write the modified lines
        if found:
            fullpath.write_text('\n'.join(lines), encoding='utf-8')
            break

    # If the option didn't appear in either config file, add it
    if not found:
        # OFF options are added as disabled items so they appear
        # in config dumps. Useful for custom settings.
        prefix = ""
        if val == "off":
            prefix, val = "//", ""  # Item doesn't appear in config dump
            #val = "false"          # Item appears in config dump

        # Uppercase the option unless already mixed/uppercase
        added = name.upper() if name.islower() else name

        # Add the provided value after the name
        if val != "on" and val != "" and val is not None:
            added += " " + val

        # Prepend the new option after the first set of #define lines
        fullpath = config_path("Configuration.h")
        with fullpath.open(encoding='utf-8') as f:
            lines = f.readlines()
            linenum = 0
            gotdef = False
            for line in lines:
                isdef = line.startswith("#define")
                if not gotdef:
                    gotdef = isdef
                elif not isdef:
                    break
                linenum += 1
            currtime = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            lines.insert(linenum, f"{prefix}#define {added:30} // Added by config.ini {currtime}\n")
            fullpath.write_text(''.join(lines), encoding='utf-8')

# Disable all (most) defined options in the configuration files.
# Everything in the named sections. Section hint for exceptions may be added.
def disable_all_options():
    # Create a regex to match the option and capture parts of the line
    regex = re.compile(r'^(\s*)(#define\s+)([A-Z0-9_]+\b)(\s?)(\s*)(.*?)(\s*)(//.*)?$', re.IGNORECASE)

    # Disable all enabled options in both Config files
    for file in ("Configuration.h", "Configuration_adv.h"):
        fullpath = config_path(file)
        lines = fullpath.read_text(encoding='utf-8').split('\n')
        found = False
        for i in range(len(lines)):
            line = lines[i]
            match = regex.match(line)
            if match:
                name = match[3].upper()
                if name in ('CONFIGURATION_H_VERSION', 'CONFIGURATION_ADV_H_VERSION', 'CONFIG_EXAMPLES_DIR'): continue
                if name.startswith('_'): continue
                found = True
                # Comment out the define
                # TODO: Comment more lines in a multi-line define with \ continuation
                lines[i] = re.sub(r'^(\s*)(#define)(\s{1,3})?(\s*)', r'\1//\2 \4', line)
                blab(f"Disable {name}")

        # If the option was found, write the modified lines
        if found:
            fullpath.write_text('\n'.join(lines), encoding='utf-8')

# Fetch configuration files from GitHub given the path.
# Return True if any files were fetched.
def fetch_example(url):
    if url.endswith("/"): url = url[:-1]
    if not url.startswith('http'):
        brch = "bugfix-2.1.x"
        if '@' in url: url, brch = map(str.strip, url.split('@'))
        if url == 'examples/default': url = 'default'
        url = f"https://raw.githubusercontent.com/MarlinFirmware/Configurations/{brch}/config/{url}"
    url = url.replace("%", "%25").replace(" ", "%20")

    # Find a suitable fetch command
    if shutil.which("curl") is not None:
        fetch = "curl -L -s -S -f -o"
    elif shutil.which("wget") is not None:
        fetch = "wget -q -O"
    else:
        blab("Couldn't find curl or wget", -1)
        return False

    import os

    # Reset configurations to default
    os.system("git checkout HEAD Marlin/*.h")

    # Try to fetch the remote files
    gotfile = False
    for fn in ("Configuration.h", "Configuration_adv.h", "_Bootscreen.h", "_Statusscreen.h"):
        if os.system(f"{fetch} wgot {url}/{fn} >/dev/null 2>&1") == 0:
            shutil.move('wgot', config_path(fn))
            gotfile = True

    if Path('wgot').exists(): shutil.rmtree('wgot')

    return gotfile

def section_items(cp, sectkey):
    return cp.items(sectkey) if sectkey in cp.sections() else []

# Apply all items from a config section. Ignore ini_ items outside of config:base and config:root.
def apply_ini_by_name(cp, sect):
    iniok = True
    if sect in ('config:base', 'config:root'):
        iniok = False
        items = section_items(cp, 'config:base') + section_items(cp, 'config:root')
    else:
        items = section_items(cp, sect)

    for item in items:
        if iniok or not item[0].startswith('ini_'):
            apply_opt(item[0], item[1])

# Apply all config sections from a parsed file
def apply_all_sections(cp):
    for sect in cp.sections():
        if sect.startswith('config:'):
            apply_ini_by_name(cp, sect)

# Apply certain config sections from a parsed file
def apply_sections(cp, ckey='all'):
    blab(f"Apply section key: {ckey}")
    if ckey == 'all':
        apply_all_sections(cp)
    else:
        # Apply the base/root config.ini settings after external files are done
        if ckey in ('base', 'root'):
            apply_ini_by_name(cp, 'config:base')

        # Apply historically 'Configuration.h' settings everywhere
        if ckey == 'basic':
            apply_ini_by_name(cp, 'config:basic')

        # Apply historically Configuration_adv.h settings everywhere
        # (Some of which rely on defines in 'Conditionals-2-LCD.h')
        elif ckey in ('adv', 'advanced'):
            apply_ini_by_name(cp, 'config:advanced')

        # Apply a specific config:<name> section directly
        elif ckey.startswith('config:'):
            apply_ini_by_name(cp, ckey)

# Apply settings from a top level config.ini
def apply_config_ini(cp):
    blab("=" * 20 + " Gather 'config.ini' entries...")

    # Pre-scan for ini_use_config to get config_keys
    base_items = section_items(cp, 'config:base') + section_items(cp, 'config:root')
    config_keys = ['base']
    for ikey, ival in base_items:
        if ikey == 'ini_use_config':
            config_keys = map(str.strip, ival.split(','))

    # For each ini_use_config item perform an action
    for ckey in config_keys:
        addbase = False

        # For a key ending in .ini load and parse another .ini file
        if ckey.endswith('.ini'):
            sect = 'base'
            if '@' in ckey: sect, ckey = map(str.strip, ckey.split('@'))
            cp2 = configparser.ConfigParser()
            cp2.read(config_path(ckey), encoding='utf-8')
            apply_sections(cp2, sect)
            ckey = 'base'

        # (Allow 'example/' as a shortcut for 'examples/')
        elif ckey.startswith('example/'):
            ckey = 'examples' + ckey[7:]

        # For 'examples/<path>' fetch an example set from GitHub.
        # For https?:// do a direct fetch of the URL.
        if ckey.startswith('examples/') or ckey.startswith('http'):
            fetch_example(ckey)
            ckey = 'base'

        #
        # [flatten] Write out Configuration.h and Configuration_adv.h files with
        #           just the enabled options and all other content removed.
        #
        #if ckey == '[flatten]':
        #   write_flat_configs()

        if ckey == '[disable]':
            disable_all_options()

        elif ckey == 'all':
            apply_sections(cp)

        else:
            # Apply keyed sections after external files are done
            apply_sections(cp, 'config:' + ckey)

if __name__ == "__main__":
    #
    # From command line use the given file name
    #
    import sys
    args = sys.argv[1:]
    if len(args) > 0:
        if args[0].endswith('.ini'):
            ini_file = args[0]
        else:
            print("Usage: %s <.ini file>" % os.path.basename(sys.argv[0]))
    else:
        ini_file = config_path('config.ini')

    if ini_file:
        user_ini = configparser.ConfigParser()
        user_ini.read(ini_file, encoding='utf-8')
        apply_config_ini(user_ini)

else:
    #
    # From within PlatformIO use the loaded INI file
    #
    import pioutil
    if pioutil.is_pio_build():
        try:
            verbose = int(pioutil.env.GetProjectOption('custom_verbose'))
        except:
            pass

        from platformio.project.config import ProjectConfig
        apply_config_ini(ProjectConfig())
