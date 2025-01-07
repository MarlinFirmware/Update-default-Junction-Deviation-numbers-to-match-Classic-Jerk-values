#!/usr/bin/env python
"""
  mc-apply.py
  Create Marlin firmware configuration files from a JSON file (marlin_config.json).

  usage: mc-apply.py [-h] [--opt] [--bare-output] [config_file]

  Process Marlin firmware configuration.

  positional arguments:
    config_file    Path to the configuration file.

  optional arguments:
    -h, --help     show this help message and exit
    --opt          Enable optional output format.
    --bare-output  Disable output suffix.
"""
import json, sys, shutil
import os, re
import argparse
import logging
from typing import Dict, List

logging.basicConfig(level=logging.INFO)

MARLIN_CONFIG_FILES = ('Configuration.h', 'Configuration_adv.h')

# Load and return the JSON configuration from the specified file path.
# Exit with an error message if the file is not found, JSON decoding fails, or CONFIG_EXPORT is not 1.
def load_config(file_path: str) -> Dict:
    try:
        with open(file_path, 'r') as file:
            config = json.load(file)
            if config.get("CONFIG_EXPORT") != "1":
                logging.error('CONFIG_EXPORT is not set to "1" in the configuration file.')
                sys.exit(1)
            return config
    except FileNotFoundError:
        logging.error(f'{file_path} not found.')
        sys.exit(1)
    except json.JSONDecodeError:
        logging.error(f'Failed to decode JSON from {file_path}.')
        sys.exit(1)

# Writes the given content to the specified file path.
# Exits with an error message if writing to the file fails.
def write_output_file(file_path: str, content: str) -> None:
    try:
        with open(file_path, 'w') as outfile:
            outfile.write(content)
    except IOError as e:
        logging.error(f'Failed to write to {file_path}. {e}')
        sys.exit(1)

# Move a file from the source to the destination path.
# Exit with an error message if the move operation fails.
def move_file(src: str, dst: str) -> None:
    try:
        shutil.move(src, dst)
    except IOError as e:
        logging.error(f'Failed to move {src} to {dst}. {e}')
        sys.exit(1)

# Move files from the source list to the destination list.
# Exit with an error message if the source and destination lists have different lengths.
def move_list_files(src_list: List[str], dst_list: List[str]) -> None:
    if len(src_list) != len(dst_list):
        logging.error('Source / destination length mismatch.')
        sys.exit(1)
    for src, dst in zip(src_list, dst_list):
        move_file(src, dst)

# Process the configuration dictionary and write the output configuration files.
# Handles both standard and optional output formats.
# Move original files to backup locations if an output suffix is specified.
def process_configuration(conf: Dict, opt_output: bool, output_suffix: str) -> None:
    output_file_path = os.path.join('Marlin', MARLIN_CONFIG_FILES[0] + output_suffix)
    content = ''

    for key, values in conf.items():
        if key in ('__INITIAL_HASH', 'VERSION'):
            if key == 'VERSION':
                for k, v in sorted(values.items()):
                    logging.info(f'{k}: {v}')
            continue

        if opt_output:
            if values:
                if '"' in values:
                    values = f"'{values}'"
                elif ' ' in values:
                    values = f'"{values}"'
                define = f'opt_set {key} {values}\n'
            else:
                define = f'opt_enable {key}\n'
        else:
            define = f'#define {key} {values}\n'
        content += define.strip() + "\n"

    write_output_file(output_file_path, content)

    if output_suffix:
        original_file_paths = [os.path.join('Marlin', config) for config in MARLIN_CONFIG_FILES]
        backup_file_paths = [path + '.orig' for path in original_file_paths]
        move_list_files(original_file_paths, backup_file_paths)

        lines = []
        separator = "#mc-apply-separator"

        for backup_file_path in backup_file_paths:
            try:
                with open(backup_file_path, 'r') as file:
                    lines.extend(file.read().split('\n'))
                lines.append(separator)
            except IOError as e:
                logging.error(f'Failed to read from {backup_file_path}. {e}')
                sys.exit(1)

        content = ''
        conf.pop('__INITIAL_HASH', None)
        conf.pop('VERSION', None)
        for line in lines:
            sline = re.sub(r'//\s*(#define)', '$1', line)
            if sline.startswith("#define"):
                indent = line[:len(line) - len(line.lstrip())]
                trailing_comment = ''
                if '//' in sline:
                    cpart = sline.split('//', 1)
                    sline = cpart[0]
                    trailing_comment = '//' + cpart[1]
                    trailing_comment_ws = sline[len(sline.rstrip()):]
                    trailing_comment = trailing_comment_ws + trailing_comment
                kv = sline[8:].strip().split()
                mid_whitespace = sline.split(kv[0], 1)[1].rsplit(kv[1], 1)[0] if len(kv) > 1 else ''
                if kv[0] in conf:
                    line = f'{indent}#define {kv[0]}{mid_whitespace}{conf[kv[0]]}{trailing_comment}'
                    del conf[kv[0]]
            content += line + '\n'

        for k, v in sorted(conf.items()):
            content += f'#define {k} {v}\n'

        seen_separator = 0
        config_data = ''
        config_adv_data = ''
        for line in content.split('\n'):
            if separator in line:
                seen_separator += 1
                continue
            if seen_separator == 0 or seen_separator >= 2:
                config_data += line + '\n'
            else:
                config_adv_data += line + '\n'
        write_output_file(original_file_paths[0], config_data)
        write_output_file(original_file_paths[1], config_adv_data)

    logging.info(f'Output configuration written to: {output_file_path}')

# Parse command line arguments to get config file path, output format, and output suffix.
# Load the configuration and process it.
def main() -> None:
    parser = argparse.ArgumentParser(description='Process Marlin firmware configuration.')
    parser.add_argument('--opt', action='store_true', help='Enable optional output format.')
    parser.add_argument('--bare-output', action='store_true', help='Disable output suffix.')
    parser.add_argument('config_file', nargs='?', default='marlin_config.json', help='Path to the configuration file.')

    args = parser.parse_args()

    opt_output = args.opt
    output_suffix = '.sh' if opt_output else '' if args.bare_output else '.gen'
    config_file_path = args.config_file

    conf = load_config(config_file_path)
    process_configuration(conf, opt_output, output_suffix)

if __name__ == '__main__':
    main()
