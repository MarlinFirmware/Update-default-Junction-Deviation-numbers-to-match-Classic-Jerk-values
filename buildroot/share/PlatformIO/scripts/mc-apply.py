#!/usr/bin/env python
"""
mc-apply.py v2.0

This script creates Marlin firmware configuration files from a JSON file (marlin_config.json).

Functions:
    load_config(file_path: str) -> Dict:
        Loads and returns the JSON configuration from the specified file path.
        Exits with an error message if the file is not found, if JSON decoding fails, or if CONFIG_EXPORT is not set to "1".

    write_output_file(file_path: str, content: str) -> None:
        Writes the given content to the specified file path.
        Exits with an error message if writing to the file fails.

    move_file(src: str, dst: str) -> None:
        Moves a file from the source path to the destination path.
        Exits with an error message if the move operation fails.

    move_list_files(src_list: List[str], dst_list: List[str]) -> None:
        Moves files from the source list to the destination list.
        Exits with an error message if the source and destination lists have different lengths.

    process_configuration(conf: Dict, opt_output: bool, output_suffix: str) -> None:
        Processes the configuration dictionary and writes the output configuration files.
        Handles both standard and optional output formats.
        Moves original files to backup locations if an output suffix is specified.

    main() -> None:
        Main entry point of the script.
        Parses command line arguments to determine the configuration file path, output format, and output suffix.
        Loads the configuration and processes it.

Command Line Options:
    --opt:
        Enables optional output format where configuration values are set using 'opt_set' and 'opt_enable' commands.

    --bare-output:
        Disables the output suffix, resulting in '.gen' suffix for generated files.

    -h, --help:
        Show this help message and exit.
"""
import json
import sys
import shutil
import os
import argparse
import logging
from typing import Dict, List

logging.basicConfig(level=logging.INFO)

MARLIN_CONFIG_FILES = ('Configuration.h', 'Configuration_adv.h')

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

def write_output_file(file_path: str, content: str) -> None:
    try:
        with open(file_path, 'w') as outfile:
            outfile.write(content)
    except IOError as e:
        logging.error(f'Failed to write to {file_path}. {e}')
        sys.exit(1)

def move_file(src: str, dst: str) -> None:
    try:
        shutil.move(src, dst)
    except IOError as e:
        logging.error(f'Failed to move {src} to {dst}. {e}')
        sys.exit(1)

def move_list_files(src_list: List[str], dst_list: List[str]) -> None:
    if len(src_list) != len(dst_list):
        logging.error('Source and destination lists must have the same length.')
        sys.exit(1)
    for src, dst in zip(src_list, dst_list):
        move_file(src, dst)

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
            sline = line.strip(" \t\n\r")
            if sline.startswith("//#define"):
                sline = sline[2:]
            if sline.startswith("#define"):
                leading_whitespace = line[:len(line) - len(line.lstrip())]
                trailing_comment = ''
                if '//' in sline:
                    trailing_comment_end = '//' + sline.split('//', 1)[1]
                    sline = sline.split('//', 1)[0]
                    trailing_comment_whitespace = sline[len(sline.rstrip()):]
                    trailing_comment = trailing_comment_whitespace + trailing_comment_end
                kv = sline[8:].strip().split()
                mid_whitespace = ''
                if len(kv) > 1:
                    mid_whitespace = sline.split(kv[0], 1)[1].rsplit(kv[1], 1)[0]
                if kv[0] in conf:
                    content += f'{leading_whitespace}#define {kv[0]}{mid_whitespace}{conf[kv[0]]}{trailing_comment}\n'
                    del conf[kv[0]]
                else:
                    content += line + '\n'
            else:
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
