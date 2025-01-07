#!/usr/bin/env python
#
# mc-apply.py
#
#  Apply firmware configuration from a JSON file (marlin_config.json).
#
#  usage: mc-apply.py [-h] [--opt] [config_file]
#
#  Process Marlin firmware configuration.
#
#  positional arguments:
#    config_file  Path to the configuration file.
#
#  optional arguments:
#    -h, --help   show this help message and exit
#    --opt        Enable optional output format.
#
import json, sys, shutil
import config
import argparse

def report_version(conf):
    if 'VERSION' in conf:
        for k, v in sorted(conf['VERSION'].items()):
            print(k + ': ' + v)

def write_opt_file(conf, outpath='Marlin/apply_config.sh'):
    with open(outpath, 'w') as outfile:
        for key, val in conf.items():
            if key in ('__INITIAL_HASH', 'VERSION'): continue

            # Other keys are assumed to be configs
            if not type(val) is dict:
                continue

            # Write config commands to the script file
            lines = []
            for k, v in sorted(val.items()):
                if v != '':
                    v.replace('"', '\\"').replace("'", "\\'").replace(' ', '\\ ')
                    lines += [f'opt_set {k} {v}']
                else:
                    lines += [f'opt_enable {k}']

            outfile.write('\n'.join(lines))

        print('Config script written to: ' + outpath)

def apply_config(conf):
    for key in conf:
        if key in ('__INITIAL_HASH', 'VERSION'): continue
        for k, v in conf[key].items():
            if v:
                config.set('Marlin/' + key, k, v)
            else:
                config.enable('Marlin/' + key, k)

def main():
    parser = argparse.ArgumentParser(description='Process Marlin firmware configuration.')
    parser.add_argument('--opt', action='store_true', help='Enable optional output format.')
    parser.add_argument('config_file', nargs='?', default='marlin_config.json', help='Path to the configuration file.')

    args = parser.parse_args()

    opt_output = args.opt

    try:
        infile = open('marlin_config.json', 'r')
    except:
        print('No marlin_config.json found.')
        sys.exit(1)

    conf = json.load(infile)

    report_version(conf)

    if opt_output:
        write_opt_file(conf)
    else:
        apply_config(conf)

if __name__ == '__main__':
    main()
