#!/usr/bin/env python3
'''
languageExport.py [--single]

Export LCD language strings to CSV files for easier translation.
Use languageImport.py to import CSV into the language files.

Use --single to export all languages to a single CSV file.
'''

import re
from pathlib import Path
from sys import argv
from languageUtil import namebyid

LANGHOME = "Marlin/src/lcd/language"

# Write multiple sheets if true, otherwise write one giant sheet
MULTISHEET = '--single' not in argv[1:]
OUTDIR = 'out-csv'

# Check for the path to the language files
if not Path(LANGHOME).is_dir():
    print("Error: Couldn't find the '%s' directory." % LANGHOME)
    print("Edit LANGHOME or cd to the root of the repo before running.")
    exit(1)

# A limit just for testing
LIMIT = 0

# A dictionary to contain strings for each language.
# Init with 'en' so English will always be first.
language_strings = { 'en': {} }

# A dictionary to contain all distinct LCD string names
names = {}

# Get all "language_*.h" files
langfiles = sorted(list(Path(LANGHOME).glob('language_*.h')))

# Read each language file
for langfile in langfiles:
    # Get the language code from the filename
    langcode = langfile.name.replace('language_', '').replace('.h', '')

    # Skip 'test' and any others that we don't want
    if langcode in ['test']: continue

    # Open the file
    f = open(langfile, 'r', encoding='utf-8')
    if not f: continue

    # Flags to indicate a wide or tall section
    wideflag, tallflag = False, False
    # A counter for the number of strings in the file
    stringcount = 0
    # A dictionary to hold all the strings
    strings = { 'narrow': {}, 'wide': {}, 'tall': {} }
    # Read each line in the file
    for line in f:
        # Clean up the line for easier parsing
        line = line.split("//")[0].strip()
        if line.endswith(';'): line = line[:-1].strip()

        # Check for wide or tall sections, assume no complicated nesting
        if line.startswith("#endif") or line.startswith("#else"):
            wideflag, tallflag = False, False
        elif re.match(r'#if.*WIDTH\s*>=?\s*2[01].*', line): wideflag = True
        elif re.match(r'#if.*LCD_HEIGHT\s*>=?\s*4.*', line): tallflag = True

        # For string-defining lines capture the string data
        match = re.match(r'LSTR\s+([A-Z0-9_]+)\s*=\s*(.+)\s*', line)
        if match:
            # Name and quote-sanitized value
            name, value = match.group(1), match.group(2).replace('\\"', '$$$')

            # Remove all _UxGT wrappers from the value in a non-greedy way
            value = re.sub(r'_UxGT\((".*?")\)', r'\1', value)

            # Multi-line strings get one or more bars | for identification
            multiline = 0
            multimatch = re.match(r'.*MSG_(\d)_LINE\s*\(\s*(.+?)\s*\).*', value)
            if multimatch:
                multiline = int(multimatch.group(1))
                value = '|' + re.sub(r'"\s*,\s*"', '|', multimatch.group(2))

            # Wrap inline defines in parentheses
            value = re.sub(r' *([A-Z0-9]+_[A-Z0-9_]+) *', r'(\1)', value)
            # Remove quotes around strings
            value = re.sub(r'"(.*?)"', r'\1', value).replace('$$$', '""')
            # Store all unique names as dictionary keys
            names[name] = 1
            # Store the string as narrow or wide
            strings['tall' if tallflag else 'wide' if wideflag else 'narrow'][name] = value

            # Increment the string counter
            stringcount += 1
            # Break for testing
            if LIMIT and stringcount >= LIMIT: break

    # Close the file
    f.close()
    # Store the array in the dict
    language_strings[langcode] = strings

# Get the language codes from the dictionary
langcodes = list(language_strings.keys())

# Print the array
#print(language_strings)

# Report the total number of unique strings
print("Found %s distinct LCD strings." % len(names))

# Write a single language entry to the CSV file with narrow, wide, and tall strings
def write_csv_lang(f, strings, name):
    f.write(',')
    if name in strings['narrow']: f.write('"%s"' % strings['narrow'][name])
    f.write(',')
    if name in strings['wide']: f.write('"%s"' % strings['wide'][name])
    f.write(',')
    if name in strings['tall']: f.write('"%s"' % strings['tall'][name])

if MULTISHEET:
    #
    # Export a separate sheet for each language
    #
    Path.mkdir(Path(OUTDIR), exist_ok=True)

    for lang in langcodes:
        with open("%s/language_%s.csv" % (OUTDIR, lang), 'w', encoding='utf-8') as f:
            lname = lang + ' ' + namebyid(lang)
            header = ['name', lname, lname + ' (wide)', lname + ' (tall)']
            f.write('"' + '","'.join(header) + '"\n')

            for name in names.keys():
                f.write('"' + name + '"')
                write_csv_lang(f, language_strings[lang], name)
                f.write('\n')

else:
    #
    # Export one large sheet containing all languages
    #
    with open("languages.csv", 'w', encoding='utf-8') as f:
        header = ['name']
        for lang in langcodes:
            lname = lang + ' ' + namebyid(lang)
            header += [lname, lname + ' (wide)', lname + ' (tall)']
        f.write('"' + '","'.join(header) + '"\n')

        for name in names.keys():
            f.write('"' + name + '"')
            for lang in langcodes: write_csv_lang(f, language_strings[lang], name)
            f.write('\n')
