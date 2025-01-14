#!/usr/bin/env python3

#
# Formatter script for pins_MYPINS.h files
#
# Usage: pinsformat.py [infile] [outfile]
#
# With no parameters convert STDIN to STDOUT
#

import sys, re

do_log = False
def logmsg(msg, line):
    if do_log: print(msg, line)

col_comment = 50

# String lpad / rpad
def lpad(astr, fill, c=' '):
    if not fill: return astr
    need = fill - len(astr)
    return astr if need <= 0 else (need * c) + astr

def rpad(astr, fill, c=' '):
    if not fill: return astr
    need = fill - len(astr)
    return astr if need <= 0 else astr + (need * c)

# Concatenate a string, adding a space if necessary
# to avoid merging two words
def concat_with_space(s1, s2):
    if not s1.endswith(' ') and not s2.startswith(' '):
        s1 += ' '
    return s1 + s2

# Pin patterns
mpatt = [ r'-?\d{1,3}', r'P[A-I]\d+', r'P\d_\d+', r'Pin[A-Z]\d\b' ]
mstr = '|'.join(mpatt)
mexpr = [ re.compile(f'^{m}$') for m in mpatt ]

# Corrsponding padding for each pattern
ppad = [ 3, 4, 5, 5 ]

# Match a define line
definePinPatt = re.compile(rf'^\s*(//)?#define\s+[A-Z_][A-Z0-9_]+?_PIN\s+({mstr})\s*(//.*)?$')

def format_pins(argv):
    src_file = 'stdin'
    dst_file = None

    scnt = 0
    for arg in argv:
        if arg == '-v':
            global do_log
            do_log = True
        elif scnt == 0:
            # Get a source file if specified. Default destination is the same file
            src_file = dst_file = arg
            scnt += 1
        elif scnt == 1:
            # Get destination file if specified
            dst_file = arg
            scnt += 1

    # No text to process yet
    file_text = ''

    if src_file == 'stdin':
        # If no source file specified read from STDIN
        file_text = sys.stdin.read()
    else:
        # Open and read the file src_file
        with open(src_file, 'r') as rf: file_text = rf.read()

    if len(file_text) == 0:
        print('No text to process')
        return

    # Read from file or STDIN until it terminates
    filtered = process_text(file_text)
    if dst_file:
        with open(dst_file, 'w') as wf: wf.write(filtered)
    else:
        print(filtered)

# Find the pin pattern so non-pin defines can be skipped
def get_pin_pattern(txt):
    r = ''
    m = 0
    match_count = [ 0, 0, 0, 0 ]

    # Find the most common matching pattern
    match_threshold = 5
    for line in txt.split('\n'):
        r = definePinPatt.match(line)
        if r == None: continue
        ind = -1
        for p in mexpr:
            ind += 1
            if not p.match(r[2]): continue
            match_count[ind] += 1
            if match_count[ind] >= match_threshold:
                return { 'match': mpatt[ind], 'pad':ppad[ind] }
    return None

def process_text(txt):
    if len(txt) == 0: return '(no text)'
    patt = get_pin_pattern(txt)
    if patt == None: return txt

    pmatch = patt['match']
    pindefPatt = re.compile(rf'^(\s*(//)?#define)\s+([A-Z_][A-Z0-9_]+)\s+({pmatch})\s*(//.*)?$')
    noPinPatt  = re.compile(r'^(\s*(//)?#define)\s+([A-Z_][A-Z0-9_]+)\s+(-1)\s*(//.*)?$')
    skipPatt1  = re.compile(r'^(\s*(//)?#define)\s+(AT90USB|USBCON|(BOARD|DAC|FLASH|HAS|IS|USE)_.+|.+_(ADDRESS|AVAILABLE|BAUDRATE|CLOCK|CONNECTION|DEFAULT|ERROR|EXTRUDERS|FREQ|ITEM|MKS_BASE_VERSION|MODULE|NAME|ONLY|ORIENTATION|PERIOD|RANGE|RATE|READ_RETRIES|SERIAL|SIZE|SPI|STATE|STEP|TIMER|VERSION))\s+(.+)\s*(//.*)?$')
    skipPatt2  = re.compile(r'^(\s*(//)?#define)\s+([A-Z_][A-Z0-9_]+)\s+(0x[0-9A-Fa-f]+|\d+|.+[a-z].+)\s*(//.*)?$')
    skipPatt3  = re.compile(r'^\s*#e(lse|ndif)\b.*$')
    aliasPatt  = re.compile(r'^(\s*(//)?#define)\s+([A-Z_][A-Z0-9_]+)\s+([A-Z_][A-Z0-9_()]+)\s*(//.*)?$')
    switchPatt = re.compile(r'^(\s*(//)?#define)\s+([A-Z_][A-Z0-9_]+)\s*(//.*)?$')
    undefPatt  = re.compile(r'^(\s*(//)?#undef)\s+([A-Z_][A-Z0-9_]+)\s*(//.*)?$')
    defPatt    = re.compile(r'^(\s*(//)?#define)\s+([A-Z_][A-Z0-9_]+)\s+([-_\w]+)\s*(//.*)?$')
    condPatt   = re.compile(r'^(\s*(//)?#(if|ifn?def|elif)(\s+\S+)*)\s+(//.*)$')
    commPatt   = re.compile(r'^\s{20,}(//.*)?$')

    col_value_lj = col_comment - patt['pad'] - 2
    col_value_rj = col_comment - 3

    #
    # #define SKIP_ME
    #
    def trySkip1(d):
        if skipPatt1.match(d['line']) == None: return False
        logmsg("skip:", d['line'])
        return True

    #
    # #define MY_PIN [pin]
    #
    def tryPindef(d):
        line = d['line']
        r = pindefPatt.match(line)
        if r == None: return False
        logmsg("pin:", line)
        pinnum = r[4] if r[4][0] == 'P' else lpad(r[4], patt['pad'])
        line = f'{r[1]} {r[3]}'
        line = concat_with_space(rpad(line, col_value_lj), pinnum)
        if r[5]: line = rpad(line, col_comment) + r[5]
        d['line'] = line
        return True

    #
    # #define MY_PIN -1
    #
    def tryNoPin(d):
        line = d['line']
        r = noPinPatt.match(line)
        if r == None: return False
        logmsg("pin -1:", line)
        line = f'{r[1]} {r[3]}'
        line = concat_with_space(rpad(line, col_value_lj), '-1')
        if r[5]: line = rpad(line, col_comment) + r[5]
        d['line'] = line
        return True

    #
    # #define SKIP_ME_TOO
    #
    def trySkip2(d):
        if skipPatt2.match( d['line']) == None: return False
        logmsg("skip:", d['line'])
        return True

    #
    # #else|endif
    #
    def trySkip3(d):
        if skipPatt3.match( d['line']) == None: return False
        logmsg("skip:", d['line'])
        return True

    #
    # #define ALIAS OTHER
    #
    def tryAlias(d):
        line = d['line']
        r = aliasPatt.match(line)
        if r == None: return False
        logmsg("alias:", line)
        line = f'{r[1]} {r[3]}'
        line = concat_with_space(line, lpad(r[4], col_value_rj + 1 - len(line)))
        if r[5]: line = concat_with_space(rpad(line, col_comment), r[5])
        d['line'] = line
        return True

    #
    # #define SWITCH
    #
    def trySwitch(d):
        line = d['line']
        r = switchPatt.match(line)
        if r == None: return False
        logmsg("switch:", line)
        line = f'{r[1]} {r[3]}'
        if r[4]: line = concat_with_space(rpad(line, col_comment), r[4])
        d['line'] = line
        d['check_comment_next'] = True
        return True

    #
    # #define ...
    #
    def tryDef(d):
        line = d['line']
        r = defPatt.match(line)
        if r == None: return False
        logmsg("def:", line)
        line = f'{r[1]} {r[3]} '
        line = concat_with_space(line, lpad(r[4], col_value_rj + 1 - len(line)))
        if r[5]: line = rpad(line, col_comment - 1) + ' ' + r[5]
        d['line'] = line
        return True

    #
    # #undef ...
    #
    def tryUndef(d):
        line = d['line']
        r = undefPatt.match(line)
        if r == None: return False
        logmsg("undef:", line)
        line = f'{r[1]} {r[3]}'
        if r[4]: line = concat_with_space(rpad(line, col_comment), r[4])
        d['line'] = line
        return True

    #
    # #if|ifdef|ifndef|elif ...
    #
    def tryCond(d):
        line = d['line']
        r = condPatt.match(line)
        if r == None: return False
        logmsg("cond:", line)
        line = concat_with_space(rpad(r[1], col_comment), r[5])
        d['line'] = line
        d['check_comment_next'] = True
        return True

    out = ''
    wDict = { 'check_comment_next': False }

    # Transform each line and add it to the output
    for line in txt.split('\n'):
        wDict['line'] = line
        if wDict['check_comment_next']:
            r = commPatt.match(line)
            wDict['check_comment_next'] = (r != None)

        if wDict['check_comment_next']:
            # Comments in column 50
            line = rpad('', col_comment) + r[1]

        elif trySkip1(wDict):   pass  #define SKIP_ME
        elif tryPindef(wDict):  pass  #define MY_PIN [pin]
        elif tryNoPin(wDict):   pass  #define MY_PIN -1
        elif trySkip2(wDict):   pass  #define SKIP_ME_TOO
        elif trySkip3(wDict):   pass  #else|endif
        elif tryAlias(wDict):   pass  #define ALIAS OTHER
        elif trySwitch(wDict):  pass  #define SWITCH
        elif tryDef(wDict):     pass  #define ...
        elif tryUndef(wDict):   pass  #undef ...
        elif tryCond(wDict):    pass  #if|ifdef|ifndef|elif ...

        out += wDict['line'].rstrip() + '\n'

    return re.sub('\n\n$', '\n', re.sub(r'\n\n+', '\n\n', out))

# Python standard startup for command line with arguments
if __name__ == '__main__':
    format_pins(sys.argv[1:])
