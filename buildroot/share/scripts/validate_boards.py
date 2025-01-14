#!/usr/bin/env python3
#
# buildroot/share/scripts/validate_boards.py
# Assert standards for boards.h and pins.h
#

import sys, re

do_log = False
def logmsg(msg, line):
    if do_log: print(msg, line)

# Print a formatted error
def err(board, msg):
  print(f'[ERROR] {board:30} {msg}')

# Print a formatted warning
def warn(board, msg):
  print(f'[WARNING] {board:30} {msg}')

def bshort(board):
    return board.replace('BOARD_', '')

#
# Run standards checks on boards.h and pins.h
#
def boards_checks(argv):
    ERRS = 0
    src_file = 'Marlin/src/core/boards.h'

    scnt = 0
    for arg in argv:
        if arg == '-v':
            global do_log
            do_log = True
        elif scnt == 0:
            src_file = arg
            scnt += 1

    logmsg('Checking boards file:', src_file)

    # Open the file
    with open(src_file, 'r') as f:
        lines = f.readlines()

    # Get the board names and numbers
    boards = []
    for line in lines:
        m = re.match(r'^\s*#define\s+(BO\w+)\s+(\d+)(\s+//\s*(.+))?', line)
        if not m: continue
        board, number, comment = m.group(1), int(m.group(2)), m.group(4)
        boards.append((board, number, comment))

    #
    # Examine boards.h to check the formatting of the file
    #
    last_number, last_groun = (-1, -1)

    for board, number, comment in boards:
        logmsg('Checking:', board)
        group = int(number / 100)
        if not re.match(r'^BOARD_\w+$', board):
            err(board, 'is not of the form BOARD_NAME')
            ERRS += 1
        if number != last_number + 1:
            if int(number / 100) != int(last_number / 100):
                if number % 100 != 0 and number < 9900:
                    err(board, f'is {number} (should be {group * 100}?)')
                    ERRS += 1
            elif number > 1040:
                err(board, f'is {number} but previous board is {last_number}')
                ERRS += 1
        if not comment:
            err(board, ' has no comment')
            ERRS += 1
        else:
            cshor = bshort(board)
            cbore = cshor.replace('_', '')
            if comment == board or comment == cshor or comment == cbore:
                warn(board, f'comment needs more detail')
        last_number = number
        last_group = number % 100

    #
    # Validate that pins.h has all the boards mentioned in it
    #
    pins_boards = []
    with open('Marlin/src/pins/pins.h', 'r') as f:
        lines = f.readlines()
        if_count = 0
        for line in lines:
            m = re.search(r'#(if|elif)\s+MB\(([^)]+)\)', line)
            if not m: continue
            if (m.group(1) == 'if'):
                if_count += 1
                if if_count == 3: break
            if if_count == 2:
                mb_items = m.group(2).split(',')
                for board in mb_items:
                    pins_boards.append('BOARD_' + board.strip())

    # Check that the list from boards.h matches the list from pins.h
    boards_boards = [b[0] for b in boards]
    if set(pins_boards) != set(boards_boards):
        ERRS += 1
        print(f'[ERROR] Boards in pins.h do not match boards.h')
        # Show the differences only
        for b in boards:
            if b[0] not in pins_boards:
                print(f'   pins.h missing: {b[0]}')
        for b in pins_boards:
            if b not in boards_boards:
                print(f' boards.h missing: {b}')

    # Check that boards_boards and pins_boards are in the same order
    for i in range(len(boards_boards)):
        if boards_boards[i] != pins_boards[i]:
            ERRS += 1
            print(f'[ERROR] Non-matching boards order in pins.h. Expected {bshort(boards_boards[i])} but got {bshort(pins_boards[i])}')
            break

    return ERRS;

if __name__ == '__main__':
    ERR_COUNT = boards_checks(sys.argv[1:])
    if ERR_COUNT:
        print(f'\nFound {ERR_COUNT} errors')
        sys.exit(1)
