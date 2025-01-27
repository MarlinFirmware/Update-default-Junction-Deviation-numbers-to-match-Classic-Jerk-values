'''
config.py - Helper functions for config manipulation

Make sure both copies always match:
 - buildroot/bin/config.py
 - buildroot/share/PlatformIO/scripts/config.py

'''
import re

FILES = ('Marlin/Configuration.h', 'Marlin/Configuration_adv.h')

def set(file_path, define_name, value):
    '''
    Replaces a define in a file with a new value.
    Returns True if the define was found and replaced, False otherwise.
    '''
    # Read the contents of the file
    with open(file_path, 'r') as f:
        content = f.readlines()

    modified = False
    for i in range(len(content)):
        # Regex to match the desired pattern
        match = re.match(r'^(\s*)(/*)(\s*)(#define\s+{})\s+(.*?)\s*(//.*)?$'.format(re.escape(define_name)), content[i])
        if match:
            modified = True
            comm = '' if match[6] is None else ' ' + match[6]
            oldval = '' if match[5] is None else match[5]
            if match[2] or value != oldval:
                content[i] = f"{match[1]}{match[3]}{match[4]} {value} // {match[5]}{comm}\n"

    # Write the modified content back to the file only if changes were made
    if modified:
        with open(file_path, 'w') as f:
            f.writelines(content)
            return True

    return False

def add(file_path, define_name, value=""):
    '''
    Insert a define on the first blank line in a file.
    '''
    with open(file_path, 'r') as f:
        content = f.readlines()

    # Prepend a space to the value if it's not empty
    if value != "":
        value = " " + value

    # Find the first blank line to insert the new define
    for i in range(len(content)):
        if content[i].strip() == '':
            # Insert the define at the first blank line
            content.insert(i, f"#define {define_name}{value}\n")
            break
    else:
        # If no blank line is found, append to the end
        content.append(f"#define {define_name}{value}\n")

    with open(file_path, 'w') as f:
        f.writelines(content)

def enable(file_path, define_name, enable=True):
    '''
    Uncomment or comment the named defines in the given file path.
    Returns True if the define was found, False otherwise.
    '''
    # Read the contents of the file
    with open(file_path, 'r') as f:
        content = f.readlines()

    # Prepare the regex
    regex = re.compile(r'^(\s*)(/*)(\s*)(#define\s+{}\b.*?)(\s*//.*)?$'.format(re.escape(define_name)))

    # Find the define in the file and uncomment or comment it
    found = False
    modified = False
    for i in range(len(content)):
        match = regex.match(content[i])
        if not match: continue
        found = True
        if enable:
            if match[2]:
                modified = True
                comment = '' if match[5] is None else '  ' + match[5]
                content[i] = f"{match[1]}{match[3]}{match[4]}{comment}\n"
        else:
            if not match[2]:
                modified = True
                comment = '' if match[5] is None else match[5]
                if comment.startswith('  '): comment = comment[2:]
                content[i] = f"{match[1]}//{match[3]}{match[4]}{comment}\n"
        break

    # Write the modified content back to the file only if changes were made
    if modified:
        with open(file_path, 'w') as f:
            f.writelines(content)

    return found
