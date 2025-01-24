#
# simulator.py
# PlatformIO pre: script for simulator builds
#

import pioutil
if pioutil.is_pio_build():
    # Get the environment thus far for the build
    env = pioutil.env

    #print(env.Dump())

    #
    # Give the binary a distinctive name
    #

    env['PROGNAME'] = "MarlinSimulator"

    #
    # Check for a valid GCC and available OpenGL on macOS
    #
    emsg = ''
    fatal = 0
    import sys
    if sys.platform == 'darwin':

        import shutil
        gcc = shutil.which('gcc')
        if gcc == '' or gcc == '/usr/bin/gcc':
            if gcc == '':
                emsg = "\u001b[31mNo GCC found in your configured shell PATH."
            elif gcc == '/usr/bin/gcc':
                emsg = "\u001b[31mCan't build Marlin Native on macOS using the included version of GCC (clang)."
            emsg += "\n\u001b[31mSee 'native.ini' for instructions to install GCC with MacPorts or Homebrew."
            fatal = 1

        else:

            #
            # Silence half of the ranlib warnings. (No equivalent for 'ARFLAGS')
            #
            env['RANLIBFLAGS'] += [ "-no_warning_for_no_symbols" ]

            # Default paths for Xcode and a lucky GL/gl.h dropped by Mesa
            xcode_path = "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/System/Library/Frameworks"
            mesa_path = "/opt/local/include/GL/gl.h"

            import os.path

            if os.path.exists(xcode_path):

                env['BUILD_FLAGS'] += [ "-F" + xcode_path ]
                emsg = "\u001b[33mUsing OpenGL framework headers from Xcode.app"

            elif os.path.exists(mesa_path):

                env['BUILD_FLAGS'] += [ '-D__MESA__' ]
                emsg = f"\u001b[33mUsing OpenGL header from {mesa_path}"

            else:

                emsg = "\u001b[31mNo OpenGL headers found. Install Xcode for matching headers, or use 'sudo port install mesa' to get a GL/gl.h."
                fatal = 1

    # Print error message, if any
    if emsg: print(f"\n\n{emsg}\n\n")

    # Break out of the PIO build immediately
    if fatal: sys.exit(1)
