#!/usr/bin/env python3
#
# Make a DWIN .ico file from a directory of JPEG icon files.
#
#  Copyright (c) 2020 Brent Burton
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.
#----------------------------------------------------------------

import os.path, argparse, DWIN_ICO

version = '2.0.7'

#----------------
if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(description='Make .ico from JPEG files')
        parser.add_argument('iconDir', type=str, nargs=1,
                            help='name of directory containing icon JPGs')
        parser.add_argument('filename', type=str, nargs=1,
                            help='name of new .ico file to create')
        args = parser.parse_args()

        filename = args.filename[0]
        iconDir = args.iconDir[0]

        if os.path.isfile(filename):
            raise RuntimeError("ICO file '%s' already exists." % (filename))

        if not os.path.exists(iconDir):
            raise RuntimeError("Icon directory '%s' doesn't exist." % (iconDir))

        print("Making .ico file '%s' from contents of '%s'" % (filename, iconDir))
        ico = DWIN_ICO.DWIN_ICO_File()
        ico.createFile(iconDir, filename)

    except Exception as e:
        print('Error: ', e)
