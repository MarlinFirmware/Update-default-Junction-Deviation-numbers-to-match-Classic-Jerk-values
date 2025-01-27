#!/usr/bin/env python3
#
# Split a DWIN .ico file into separate images.
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
        parser = argparse.ArgumentParser(description='Split .ico into JPEG files')
        parser.add_argument('filename', type=str, nargs=1,
                            help='name of input .ico file to split')
        parser.add_argument('outputDir', type=str, nargs=1,
                            help='name of output directory to create')
        args = parser.parse_args()

        filename = args.filename[0]
        outputDir = args.outputDir[0]

        if not os.path.isfile(filename):
            raise RuntimeError("ICO file '%s' doesn't exist" % (filename))

        if os.path.exists(outputDir):
            raise RuntimeError("Output directory '%s' already exists." % (outputDir))

        print('Splitting %s into dir %s' % (filename, outputDir))
        ico = DWIN_ICO.DWIN_ICO_File()
        ico.splitFile(filename, outputDir)

    except Exception as e:
        print('Error: ', e)
