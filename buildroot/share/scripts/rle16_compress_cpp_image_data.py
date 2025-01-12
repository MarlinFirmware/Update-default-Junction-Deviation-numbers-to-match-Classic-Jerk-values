#!/usr/bin/env python3
#
# Utility to compress Marlin RGB565 TFT data to RLE16 format.
# Reads the existing Marlin RGB565 cpp file and generates a new file with the additional RLE16 data.
#
# Usage: rle16_compress_cpp_image_data.py INPUT_FILE.cpp OUTPUT_FILE.cpp
#
import sys, struct, re

def addCompressedData(input_file, output_file):
    ofile = open(output_file, 'wt')

    c_data_section = False
    c_skip_data = False
    c_footer = False
    raw_data = []
    rle_value = []
    rle_count = []
    arrname = ''

    line = input_file.readline()
    while line:
        if not c_footer:
            if not c_skip_data: ofile.write(line)

        if "};" in line:
            c_skip_data = False
            c_data_section = False
            c_footer = True

        if c_data_section:
            cleaned = re.sub(r"\s|,|\n", "", line)
            as_list = cleaned.split("0x")
            as_list.pop(0)
            raw_data += [int(x, 16) for x in as_list]

        if "const uint" in line:
            # e.g.: const uint16_t marlin_logo_480x320x16[153600] = {
            if "_rle16" in line:
                c_skip_data = True
            else:
                c_data_section = True
                arrname = line.split('[')[0].split(' ')[-1]
                print("Found data array", arrname)

        line = input_file.readline()

    input_file.close()

    #
    # RLE16 (run length 16) encoding
    # Convert data from from raw RGB565 to a simple run-length-encoded format for each word of data.
    # - Each sequence begins with a count byte N.
    #   - If the high bit is set in N the run contains N & 0x7F + 1 unique words.
    #   - Otherwise it repeats the following word N + 1 times.
    # - Each RGB565 word is stored in MSB / LSB order.
    #
    def rle_encode(data):
        warn = "This may take a while" if len(data) > 300000 else ""
        print("Compressing image data...", warn)
        rledata = []
        distinct = []
        i = 0
        while i < len(data):
            v = data[i]
            i += 1
            rsize = 1
            for j in range(i, len(data)):
                if v != data[j]: break
                i += 1
                rsize += 1
                if rsize >= 128: break

            # If the run is one, add to the distinct values
            if rsize == 1: distinct.append(v)

            # If distinct length >= 127, or the repeat run is 2 or more,
            # store the distinct run.
            nr = len(distinct)
            if nr and (nr >= 128 or rsize > 1 or i >= len(data)):
                rledata += [(nr - 1) | 0x80] + distinct
                distinct = []

            # If the repeat run is 2 or more, store the repeat run.
            if rsize > 1: rledata += [rsize - 1, v]

        return rledata

    def append_byte(data, byte, cols=240):
        if data == '': data = '  '
        data += ('0x{0:02X}, '.format(byte)) # 6 characters
        if len(data) % (cols * 6 + 2) == 0: data = data.rstrip() + "\n  "
        return data

    def rle_emit(ofile, arrname, rledata, rawsize):
        col = 0
        i = 0
        outstr = ''
        size = 0
        while i < len(rledata):
            rval = rledata[i]
            i += 1
            if rval & 0x80:
                count = (rval & 0x7F) + 1
                outstr = append_byte(outstr, rval)
                size += 1
                for j in range(count):
                    outstr = append_byte(outstr, rledata[i + j] >> 8)
                    outstr = append_byte(outstr, rledata[i + j] & 0xFF)
                    size += 2
                i += count
            else:
                outstr = append_byte(outstr, rval)
                outstr = append_byte(outstr, rledata[i] >> 8)
                outstr = append_byte(outstr, rledata[i] & 0xFF)
                i += 1
                size += 3

        outstr = outstr.rstrip()[:-1]
        ofile.write("\n// Saves %i bytes\nconst uint8_t %s_rle16[%d] = {\n%s\n};\n" % (rawsize - size, arrname, size, outstr))

        (w, h, d) = arrname.split("_")[-1].split('x')
        ofile.write("\nconst tImage MarlinLogo{0}x{1}x16 = MARLIN_LOGO_CHOSEN({0}, {1});\n".format(w, h))
        ofile.write("\n#endif // HAS_GRAPHICAL_TFT && SHOW_BOOTSCREEN\n".format(w, h))

    # Encode the data, write it out, close the file
    rledata = rle_encode(raw_data)
    rle_emit(ofile, arrname, rledata, len(raw_data) * 2)
    ofile.close()

if len(sys.argv) <= 2:
    print("Utility to compress Marlin RGB565 TFT data to RLE16 format.")
    print("Reads a Marlin RGB565 cpp file and generates a new file with the additional RLE16 data.")
    print("Usage: rle16_compress_cpp_image_data.py INPUT_FILE.cpp OUTPUT_FILE.cpp")
    exit(1)

output_cpp = sys.argv[2]
inname = sys.argv[1].replace('//', '/')
input_cpp = open(inname)
print("Processing", inname, "...")
addCompressedData(input_cpp, output_cpp)
