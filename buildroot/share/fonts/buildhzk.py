# Generate a 'HZK' font file for the T5UIC1 DWIN LCD
# from multiple bdf font files.
# Note: the 16x16 glyphs are not produced
# Author: Taylor Talkington
# License: GPL

import bdflib.reader, math

def glyph_bits(size_x, size_y, font, glyph_ord):
    asc = font[b'FONT_ASCENT']
    desc = font[b'FONT_DESCENT']
    bits = [0 for y in range(size_y)]

    glyph_bytes = math.ceil(size_x / 8)
    try:
        glyph = font[glyph_ord]
        for y, row in enumerate(glyph.data):
            v = row
            rpad = size_x - glyph.bbW
            if rpad < 0: rpad = 0
            if glyph.bbW > size_x: v = v >> (glyph.bbW - size_x) # some glyphs are actually too wide to fit!
            v = v << (glyph_bytes * 8) - size_x + rpad
            v = v >> glyph.bbX
            bits[y + desc + glyph.bbY] |= v
    except KeyError:
        pass

    bits.reverse()
    return bits

def marlin_font_hzk():
    fonts = [
        [6,12,'marlin-6x12-3.bdf'],
        [8,16,'marlin-8x16.bdf'],
        [10,20,'marlin-10x20.bdf'],
        [12,24,'marlin-12x24.bdf'],
        [14,28,'marlin-14x28.bdf'],
        [16,32,'marlin-16x32.bdf'],
        [20,40,'marlin-20x40.bdf'],
        [24,48,'marlin-24x48.bdf'],
        [28,56,'marlin-28x56.bdf'],
        [32,64,'marlin-32x64.bdf']
    ]

    with open('marlin_fixed.hzk','wb') as output:
        for f in fonts:
            with open(f[2], 'rb') as file:
                print(f'{f[0]}x{f[1]}')
                font = bdflib.reader.read_bdf(file)
                for glyph in range(128):
                    bits = glyph_bits(f[0], f[1], font, glyph)
                    glyph_bytes = math.ceil(f[0]/8)

                    for b in bits:
                        try:
                            z = b.to_bytes(glyph_bytes, 'big')
                            output.write(z)
                        except OverflowError:
                            print('Overflow')
                            print(f'{glyph}')
                            print(font[glyph])
                            for b in bits: print(f'{b:0{f[0]}b}')
                            return
