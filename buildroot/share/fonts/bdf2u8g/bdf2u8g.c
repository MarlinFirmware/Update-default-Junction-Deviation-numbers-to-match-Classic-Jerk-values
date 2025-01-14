/*
  general font collections
    https://www.smashingmagazine.com/2007/11/08/40-excellent-freefonts-for-professional-design/
    https://techmagazine.ws/most-popular-free-quality-fonts/
    https://openfontlibrary.org/
  bitmap font collections
    https://www.orgdot.com/aliasfonts/ (includes links)
    https://www.04.jp.org/
    https://www.miniml.com
    https://www.fontspace.com/010bus

  https://en.wikipedia.org/wiki/Unicode_typeface
      da könnten auch ein paar fonts dabei sein, die die m2tklib sonderzeichen beinhalten:
      Caslon Roman              https://en.wikipedia.org/wiki/Caslon_Roman
      Charis Sil                https://en.wikipedia.org/wiki/Charis_SIL
      DejaVu Sans       https://en.wikipedia.org/wiki/DejaVu_fonts
      Doulos            https://en.wikipedia.org/wiki/Doulos_SIL
      Free Serif        https://en.wikipedia.org/wiki/FreeSerif          https://ftp.gnu.org/gnu/freefont/
            --> keine box, aber es gibt pfeile/invertierte pfeile und kreise für m2tklib
      Gentium Plus ????    https://scripts.sil.org/cms/scripts/page.php?site_id=nrsi&id=Gentium_download#02b091ae
            --> keine graphic
      GNU Unifont       https://en.wikipedia.org/wiki/GNU_Unifont, https://unifoundry.com/unifont.html

      Titus cyberbit Basic      https://en.wikipedia.org/wiki/TITUS_Cyberbit_Basic

  fonts
    Gentium
      https://openfontlibrary.org/font/gentium
      license: OFL
    Old-Standard
      https://openfontlibrary.org/font/old-standard
      license: OFL
    Hanuman
      https://openfontlibrary.org/font/hanumanb
      license: OFL
    FreeUniversal
      https://openfontlibrary.org/font/freeuniversal
      license: OFL
    FriendShip-Code  <--- nicht so sicher...
      https://openfontlibrary.org/font/friendship-code
      license: CC-BY-SA
    LinuxLibertine
      https://de.wikipedia.org/wiki/Linux_Libertine
      https://sourceforge.net/projects/linuxlibertine/files/linuxlibertine/5.1.3-2/
      license: OFL
    DidactGothic
      source: https://openfontlibrary.org/
    judson
      source: https://openfontlibrary.org/
    unicons
      source: https://openfontlibrary.org/
      license: OFL
      suggested pt: 26, 30
    org_V01, fixed_V0
      source: https://www.orgdot.com/aliasfonts/
      license: open source, individual, cite required
      suggested pt: 8
    04b_03b.zip  04b_03.zip  04b_09.zip  04b_11.zip  04b_19.zip  04b_21.zip  04b_25.zip  04b_30.zip
      source: https://www.04.jp.org/
      license: "Freeware: You may use them as you like"
    7px4bus
      source: https://www.fontspace.com/010bus
      license: Licensed as: Freeware, Commercial use allowed!
      suggested 7pt
    8pxbus
      source: https://www.fontspace.com/010bus
      license: Licensed as: Freeware, Commercial use allowed!
      suggested 8pt

*/

/*
  only supports metric set "0"
  assume DWIDTH second arg to be 0 for all glyphs
  assumes that (0,0) of the BBX is placed on the base line
  assumes ISO10646 encoding of the BDF file

  font information
  offset
  0             font format
  1             FONTBOUNDINGBOX width           unsigned
  2             FONTBOUNDINGBOX height          unsigned
  3             FONTBOUNDINGBOX x-offset         signed
  4             FONTBOUNDINGBOX y-offset        signed
  5             capital A height                                unsigned
  6             start 'A'
  8             start 'a'
  10            encoding start
  11            encoding end
  12            descent 'g'                     negative: below baseline
  13            font max ascent
  14            font min decent             negative: below baseline
  15            xascent (ascent of "(")
  16            xdescent (descent of ")")

format 0
    glyph information
    offset
    0             BBX width                                       unsigned
    1             BBX height                                      unsigned
    2             data size                                          unsigned    (BBX width + 7)/8 * BBX height
    3             DWIDTH                                          signed
    4             BBX xoffset                                    signed
    5             BBX yoffset                                    signed

format 1
  0             BBX xoffset                                    signed   --> upper 4 Bit
  0             BBX yoffset                                    signed --> lower 4 Bit
  1             BBX width                                       unsigned --> upper 4 Bit
  1             BBX height                                      unsigned --> lower 4 Bit
  2             data size                                           unsigned -(BBX width + 7)/8 * BBX height  --> lower 4 Bit
  2             DWIDTH                                          signed --> upper  4 Bit
  byte 0 == 255 indicates empty glyph

format 2
  like format 0, but 4 gray levels for the glyph (4 pixel per byte in the glyph data)

  The glyph bitmap size is defined by BBX width and BBX height
  number of bytes in the bitmap data (BBX width + 7)/8 * BBX height (format 0 and 1)

  draw_text(x,y,str)
  get_text_frame(x,y,str, &x1, &y1, &width, &height)
  frame( x1, y1, width, height)
  underline( x1, y-1, width )

  size of the surrounding bbox

  width = - xoffset(c1) + DWIDTH(c1) + DWIDTH(c2) + ... + DWIDTH(cn-1) + width(cn) + xoffset(cn)
  height = FONTBOUNDINGBOX height
  x1 = x + xoffset(c1)
  y1 = y + yoffset(c1)

ISO-8859-1 was incorporated as the first 256 code points of ISO/IEC 10646 and Unicode.

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define BDF2U8G_COMPACT_OUTPUT
#define BDF2U8G_VERSION "1.01"
//#define VERBOSE

/*=== forward declaration ===*/
void bdf_aa_ClearDoShow(void);
void bdf_aa_Do(void);

/*=== result data ===*/
#define DATA_BUF_SIZE (1024 * 64)
unsigned char data_buf[DATA_BUF_SIZE];
int data_pos = 0;

void data_Init(void) {
  data_pos = 0;
}

void data_Put(unsigned char c) {
  data_buf[data_pos] = c;
  data_pos++;
}

void data_Write(FILE *out_fp, const char *indent) {
  int i;
  int bytes_per_line = 16;
  for (i = 0; i < data_pos; i++) {
    fprintf(out_fp, "0x%02X", data_buf[i]);
    if (i + 1 != data_pos)
      fprintf(out_fp, ",");
    if ((i + 1) % bytes_per_line == 0)
      fprintf(out_fp, "\n%s", indent);
  }
}

/*=== low level parser ===*/
#define P_BUF_SIZE (1024 * 4)
int p_current_char;
const char *p_line;
char p_buf[P_BUF_SIZE];

/* put next char into p_current_char */
static int p_next_char(void) {
  p_current_char = *p_line;
  if (p_current_char == '\0')
    return 0;
  p_line++;
  return 1;
}

int p_first_char(const char *line) {
  p_line = line;
  return p_next_char();
}

void p_err(const char *msg) {
}

int p_skip_space(void) {
  for (;;) {
    if (p_current_char == 0 || p_current_char > 32)
      break;
    if (p_next_char() == 0)
      return 0;
  }
  return 1;
}

/* put identifier into p_buf */
int p_get_identifier(void) {
  int i = 0;
  if (p_current_char == '\0')
    return p_err("unexpected EOF (expected identifier)"), 0;
  for (;;) {
    if (p_current_char <= 32)
      break;
    p_buf[i++] =  p_current_char;
    if (p_next_char() == 0)
      break;
  }
  p_buf[i++] =  '\0';
  p_skip_space();
  return 1;
}

/* put identifier into p_buf */
int p_get_identifier_with_blank(void) {
  int i = 0;
  for (;;) {
    if (p_current_char < 32)
      break;
    p_buf[i++] =  p_current_char;
    if (p_next_char() == 0)
      break;
  }
  p_buf[i++] =  '\0';
  p_skip_space();
  return 1;
}

int p_get_string(void) {
  int i = 0;
  if (p_current_char == '\0')
    return 0;
  if (p_current_char != '\"')
    return p_err("\" expected"), 0;
  if (p_next_char() == 0)
    return p_err("unexpected EOF (\")"), 0;
  for (;;) {
    if (p_current_char == '\\') {
      if (p_next_char() == 0)
        return p_err("unexpected EOF (\\)"), 0;
    }
    else if (p_current_char == '\"') {
      p_next_char();
      break;
    }
    p_buf[i++] =  p_current_char;
    if (p_next_char() == 0)
      return p_err("unexpected EOF (\")"), 0;
  }
  p_buf[i] =  '\0';
  return 1;
}

int p_get_any(void) {
  if (p_current_char == '\"')
    return p_get_string();
  return p_get_identifier();
}

int p_get_val(void) {
  p_get_any();
  return atoi(p_buf);
}

int p_get_hex(void) {
  int value = 0;
  if (p_current_char >= '0' && p_current_char <= '9')
    value = p_current_char - '0';
  else if (p_current_char >= 'a' && p_current_char <= 'f')
    value = p_current_char - 'a' + 10;
  else if (p_current_char >= 'A' && p_current_char <= 'F')
    value = p_current_char - 'A' + 10;
  p_next_char();
  return value;
}

int p_get_hex_byte(void) {
  int v;
  v  = p_get_hex();
  v *= 16;
  v += p_get_hex();
  return v;
}

/*=== encoding mapping ===*/
/* the internal u8g index number (0..255) is mapped to the unicode number */
/* for the conversion we need the reverse search */
/* 0 is special and means not found */
int map_u8g_to_unicode[256];

int map_UnicodeToU8G(int unicode) {
  int i;
  for (i = 0; i < 256; i++)
    if (map_u8g_to_unicode[i] == unicode)
      return i;
  return 0;
}

void map_Init(void) {
  int i;
  map_u8g_to_unicode[0] = 0;
  for (i = 0; i < 256; i++) map_u8g_to_unicode[i] = i;
}

void map_UpperLowerPage(int lower_page, int upper_page, int shift, int upper_shift) {
  int i;
  int encoding;
  int tmp[256];
  // map_u8g_to_unicode[0] = 0;
  for (i = 0; i < 128; i++) {
    encoding              = i + lower_page * 128;
    map_u8g_to_unicode[i] = encoding;
  }

  for (i = 128; i < 256; i++) {
    encoding = i - 128 + upper_page * 128;
    if (i + upper_shift < 256)
      map_u8g_to_unicode[i + upper_shift] = encoding;
  }

  for (i = 0; i < 256; i++) tmp[i] = map_u8g_to_unicode[i];

  for (i = 0; i < shift; i++) map_u8g_to_unicode[i] = -1;

  for (i = shift; i < 256; i++) map_u8g_to_unicode[i] = tmp[(i + 256 - shift) % 256];
  /*
  printf("map_u8g_to_unicode[ 32 ] = %d\n", map_u8g_to_unicode[ 32 ]);
  printf("map_u8g_to_unicode[ 33 ] = %d\n", map_u8g_to_unicode[ 33 ]);
  */
}

/*=== store bdf file positions ===*/

long bdf_last_line_start_pos;
long bdf_encoding_pos[256];

void bdf_InitFilePos(void) {
  int i;
  for (i = 0; i < 256; i++) bdf_encoding_pos[i] = 0;
}

void bdf_SetFilePos(FILE *fp, int encoding) {
  if (encoding < 0)
    return;
  if (bdf_encoding_pos[encoding] == 0L)
    return;
  fseek(fp, bdf_encoding_pos[encoding], SEEK_SET);
  // fprintf(stderr, "setting file for encoding %d to pos %ld\n", encoding, bdf_encoding_pos[encoding]);
}

int bdf_IsEncodingAvailable(int encoding) {
  if (bdf_encoding_pos[encoding] == 0L)
    // printf("encoding %d not availabe\n", encoding);
    return 0;
  return 1;
}

void bdf_StoreFilePos(int encoding, long pos) {
  // if ( encoding == 33 )
  // printf("encoding %d at pos %ld\n", encoding, pos);
  if (bdf_encoding_pos[encoding] != 0L)
    return;
  bdf_encoding_pos[encoding] = pos;
}

/*=== bdf file read ===*/

int bdf_font_format = 0;

#define BDF_STATE_FONT_DATA 0
#define BDF_STATE_ENCODING 1

int bdf_state              = BDF_STATE_FONT_DATA;
int bdf_requested_encoding = 0;

#define BDF_LINE_MAX (1024 * 4)
#define BDF_LINE_STATE_KEYWORDS 0
#define BDF_LINE_STATE_BITMAP 1
#define BDF_MAX_HEIGHT 200
#define BDF_AA_OFFSET 1

char bdf_copyright[BDF_LINE_MAX];
char bdf_font[BDF_LINE_MAX];
unsigned char bdf_bitmap_line[BDF_MAX_HEIGHT][20];
unsigned char bdf_aa_bitmap_line[BDF_MAX_HEIGHT + 2 * BDF_AA_OFFSET][(20 + 2 * BDF_AA_OFFSET) * 8];

int bdf_line_state   = BDF_LINE_STATE_KEYWORDS;
int bdf_line_bm_line = 0;

int bdf_font_size;            /* point font size */

int bdf_font_width;           /* FONTBOUNDINGBOX arg 1 */
int bdf_font_height;          /* FONTBOUNDINGBOX arg 2 */
int bdf_font_x;               /* FONTBOUNDINGBOX arg 3 */
int bdf_font_y;               /* FONTBOUNDINGBOX arg 4 */

int bdf_capital_A_height;
int bdf_capital_1_height;
int bdf_lower_g_descent;

int bdf_char_width;           /* BBX arg 1 */
int bdf_char_max_width;
int bdf_char_height;          /* BBX arg 2 */
int bdf_char_ascent;          /* defined as BBX arg 2 + BBX arg 4 */
int bdf_char_xascent;
int bdf_char_xdescent;
int bdf_char_max_ascent;
int bdf_char_max_height;
int bdf_char_x;               /* BBX arg 3 */
int bdf_char_max_x;
int bdf_char_min_x;
int bdf_char_y;               /* BBX arg 4 */
int bdf_char_max_y;
int bdf_char_min_y;

int bdf_delta_x_default = -1;
int bdf_delta_x         = -1; /* DWIDTH arg 1 */
int bdf_delta_max_x;
int bdf_delta_min_x;
int bdf_delta_y;              /* DWIDTH arg 2 */
int bdf_delta_max_y;
int bdf_delta_min_y;

int bdf_glyph_data_len;
int bdf_glyph_data_max_len;

int bdf_encoding;

int bdf_encoding_65_pos;
int bdf_encoding_97_pos;
int bdf_is_encoding_successfully_done;

char bdf_info[32000 * 2];

int bdf_is_put_glyph_completed = 0; /* indicator, when the glyph has been processed */

void bdf_ResetMax(void) {
  bdf_char_max_width     = 0;
  bdf_char_max_height    = 0;
  bdf_char_max_x         = 0;
  bdf_char_max_y         = 0;
  bdf_delta_max_x        = 0;
  bdf_delta_max_y        = 0;
  bdf_char_min_x         = 0;
  bdf_char_min_y         = 0;
  bdf_delta_min_x        = 0;
  bdf_delta_min_y        = 0;
  bdf_glyph_data_max_len = 0;
  bdf_char_max_ascent    = 0;
  bdf_char_xascent       = 0;
  bdf_char_xdescent      = 0;
}

void bdf_UpdateMax(void) {
  if (bdf_char_max_width < bdf_char_width)
    bdf_char_max_width = bdf_char_width;
  if (bdf_char_max_height < bdf_char_height)
    bdf_char_max_height = bdf_char_height;

  if (bdf_char_max_x < bdf_char_x)
    bdf_char_max_x = bdf_char_x;

  if (bdf_char_max_y < bdf_char_y)
    bdf_char_max_y = bdf_char_y;

  if (bdf_delta_max_x < bdf_delta_x)
    bdf_delta_max_x = bdf_delta_x;
  if (bdf_delta_max_y < bdf_delta_y)
    bdf_delta_max_y = bdf_delta_y;

  if (bdf_char_min_x > bdf_char_x)
    bdf_char_min_x = bdf_char_x;

  if (bdf_char_min_y > bdf_char_y)
    bdf_char_min_y = bdf_char_y;

  if (bdf_delta_min_x > bdf_delta_x)
    bdf_delta_min_x = bdf_delta_x;
  if (bdf_delta_min_y > bdf_delta_y)
    bdf_delta_min_y = bdf_delta_y;

  if (bdf_glyph_data_max_len < bdf_glyph_data_len)
    bdf_glyph_data_max_len = bdf_glyph_data_len;

  if (bdf_char_max_ascent < bdf_char_ascent)
    bdf_char_max_ascent = bdf_char_ascent;
}

void bdf_ShowGlyph(void) {
  #ifdef VERBOSE
    int x, y, byte, bit;
    int gx, gy;
    char *p;
    gy = bdf_char_height - 1 + bdf_char_y;
    printf("bbx %d %d %d %d encoding %d\n", bdf_char_width, bdf_char_height, bdf_char_x, bdf_char_y, bdf_encoding);
    for (y = 0; y < bdf_line_bm_line; y++) {
      printf("%02d ", gy);
      gx = bdf_char_x;
      for (x = 0; x < bdf_char_width; x++) {
        byte = x >> 3;
        bit  = 7 - (x & 7);
        if ((bdf_bitmap_line[y][byte] & (1 << bit)) == 0)
          p = " .";
        else
          p = " *";
        if (gy == 0 && gx == 0)
          p = " o";
        printf("%s", p);
        gx++;
      }
      printf(" ");
      for (x = 0; x < ((bdf_char_width + 7) / 8); x++) printf( "%02X", bdf_bitmap_line[y][x]);
      gy--;
      printf("\n");
    }
  #else
    printf("bbx %d %d %d %d encoding %d\n", bdf_char_width, bdf_char_height, bdf_char_x, bdf_char_y, bdf_encoding);
    fflush(stdout);
  #endif
}

void bdf_ClearGlyphBuffer(void) {
  int x, y;
  for (y = 0; y < BDF_MAX_HEIGHT; y++)
    for (x = 0; x < 20; x++) bdf_bitmap_line[y][x] = 0;
}

void bdf_PutGlyph(void) {
  int len;
  int y, x;

  if (bdf_state == BDF_STATE_ENCODING) {
    //if (bdf_char_width == 0 && bdf_char_height == 0) bdf_char_y = 0;

    bdf_char_ascent = bdf_char_height + bdf_char_y;
    //printf("h:%d w:%d ascent: %d\n", bdf_char_height, bdf_char_width, bdf_char_ascent);

    if (bdf_encoding == 'A')
      bdf_capital_A_height = bdf_char_height;
    if (bdf_encoding == '1')
      bdf_capital_1_height = bdf_char_height;

    if (bdf_encoding == 'g')
      bdf_lower_g_descent = bdf_char_y;

    if (bdf_char_xascent < bdf_capital_A_height)
      bdf_char_xascent = bdf_capital_A_height;
    if (bdf_char_xascent < bdf_capital_1_height)
      bdf_char_xascent = bdf_capital_1_height;
    if (bdf_encoding == '(')
      if (bdf_char_xascent < bdf_char_ascent)
        bdf_char_xascent = bdf_char_ascent;
    if (bdf_encoding == '[')
      if (bdf_char_xascent < bdf_char_ascent)
        bdf_char_xascent = bdf_char_ascent;

    if (bdf_char_xdescent > bdf_lower_g_descent)
      bdf_char_xdescent = bdf_lower_g_descent;
    if (bdf_encoding == '(')
      if (bdf_char_xdescent > bdf_char_y)
        bdf_char_xdescent = bdf_char_y;
    if (bdf_encoding == '[')
      if (bdf_char_xdescent > bdf_char_y)
        bdf_char_xdescent = bdf_char_y;

    if (bdf_requested_encoding != bdf_encoding)
      return;

    assert( bdf_line_bm_line == bdf_char_height);

    bdf_ShowGlyph();
    #ifdef VERBOSE
      bdf_aa_ClearDoShow();
    #endif

    bdf_UpdateMax();

    if (bdf_font_format <= 1)
      len = (bdf_char_width + 7) / 8 * bdf_char_height;
    else
      len = (bdf_char_width + 2 * BDF_AA_OFFSET + 3) / 4 * (bdf_char_height + 2 * BDF_AA_OFFSET);
    if (len > 255) {
      fprintf(stderr, "Glyph with encoding %d is too large (%d > 255)\n", bdf_encoding, len);
      exit(1);
    }

    bdf_glyph_data_len = len;

    /*
      format 0 and format 2
      glyph information
      offset
      0             BBX width                                       unsigned
      1             BBX height                                      unsigned
      2             data size                                          unsigned    (BBX width + 7)/8 * BBX height
      3             DWIDTH                                          signed
      4             BBX xoffset                                    signed
      5             BBX yoffset                                    signed
    */

    if (bdf_font_format == 0) {
      data_Put(bdf_char_width);
      data_Put(bdf_char_height);
      data_Put(bdf_glyph_data_len);
      data_Put(bdf_delta_x);
      data_Put(bdf_char_x);
      data_Put(bdf_char_y);
      // data_Put(bdf_encoding);
      bdf_is_encoding_successfully_done = 1;
    }
    else if (bdf_font_format == 2) {
      data_Put(bdf_char_width + 2 * BDF_AA_OFFSET);
      data_Put(bdf_char_height + 2 * BDF_AA_OFFSET);
      data_Put(bdf_glyph_data_len);
      data_Put(bdf_delta_x);
      data_Put(bdf_char_x - BDF_AA_OFFSET);
      data_Put(bdf_char_y - BDF_AA_OFFSET);
      // data_Put(bdf_encoding);
      bdf_is_encoding_successfully_done = 1;
    }
    else {
      /**
       * format 1
       *   0             BBX xoffset         signed   --> upper 4 Bit
       *   0             BBX yoffset         signed --> lower 4 Bit
       *   1             BBX width           unsigned --> upper 4 Bit
       *   1             BBX height          unsigned --> lower 4 Bit
       *   2             data size           unsigned -(BBX width + 7)/8 * BBX height  --> lower 4 Bit
       *   2             DWIDTH              signed --> upper  4 Bit
       *   byte 0 == 255 indicates empty glyph
       */
      if (bdf_glyph_data_len < 0 || bdf_glyph_data_len > 15) {
        fprintf(stderr, "Glyph with encoding %d does not fit for format 1 (data len = %d)\n", bdf_encoding, bdf_glyph_data_len);
        exit(1);
      }
      if (bdf_delta_x < 0 || bdf_delta_x > 15) {
        fprintf(stderr, "Glyph with encoding %d does not fit for format 1 (DWIDTH = %d)\n", bdf_encoding, bdf_delta_x);
        exit(1);
      }
      if (bdf_char_x < 0 || bdf_char_x > 15) {
        fprintf(stderr, "Glyph with encoding %d does not fit for format 1 (x-off = %d)\n", bdf_encoding, bdf_char_x);
        exit(1);
      }
      if (bdf_char_y < -2 || bdf_char_y > 13) {
        fprintf(stderr, "Glyph with encoding %d does not fit for format 1 (y-off = %d [%d..%d])\n", bdf_encoding, bdf_char_y, bdf_char_min_y, bdf_char_max_y);
        exit(1);
      }
      if (bdf_char_width < 0 || bdf_char_width > 15) {
        fprintf(stderr, "Glyph with encoding %d does not fit for format 1 (width = %d)\n", bdf_encoding, bdf_char_width);
        exit(1);
      }
      if (bdf_char_height < 0 || bdf_char_height > 15) {
        fprintf(stderr, "Glyph with encoding %d does not fit for format 1 (height = %d)\n", bdf_encoding, bdf_char_height);
        exit(1);
      }
      // data_Put(bdf_encoding);
      if (((bdf_char_x << 4) | (bdf_char_y + 2)) == 255) {
        fprintf(stderr, "Glyph with encoding %d does not fit for format 1 (skip mark generated)\n", bdf_encoding);
        exit(1);
      }
      data_Put((bdf_char_x << 4) | (bdf_char_y + 2));
      data_Put((bdf_char_width << 4) | bdf_char_height );
      data_Put((bdf_delta_x << 4) | bdf_glyph_data_len );
      bdf_is_encoding_successfully_done = 1;
    }

    sprintf(bdf_info + strlen(bdf_info), "/* encoding %d %c, bbx %d %d %d %d  asc %d dx %d*/\n",
      bdf_encoding,
      bdf_encoding > 32 && bdf_encoding <= 'z' ? bdf_encoding : ' ',
      bdf_char_width,
      bdf_char_height,
      bdf_char_x,
      bdf_char_y,
      bdf_char_ascent,
      bdf_delta_x);

    if (bdf_font_format <= 1) {
      for (y = 0; y < bdf_char_height; y++)
        for (x = 0; x < ((bdf_char_width + 7) / 8); x++) {
          data_Put(bdf_bitmap_line[y][x]);
          len--;
        }
      assert( len == 0 );
      bdf_is_put_glyph_completed = 1;
    }
    else { /* format == 2 */
      int b, cnt;
      bdf_aa_Do();
      for (y = 0; y < bdf_char_height + 2 * BDF_AA_OFFSET; y++) {
        b   = 0;
        cnt = 0;
        for (x = 0; x < bdf_char_width + 2 * BDF_AA_OFFSET; x++) {
          b <<= 2;
          b  |= bdf_aa_bitmap_line[y][x] & 3;
          cnt++;
          if (cnt == 4) {
            data_Put(b);
            b   = 0;
            cnt = 0;
            len--;
          }
        }

        if (cnt != 0) {
          b <<= 2 * (4 - cnt);
          data_Put(b);
          b   = 0;
          cnt = 0;
          len--;
        }
      }
      assert( len == 0 );
    }
  }
}

/*=========================================================================*/
/* Glyph Clipping */

int bdf_IsColZero(int x) {
  int y, byte, bit;
  for (y = 0; y < bdf_char_height; y++) {
    byte = x >> 3;
    bit  = 7 - (x & 7);
    if ((bdf_bitmap_line[y][byte] & (1 << bit)) != 0)
      return 0;
  }
  return 1;
}

int bdf_IsRowZero(int y) {
  int x, byte, bit;
  for (x = 0; x < bdf_char_width; x++) {
    byte = x >> 3;
    bit  = 7 - (x & 7);
    if ((bdf_bitmap_line[y][byte] & (1 << bit)) != 0)
      return 0;
  }
  return 1;
}

void bdf_DeleteFirstCol(void) {
  int m = (bdf_char_width + 7) / 8;
  int x, y;
  for (y = 0; y < bdf_char_height; y++)
    for (x = 0; x < m; x++) {
      bdf_bitmap_line[y][x] <<= 1;
      bdf_bitmap_line[y][x]  |= bdf_bitmap_line[y][x + 1] >> 7;
    }
}

void bdf_DeleteFirstRow(void) {
  int m = (bdf_char_width + 7) / 8;
  int x, y;
  for (y = 0; y + 1 < bdf_char_height; y++)
    for (x = 0; x < m; x++)
      bdf_bitmap_line[y][x] = bdf_bitmap_line[y + 1][x];
}

void bdf_ReduceGlyph(void) {
  while (bdf_char_width > 0) {
    if (bdf_IsColZero(bdf_char_width - 1) == 0)
      break;
    bdf_char_width--;
  }

  while (bdf_char_height > 0) {
    if (bdf_IsRowZero(bdf_char_height - 1) == 0)
      break;
    bdf_line_bm_line--;
    bdf_char_height--;
    bdf_char_y++;
  }

  while (bdf_IsColZero(0) != 0 && bdf_char_width > 0) {
    bdf_DeleteFirstCol();
    bdf_char_x++;
    bdf_char_width--;
  }

  while (bdf_char_height > 0) {
    if (bdf_IsRowZero(0) == 0)
      break;
    bdf_DeleteFirstRow();
    bdf_line_bm_line--;
    bdf_char_height--;
  }
}

/*=========================================================================*/
/* Anti Aliasing / Graylevel Glyph */

int bdf_GetXYVal(int x, int y) {
  int byte, bit;

  if (x < 0) return 0;
  if (y < 0) return 0;
  if (x >= bdf_char_width) return 0;
  if (y >= bdf_char_height) return 0;

  byte = x >> 3;
  bit  = 7 - (x & 7);
  if ((bdf_bitmap_line[y][byte] & (1 << bit)) != 0) return 1;
  return 0;
}

void bdf_aa_Clear(void) {
  int x, y;
  for (y = 0; y < BDF_MAX_HEIGHT + 2 * BDF_AA_OFFSET; y++)
    for (x = 0; x < (20 + 2 * BDF_AA_OFFSET) * 8; x++)
      bdf_aa_bitmap_line[y][x] = 0;
}

void bdf_aa_SetXYVal(int x, int y, int val) {
  bdf_aa_bitmap_line[y][x] = val;
}

int bdf_aa_matrix[9] = {
  1, 3, 1,
  3, 4, 3,
  1, 3, 1
};
int bdf_aa_sum = 20;
int bdf_aa_gray_levels = 4;

void bdf_aa_Do(void) {
  int x, y, val, sx, sy, sum, gray;
  bdf_aa_Clear();
  for (y = 0; y < bdf_char_height + 2 * BDF_AA_OFFSET; y++)
    for (x = 0; x < bdf_char_width + 2 * BDF_AA_OFFSET; x++) {
      if (bdf_GetXYVal(x - BDF_AA_OFFSET, y - BDF_AA_OFFSET) == 0) {
        sum = 0;
        for (sy = -BDF_AA_OFFSET; sy <= BDF_AA_OFFSET; sy++)
          for (sx = -BDF_AA_OFFSET; sx <= BDF_AA_OFFSET; sx++) {
            val  = bdf_GetXYVal(x + sx - BDF_AA_OFFSET, y + sy - BDF_AA_OFFSET);
            val *= bdf_aa_matrix[(sy + BDF_AA_OFFSET) * (2 * BDF_AA_OFFSET + 1) + sx + BDF_AA_OFFSET];
            sum += val;
          }
        if (sum <= 5)
          gray = 0;
        else
          gray = (sum * (bdf_aa_gray_levels - 1) + (bdf_aa_sum / 2)) / bdf_aa_sum;
        if (gray >= bdf_aa_gray_levels)
          gray = bdf_aa_gray_levels - 1;
      }
      else {
        gray = bdf_aa_gray_levels - 1;
      }
      bdf_aa_SetXYVal(x, y, gray);
    }
}

void bdf_aa_Show(void) {
  int x, y;
  if (bdf_font_format == 2) {
    for (y = 0; y < bdf_char_height + 2 * BDF_AA_OFFSET; y++) {
      for (x = 0; x < bdf_char_width + 2 * BDF_AA_OFFSET; x++)
        switch (bdf_aa_bitmap_line[y][x]) {
          case 0: printf("."); break;
          case 1: printf("-"); break;
          case 2: printf("+"); break;
          case 3: printf("#"); break;
        }
      printf("\n");
    }
  }
}

void bdf_aa_ClearDoShow(void) {
  bdf_aa_Do();
  bdf_aa_Show();
}

/*=========================================================================*/
/* Parser */

void bdf_ReadLine(const char *s) {
  /*
  if ( bdf_line_state == BDF_LINE_STATE_BITMAP && bdf_requested_encoding != bdf_encoding && *s != 'E' )
    return;
  */

  if (p_first_char(s) == 0) return;
  if (p_skip_space() == 0) return;

  if (bdf_line_state == BDF_LINE_STATE_KEYWORDS) {
    p_get_identifier();
    if (strcmp(p_buf, "COPYRIGHT") == 0) {
      p_get_any();
      strcpy(bdf_copyright, p_buf);
    }
    else if (strcmp(p_buf, "FONT") == 0) {
      /* p_get_any(); */
      p_get_identifier_with_blank();
      strcpy(bdf_font, p_buf);
    }
    else if (strcmp(p_buf, "SIZE") == 0) {
      bdf_font_size = p_get_val();
    }
    else if (strcmp(p_buf, "ENCODING") == 0) {
      bdf_encoding = map_UnicodeToU8G(p_get_val());
      bdf_StoreFilePos(bdf_encoding, bdf_last_line_start_pos);
    }
    else if (strcmp(p_buf, "DWIDTH") == 0) {
      bdf_delta_x = p_get_val();
      bdf_delta_y = p_get_val();
    }
    else if (strcmp(p_buf, "FONTBOUNDINGBOX") == 0) {
      bdf_font_width  = p_get_val();
      bdf_font_height = p_get_val();
      bdf_font_x      = p_get_val();
      bdf_font_y      = p_get_val();
    }
    else if (strcmp(p_buf, "BBX") == 0) {
      bdf_char_width  = p_get_val();
      bdf_char_height = p_get_val();
      bdf_char_x      = p_get_val();
      bdf_char_y      = p_get_val();
      bdf_char_ascent = bdf_char_height + bdf_char_y;
      // printf("h:%d w:%d ascent: %d\n", bdf_char_height, bdf_char_width, bdf_char_ascent);
    }
    else if (strcmp(p_buf, "CHARS") == 0) {
      if (bdf_delta_x < 0)
        bdf_delta_x = 0;
      if (bdf_delta_x_default < 0)
        bdf_delta_x_default = bdf_delta_x;
    }
    else if (strcmp(p_buf, "STARTCHAR") == 0) {
      if (bdf_delta_x_default < 0)
        bdf_delta_x_default = 0;
      bdf_delta_x = bdf_delta_x_default;
    }
    else if (strcmp(p_buf, "BITMAP") == 0) {
      bdf_line_state   = BDF_LINE_STATE_BITMAP;
      bdf_line_bm_line = 0;
    }
  }
  else if (bdf_line_state == BDF_LINE_STATE_BITMAP) {
    if (strncmp(s, "ENDCHAR", 7) == 0) {
      bdf_ReduceGlyph();
      bdf_PutGlyph();
      bdf_line_state   = BDF_LINE_STATE_KEYWORDS;
      bdf_line_bm_line = 0;
    }
    else if (bdf_requested_encoding == bdf_encoding)  {
      int i = 0;
      for (;;) {
        if (p_current_char < '0') break;
        bdf_bitmap_line[bdf_line_bm_line][i] = p_get_hex_byte();
        i++;
      }
      bdf_line_bm_line++;
      assert(bdf_line_bm_line < BDF_MAX_HEIGHT);
    }
  }
}

int bdf_ReadFP(FILE *fp) {
  static char bdf_line[BDF_LINE_MAX];
  bdf_is_put_glyph_completed = 0;
  for (;;) {
    bdf_last_line_start_pos = ftell(fp);
    if (fgets(bdf_line, BDF_LINE_MAX - 1, fp) == NULL)
      break;
    bdf_ReadLine(bdf_line);
    if (bdf_is_put_glyph_completed != 0)
      break;
  }
  return 1;
}

int bdf_ReadFile(const char *filename, int encoding) {
  int r;
  FILE *fp;
  fp = fopen(filename, "rb");
  if (fp != NULL) {
    bdf_SetFilePos(fp, encoding);
    r = bdf_ReadFP(fp);
    fclose(fp);
    return r;
  }
  return 0; /* open error */
}

void bdf_GenerateFontData(const char *filename, int begin, int end) {
  bdf_state = BDF_STATE_FONT_DATA;
  bdf_ReadFile(filename, -1);
  /**
   * font information
   *
   * offset
   *   0          font format
   *   1          FONTBOUNDINGBOX width         unsigned
   *   2          FONTBOUNDINGBOX height        unsigned
   *   3          FONTBOUNDINGBOX x-offset      signed
   *   4          FONTBOUNDINGBOX y-offset      signed
   *   5          Capital A Height              unsigned
   *   6          position of encoding 65 'A'   high byte first
   *   8          position of encoding 97 'a'   high byte first
   */
  data_Put(bdf_font_format);
  data_Put(bdf_font_width);
  data_Put(bdf_font_height);
  data_Put(bdf_font_x);
  data_Put(bdf_font_y);
  data_Put(bdf_capital_A_height > 0 ? bdf_capital_A_height : bdf_capital_1_height);
  data_Put(0);
  data_Put(0);
  data_Put(0);
  data_Put(0);
  data_Put(begin);
  data_Put(end);                /* will be overwritten later */
  data_Put(0);                  /* lower g descent */
  data_Put(0);                  /* max ascent */
  data_Put(0);                  /* min y = descent */
  data_Put(0);                  /* x ascent */
  data_Put(0);                  /* x descent */
}

void bdf_GenerateGlyph(const char *filename, int encoding) {
  bdf_ClearGlyphBuffer();
  bdf_requested_encoding = encoding;
  bdf_state = BDF_STATE_ENCODING;
  bdf_ReadFile(filename, encoding);
}

void bdf_Generate(const char *filename, int begin, int end) {
  int i;
  int last_valid_encoding;

  bdf_encoding_65_pos = 0;
  bdf_encoding_97_pos = 0;

  bdf_InitFilePos();
  bdf_ResetMax();
  bdf_info[0]      = '\0';
  bdf_font[0]      = '\0';
  bdf_copyright[0] = '\0';

  bdf_GenerateFontData(filename, begin, end);
  for (i = begin; i <= end; i++) {

    if (i == 65) bdf_encoding_65_pos = data_pos;
    if (i == 97) bdf_encoding_97_pos = data_pos;

    bdf_is_encoding_successfully_done = 0;
    if (bdf_IsEncodingAvailable(i))
      bdf_GenerateGlyph(filename, i);
    if (bdf_is_encoding_successfully_done == 0)
      data_Put(255);          /* no char encoding */
    if (bdf_is_encoding_successfully_done != 0)
      last_valid_encoding = i;
  }
  /* data_Put(255); obsolete, not required any more for format 0 */         /* encoding 255, end of font data (format 0) */

  data_buf[5] = bdf_capital_A_height > 0 ? bdf_capital_A_height : bdf_capital_1_height;

  data_buf[6] = (bdf_encoding_65_pos >> 8);
  data_buf[7] = (bdf_encoding_65_pos & 255);
  data_buf[8] = (bdf_encoding_97_pos >> 8);
  data_buf[9] = (bdf_encoding_97_pos & 255);

  data_buf[12] = bdf_lower_g_descent;
  data_buf[13] = bdf_char_max_ascent;
  data_buf[14] = bdf_char_min_y;
  data_buf[15] = bdf_char_xascent;
  data_buf[16] = bdf_char_xdescent;

  if (0) data_buf[11] = last_valid_encoding;

}

void bdf_WriteC(const char *outname, const char *fontname) {
  int capital_ascent;
  FILE *out_fp;
  out_fp = fopen(outname, "wb");
  assert( out_fp != NULL );

  capital_ascent = bdf_capital_A_height > 0 ? bdf_capital_A_height : bdf_capital_1_height;

  fprintf(out_fp, "/**\n");
  fprintf(out_fp, " * Fontname: %s\n", bdf_font);
  fprintf(out_fp, " * Copyright: %s\n", bdf_copyright);
  fprintf(out_fp, " * Capital A Height: %d, '1' Height: %d\n", bdf_capital_A_height, bdf_capital_1_height);
  fprintf(out_fp, " * Calculated Max Values w=%2d h=%2d x=%2d y=%2d dx=%2d dy=%2d ascent=%2d len=%2d\n",
    bdf_char_max_width, bdf_char_max_height, bdf_char_max_x, bdf_char_max_y, bdf_delta_max_x, bdf_delta_max_y,
    bdf_char_max_ascent, bdf_glyph_data_max_len);
  fprintf(out_fp, " * Font Bounding box     w=%2d h=%2d x=%2d y=%2d\n",
    bdf_font_width, bdf_font_height, bdf_font_x, bdf_font_y);
  fprintf(out_fp, " * Calculated Min Values           x=%2d y=%2d dx=%2d dy=%2d\n",
    bdf_char_min_x, bdf_char_min_y, bdf_delta_min_x, bdf_delta_min_y);

  fprintf(out_fp, " * Pure Font   ascent =%2d descent=%2d\n", capital_ascent, bdf_lower_g_descent);
  fprintf(out_fp, " * X Font      ascent =%2d descent=%2d\n", bdf_char_xascent, bdf_char_xdescent);
  fprintf(out_fp, " * Max Font    ascent =%2d descent=%2d\n", bdf_char_max_ascent, bdf_char_min_y);

  fprintf(out_fp, " */\n");
  fprintf(out_fp, "const u8g_fntpgm_uint8_t %s[%d] U8G_FONT_SECTION(\"%s\") = {\n", fontname, data_pos, fontname);
  fprintf(out_fp, "  ");
  data_Write(out_fp, "  ");
  fprintf(out_fp, "};\n");
  #ifndef BDF2U8G_COMPACT_OUTPUT
    fprintf(out_fp, "%s\n", bdf_info);
  #endif

  fclose(out_fp);
}

int ga_argc;
char **ga_argv;

void ga_remove_arg(void) {
  if (ga_argc == 0) return;
  ga_argc--;
  ga_argv++;
}

int ga_is_arg(char opt) {
  if (ga_argc == 0) return 0;
  if (ga_argv[0] == NULL) return 0;
  if (ga_argv[0][0] != '-') return 0;
  if (ga_argv[0][1] != opt) return 0;
  ga_remove_arg();
  return 1;
}

int main(int argc, char **argv) {
  int lower_page          = 0;
  int upper_page          = 1;
  int mapping_shift       = 0;
  int upper_mapping_shift = 0;
  int begin               = 32;
  int end                 = 255;

  if (argc < 4) {
    printf("bdf to u8glib font format converter v" BDF2U8G_VERSION "\n");
    printf("%s [-l page] [-u page] [-s shift] [-S upper-shift] [-b begin] [-e end] [-f format] fontfile fontname outputfile\n", argv[0]);
    return 1;
  }

  ga_argc = argc;
  ga_argv = argv;
  ga_remove_arg();              /* remove program name */

  for (;;) {
    if (ga_is_arg('l')) {
      lower_page = atoi(ga_argv[0]);
      ga_remove_arg();
    }
    else if (ga_is_arg('u')) {
      upper_page = atoi(ga_argv[0]);
      ga_remove_arg();
    }
    else if (ga_is_arg('s')) {
      mapping_shift = atoi(ga_argv[0]);
      ga_remove_arg();
    }
    else if (ga_is_arg('S')) {
      upper_mapping_shift = atoi(ga_argv[0]);
      ga_remove_arg();
    }
    else if (ga_is_arg('b')) {
      begin = atoi(ga_argv[0]);
      ga_remove_arg();
    }
    else if (ga_is_arg('e')) {
      end = atoi(ga_argv[0]);
      ga_remove_arg();
    }
    else if (ga_is_arg('f')) {
      bdf_font_format = atoi(ga_argv[0]);
      ga_remove_arg();
    }
    else {
      break;
    }
  }

  printf("encoding range %d..%d\n", begin, end);

  data_Init();
  map_Init();
  map_UpperLowerPage(lower_page, upper_page, mapping_shift, upper_mapping_shift);

  /*
  puts(bdf_font);
  puts(bdf_copyright);
  if (ga_argc < 3) {
    printf("from page %d to page %d\n", lower_page, upper_page);
    return 1;
  }
  */

  bdf_Generate(ga_argv[0], begin, end);
  bdf_WriteC(ga_argv[2], ga_argv[1]);

  printf("input file '%s'\n", ga_argv[0]);
  printf("u8g font name '%s'\n", ga_argv[1]);
  printf("output file '%s'\n", ga_argv[2]);

  return 0;
}
