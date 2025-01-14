## Language engine and tools

### Supported hardware

Marlin supports HD44780 character LCD and 128x64 graphical LCD via U8GLIB. Because of the limitation of HD44780 hardwares, Marlin can only support three character sets for that hardware: Japanese (kana_utf8), Russian/Cyrillic (ru), or Western (Roman characters)

For the graphical LCD modules, there's far less limitation. Users and language maintainers can easily change the language translation strings when saved with UTF-8 encoding. The only extra step is to regenerate the font data from an existing BDF font file.

### How to support a new language?

1. Prepare the translation source file

You need to add a language header file `language_xx.h`, replacing xx with the language/country code. (e.g., "en" for English). You can start with a existing language file by copying it to a new location:

```bash
cp language_zh_CN.h language_zh_TW.h
```

Then simply replace the strings inside the `_UxGT()` macros.

Make sure the file is saved as UTF-8 encoded.

2. Generate the font data file

This step gathers the glyphs used in the language file from a 9pt bitmap font and saves the data in a language_data_xx.h file. So the font used for conversion should contain all the characters in your language file.

The default bitmap font is WQY's 9pt bitmap font.

Download from [here](http://wenq.org/daily/wqy-bitmapfont-bdf-gb18030-nightly_build.tar.gz), or to install in a Debian/Ubuntu system:

```
sudo apt-get install xfonts-wqy
```

You also need to compile the `bdf2u8g` binary to convert BDF font files into U8glib data structures:

```bash
cd marlin-git/buildroot/share/fonts/
./get-bdf2u8g.sh
```

The `genallfont.sh` script generates font data for all language translation files.

You may specify a list of languages to process. For example:

```bash
MARLIN_LANGS="zh_CN zh_TW"
```

and run the script to generate the font data (`language_data_xx.h`):

```bash
cd marlin-git/Marlin/
MARLIN_LANGS="zh_CN zh_TW" ../buildroot/share/fonts/genallfont.sh
```

3. Change the language settings

To compile Marlin with your language, choose a language in `Configuration.h`. For Chinese (Taiwan) you would use:

```cpp
#define LCD_LANGUAGE zh_TW
```

4. Compile and Upload the firmware

Open `Marlin.ino` in your IDE and compile the firmware. Once the build succeeds, upload it to your board.

### Update the language translation

Whenever language files are changed, you need to run the script `genallfont.sh` again to update the font data file.

### Use a cool font

You may need to use a different font to support your own language, because the default 9pt font is not complete. (You may also support them by adding the missing glyphs to the font.)

After you've prepared your font, specify the font file path as an argument to `genallfont.sh`, so that the font used for your language is your new font. For example, if your font is named `newfont.bdf` run the following command:

```bash
cd Marlin/
../buildroot/share/fonts/genallfont.sh ./newfont.bdf
```

...or to regenerate the language font data for a specific language:

```bash
MARLIN_LANGS="zh_TW" ../buildroot/share/fonts/genallfont.sh ./newfont.bdf
```

### Suggestions for Maintainers

The tool and the language engine can be easily updated. Since it uses common bitmap font files and UTF-8 text, the maintainer needs to confirm that the font contains the glyphs in the language files.

At this time, the font file `marlin-6x12-3.bdf` is used to generate the font data. It combines all of Marlin's ISO10646-1 fonts and the WQY 9pt bitmap font.

### Related resources

#### Fonts

- [WQY](http://wenq.org/)
- [WQY 9pt bitmap font](http://wenq.org/daily/wqy-bitmapfont-bdf-gb18030-nightly_build.tar.gz)
- [unifont (16x8 or 16x16)](https://unifoundry.com/unifont.html)

Documents related to the old version of the language engine:

- [Marlin Fonts Documentation](https://marlinfw.org/docs/development/fonts.html)
- [Marlin LCD Language](https://marlinfw.org/docs/development/lcd_language.html)
- [U8GLIB](https://github.com/olikraus/u8glib.git)
- [UTF-8 for U8GLIB](https://github.com/yhfudev/u8glib-fontutf8.git)
- [Standalone test project for the Marlin UTF-8 language engine](https://github.com/yhfudev/marlin-fontutf8.git)
