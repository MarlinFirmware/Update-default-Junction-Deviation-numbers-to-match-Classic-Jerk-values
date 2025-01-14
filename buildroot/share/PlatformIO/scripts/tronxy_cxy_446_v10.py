#
# tronxy_cxy_446_v10.py
# Build customizations for env:TRONXY_CXY_446_V10
#
import pioutil
if pioutil.is_pio_build():

    import marlin, os

    env = pioutil.env

    # Check whether the "update" folder exists
    outpath = f"{env['PROJECT_BUILD_DIR']}/{env['PIOENV']}/update"
    if not os.path.exists(outpath): os.makedirs(outpath)

    # Build "fmw_tronxy.hex" and place in "update" folder
    def output_target_hex():
        hex_path  = f"update/fmw_tronxy.hex"
        hex_long = f"$PROJECT_BUILD_DIR/$PIOENV/{hex_path}"
        env.AddPostAction(
            "$BUILD_DIR/${PROGNAME}.elf",
            env.VerboseAction(" ".join([
                "$OBJCOPY", "-O", "ihex", "-R", ".eeprom",
                "$BUILD_DIR/${PROGNAME}.elf", hex_long
            ]), f"Building {hex_path}")
        )

    # Build "fmw_tronxy.bin" and place in "update" folder
    def output_target_bin():
        bin_path  = f"update/fmw_tronxy.bin"
        bin_long = f"$PROJECT_BUILD_DIR/$PIOENV/{bin_path}"
        env.AddPostAction(
            "$BUILD_DIR/${PROGNAME}.elf",
            env.VerboseAction(" ".join([
                "$OBJCOPY", "-O", "binary", "-R", ".eeprom",
                "$BUILD_DIR/${PROGNAME}.elf", bin_long
            ]), f"Building {bin_path}")
        )

    output_target_hex()
    output_target_bin()
