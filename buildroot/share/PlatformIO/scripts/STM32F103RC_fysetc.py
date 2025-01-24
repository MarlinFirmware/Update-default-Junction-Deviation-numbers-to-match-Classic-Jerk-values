#
# STM32F103RC_fysetc.py
#
import pioutil
if pioutil.is_pio_build():
    from os.path import join, expandvars
    env = pioutil.env

    # Custom HEX from ELF
    env.AddPostAction(
        join("$BUILD_DIR", "${PROGNAME}.elf"),
        env.VerboseAction(" ".join([
            "$OBJCOPY", "-O ihex", "$TARGET",
            "\"" + join("$BUILD_DIR", "${PROGNAME}.hex") + "\"", # Note: $BUILD_DIR is a full path
        ]), "Building $TARGET"))

    # In-line command with arguments
    UPLOAD_TOOL="stm32flash"
    platform = env.PioPlatform()
    if platform.get_package_dir("tool-stm32duino") != None:
        UPLOAD_TOOL=expandvars("\"" + join(platform.get_package_dir("tool-stm32duino"),"stm32flash","stm32flash") + "\"")

    env.Replace(
        UPLOADER=UPLOAD_TOOL,
        UPLOADCMD=expandvars(UPLOAD_TOOL + " -v -i rts,-dtr,dtr -R -b 115200 -g 0x8000000 -w \"" + join("$BUILD_DIR","${PROGNAME}.hex")+"\"" + " $UPLOAD_PORT")
    )
