#
# Convert the ELF to an SREC file suitable for some bootloaders
#
import pioutil
if pioutil.is_pio_build():
    from os.path import join
    env = pioutil.env

    board = env.BoardConfig()
    board_keys = board.get("build").keys()
    if 'encode' in board_keys:
        env.AddPostAction(
            join("$BUILD_DIR", "${PROGNAME}.bin"),
            env.VerboseAction(" ".join([
                "$OBJCOPY", "-O", "srec",
                "\"$BUILD_DIR/${PROGNAME}.elf\"", "\"" + join("$BUILD_DIR", board.get("build.encode")) + "\""
            ]), "Building " + board.get("build.encode"))
        )
