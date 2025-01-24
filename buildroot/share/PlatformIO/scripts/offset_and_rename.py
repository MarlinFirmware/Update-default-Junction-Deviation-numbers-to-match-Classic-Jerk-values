#
# offset_and_rename.py
#
# - If 'board_build.offset' is provided, either by JSON or by the environment...
#   - Set linker flag LD_FLASH_OFFSET and relocate the VTAB based on 'build.offset'.
#   - Set linker flag LD_MAX_DATA_SIZE based on 'build.maximum_ram_size'.
#   - Define STM32_FLASH_SIZE from 'upload.maximum_size' for use by Flash-based EEPROM emulation.
#
# - For 'board_build.rename' add a post-action to rename the firmware file.
#
import pioutil
if pioutil.is_pio_build():
    import marlin

    env = marlin.env
    board = env.BoardConfig()
    board_keys = board.get("build").keys()

    #
    # For build.offset define LD_FLASH_OFFSET, used by ldscript.ld
    #
    if 'offset' in board_keys:
        LD_FLASH_OFFSET = board.get("build.offset")
        marlin.relocate_vtab(LD_FLASH_OFFSET)

        # Flash size
        maximum_flash_size = board.get("upload.maximum_size") // 1024
        marlin.replace_define('STM32_FLASH_SIZE', maximum_flash_size)

        # Get upload.maximum_ram_size (defined by /buildroot/share/PlatformIO/boards/VARIOUS.json)
        maximum_ram_size = board.get("upload.maximum_ram_size")

        for i, flag in enumerate(env["LINKFLAGS"]):
            if "-Wl,--defsym=LD_FLASH_OFFSET" in flag:
                env["LINKFLAGS"][i] = "-Wl,--defsym=LD_FLASH_OFFSET=" + LD_FLASH_OFFSET
            if "-Wl,--defsym=LD_MAX_DATA_SIZE" in flag:
                env["LINKFLAGS"][i] = "-Wl,--defsym=LD_MAX_DATA_SIZE=" + str(maximum_ram_size - 40)

    #
    # For build.encrypt_mks rename and encode the firmware file.
    #
    if 'encrypt_mks' in board_keys:

        # Encrypt ${PROGNAME}.bin and save it with the name given in build.encrypt_mks
        def encrypt(source, target, env):
            marlin.encrypt_mks(source, target, env, board.get("build.encrypt_mks"))

        if board.get("build.encrypt_mks") != "":
            marlin.add_post_action(encrypt)

    #
    # For build.rename simply rename the firmware file.
    #
    if 'rename' in board_keys:

        # If FIRMWARE_BIN is defined by config, override all
        mf = env["MARLIN_FEATURES"]
        if "FIRMWARE_BIN" in mf: new_name = mf["FIRMWARE_BIN"]
        else: new_name = board.get("build.rename")

        def rename_target(source, target, env):
            from pathlib import Path
            from datetime import datetime
            from os import path
            _newpath = Path(target[0].dir.path, datetime.now().strftime(new_name.replace('{date}', '%Y%m%d').replace('{time}', '%H%M%S')))
            Path(target[0].path).replace(_newpath)
            env['PROGNAME'] = path.splitext(_newpath)[0]

        marlin.add_post_action(rename_target)
