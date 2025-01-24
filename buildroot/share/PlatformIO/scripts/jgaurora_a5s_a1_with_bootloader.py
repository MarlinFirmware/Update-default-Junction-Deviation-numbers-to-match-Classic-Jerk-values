#
# jgaurora_a5s_a1_with_bootloader.py
# Customizations for env:jgaurora_a5s_a1
#
import pioutil
if pioutil.is_pio_build():

    # Append ${PROGNAME}.bin firmware after bootloader and save it as 'jgaurora_firmware.bin'
    def addboot(source, target, env):
        from pathlib import Path

        fw_path = Path(target[0].path)
        fwb_path = fw_path.parent / 'firmware_with_bootloader.bin'
        with fwb_path.open("wb") as fwb_file:
            bl_path = Path("buildroot/share/PlatformIO/scripts/jgaurora_bootloader.bin")
            bl_file = bl_path.open("rb")
            while True:
                b = bl_file.read(1)
                if b == b'': break
                else: fwb_file.write(b)

            with fw_path.open("rb") as fw_file:
                while True:
                    b = fw_file.read(1)
                    if b == b'': break
                    else: fwb_file.write(b)

        fws_path = Path(target[0].dir.path, 'firmware_for_sd_upload.bin')
        if fws_path.exists():
            fws_path.unlink()

        fw_path.rename(fws_path)

    import marlin
    marlin.add_post_action(addboot)
