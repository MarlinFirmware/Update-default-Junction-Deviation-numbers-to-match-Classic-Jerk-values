#
# fix_framework_weakness.py
#
import pioutil
if pioutil.is_pio_build():

    import shutil
    from os.path import join, isfile
    from pprint import pprint

    env = pioutil.env
    if env.MarlinHas("POSTMORTEM_DEBUGGING"):
        FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-arduinoststm32-maple")
        patchflag_path = join(FRAMEWORK_DIR, ".exc-patching-done")

        # patch file only if we didn't do it before
        if not isfile(patchflag_path):
            print("Patching libmaple exception handlers")
            original_file = join(FRAMEWORK_DIR, "STM32F1", "cores", "maple", "libmaple", "exc.S")
            backup_file = join(FRAMEWORK_DIR, "STM32F1", "cores", "maple", "libmaple", "exc.S.bak")
            src_file = join("buildroot", "share", "PlatformIO", "scripts", "exc.S")

            assert isfile(original_file) and isfile(src_file)
            shutil.copyfile(original_file, backup_file)
            shutil.copyfile(src_file, original_file)

            def _touch(path):
                with open(path, "w") as fp:
                    fp.write("")

            env.Execute(lambda *args, **kwargs: _touch(patchflag_path))
            print("Done patching exception handler")

        print("Libmaple modified and ready for post mortem debugging")
