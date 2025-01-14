#
# STM32F1_create_variant.py
#
import pioutil
if pioutil.is_pio_build():
    from pathlib import Path

    source_root_str = "buildroot/share/PlatformIO/variants"
    source_root = Path(source_root_str)
    assert source_root.is_dir()

    env = pioutil.env
    board = env.BoardConfig()
    variant = board.get("build.variant")

    source_dir = source_root / variant
    assert source_dir.is_dir()

    if True:
        # Copying to the platform folder is still needed by STM32F1 (Maple).
        # The alternative code below comes close. See if you can solve it!
        platform = env.PioPlatform()
        FRAMEWORK_DIR = Path(platform.get_package_dir("framework-arduinoststm32-maple"))
        assert FRAMEWORK_DIR.is_dir()

        variant_dir = FRAMEWORK_DIR / "STM32F1/variants" / variant

        if variant_dir.is_dir():
            import shutil
            shutil.rmtree(variant_dir)

        if not variant_dir.is_dir():
            variant_dir.mkdir()

        import marlin
        marlin.copytree(source_dir, variant_dir)

    else:

        # The following almost works, but __start__ (from wirish/start.S) is not seen by common.inc
        board.update("build.variants_dir", source_root_str);
        src = str(source_dir)
        env.Append(BUILD_FLAGS=[f"-I{src}", f"-L{src}/ld"])  # Add include path for variant
