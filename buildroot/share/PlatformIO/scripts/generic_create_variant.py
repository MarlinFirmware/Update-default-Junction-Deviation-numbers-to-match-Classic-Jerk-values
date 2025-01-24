#
# generic_create_variant.py
#
# Copy one of the variants from buildroot/platformio/variants into
# the appropriate framework variants folder, so that its contents
# will be picked up by PlatformIO just like any other variant.
#
import pioutil, re

marlin_variant_pattern = re.compile("marlin_.*")
if pioutil.is_pio_build():
    import shutil, marlin
    from pathlib import Path

    #
    # Get the platform name from the 'platform_packages' option,
    # or look it up by the platform.class.name.
    #
    env = pioutil.env
    platform = env.PioPlatform()

    from platformio.package.meta import PackageSpec
    platform_packages = env.GetProjectOption('platform_packages')

    # Remove all tool items from platform_packages
    platform_packages = [x for x in platform_packages if not x.startswith("platformio/tool-")]

    if len(platform_packages) == 0:
        framewords = {
            "Ststm32Platform": "framework-arduinoststm32",
            "AtmelavrPlatform": "framework-arduino-avr"
        }
        platform_name = framewords[platform.__class__.__name__]
    else:
        spec = PackageSpec(platform_packages[0])
        if spec.uri and '@' in spec.uri:
            platform_name = re.sub(r'@.+', '', spec.uri)
        else:
            platform_name = spec.name

    FRAMEWORK_DIR = Path(platform.get_package_dir(platform_name))
    assert FRAMEWORK_DIR.is_dir()

    #
    # Point variants_dir to our variant folder when board_build.variant
    # is provided and the variant name begins with "marlin_".
    #
    board = env.BoardConfig()
    variant = board.get("build.variant")
    #mcu_type = board.get("build.mcu")[:-2]
    #series = mcu_type[:7].upper() + "xx"

    # Make sure the local variant sub-folder exists
    if marlin_variant_pattern.match(str(variant).lower()):
        here = Path.cwd()
        variants_dir = here / 'buildroot' / 'share' / 'PlatformIO' / 'variants'
        source_dir = variants_dir / variant
        assert source_dir.is_dir()
        board.update("build.variants_dir", str(variants_dir))
