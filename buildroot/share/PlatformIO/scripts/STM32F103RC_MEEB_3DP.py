#
# STM32F103RC_MEEB_3DP.py
#
import pioutil
if pioutil.is_pio_build():
    env = pioutil.env

    flash_size = 0
    vect_tab_addr = 0

    for define in env['CPPDEFINES']:
        if define[0] == "VECT_TAB_ADDR":
            vect_tab_addr = define[1]
        if define[0] == "STM32_FLASH_SIZE":
            flash_size = define[1]

    print('Use the {0:s} address as the marlin app entry point.'.format(vect_tab_addr))
    print('Use the {0:d}KB flash version of stm32f103rct6 chip.'.format(flash_size))
