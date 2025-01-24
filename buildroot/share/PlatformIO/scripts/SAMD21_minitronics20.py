#
# SAMD21_minitronics20.py
# Customizations for env:SAMD21_minitronics20
#
import pioutil
if pioutil.is_pio_build():
    from os.path import join, isfile
    import shutil
    env = pioutil.env

    mf = env["MARLIN_FEATURES"]
    rxBuf = mf["RX_BUFFER_SIZE"] if "RX_BUFFER_SIZE" in mf else "0"
    txBuf = mf["TX_BUFFER_SIZE"] if "TX_BUFFER_SIZE" in mf else "0"

    serialBuf = str(max(int(rxBuf), int(txBuf), 350))

    build_flags = env.get('BUILD_FLAGS')
    env.Replace(BUILD_FLAGS=build_flags)
