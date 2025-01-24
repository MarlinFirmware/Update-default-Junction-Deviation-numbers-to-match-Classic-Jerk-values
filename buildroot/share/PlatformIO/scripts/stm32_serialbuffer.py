#
# stm32_serialbuffer.py
#
import pioutil
if pioutil.is_pio_build():
    env = pioutil.env

    # Get a build flag's value or None
    def getBuildFlagValue(name):
        for flag in build_flags:
            if isinstance(flag, list) and flag[0] == name:
                return flag[1]

        return None

    # Get an overriding buffer size for RX or TX from the build flags
    def getInternalSize(side):
        return  getBuildFlagValue(f"MF_{side}_BUFFER_SIZE") or \
                getBuildFlagValue(f"SERIAL_{side}_BUFFER_SIZE") or \
                getBuildFlagValue(f"USART_{side}_BUF_SIZE")

    # Get the largest defined buffer size for RX or TX
    def getBufferSize(side, default):
        # Get a build flag value or fall back to the given default
        internal = int(getInternalSize(side) or default)
        flag = side + "_BUFFER_SIZE"
        # Return the largest value
        return max(int(mf[flag]), internal) if flag in mf else internal

    # Add a build flag if it's not already defined
    def tryAddFlag(name, value):
        if getBuildFlagValue(name) is None:
            env.Append(BUILD_FLAGS=[f"-D{name}={value}"])

    # Marlin uses the `RX_BUFFER_SIZE` \ `TX_BUFFER_SIZE` options to
    # configure buffer sizes for receiving \ transmitting serial data.
    # Stm32duino uses another set of defines for the same purpose, so this
    # script gets the values from the configuration and uses them to define
    # `SERIAL_RX_BUFFER_SIZE` and `SERIAL_TX_BUFFER_SIZE` as global build
    # flags so they are available for use by the platform.
    #
    # The script will set the value as the default one (64 bytes)
    # or the user-configured one, whichever is higher.
    #
    # Marlin's default buffer sizes are 128 for RX and 32 for TX.
    # The highest value is taken (128/64).
    #
    # If MF_*_BUFFER_SIZE, SERIAL_*_BUFFER_SIZE, USART_*_BUF_SIZE, are
    # defined, the first of these values will be used as the minimum.
    build_flags = env.ParseFlags(env.get('BUILD_FLAGS'))["CPPDEFINES"]
    mf = env["MARLIN_FEATURES"]

    # Get the largest defined buffer sizes for RX or TX, using defaults for undefined
    rxBuf = getBufferSize("RX", 128)
    txBuf = getBufferSize("TX",  64)

    # Provide serial buffer sizes to the stm32duino platform
    tryAddFlag("SERIAL_RX_BUFFER_SIZE", rxBuf)
    tryAddFlag("SERIAL_TX_BUFFER_SIZE", txBuf)
    tryAddFlag("USART_RX_BUF_SIZE", rxBuf)
    tryAddFlag("USART_TX_BUF_SIZE", txBuf)
