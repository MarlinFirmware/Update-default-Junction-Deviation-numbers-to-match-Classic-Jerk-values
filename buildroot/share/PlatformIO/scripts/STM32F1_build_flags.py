#!/usr/bin/env python
#
# STM32F1_build_flags.py
# Add build_flags for the base STM32F1_maple environment (stm32f1-maple.ini)
#
from __future__ import print_function
import sys

# Dynamic build flags for generic compile options
if __name__ == "__main__":

    # Print these plus the given args when running directly on the command-line
    args = [
        "-std=gnu++14",
        "-Os",
        "-mcpu=cortex-m3",
        "-mthumb",

        "-fsigned-char",
        "-fno-move-loop-invariants",
        "-fno-strict-aliasing",
        "-fsingle-precision-constant",

        "--specs=nano.specs",
        "--specs=nosys.specs",

        "-MMD", "-MP",

        "-IMarlin/src/HAL/STM32F1",

        "-DTARGET_STM32F1",
        "-DARDUINO_ARCH_STM32",
        "-DPLATFORM_M997_SUPPORT"
    ] + sys.argv[1:]

    print(" ".join(args))

else:

    # Extra script for stm32f1-maple.ini build_flags

    import pioutil
    if pioutil.is_pio_build():
        pioutil.env.Append(
            ARFLAGS=["rcs"],

            ASFLAGS=["-x", "assembler-with-cpp"],

            CXXFLAGS=[
                "-fabi-version=0",
                "-fno-use-cxa-atexit",
                "-fno-threadsafe-statics"
            ],
            LINKFLAGS=[
                "-Os",
                "-mcpu=cortex-m3",
                "-ffreestanding",
                "-mthumb",
                "--specs=nano.specs",
                "--specs=nosys.specs",
                "-u_printf_float",
            ],
        )
