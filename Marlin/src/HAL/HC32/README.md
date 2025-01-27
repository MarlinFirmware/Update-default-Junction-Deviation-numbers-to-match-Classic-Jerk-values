# HC32F460 HAL

This document provides notes on the HAL for the HC32F460 MCU.

## Adding support for a new board

The HC32F460 HAL is designed to be generic enough for any HC32F460-based board. Adding support for a new HC32F460-based board will require the following steps:

1. Follow [the usual instructions](https://marlinfw.org/docs/development/boards.html#adding-a-new-board) to add a new board to Marlin. (i.e., Add a pins file, edit `boards.h` and `pins.h`, etc.)
2. Determine the flash size your board uses:
   - Examine the board's main processor. (Refer the naming key in `hc32.ini`.)
   - Extend the `HC32F460C_common` base env for 256K, or `HC32F460E_common` for 512K.
3. Determine your board's application start address (see [below](#finding-the-application-start-address))
4. Set `board_upload.offset_address` to the app start address once you've found it. If your board doesn't use a bootloader, you may be able to use the "ICSP" header or DFU. This document will be updated once we have more information about flashing without a bootloader.

### Finding the application start address

If the board contains a bootloader you'll need to find the application address. This is the address the bootloader jumps to after it's done. You can find this address in a few different ways:

#### 1. Using log messages

If you're lucky, the bootloader may print the app start address on the serial output during boot. To check for this, use your favorite serial monitor to observe the serial output when you power on the board. Look for a message like "Jumping to 0xC000" or "GotoApp->addr=0xC000". This line would be printed before Marlin's "start" message.

Example:

```
[...]
version 1.2
sdio init success!
Disk init
Tips ------ None Firmware file
GotoApp->addr=0xC000

start
[...]
```

#### 2. Using published source code

If the vendor has published Marlin source code that includes the bootloader, you can search the bootloader source code for the address. Begin your search with the following steps:

1. Find the code that sets the vector table offset

The vector table offset is usually set using a line like this:

```c
SCB->VTOR = ((uint32_t) APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);
```

Just searching for `SCB->VTOR` should yield some results. From there, you just need to look at the value that's assigned to it. The example uses `APP_START_ADDRESS`.

> [!NOTE]
> Some vendors publish incomplete source code. But they sometimes leave version control related files in the repo, which can contain previous version of files that were removed. Find these by including folders like `.git` or `.svn` in your search.

> [!NOTE]
> The example is based on the [Voxelab-64/Aquila_X2](https://github.com/Voxelab-64/Aquila_X2/blob/d1f23adf96920996b979bc31023d1dce236d05db/firmware/Sources/.svn/pristine/ec/ec82bcb480b511906bc3e6658450e3a803ab9813.svn-base#L96) which actually includes deleted files in its repo.

2. Using a linker script

If the repository contains a linker script, look at the memory regions, specifically a region named `FLASH` or similar. The `ORIGIN` of that region will be the application start address.

**Example:**

```ld
MEMORY
{
    FLASH       (rx): ORIGIN = 0x0000C000, LENGTH = 512K
    OTP         (rx): ORIGIN = 0x03000C00, LENGTH = 1020
    RAM        (rwx): ORIGIN = 0x1FFF8000, LENGTH = 188K
    RET_RAM    (rwx): ORIGIN = 0x200F0000, LENGTH = 4K
}
```

> [!NOTE]
> This example is based on [Voxelab-64/Aquila_X2](https://github.com/Voxelab-64/Aquila_X2/blob/d1f23adf96920996b979bc31023d1dce236d05db/firmware/Sources/main/hdsc32core/hc32f46x_flash.ld#L55)

## Documentation on the HC32F460

Due to uncertain licensing (w/r/t STMicro), documentation for the HC32F460 is only available upon request. Documentation includes the following:

- Datasheet, user manual, reference manual
- Application notes for the DDL
- DDL source code
- IDE support packages (Keil, IAR, ...) including .svd files
- Programming software
- Emulator / debugger drivers
- Development board documentation and schematics
- Errata documents
- (Limited) sales information
- Full Voxelab firmware source code
- Documents in Chinese or English (machine translated)

Contact me on Discord (@shadow578) if you need it.

## Dependencies

This HAL depends on the following projects:

- [shadow578/platform-hc32f46x](https://github.com/shadow578/platform-hc32f46x) (PlatformIO platform for HC32F46x)
- [shadow578/framework-arduino-hc32f46x](https://github.com/shadow578/framework-arduino-hc32f46x) (Arduino framework for HC32F46x)
- [shadow578/framework-hc32f46x-ddl](https://github.com/shadow578/framework-hc32f46x-ddl) (HC32F46x DDL framework)

## Credits

This HAL wouldn't be possible without the following projects:

- [Voxelab-64/Aquila_X2](https://github.com/Voxelab-64/Aquila_X2) (original implementation)
- [alexqzd/Marlin-H32](https://github.com/alexqzd/Marlin-H32) (misc. fixes to the original implementation)
- [kgoveas/Arduino-Core-Template](https://github.com/kgoveas/Arduino-Core-Template) (template for Arduino headers)
- [stm32duino/Arduino_Core_STM32](https://github.com/stm32duino/Arduino_Core_STM32) (misc. Arduino functions)
