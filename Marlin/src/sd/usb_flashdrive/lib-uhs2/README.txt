
==== USB HOST SHIELD 2.0 LIBRARY ====

The lib/ folder contains a subset of the files from the USB Host Shield 2.0
library:

  https://github.com/felis/USB_Host_Shield_2.0

While the original library was released under the GPLv2 and could not be
commingled with Marlin, the developers have graciously re-licenced the
files needed for Marlin as "GPLv2 or later", as documented in this thread.

  https://github.com/felis/USB_Host_Shield_2.0/issues/364

Small modifications have been made to the source. Please search for
HAS_USB_FLASH_DRIVE or look at the patch file to see what was changed.

==== LICENSE SUMMARY ====

Source Path:                 Repository:                           License:
------------                 -----------                           --------
lib-uhs3/                    github.com/felis/USB_Host_Shield_2.0  GPLv2 or later
lib-uhs3/lib/masstorage.cpp  github.com/greiman/UsbFat [1]         MIT
lib-uhs3/lib/settings.h      github.com/greiman/UsbFat [1]         MIT

[1] Changes related to SKIP_WRITE_PROTECT and DELAY only

==== PERFORMANCE ENHANCEMENTS FOR USB DRIVES ====

There are also some small performance enhancements from Bill Greiman, regarding
SKIP_WRITE_PROTECT and DELAY. These changes came from the following repo:

   https://github.com/greiman/UsbFat

While the original library was released under the GPLv2 and could not be
commingled with Marlin, the developer has graciously re-licenced his changes
under the "MIT" license, as documented here:

   https://github.com/greiman/UsbFat/issues/8

==== MARLIN INTEGRATION WORK ====

All additional work done to integrate USB into Marlin was performed by LulzBot and is licensed under the GPLv3.
