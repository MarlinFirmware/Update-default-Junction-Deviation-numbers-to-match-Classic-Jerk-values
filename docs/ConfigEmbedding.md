# Configuration Embedding

Starting with version 2.0.9.3, Marlin can automatically extract the configuration used to generate the firmware and store it in the firmware binary. This is enabled by defining `CONFIGURATION_EMBEDDING` in `Configuration_adv.h`.

## How it's done
At the start of the PlatformIO build process, we create an embedded configuration by extracting all active options from the Configuration files and writing them out as JSON to `marlin_config.json`, which also includes specific build information (like the git revision, the build date, and some version information). The JSON file is then compressed in a ZIP archive called `.pio/build/mc.zip` which is converted into a C array and stored in a C++ file called `mc.h` which is included in the build.

## Extracting configurations from a Marlin binary
To get the configuration out of a binary firmware, you'll need a non-write-protected SD card inserted into the printer while running the firmware.
Send the command `M503 C` to write the file `mc.zip` to the SD card. Copy the file to your computer, ideally in the same folder as the Marlin repository.

Run the following commands to extract and apply the configuration:
```
$ git checkout -f
$ unzip mc.zip
$ python buildroot/share/PlatformIO/scripts/mc-apply.py
```

This will attempt to update the configuration files to match the settings used for the original build. It will also dump the git reference used to build the code (which may be accessible if the firmware was built from the main repository. As a fallback it also includes the `STRING_DISTRIBUTION_DATE` which is unlikely to be modified in a fork).
