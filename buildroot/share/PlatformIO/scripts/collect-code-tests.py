#
# collect-code-tests.py
# Convenience script to collect all code tests. Used by env:linux_native_test in native.ini.
#

import pioutil
if pioutil.is_pio_build():

    import os, re
    env = pioutil.env

    os.environ['PATH'] = f"./buildroot/bin/:./buildroot/tests/:{os.environ['PATH']}"

    def collect_test_suites():
        """Get all the test suites"""
        from pathlib import Path
        return sorted(list(Path("./test").glob("*.ini")))

    def register_test_suites():
        """Register all the test suites"""
        targets = []
        test_suites = collect_test_suites()
        for path in test_suites:
            name = re.sub(r'^\d+-|\.ini$', '', path.name)
            targets += [name];

            env.AddCustomTarget(
                name = f"marlin_{name}",
                dependencies = None,
                actions = [
                    f"echo ====== Configuring for marlin_{name} ======",
                    "restore_configs",
                    f"cp -f {path} ./Marlin/config.ini",
                    "python ./buildroot/share/PlatformIO/scripts/configuration.py",
                    f"platformio test -e linux_native_test -f {name}",
                    "restore_configs",
                ],
                title = "Marlin: {}".format(name.lower().title().replace("_", " ")),
                description = (
                    f"Run a Marlin test suite, with the appropriate configuration, "
                    f"that sits in {path}"
                )
            )

        env.AddCustomTarget(
            name = "test-marlin",
            dependencies = None,
            actions = [
                f"platformio run -t marlin_{name} -e linux_native_test"
                for name in targets
            ],
            title = "Marlin: Test all code test suites",
            description = (
                f"Run all Marlin code test suites ({len(targets)} found)."
            ),
        )

    register_test_suites()
