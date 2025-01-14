# Marlin Code Maintenance

In addition to the general work of fixing bugs, supporting new hardware, developing new features, and refining existing features we have to also maintain supporting files that change in tandem with the codebase. Some of this maintenance can be done as part of CI, and we may even generate commits to the development branch using periodic checks whenever any auto-generated files need to be regenerated. (See `bump-date.yml` for one example.)

## Configuration Tasks

- Add CI build tests and Unit Testing to catch future breaking changes.
- Periodically run `buildroot/bin/build_all_examples` to make sure configs and code are buildable.
- Periodically run `buildroot/share/scripts/makeBaseConfigs.py` to generate new base configurations.

## Documentation Tasks

- For any G-code command changes update G-code pages and the RepRap wiki G-code page.
- For a new feature or changes to an existing feature update Configuration page and any articles about the feature.
