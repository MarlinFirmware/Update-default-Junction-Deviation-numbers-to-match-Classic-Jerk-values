#
# preprocessor.py
#
import subprocess

nocache = 1
verbose = 0

def blab(str):
    if verbose:
        print(str)

################################################################################
#
# Invoke GCC to run the preprocessor and extract enabled features
#
preprocessor_cache = {}

def run_preprocessor(env, fn=None):
    filename = fn or 'buildroot/share/PlatformIO/scripts/common-dependencies.h'
    if filename in preprocessor_cache:
        return preprocessor_cache[filename]

    # Process defines
    build_flags = env.get('BUILD_FLAGS')
    build_flags = env.ParseFlagsExtended(build_flags)

    cxx = search_compiler(env)
    cmd = ['"' + cxx + '"']

    # Build flags from board.json
    #if 'BOARD' in env:
    #   cmd += [env.BoardConfig().get("build.extra_flags")]
    for s in build_flags['CPPDEFINES']:
        if isinstance(s, tuple):
            cmd += ['-D' + s[0] + '=' + str(s[1])]
        else:
            cmd += ['-D' + s]

    cmd += ['-D__MARLIN_DEPS__ -w -dM -E -x c++']
    depcmd = cmd + [ filename ]
    cmd = ' '.join(depcmd)
    blab(cmd)
    try:
        define_list = subprocess.check_output(cmd, shell=True).splitlines()
    except:
        define_list = {}
    preprocessor_cache[filename] = define_list
    return define_list


################################################################################
#
# Find a compiler, considering the OS
#
def search_compiler(env):
    global nocache

    from pathlib import Path, PurePath

    ENV_BUILD_PATH = Path(env['PROJECT_BUILD_DIR'], env['PIOENV'])
    GCC_PATH_CACHE = ENV_BUILD_PATH / ".gcc_path"

    gccpath = None
    try:
        gccpath = env.GetProjectOption('custom_gcc')
        blab("Getting compiler from env")
        return gccpath
    except:
        pass

    # Warning: The cached .gcc_path will obscure a newly-installed toolkit
    if not nocache and GCC_PATH_CACHE.exists():
        blab("Getting g++ path from cache")
        return GCC_PATH_CACHE.read_text()

    path_separator = ':'
    gcc_exe = '*g++'
    if env['PLATFORM'] == 'win32':
        path_separator = ';'
        gcc_exe += ".exe"

    envpath = map(Path, env['ENV']['PATH'].split(path_separator))

    # Search for the compiler in PATH
    for ppath in envpath:
        # Use any item in $PATH corresponding to a platformio toolchain bin folder
        if ppath.match(env['PROJECT_PACKAGES_DIR'] + "/**/bin"):
            for gpath in ppath.glob(gcc_exe):
                # Skip '*-elf-g++' (crosstool-NG) except for xtensa32
                if not gpath.stem.endswith('-elf-g++') or "xtensa32" in str(gpath):
                    gccpath = str(gpath.resolve())
                    break

    if not gccpath:
        for ppath in envpath:
            for gpath in ppath.glob(gcc_exe):
                # Skip macOS Clang
                if not (gpath == 'usr/bin/g++' and env['PLATFORM'] == 'darwin'):
                    gccpath = str(gpath.resolve())
                    break

    if not gccpath:
        gccpath = env.get('CXX')
        blab("Couldn't find a compiler! Fallback to '%s'" % gccpath)
        nocache = 1

    # Cache the g++ path to speed up the next build
    if not nocache and gccpath and ENV_BUILD_PATH.exists():
        blab("Caching g++ for current env")
        GCC_PATH_CACHE.write_text(gccpath)

    return gccpath
