# pixi_ci

Builds and tests one Gazebo library on a CI agent: it installs the third-party
dependencies of a `conda/envs/<env>` pixi environment, builds the Gazebo
dependencies from source (gazebodistro + colcon), then builds and tests the
library. It supports Windows on amd64 and macOS on arm64 (see
[macOS](#macos)). On Windows the Jenkins jobs call
`jenkins-scripts/lib/colcon-default-devel-windows.bat <library>`, which
downloads pixi, creates the bootstrap environment and runs the driver. The
`jenkins-scripts/<lib>-default-devel-windows-amd64.bat` files only forward to
it, for the jobs generated before the DSL called it directly.

It runs with the Python of the `conda/config-detector` pixi environment (the
"bootstrap" environment, which has pyyaml):

```bat
"%PIXI_BOOTSTRAP_PROJECT_PATH%\.pixi\envs\default\python.exe" -u jenkins-scripts\tools\run_pixi_ci.py gz-fuel-tools
```

The argument is the library name in `jenkins-scripts/dsl/gz-collections.yaml`,
which must give the library a CI job on this platform.
`run_pixi_ci.py` puts `jenkins-scripts/tools` on `sys.path` and `pixi_ci` adds
`jenkins-scripts/dsl/tools`; no `PYTHONPATH` is set, so nothing leaks into the
builds and tests.

## Job variables

The variables are the ones `colcon-default-devel-windows.bat` used. A variable
is *set* when it is defined and not empty, as in `cmd`.

| Variable | Meaning | Default |
|---|---|---|
| `WORKSPACE` | Jenkins workspace | required |
| `VCS_DIRECTORY` | checkout of the sources under test, relative to `WORKSPACE` | the library name |
| `COLCON_PACKAGE` | colcon name without major version | from `packages.py` (`gz-fuel_tools` for `gz-fuel-tools`) |
| `COLCON_AUTO_MAJOR_VERSION` | try `COLCON_PACKAGE` + major version first | `true` |
| `COLCON_PACKAGE_EXTRA_CMAKE_ARGS` | extra CMake args for the library under test | from `packages.py` |
| `BUILD_TYPE` | CMake build type | `Release` (macOS: `RelWithDebInfo`) |
| `ENABLE_TESTS` | run the tests (`TRUE`/`FALSE`, any case); the library is always built with `BUILD_TESTING=1` | `TRUE` |
| `KEEP_WORKSPACE` | when set (any value), keep `WORKSPACE\ws` before and after the build | unset |
| `CONDA_ENV_NAME` | env in `conda/envs` to use | detected from `gz-collections.yaml` for the platform (windows/amd64, darwin/arm64) |
| `GAZEBODISTRO_FILE` | gazebodistro file | `<VCS_DIRECTORY><major>.yaml` |
| `GAZEBODISTRO_BRANCH` | gazebodistro branch | `master` |
| `ghprbSourceBranch` | PR branch: a `ci_matching_branch/*` name checks out the same gazebodistro branch | unset |
| `MAKE_JOBS` | colcon `--parallel-workers` and `MAKEFLAGS=-j` | number of CPUs |
| `GPU_SUPPORT_NEEDED` | `true`: fail unless `dxdiag` reports an NVIDIA GPU (macOS: nothing to check) | `requirements.nvidia_gpu` in `gz-collections.yaml` |
| `REUSE_PIXI_INSTALLATION` | when set (any value), reuse the installed pixi env | unset |
| `PIXI_TMP` | pixi executable | `%PROGRAMDATA%\pixi\pixi.exe` (macOS: [below](#macos)) |
| `PIXI_PROJECT_PATH` | pixi project with the Gazebo env | `%PROGRAMDATA%\pixi\project` (macOS: [below](#macos)) |

The log starts with a `pixi_ci configuration` section listing the effective
values and the variables that were set.

Outputs: JUnit files in `WORKSPACE\build\test_results`, and
`PIXI_PROJECT_PATH\hooks.bat` to activate the env in a command prompt.

## Running it outside the CI agents

The defaults are the paths where `lib/windows_env_vars.bat` provisions pixi and
the bootstrap environment on the agents. On a developer machine, set the two
pixi variables to paths of your own, from the release-tools checkout:

```bat
:: the PIXI_VERSION of lib\windows_env_vars.bat, the one the agents download
set "PIXI_TMP=C:\tools\pixi.exe"
:: pixi_ci deletes and recreates this directory on every run
set "PIXI_PROJECT_PATH=%LOCALAPPDATA%\pixi_ci\project"
:: holds the checkout under test: %WORKSPACE%\gz-fuel-tools
set "WORKSPACE=C:\ci"

:: the bootstrap environment, as windows_library.bat installs it
mkdir "%LOCALAPPDATA%\pixi_ci\bootstrap"
copy conda\config-detector\pixi.* "%LOCALAPPDATA%\pixi_ci\bootstrap"
"%PIXI_TMP%" install --locked --manifest-path "%LOCALAPPDATA%\pixi_ci\bootstrap\pixi.toml"

"%LOCALAPPDATA%\pixi_ci\bootstrap\.pixi\envs\default\python.exe" -u jenkins-scripts\tools\run_pixi_ci.py gz-fuel-tools
```

pixi_ci does not fall back to a `pixi` found in `PATH`: the CI jobs must use
the pinned version.

## macOS

On the macOS arm64 agents the jobs call
`jenkins-scripts/lib/colcon-default-devel-macos.bash <library>`. Unless
`REUSE_PIXI_INSTALLATION` is set, the wrapper downloads pixi (its
`PIXI_VERSION`, the same as in `lib/windows_env_vars.bat`), checks the
SHA-256 that the pixi release publishes and installs `conda/config-detector` as
the bootstrap environment. Then it runs the driver with the bootstrap python
(`-u -I`: the `PYTHON*` variables of the job do not reach it), and the exit
code of the driver is the job result. A Mac with the Xcode Command Line Tools
runs it as is:

```bash
# $WORKSPACE holds the checkout under test: $WORKSPACE/gz-math
WORKSPACE=$HOME/ci bash jenkins-scripts/lib/colcon-default-devel-macos.bash gz-math
```

Everything lives in the Jenkins workspace:

| Variable | macOS default |
|---|---|
| `PIXI_TMP` | `$WORKSPACE/.pixi_ci/pixi` |
| `PIXI_PROJECT_PATH` | `$WORKSPACE/.pixi_ci/project` |
| `PIXI_BOOTSTRAP_PROJECT_PATH` | `$WORKSPACE/.pixi_ci/bootstrap` (read by the wrapper only) |
| `BUILD_TYPE` | `RelWithDebInfo`, as the Homebrew jobs |

With `REUSE_PIXI_INSTALLATION` set, the wrapper and the driver skip the
download and the installs and use what the previous run left in `.pixi_ci`;
both stop with an error naming the missing piece when it is not there.

The compilers are the conda-forge clang (`clang_osx-arm64`,
`clangxx_osx-arm64`). Their activation sets `CC`, `CXX`, the flags and
`SDKROOT`, which comes from the Command Line Tools. After the activation the
driver checks that `CC` and `CXX` resolve inside the conda env and that
`SDKROOT` exists, and logs the macOS, SDK and compiler versions in the
`macOS toolchain` section.
`PIXI_PROJECT_PATH/hooks.sh` activates the env in a terminal (`source` it).

### Isolation from Homebrew

Homebrew stays installed on the agents for the other jobs. Before anything
runs, pixi included, the driver removes from the job environment:

- the `PATH` entries under `/opt/homebrew` and `/usr/local`;
- `HOMEBREW_*` and `DYLD_*`;
- `CMAKE_PREFIX_PATH`, `CMAKE_INCLUDE_PATH`, `CMAKE_LIBRARY_PATH`,
  `CMAKE_PROGRAM_PATH`, `CMAKE_FRAMEWORK_PATH`, `PKG_CONFIG_PATH`,
  `PKG_CONFIG_LIBDIR`, `CPATH`, `C_INCLUDE_PATH`, `CPLUS_INCLUDE_PATH`,
  `LIBRARY_PATH`, `PYTHONPATH` and `PYTHONHOME`;
- `CC`, `CXX`, `CPP`, `OBJC`, `CFLAGS`, `CXXFLAGS`, `CPPFLAGS` and `LDFLAGS`:
  the clang activation keeps or extends values set before it;
- every other variable whose value mentions `/opt/homebrew` or `/usr/local`
  (`AR`, `JAVA_HOME`, ...).

The `macOS: remove Homebrew from the job environment` section lists what it
removed, by name only. Both colcon builds also get
`-DCMAKE_IGNORE_PREFIX_PATH=/opt/homebrew;/usr/local`, and
`-DCMAKE_FIND_FRAMEWORK=LAST -DCMAKE_FIND_APPBUNDLE=LAST` as conda-forge
builds its macOS packages: CMake looks in the env before the SDK frameworks,
otherwise the headers of the SDK's `Ruby.framework` win over the env's ruby.

### Leak check

Between the build and the tests, `leakcheck.py` looks for anything the build
took from outside the conda env, and the job fails listing each offender with
its file:

- every Mach-O file in `ws/install`: the libraries it loads and its rpaths
  (`otool -l`) must be relative to `@rpath/`, `@loader_path` or
  `@executable_path`, or be under `$CONDA_PREFIX`, `ws`, `/usr/lib` or
  `/System/Library`. `/usr/lib/libc++.1.dylib` is refused: conda libraries load
  `@rpath/libc++.1.dylib`, and two C++ runtimes in one process break.
  `/usr/lib/libc++abi.dylib` is fine: conda-forge has no libc++abi, its libc++
  uses the system one.
- every `ws/build/*/CMakeCache.txt`: the `PATH` and `FILEPATH` entries must be
  under `$CONDA_PREFIX`, `ws`, `$SDKROOT`, `xcode-select -p`, `/usr/bin`,
  `/bin`, `/usr/sbin`, `/sbin`, `/usr/lib` or `/System/Library`, and
  `CMAKE_C_COMPILER`/`CMAKE_CXX_COMPILER` under `$CONDA_PREFIX`. The
  `CMAKE_INSTALL_<dir>` destinations of GNUInstallDirs are not checked: they
  say where the install goes, and `CMAKE_INSTALL_OLDINCLUDEDIR` is
  `/usr/include` in every cache.

`$CONDA_PREFIX`, `ws`, `$SDKROOT` and `xcode-select -p` also count by their
physical path: CMake and colcon record resolved paths when the workspace is
behind a symlink (`/tmp` on macOS), and the SDK path is a symlink.

It is an allowlist: prefixes nobody listed (`/opt/X11`, `/Library/Frameworks`,
MacPorts) fail too. When a refused path is legitimate, add its root to
`SYSTEM_LIBRARY_ROOTS` (libraries and caches) or `SYSTEM_TOOL_ROOTS` (caches
only) in `leakcheck.py`, with a case in `tests/test_leakcheck.py`.

## Modules

| Module | Responsibility |
|---|---|
| `__main__.py` | the job steps, in order, with `# BEGIN SECTION` markers |
| `config.py` | job variables to `JobConfig` |
| `packages.py` | libraries and GPU needs from `gz-collections.yaml` (`find_conda_libraries` in `dsl/tools`), colcon name and CMake args that differ |
| `platforms.py` | per-OS facts: pixi paths, variable expansion, OGRE paths, GPU check; on macOS the Homebrew isolation and the toolchain checks |
| `leakcheck.py` | macOS: Homebrew and system paths in the install tree (`otool -l`) and in the CMake caches |
| `envs.py` | major version, env selection, `pixi install --locked`, activation with `pixi shell-hook --json` |
| `envvars.py` | merge of the activation variables, `%NAME%` expansion |
| `workspace.py` | gazebodistro, copy of the sources, colcon package name |
| `colcon.py` | colcon build and test, JUnit export |
| `runner.py` | subprocesses (absolute executables from the activated `PATH`), log sections |
| `fsops.py` | `rmdir /s /q` and `xcopy` equivalents |

## Tests

```bash
uv venv --python 3.10 /tmp/pixi_ci_venv
VIRTUAL_ENV=/tmp/pixi_ci_venv uv pip install pytest pyyaml
/tmp/pixi_ci_venv/bin/python -m pytest jenkins-scripts/tools/pixi_ci/tests jenkins-scripts/dsl/tools/tests
```

`tests/test_recorded_commands.py` runs the driver with a fake runner and
compares the commands it records with the ones `colcon-default-devel-windows.bat`
ran. `tests/test_recorded_commands_macos.py` does the same for macOS, on top of
the Homebrew isolation, and `tests/test_macos_wrapper.py` runs the macOS
wrapper with fake `curl`, `shasum` and `pixi`. The `macos_smoke` job of
`.github/workflows/ci-pixi-ci.yaml` builds and tests gz-math for real with the
wrapper on GitHub's macOS runners, which have Homebrew installed like the
agents.
