# pixi_ci

Builds and tests one Gazebo library on a CI agent: it installs the third-party
dependencies of a `conda/envs/<env>` pixi environment, builds the Gazebo
dependencies from source (gazebodistro + colcon), then builds and tests the
library. Today it supports Windows only: the `jenkins-scripts/<lib>-default-devel-windows-amd64.bat`
entry points call `jenkins-scripts/lib/colcon-default-devel-windows.bat <library>`,
which downloads pixi, creates the bootstrap environment and runs the driver.

It runs with the Python of the `conda/config-detector` pixi environment (the
"bootstrap" environment, which has pyyaml):

```bat
"%PIXI_BOOTSTRAP_PROJECT_PATH%\.pixi\envs\default\python.exe" -u jenkins-scripts\tools\run_pixi_ci.py gz-fuel-tools
```

The argument is the library name in `jenkins-scripts/dsl/gz-collections.yaml`.
`run_pixi_ci.py` puts `jenkins-scripts/tools` on `sys.path`; no `PYTHONPATH` is
set, so nothing leaks into the builds and tests.

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
| `BUILD_TYPE` | CMake build type | `Release` |
| `ENABLE_TESTS` | run the tests (`TRUE`/`FALSE`, any case); the library is always built with `BUILD_TESTING=1` | `TRUE` |
| `KEEP_WORKSPACE` | when set (any value), keep `WORKSPACE\ws` before and after the build | unset |
| `CONDA_ENV_NAME` | env in `conda/envs` to use | detected from `gz-collections.yaml` for windows/amd64 |
| `GAZEBODISTRO_FILE` | gazebodistro file | `<VCS_DIRECTORY><major>.yaml` |
| `GAZEBODISTRO_BRANCH` | gazebodistro branch | `master` |
| `ghprbSourceBranch` | PR branch: a `ci_matching_branch/*` name checks out the same gazebodistro branch | unset |
| `MAKE_JOBS` | colcon `--parallel-workers` and `MAKEFLAGS=-j` | number of CPUs |
| `GPU_SUPPORT_NEEDED` | `true`: fail unless `dxdiag` reports an NVIDIA GPU | from `packages.py` |
| `REUSE_PIXI_INSTALLATION` | when set (any value), reuse the installed pixi env | unset |
| `PIXI_TMP` | pixi executable | `%PROGRAMDATA%\pixi\pixi.exe` |
| `PIXI_PROJECT_PATH` | pixi project with the Gazebo env | `%PROGRAMDATA%\pixi\project` |

The log starts with a `pixi_ci configuration` section listing the effective
values and the variables that were set.

Outputs: JUnit files in `WORKSPACE\build\test_results`, and
`PIXI_PROJECT_PATH\hooks.bat` to activate the env in a command prompt.

## Modules

| Module | Responsibility |
|---|---|
| `__main__.py` | the job steps, in order, with `# BEGIN SECTION` markers |
| `config.py` | job variables to `JobConfig` |
| `packages.py` | per-library differences (colcon name, CMake args, GPU) |
| `platforms.py` | per-OS facts: pixi paths, variable expansion, OGRE paths, GPU check |
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
ran.
