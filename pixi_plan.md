# Plan: Refactor Conda Build Scripts from Batch to Python

## Context

The Gazebo CI conda/pixi build system is implemented in Windows batch scripts (~590 lines across 3 library files + 16 package entry points + 1 sdformat entry point). Batch is hard to maintain, test, and extend. The goal is to rewrite this in Python **keeping Windows as the target platform**, but structuring the code so that adding macOS support in the future is straightforward.

This is **not** a macOS migration — it is a language migration (batch → Python) on Windows, with platform abstraction designed in from the start.

Do one per commit per step prepare to create a PR for it to review.

---

## Step 1: Create shared utilities — `jenkins-scripts/lib/build_utils.py`

Foundation module with no platform-specific code.

**Functions:**
- `run_cmd(cmd, cwd=None, env=None, check=True)` — subprocess.run wrapper with logging and error handling. Prints command being run, streams output, raises on failure.
- `section(name)` — context manager that prints `# BEGIN SECTION: {name}` / `# END SECTION` markers (Jenkins log folding).
- `download_file(url, dest_path, retries=3)` — download with retry logic. Replaces the powershell `Invoke-WebRequest` call in `:wget` (`windows_library.bat:66-86`).
- `detect_cmake_major_version(cmakelists_path)` — regex extraction, reuses logic from `jenkins-scripts/tools/detect_cmake_major_version.py:15-28`.

**Source files being replaced:** Parts of `windows_library.bat` (`:wget` lines 66-86).

---

## Step 2: Create pixi utilities — `jenkins-scripts/lib/pixi_utils.py`

Pixi/conda environment management. Platform-aware via `sys.platform` checks for URLs and binary names.

**Constants:**
- `PIXI_VERSION = "0.44.0"`
- `get_pixi_url()` — returns platform-appropriate URL. Windows: `pixi-x86_64-pc-windows-msvc.exe`. Future macOS: `pixi-aarch64-apple-darwin` / `pixi-x86_64-apple-darwin`.
- `get_pixi_binary_name()` — `pixi.exe` on Windows, `pixi` on macOS.

**Functions:**
- `pixi_installation(pixi_tmpdir)` — download pixi binary to tmpdir. Replaces `:pixi_installation` (`windows_library.bat:201-211`).
- `pixi_create_environment(source_dir, project_path, pixi_binary)` — copy pixi.toml/pixi.lock from source_dir to project_path, run `pixi install --locked`. Replaces `:pixi_create_bootstrap_environment` (lines 216-228) and `:pixi_create_gz_environment` (lines 233-254).
- `pixi_load_shell(project_path, pixi_binary)` — run `pixi shell-hook --locked`, capture output. On Windows: writes hooks.bat, runs it, captures resulting env. On macOS (future): captures bash export statements. Replaces `:pixi_load_bootstrap_shell` (lines 257-271) and `:pixi_load_shell` (lines 274-288).
- `pixi_cmd(project_path, pixi_binary, *args)` — run pixi subcommand. Replaces `:pixi_cmd` (lines 307-320) and `:pixi_bootstrap_cmd` (lines 290-303).
- `detect_conda_env(package_name, major_version, yaml_file)` — imports and calls `get_conda_ciconfig_from_package_and_version.find_conda_configs()` directly. Replaces `:detect_conda_env` (lines 324-337).

**Source files being replaced:** Pixi-related functions from `windows_library.bat`, all of `windows_env_vars.bat`.

---

## Step 3: Create colcon utilities — `jenkins-scripts/lib/colcon_utils.py`

Colcon build/test operations and workspace management. Platform-independent.

**Functions:**
- `get_source_from_gazebodistro(distro_file, dest_dir, branch="master", pr_branch=None)` — clone gazebodistro, handle ci_matching_branch detection, vcs import. Replaces `:get_source_from_gazebodistro` (`windows_library.bat:89-117`).
- `build_workspace(package_name, build_type, extra_cmake_args=None, make_jobs=None)` — two-step colcon build: (1) dependencies without tests `--packages-skip`, (2) target with tests `--packages-select`. Replaces `:build_workspace` (lines 156-174) and `:_colcon_build_cmd` (lines 120-148).
- `run_tests(package_name)` — `colcon test --packages-select` + `colcon test-result --all`. Replaces `:tests_in_workspace` (lines 183-196).
- `list_workspace_pkgs()` — `colcon list -t` + `vcs export --exact`. Replaces `:list_workspace_pkgs` (lines 177-180).
- `resolve_colcon_package_name(package_original, major_version, workspace_dir)` — handle the gz→ignition name fallback chain (with/without major version, gz→ignition renaming, sim→gazebo renaming). Replaces the complex name resolution logic in `colcon-default-devel-windows.bat:161-194`.

**Source files being replaced:** Colcon and workspace functions from `windows_library.bat`, name resolution from `colcon-default-devel-windows.bat`.

---

## Step 4: Create Windows platform utilities — `jenkins-scripts/lib/windows_utils.py`

Windows-specific operations isolated into their own module. This is the module that gets a macOS counterpart in the future.

**Functions:**
- `configure_msvc_compiler(platform="x86_amd64")` — find and call vcvarsall.bat, capture resulting environment. Replaces `:configure_msvc2019_compiler` (`windows_library.bat:7-62`).
- `check_gpu_support(workspace)` — run dxdiag, check for NVIDIA GPU. Replaces the GPU check in `colcon-default-devel-windows.bat:48-60`.
- `get_ogre_env_vars(conda_prefix)` — return dict with OGRE_RESOURCE_PATH and OGRE2_RESOURCE_PATH using Windows paths (`Library\bin`, `Library\bin\OGRE-Next`). Replaces lines 123-124 of `colcon-default-devel-windows.bat`.

---

## Step 5: Create main orchestrator — `jenkins-scripts/lib/colcon_default_devel.py`

Single entry point replacing all 17 `.bat` entry points + `colcon-default-devel-windows.bat`.

**CLI interface:**
```
python colcon_default_devel.py \
  --package gz-rendering \
  --auto-major-version \
  --gpu-support \
  --conda-env-name legacy_ogre23
```

**Parameters** (replace the env vars set in each `.bat` file):
| CLI argument | Old env var | Default |
|---|---|---|
| `--package` (required) | `COLCON_PACKAGE` | — |
| `--vcs-directory` | `VCS_DIRECTORY` | same as --package |
| `--auto-major-version` | `COLCON_AUTO_MAJOR_VERSION` | false |
| `--build-type` | `BUILD_TYPE` | Release |
| `--enable-tests / --disable-tests` | `ENABLE_TESTS` | enabled |
| `--conda-env-name` | `CONDA_ENV_NAME` | auto-detect |
| `--keep-workspace` | `KEEP_WORKSPACE` | false |
| `--gazebodistro-file` | `GAZEBODISTRO_FILE` | auto |
| `--extra-cmake-args` | `COLCON_PACKAGE_EXTRA_CMAKE_ARGS` | none |
| `--gpu-support` | `GPU_SUPPORT_NEEDED` | false |
| `--platform` | `PLATFORM_TO_BUILD` | x86_amd64 |

**Build flow** (same as current `colcon-default-devel-windows.bat`):
1. Parse CLI arguments
2. Validate inputs (VCS_DIRECTORY exists in WORKSPACE)
3. Platform-specific: configure MSVC compiler (via `windows_utils`)
4. Platform-specific: GPU check if requested (via `windows_utils`)
5. Install pixi binary (via `pixi_utils`)
6. Create bootstrap environment (via `pixi_utils`)
7. Load bootstrap shell (via `pixi_utils`)
8. Auto-detect major version if requested (via `build_utils`)
9. Auto-detect conda environment (via `pixi_utils`)
10. Create Gazebo conda environment (via `pixi_utils`)
11. Load Gazebo environment shell (via `pixi_utils`)
12. Platform-specific: set OGRE env vars (via `windows_utils`)
13. Get sources from gazebodistro (via `colcon_utils`)
14. Setup workspace, move package source
15. Resolve colcon package name (via `colcon_utils`)
16. Build workspace (via `colcon_utils`)
17. Run tests and export results (via `colcon_utils`)
18. Cleanup workspace

Steps marked "platform-specific" will import from `windows_utils` (or future `macos_utils`) based on `sys.platform`.

---

## Step 6: Update DSL to call Python

Modify `jenkins-scripts/dsl/gazebo_libs.dsl`.

Change `add_win_devel_bat_call()` (lines 195-215) to call the Python script:

```groovy
void add_win_devel_bat_call(gz_win_ci_job, lib_name, ws_checkout_dir, ci_config)
{
  def conda_env = ci_config.system.version
  gz_win_ci_job.with
  {
    steps {
      batchFile("""\
            set VCS_DIRECTORY=${ws_checkout_dir}
            if "%CONDA_ENV_NAME%" == "" set CONDA_ENV_NAME=${conda_env}
            python "./scripts/jenkins-scripts/lib/colcon_default_devel.py" ^
              --package ${lib_name} ^
              --vcs-directory ${ws_checkout_dir} ^
              --auto-major-version ^
              --conda-env-name %CONDA_ENV_NAME%
            """.stripIndent())
    }
  }
  generate_label_by_requirements(gz_win_ci_job, lib_name, ci_config.requirements, 'win')
}
```

Note: keeps `batchFile()` since this is still Windows. The batch wrapper only sets env vars and calls the Python script. Future macOS migration changes this to `shell()`.

---

## Step 7: Delete old batch files

After the Python implementation is verified working:

- `jenkins-scripts/lib/colcon-default-devel-windows.bat`
- `jenkins-scripts/lib/windows_library.bat`
- `jenkins-scripts/lib/windows_env_vars.bat`
- All 16 `jenkins-scripts/gz_*-default-devel-windows-amd64.bat`
- `jenkins-scripts/sdformat-default-devel-windows-amd64.bat`

Keep `jenkins-scripts/lib/generic-default-devel-windows.bat` — it is used by non-conda builds (plain cmake/nmake) and is **out of scope**.

---

## Files summary

| Action | File | Purpose |
|--------|------|---------|
| **Create** | `jenkins-scripts/lib/build_utils.py` | Shared utilities (run_cmd, section, download) |
| **Create** | `jenkins-scripts/lib/pixi_utils.py` | Pixi/conda operations |
| **Create** | `jenkins-scripts/lib/colcon_utils.py` | Colcon build/test/workspace operations |
| **Create** | `jenkins-scripts/lib/windows_utils.py` | Windows-specific (MSVC, GPU, OGRE paths) |
| **Create** | `jenkins-scripts/lib/colcon_default_devel.py` | Main orchestrator entry point |
| **Modify** | `jenkins-scripts/dsl/gazebo_libs.dsl` | Call Python instead of .bat |
| **Delete** | `jenkins-scripts/lib/colcon-default-devel-windows.bat` | Replaced by Python |
| **Delete** | `jenkins-scripts/lib/windows_library.bat` | Replaced by Python modules |
| **Delete** | `jenkins-scripts/lib/windows_env_vars.bat` | Replaced by pixi_utils constants |
| **Delete** | 17x `*-default-devel-windows-amd64.bat` | Replaced by CLI args |

## Files unchanged

- `conda/envs/*/pixi.toml` and `pixi.lock` — environment definitions
- `conda/config-detector/pixi.toml` — bootstrap environment
- `jenkins-scripts/dsl/tools/get_conda_ciconfig_from_package_and_version.py` — imported by pixi_utils
- `jenkins-scripts/dsl/tools/get_ciconfigs_from_package_and_version.py` — core detection logic
- `jenkins-scripts/tools/detect_cmake_major_version.py` — kept for standalone use
- `jenkins-scripts/dsl/gz-collections.yaml` — no changes needed for Windows
- `jenkins-scripts/lib/generic-default-devel-windows.bat` — out of scope (non-conda builds)

---

## Future macOS migration (out of scope, design only)

When macOS support is needed:
1. Create `jenkins-scripts/lib/macos_utils.py` with macOS equivalents (no MSVC, no GPU check, macOS OGRE paths)
2. Update `pixi_utils.py` URL/binary logic (already has `sys.platform` abstraction points)
3. Update `conda/envs/*/pixi.toml` to add `osx-64`/`osx-arm64` platforms
4. Add macOS ci_configs in `gz-collections.yaml`
5. Add `shell()` DSL step for macOS jobs in `gazebo_libs.dsl`

---

## Verification

1. **Syntax check**: `python -m py_compile` on each new .py file
2. **CLI test**: `python colcon_default_devel.py --help` produces expected usage
3. **Unit-level**: Verify each module imports cleanly and functions have correct signatures
4. **DSL validation**: Run the DSL seed job to ensure generated XML is valid
5. **End-to-end**: Run on a Windows Jenkins agent with `gz-cmake` (simplest package, no GPU needed). Use the jenkins-cli skill with https://build.osrfoundation.org/job/gz_cmake-pr-cnlwin/
