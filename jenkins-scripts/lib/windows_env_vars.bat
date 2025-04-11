set "PIXI_VERSION=0.44.0"
set "PIXI_URL=https://github.com/prefix-dev/pixi/releases/download/v%PIXI_VERSION%/pixi-x86_64-pc-windows-msvc.exe"
if NOT DEFINED PIXI_PROJECT_PATH (
  set "PIXI_PROJECT_PATH=%TMP%\pixi\project"
)
set "PIXI_TMPDIR=%TMP%\pixi"
set "PIXI_TMP=%PIXI_TMPDIR%\pixi.exe"
set "CONDA_ENVS_DIR=%SCRIPT_DIR%\..\conda\envs\"

if NOT DEFINED EXIT_ON_ERROR (
  set EXIT_ON_ERROR=
)

goto :EOF
