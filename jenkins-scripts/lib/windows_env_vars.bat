set "CONDA_BASE_PATH=C:\conda"
set "miniforge_install=%CONDA_BASE_PATH%\Miniforge3"
set "CONDA_CMD=%miniforge_install%\condabin\conda.bat"

set "PIXI_VERSION=0.30.0"
set "PIXI_URL=https://github.com/prefix-dev/pixi/releases/download/v%PIXI_VERSION%/pixi-x86_64-pc-windows-msvc.exe"
set "PIXI_TMPDIR=%TMP%\pixi-%RANDOM%"
set "PIXI_TMP=%PIXI_TMPDIR%\pixi.exe"
set "MINIFORGE_ROOT=%CONDA_BASE_PATH%\.pixi\envs\default"
set "PIXI_PROJECT_PATH=%CONDA_BASE_PATH%\conda_testing"


goto :EOF
