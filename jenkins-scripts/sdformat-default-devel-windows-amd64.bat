@echo on

set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=sdformat
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set DEPEN_PKGS="eigen3 libxml2 tinyxml2"

for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set SDFORMAT_MAJOR_VERSION=%%i

if %SDFORMAT_MAJOR_VERSION% GEQ 10 (
  set COLCON_PACKAGE=sdformat
  set COLCON_AUTO_MAJOR_VERSION=true
  call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
) else if %SDFORMAT_MAJOR_VERSION% GEQ 6 (
  call "%SCRIPT_DIR%/lib/generic-default-devel-windows.bat"
) else (
  call "%SCRIPT_DIR%/lib/sdformat-base-windows.bat"
)
