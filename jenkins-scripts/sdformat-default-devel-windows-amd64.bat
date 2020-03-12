@echo on

set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=sdformat
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set SDFORMAT_MAJOR_VERSION=%%i
if %SDFORMAT_MAJOR_VERSION% GEQ 6 (
  call "%SCRIPT_DIR%/lib/generic-default-devel-windows.bat"
) else (
  call "%SCRIPT_DIR%/lib/sdformat-base-windows.bat"
)
