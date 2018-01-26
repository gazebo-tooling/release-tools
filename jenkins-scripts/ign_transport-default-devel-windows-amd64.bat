@echo on

set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=ign-transport
set PLATFORM_TO_BUILD=amd64
set IGN_CLEAN_WORKSPACE=true

for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set IGN_TRANSPORT_MAJOR_VERSION=%%i
if %IGN_TRANSPORT_MAJOR_VERSION% GEQ 4 (
  call "%SCRIPT_DIR%/lib/generic-default-devel-windows.bat"
) else (
  call "%SCRIPT_DIR%/lib/ign_transport-base-windows.bat"
)
