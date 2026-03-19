@echo on
set SCRIPT_DIR=%~dp0

if not defined VCS_DIRECTORY set VCS_DIRECTORY=gz-sensors
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true
set COLCON_PACKAGE=gz-sensors
set COLCON_AUTO_MAJOR_VERSION=true
set GPU_SUPPORT_NEEDED=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
