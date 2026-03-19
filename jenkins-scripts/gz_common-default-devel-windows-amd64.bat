set SCRIPT_DIR=%~dp0

if not defined VCS_DIRECTORY set VCS_DIRECTORY=gz-common
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set COLCON_PACKAGE=gz-common
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
