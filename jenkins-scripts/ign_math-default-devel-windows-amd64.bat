set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=gz-math
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true
set DEPEN_PKGS=pybind11
set COLCON_PACKAGE=gz-math
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
