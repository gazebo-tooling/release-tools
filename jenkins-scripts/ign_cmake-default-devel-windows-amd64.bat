set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=gz-cmake
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set COLCON_EXTRA_CMAKE_ARGS=-DBUILDSYSTEM_TESTING:BOOL=True

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
