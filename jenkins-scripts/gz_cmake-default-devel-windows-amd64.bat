set SCRIPT_DIR=%~dp0

if not defined VCS_DIRECTORY set VCS_DIRECTORY=gz-cmake
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

:: This needs to be migrated to DSL to get multi-major versions correctly
set COLCON_PACKAGE=gz-cmake
set COLCON_AUTO_MAJOR_VERSION=true
set COLCON_PACKAGE_EXTRA_CMAKE_ARGS="-DBUILDSYSTEM_TESTING:BOOL=True"

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
