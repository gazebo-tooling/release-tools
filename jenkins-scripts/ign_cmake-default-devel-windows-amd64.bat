set SCRIPT_DIR="%~dp0"

set VCS_DIRECTORY=ign-cmake
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set EXTRA_CMAKE_TEST_ARGS=-DBUILDSYSTEM_TESTING:BOOL=True

call "%SCRIPT_DIR%/lib/generic-default-devel-windows.bat"
