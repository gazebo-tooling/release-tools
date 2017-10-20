set SCRIPT_DIR="%~dp0"

set VCS_DIRECTORY=ign-cmake
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

:: The ign-cmake CI does not currently support testing
set ENABLE_TESTS=FALSE

call "%SCRIPT_DIR%/lib/generic-default-devel-windows.bat"
