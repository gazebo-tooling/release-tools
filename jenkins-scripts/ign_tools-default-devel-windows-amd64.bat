set SCRIPT_DIR="%~dp0"

set VCS_DIRECTORY=ign-tools
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true
set ENABLE_TESTS=false

call "%SCRIPT_DIR%/lib/generic-default-devel-windows.bat"
