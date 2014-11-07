@echo on

set SCRIPT_DIR=%~dp0
set PLATFORM_TO_BUILD=amd64
set IGN_TEST_DISABLE=TRUE

call %SCRIPT_DIR%/lib/haptix_comm-base-windows.bat
