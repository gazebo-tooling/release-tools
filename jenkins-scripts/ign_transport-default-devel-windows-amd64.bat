@echo on

set SCRIPT_DIR=%~dp0
set PLATFORM_TO_BUILD=amd64
set IGN_CLEAN_WORKSPACE=true

call %SCRIPT_DIR%/lib/ign_transport-base-windows.bat
