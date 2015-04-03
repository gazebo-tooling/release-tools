@echo on

set SCRIPT_DIR=%~dp0
set PLATFORM_TO_BUILD=amd64

call %SCRIPT_DIR%/lib/ign_transport-base-windows.bat
