@echo on

set SCRIPT_DIR=%~dp0
set PLATFORM_TO_BUILD=x86

cd %WORKSPACE%\haptix-comm
call "%SCRIPT_DIR%/lib/haptix_comm-make_sdk-windows.bat" %PLATFORM_TO_BUILD%
