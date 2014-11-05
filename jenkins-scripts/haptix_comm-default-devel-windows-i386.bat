@echo on

set SCRIPT_DIR=%~dp0
set PLATFORM_TO_BUILD=x86
set IGN_TEST_DISABLE=TRUE

:: We need ignition first
hg clone https://bitbucket.org/ignitionrobotics/ign-transport
call %SCRIPT_DIR%/lib/ign_transport-base-windows.bat

cd %WORKSPACE%/workspace
move haptix-comm %WORKSPACE%/workspace
call %SCRIPT_DIR%/lib/haptix_comm-base-windows.bat
