@echo on

set SCRIPT_DIR=%~dp0
set PLATFORM_TO_BUILD=i386

:: We need ignition first
hg clone https://bitbucket.org/ignitionrobotics/ign-transport
call %SCRIPT_DIR%/lib/ign_transport-base-windows.bat

cd %WORKSPACE%/workspace
mv haptix-comm %WORKSPACE%/workspace
call %SCRIPT_DIR%/lib/haptix_comm-base-windows.bat
