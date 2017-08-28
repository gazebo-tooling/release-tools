@echo on
set SCRIPT_DIR="%~dp0"

call "%SCRIPT_DIR%\lib\windows_configuration.bat"

set VCS_DIRECTORY=ign-gui
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true
set DEPEN_PKGS=

call "%SCRIPT_DIR%\lib\project-default-devel-windows.bat"
