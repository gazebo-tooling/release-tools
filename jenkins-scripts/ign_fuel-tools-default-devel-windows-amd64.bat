set SCRIPT_DIR="%~dp0"

set VCS_DIRECTORY=ign-fuel-tools
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

:: tinyxml2 from msgs
set DEPEN_PKGS="libyaml libzip tinyxml2"

call "%SCRIPT_DIR%/lib/generic-default-devel-windows.bat"
