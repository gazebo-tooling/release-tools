set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=ign-fuel-tools
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

:: tinyxml2 from msgs

set DEPEN_PKGS="libyaml libzip tinyxml2 openssl curl"
set COLCON_PACKAGE=ignition-fuel_tools
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
