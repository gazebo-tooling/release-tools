set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=gz-fuel-tools
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

:: tinyxml2 from msgs
set DEPEN_PKGS=libyaml libzip tinyxml2 openssl protobuf curl

set COLCON_PACKAGE=gz-fuel_tools
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"