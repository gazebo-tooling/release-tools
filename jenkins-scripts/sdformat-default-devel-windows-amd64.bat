@echo on

set SCRIPT_DIR=%~dp0

if not defined VCS_DIRECTORY set VCS_DIRECTORY=sdformat
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set DEPEN_PKGS="libxml2 tinyxml2"

set COLCON_PACKAGE=sdformat
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
