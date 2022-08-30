set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=gz-msgs
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set DEPEN_PKGS=protobuf tinyxml2
set COLCON_PACKAGE=gz-msgs
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
