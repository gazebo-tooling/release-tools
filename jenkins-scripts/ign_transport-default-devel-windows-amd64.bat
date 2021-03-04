@echo on

set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=ign-transport
set PLATFORM_TO_BUILD=amd64
set IGN_CLEAN_WORKSPACE=true

set DEPEN_PKGS="dlfcn-win32 cppzmq sqlite3 protobuf tinyxml2 zeromq"
set COLCON_PACKAGE=ignition-transport
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
