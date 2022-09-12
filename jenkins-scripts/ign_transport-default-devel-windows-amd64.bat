@echo on

set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=gz-transport
set PLATFORM_TO_BUILD=amd64
set IGN_CLEAN_WORKSPACE=true

set DEPEN_PKGS=cppzmq dlfcn-win32 protobuf sqlite3 tinyxml2 zeromq
set COLCON_PACKAGE=gz-transport
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
