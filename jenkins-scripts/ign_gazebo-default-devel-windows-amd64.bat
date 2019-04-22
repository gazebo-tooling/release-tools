@echo on
set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=ign-gazebo
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true
:: dlfcn
set DEPEN_PKGS="dlfcn-win32 cuda curl jsoncpp freeimage ogre gts glib fcl eigen3 ccd assimp libyaml libzip gflags protobuf"
:: This needs to be migrated to DSL to get multi-major versions correctly
set COLCON_PACKAGE=ignition-gazebo
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
