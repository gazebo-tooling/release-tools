@echo on
set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=ign-rendering
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true
:: ogre2 from osrf vcpkg-ports
set DEPEN_PKGS="boost dlfcn-win32 cuda freeimage ogre ogre2 gts glib"
:: This needs to be migrated to DSL to get multi-major versions correctly
set COLCON_PACKAGE=ignition-rendering
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
