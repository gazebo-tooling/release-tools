@echo on
set SCRIPT_DIR="%~dp0"

set VCS_DIRECTORY=ign-rendering
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true
:: dlfcn
set DEPEN_PKGS="dlfcn-win32 cuda freeimage ogre"
:: This needs to be migrated to DSL to get multi-major versions correctly
set COLCON_PACKAGE=ignition-rendering0
set GAZEBODISTRO_FILE=ign-rendering.yaml

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
