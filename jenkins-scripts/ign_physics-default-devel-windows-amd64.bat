set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=ign-physics
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set DEPEN_PKGS="eigen3"
:: This needs to be migrated to DSL to get multi-major versions correctly
set COLCON_PACKAGE=ignition-physics
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%/lib/colcon-default-devel-windows.bat"
