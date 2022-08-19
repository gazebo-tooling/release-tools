@echo on
set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=gz-utils
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

:: ign-utils shouldn't have external dependencies
:: set DEPEN_PKGS=

:: This needs to be migrated to DSL to get multi-major versions correctly
set COLCON_PACKAGE=gz-utils
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
