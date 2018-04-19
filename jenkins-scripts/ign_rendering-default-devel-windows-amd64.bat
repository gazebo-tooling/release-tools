@echo on
set SCRIPT_DIR="%~dp0"

call "%SCRIPT_DIR%\lib\windows_configuration.bat"

set VCS_DIRECTORY=ign-rendering
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true
:: set DEPEN_PKGS=qt5-x64-static-release.zip qwt_6.1.2~osrf_qt5.zip
:: This needs to be migrated to DSL to get multi-major versions  correctly
set GAZEBODISTRO_FILE="ign-rendering0.yaml"

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
