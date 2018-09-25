@echo on
set SCRIPT_DIR="%~dp0"

set VCS_DIRECTORY=ign-gui
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

:: boost for sdformat
set DEPEN_PKGS="qt5 qwt protobuf tinyxml2 cppzmq zeromq boost-any boost-variant"
:: This needs to be migrated to DSL to get multi-major versions correctly
set COLCON_PACKAGE=ignition-gui0
set GAZEBODISTRO_FILE=ign-gui0.yaml

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
