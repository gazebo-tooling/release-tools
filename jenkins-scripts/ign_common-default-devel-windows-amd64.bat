set SCRIPT_DIR="%~dp0"

set VCS_DIRECTORY=ign-common
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set DEPEN_PKGS="ffmpeg freeimage gts tinyxml2"

call "%SCRIPT_DIR%/lib/generic-default-devel-windows.bat"
