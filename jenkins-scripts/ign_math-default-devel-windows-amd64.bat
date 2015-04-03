set SCRIPT_DIR="%~dp0"

call "%SCRIPT_DIR%/lib/windows_configuration.bat"

set VCS_DIRECTORY=ignition-math
set PLATFORM_TO_BUILD=x86_amd64

call "%SCRIPT_DIR%/lib/project-default-devel-windows.bat"
