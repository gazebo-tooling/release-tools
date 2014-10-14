set SCRIPT_DIR="%~dp0"

call "%SCRIPT_DIR%/lib/windows_configuration.bat"
REM x64 or x86
set PLATFORM_TO_BUILD=x86
set ARG_CMAKE_FLAGS=

call "%SCRIPT_DIR%/lib/project-default-devel-windows.bat"
