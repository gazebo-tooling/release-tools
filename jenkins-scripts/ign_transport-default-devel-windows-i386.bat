@echo on

set SCRIPT_DIR="%~dp0"

set win_lib="%SCRIPT_DIR%/lib/windows_library.bat"

REM i386 for the moment to ignition-transport
set PLATFORM_TO_BUILD=x86

IF %PLATFORM_TO_BUILD% == x86 (
  echo "Using 32bits VS configuration"
  set BITNESS=32
) ELSE (
  echo "Using 64bits VS configuration"
  set BITNESS=64
)

call %SCRIPT_DIR%/lib/ign_transport-base-windows.bat
