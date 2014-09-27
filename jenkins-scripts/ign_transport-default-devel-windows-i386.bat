#!/bin/bash -x

echo" %~dp0"
echo" %~dp"

set SCRIPT_DIR="%~dp"

call "%SCRIPT_DIR%/lib/windows_configuration.bat"
set PLATFORM_TO_BUILD="32"
set ARG_CMAKE_ARGS="%WINNODE_BOOST_ROOT%"

call "%SCRIPT_DIR%/lib/project-default-devel-windows.bat"
