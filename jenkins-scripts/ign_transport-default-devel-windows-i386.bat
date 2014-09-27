#!/bin/bash -x

call "lib/windows_configuration.bat"
set PLATFORM_TO_BUILD="32"
set ARG_CMAKE_ARGS="%WINNODE_BOOST_ROOT%"

call "lib/project-default-devel-windows.bat"
