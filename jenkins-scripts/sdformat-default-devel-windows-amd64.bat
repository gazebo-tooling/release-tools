set SCRIPT_DIR="%~dp0"

call "%SCRIPT_DIR%/lib/windows_configuration.bat"
REM x64 or x86_amd64
set VCS_DIRECTORY=sdformat
set PLATFORM_TO_BUILD=x86_amd64
set ARG_CMAKE_FLAGS=%CMAKE_BOOST_FLAGS%
set DEPENDENCY_PKG=boost_1_56_0.zip

call "%SCRIPT_DIR%/lib/project-default-devel-windows.bat"
