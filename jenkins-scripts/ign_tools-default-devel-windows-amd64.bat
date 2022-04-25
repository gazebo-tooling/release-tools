set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=ign-tools
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set COLCON_PACKAGE=ignition-tools

:: override logic @ colcon-default-devel-windows.bat to handle ign-tools1 case on windows
setlocal ENABLEDELAYEDEXPANSION
for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set PKG_MAJOR_VERSION=%%i
echo "MAJOR_VERSION detected: !PKG_MAJOR_VERSION!"
if "!PKG_MAJOR_VERSION!" == "1" (
   set COLCON_PACKAGE=%COLCON_PACKAGE%
) else(
   set COLCON_PACKAGE=%COLCON_PACKAGE%!PKG_MAJOR_VERSION!
)

set COLCON_AUTO_MAJOR_VERSION=false

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
