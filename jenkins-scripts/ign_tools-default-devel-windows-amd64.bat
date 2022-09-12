set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=gz-tools
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set COLCON_PACKAGE=gz-tools

:: override logic @ colcon-default-devel-windows.bat to handle ign-tools1 case on windows
setlocal ENABLEDELAYEDEXPANSION
for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set PKG_MAJOR_VERSION=%%i
echo "MAJOR_VERSION detected: !PKG_MAJOR_VERSION!"
if "!PKG_MAJOR_VERSION!" == "1" (
   set COLCON_PACKAGE=%COLCON_PACKAGE%
) else (
   set COLCON_PACKAGE=%COLCON_PACKAGE%!PKG_MAJOR_VERSION!
)

echo # BEGIN SECTION: Update package !COLCON_PACKAGE! from gz to ignition
echo Packages in workspace:
colcon list --names-only

colcon list --names-only | find "!COLCON_PACKAGE!"
if errorlevel 1 (
  set COLCON_PACKAGE=!COLCON_PACKAGE:gz=ignition!
  set COLCON_PACKAGE=!COLCON_PACKAGE:sim=gazebo!
)
colcon list --names-only | find "!COLCON_PACKAGE!"
if errorlevel 1 (
  echo Failed to find package !COLCON_PACKAGE! in workspace.
  goto :error
)
echo Using package name !COLCON_PACKAGE!
echo # END SECTION


set COLCON_AUTO_MAJOR_VERSION=false

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
