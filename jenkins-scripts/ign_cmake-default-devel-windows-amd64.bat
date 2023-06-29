set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=gz-cmake
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

:: This needs to be migrated to DSL to get multi-major versions correctly
set COLCON_PACKAGE=gz-cmake
set COLCON_AUTO_MAJOR_VERSION=true
set COLCON_PACKAGE_EXTRA_CMAKE_ARGS="-DBUILDSYSTEM_TESTING:BOOL=True"

setlocal ENABLEDELAYEDEXPANSION
for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set PKG_MAJOR_VERSION=%%i
if !PKG_MAJOR_VERSION! gtr 3 (
  set COLCON_PACKAGE_EXTRA_CMAKE_ARGS="%COLCON_PACKAGE_EXTRA_CMAKE_ARGS% -DGZ_ENABLE_RELOCATABLE_INSTALL=True"
)

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
