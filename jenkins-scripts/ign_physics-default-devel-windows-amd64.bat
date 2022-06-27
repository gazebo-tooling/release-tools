set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=ign-physics
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set DEPEN_PKGS=eigen3 tinyxml2
for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set IGN_MAJOR_VERSION=%%i
if %IGN_MAJOR_VERSION% GEQ 6 (
  set DEPEN_PKGS=%DEPEN_PKGS% gdal
)
:: This needs to be migrated to DSL to get multi-major versions correctly
set COLCON_PACKAGE=ignition-physics
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%/lib/colcon-default-devel-windows.bat"
