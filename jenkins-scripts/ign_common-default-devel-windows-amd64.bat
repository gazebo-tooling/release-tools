set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=ign-common
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set DEPEN_PKGS=ffmpeg freeimage gts tinyxml2
for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set IGN_MAJOR_VERSION=%%i
if %IGN_MAJOR_VERSION% GEQ 5 (
  set DEPEN_PKGS=%DEPEN_PKGS% gdal
)

set COLCON_PACKAGE=gz-common
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
