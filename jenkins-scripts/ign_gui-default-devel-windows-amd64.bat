@echo on
set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=gz-gui
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

:: ogre2 from vcpkg-ports
set DEPEN_PKGS=qt5 qt5-winextras qwt protobuf tinyxml2 freeimage ogre ogre22
for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set GZ_MAJOR_VERSION=%%i
if %GZ_MAJOR_VERSION% GEQ 7 (
  set DEPEN_PKGS=%DEPEN_PKGS% gdal
)
:: This needs to be migrated to DSL to get multi-major versions correctly
set COLCON_PACKAGE=gz-gui
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
