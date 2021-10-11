@echo on
set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=ign-rendering
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set DEPEN_PKGS=boost-core dlfcn-win32 cuda eigen3 freeimage ogre ogre2 gts glib
for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set IGN_MAJOR_VERSION=%%i
if %IGN_MAJOR_VERSION% GEQ 5 (
  set DEPEN_PKGS=%DEPEN_PKGS% ogre22
)

:: This needs to be migrated to DSL to get multi-major versions correctly
set COLCON_PACKAGE=ignition-rendering
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
