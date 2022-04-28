set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=gz-launch
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set DEPEN_PKGS=dlfcn-win32 cuda cppzmq curl openssl jsoncpp ffmpeg freeimage ogre ogre2 ogre22 qt5 qwt gts glib fcl eigen3 ccd assimp libyaml libzip gflags protobuf tinyxml2 zeromq
for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set IGN_MAJOR_VERSION=%%i
if %IGN_MAJOR_VERSION% GEQ 6 (
  set DEPEN_PKGS=%DEPEN_PKGS% gdal
)
set COLCON_PACKAGE=ignition-launch
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
