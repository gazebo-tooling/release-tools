set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=gz-launch
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set DEPEN_PKGS=assimp boost-core ccd cppzmq cuda curl dlfcn-win32 eigen3 fcl ffmpeg freeimage gflags glib gts jsoncpp libyaml libzip ogre ogre2 ogre22 openssl protobuf qt5 qwt tinyxml2 zeromq
for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set GZ_MAJOR_VERSION=%%i
if %GZ_MAJOR_VERSION% GEQ 6 (
  set DEPEN_PKGS=%DEPEN_PKGS% gdal
)
set COLCON_PACKAGE=gz-launch
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
