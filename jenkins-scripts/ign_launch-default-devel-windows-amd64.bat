set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=ign-launch
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set DEPEN_PKGS="dlfcn-win32 cuda cppzmq curl jsoncpp ffmpeg freeimage ogre ogre2 qt5 qwt gts glib fcl eigen3 ccd assimp libyaml libzip gflags protobuf tinyxml2 zeromq"
set COLCON_PACKAGE=ignition-launch
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat"
