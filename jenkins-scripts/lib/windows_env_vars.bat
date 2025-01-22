set "PIXI_VERSION=0.30.0"
set "PIXI_URL=https://github.com/prefix-dev/pixi/releases/download/v%PIXI_VERSION%/pixi-x86_64-pc-windows-msvc.exe"
set "PIXI_PROJECT_PATH=%TMP%\pixi\project"
set "PIXI_TMPDIR=%TMP%\pixi"
set "PIXI_TMP=%PIXI_TMPDIR%\pixi.exe"
set "CONDA_ENVS_DIR=%SCRIPT_DIR%\..\conda\envs\"

if exist D:\vcpkg (
  set VCPKG_DIR=D:\vcpkg
) else if exist C:\vcpkg (
    set VCPKG_DIR=C:\vcpkg
) else (
  if defined USE_PIXI (
    :: vcpkg removed by pixi. Recreate a fake one until vcpkg is removed from nodes
    if not exist VCPKG_DIR set "VCPKG_DIR=%TMP%\%RANDOM%\vcpkg"
    mkdir !VCPKG_DIR!
  ) else (
    echo "Can not find a vcpkg installation"
    exit -1
  )
)

if NOT DEFINED EXIT_ON_ERROR (
  set EXIT_ON_ERROR=
)

if DEFINED MAKE_JOBS (
  set VCPKG_MAX_CONCURRENCY=%MAKE_JOBS%
)
set VCPKG_OSRF_DIR=%VCPKG_DIR%\osrf_vcpkg_ports
set VCPKG_INSTALLED_FILES_DIR=%VCPKG_DIR%\installed
set VCPKG_CMD=%VCPKG_DIR%\vcpkg.exe
set VCPKG_CMAKE_TOOLCHAIN_FILE=%VCPKG_DIR%/scripts/buildsystems/vcpkg.cmake
if NOT DEFINED VCPKG_SNAPSHOT (
  :: see https://github.com/microsoft/vcpkg/releases
  set VCPKG_SNAPSHOT=2022.02.23
)
:: Set of common gz dependencies expected up to Garden
set VCPKG_DEPENDENCIES_LEGACY=assimp boost-core bullet3 ccd cli11 console-bridge cppzmq cuda curl dlfcn-win32 eigen3 fcl ffmpeg freeimage gdal gflags glib gts jsoncpp libyaml libzip ogre ogre2 ogre22 openssl protobuf pybind11 qt5 qt5-winextras qwt spdlog sqlite3 tinyxml2 zeromq

goto :EOF
