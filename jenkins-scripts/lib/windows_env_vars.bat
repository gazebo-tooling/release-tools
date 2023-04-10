if exist D:\vcpkg (
  set VCPKG_DIR=D:\vcpkg
) else if exist C:\vcpkg (
    set VCPKG_DIR=C:\vcpkg
) else (
   echo "Can not find a vcpkg installation"
   exit -1
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
goto :EOF
