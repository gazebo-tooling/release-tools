if exist D:\vcpkg (
  set VCPKG_DIR=D:\vcpkg
) else if exist C:\vcpkg (
    set VCPKG_DIR=C:\vcpkg
  else
   echo "Can not find a vcpkg installation"
   exit -1
)

set VCPKG_OSRF_DIR=%VCPKG_DIR%\osrf_vcpkg_ports
set VCPKG_CMD=%VCPKG_DIR%\vcpkg.exe
set VCPKG_CMAKE_TOOLCHAIN_FILE=%VCPKG_DIR%/scripts/buildsystems/vcpkg.cmake

goto :EOF
