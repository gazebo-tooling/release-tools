echo on
 
set SCRIPT_DIR=%~dp0
set PLATFORM_TO_BUILD=amd64

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc2019_compiler
echo # END SECTION

echo # BEGIN SECTION: clone vcpkg repositories
:: Remove previous vcpkg installation
rd /s /q %VCPKG_DIR% || goto :error

:: Clone and use the new version of vcpkg
git clone https://github.com/microsoft/vcpkg %VCPKG_DIR% || goto :error
git clone https://github.com/osrf/vcpkg-ports %VCPKG_OSRF_DIR% || goto :error
echo # END SECTION

echo "Using SNAPSHOT: %VCPKG_SNAPSHOT%"
cd %VCPKG_DIR%
git checkout %VCPKG_SNAPSHOT% || goto :error
echo # END SECTION

echo # BEGIN SECTION: Custom patches to packages
:: 1. do not build debug builds to speed up CI
echo set(VCPKG_BUILD_TYPE release) >> %VCPKG_DIR%\triplets\x64-windows.cmake || goto :error
:: 2. patch dfcln-win32 to avoid debug filesystem paths
:: dfcln-win32 assumes that debug build is always there.
:: remove lines with debug word. portfile is simple enough
set dfcln_portfile=%VCPKG_DIR%\ports\dlfcn-win32\portfile.cmake
findstr  /v  /i "debug" %dfcln_portfile% > %dfcln_portfile%.new || goto :error
move %dfcln_portfile%.new %dfcln_portfile% || goto :error
:: 3. use lowercase in ogre for releaes
:: ogre formula relies on checking for Release to install manual-link
:: replace it to lowercase
set ogre_portfile=%VCPKG_DIR%\ports\ogre\portfile.cmake
powershell -Command "(Get-Content %ogre_portfile%) -replace 'Release', 'release' | Out-File -encoding ASCII %ogre_portfile%" || goto :error
:: 4. Disable DEBUG libraries in gdal
set gdal_cmakefile=%VCPKG_DIR%\ports\gdal\vcpkg-cmake-wrapper.cmake
powershell -Command "(Get-Content %gdal_cmakefile%) -replace 'find_library\(GDAL_LIBRARY_DEBUG', '#find_library(GDAL_LIBRARY_DEBUG' | Out-File -encoding ASCII %gdal_cmakefile%" || goto :error
echo # END SECTION

echo "Using SNAPSHOT: bootstrap vcpkg executable"
%VCPKG_DIR%\bootstrap-vcpkg.bat -disableMetrics || goto :error
echo # END SECTION
:: TODO: the bootstrap-vcpkg.bat file seems to make Jenkins to exit no
:: matter the return code. Be careful if you use instructions at this
:: point.
cd %WORKSPACE%
goto :EOF

:: ##################################
:error - error routine
::
echo Failed in with error #%errorlevel%.
exit /B %errorlevel%
