echo on
 
set SCRIPT_DIR=%~dp0
set PLATFORM_TO_BUILD=amd64

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc2019_compiler
echo # END SECTION

echo # BEGIN SECTION: Bootstrap vcpkg
:: Remove previous vcpkg installation
rd /s /q %VCPKG_DIR% || goto :error

:: Clone and use the new version of vcpkg
git clone https://github.com/microsoft/vcpkg %VCPKG_DIR% || goto :error
git clone https://github.com/osrf/vcpkg-ports %VCPKG_OSRF_DIR% || goto :error

echo "Using SNAPSHOT: %VCPKG_SNAPSHOT%"
cd %VCPKG_DIR%
git checkout %VCPKG_SNAPSHOT% || goto :error
:: Bootstrap vcpkg.exe
%VCPKG_DIR%\bootstrap-vcpkg.bat -disableMetrics
echo # END SECTION

echo # BEGIN SECTION: Custom patches to packages
:: 1. do not build debug builds to speed up CI
echo "set(VCPKG_BUILD_TYPE release)" >> %VCPKG_DIR%\triplets\x64-windows.cmake
:: 2. patch dfcln-win32 to avoid debug filesystem paths
:: dfcln-win32 assumes that debug build is always there.
:: remove lines with debug word. portfile is simple enough
set dfcln_portfile=%VCPKG_DIR%\ports\dfcln-win32\portfile.cmake
findstr  /v  /i "debug" %dfcln_portfile% > %dfcln_portfile%.new || goto :error
move %dfcln_portfile%.new %dfcln_portfile%
:: 3. use lowercase in ogre for releaes
:: ogre formula relies on checking for Release to install manual-link
:: replace it to lowercase
set ogre_portfile=%VCPKG_DIR%\ports\ogre\portfile.cmake
powershell -Command "(Get-Content %ogre_portfile%) -replace 'Release', 'release' | Out-File -encoding ASCII %ogre_portfile%"
echo # END SECTION
cd %WORKSPACE%
goto :EOF

:: ##################################
:error - error routine
::
echo Failed in with error #%errorlevel%.
exit /B %errorlevel%
