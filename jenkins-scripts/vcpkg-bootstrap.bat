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
:: 4. Fix for bug in hash in zeromq
:: https://github.com/microsoft/vcpkg/issues/23461
set zeromq_portfile=%VCPKG_DIR%\ports\zeromq\portfile.cmake
powershell -Command "(Get-Content %zeromq_portfile%) -replace '64e6d37ab843e5b9aa9e56ba7904423ce0a2c6b4101dbd86b7b8b22c52c384ed7ea9764f9e0a53be04e7ade09923ca95452104e9760b66ebc0ed3ffef08a75c5', '42663c9b16a09a5c30d61a027c544ea318a9f745129579dcc0d5dd2d529be42e8dbaee1b9406497c4da7815fa60fc877d2e26f807135b2bbc0ea7ea4214b8af6' | Out-File -encoding ASCII %zeromq_portfile%" || goto :error
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
