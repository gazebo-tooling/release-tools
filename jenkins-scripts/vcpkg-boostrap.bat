set SCRIPT_DIR=%~dp0

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc2019_compiler
echo # END SECTION

:: Remove previous vcpkg installation
rd /s /q %VCPKG_DIR%

:: Clone and use the new version of vcpkg
git clone https://github.com/microsoft/vcpkg -C %VCPKG_DIR% || goto :error
echo "Using SNAPSHOT: %VCPKG_SNAPSHOT%"
git checkout %VCPKG_SNAPSHOT% -C %VCPKG_DIR% || goto :error
:: Bootstrap vcpkg.exe
%VCPKG_DIR%\bootstrap-vcpkg.bat || goto :error
