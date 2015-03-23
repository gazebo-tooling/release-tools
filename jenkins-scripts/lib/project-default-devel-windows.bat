REM Windows standard file to build Visual Studio projects

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc_compiler
echo # END SECTION

echo # BEGIN SECTION: setup workspace
cd %WORKSPACE%
IF exist workspace ( rmdir /s /q workspace ) || goto %win_lib% :error
mkdir workspace
cd workspace
echo # END SECTION

echo # BEGIN SECTION: downloading and unzip dependency %DEPENDENCY_PKG%
REM Todo: support multiple dependencies
if defined DEPENDENCY_PKG (
  call %win_lib% :download_7za
  call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/%DEPENDENCY_PKG% %DEPENDENCY_PKG% || goto :error
  call %win_lib% :unzip_7za %DEPENDENCY_PKG% %DEPENDENCY_PKG% > install_boost.log || goto:error
)
echo # END SECTION

echo # BEGIN SECTION: move %VCS_DIRECTORY% source to workspace
xcopy %WORKSPACE%\%VCS_DIRECTORY% %VCS_DIRECTORY% /s /e /i > xcopy.log || goto :error
cd %VCS_DIRECTORY% || goto :error
mkdir build
cd build
echo # END SECTION

if exist ..\configure.bat (
  echo # BEGIN SECTION: configuring %VCS_DIRECTORY% using configure.bat
  call ..\configure.bat || goto :error
) else (
  echo # BEGIN SECTION: configuring %VCS_DIRECTORY% using cmake 
  cmake .. %VS_CMAKE_GEN% %VS_DEFAULT_CMAKE_FLAGS% %ARG_CMAKE_FLAGS% || goto :error
)
echo # END SECTION

echo # BEGIN SECTION: compiling %VCS_DIRECTORY%
nmake || goto %win_lib% :error
echo # END SECTION
echo # BEGIN SECTION: installing %VCS_DIRECTORY%
nmake install || goto %win_lib% :error
echo # END SECTION

echo # BEGIN SECTION: running tests
REM Need to find a way of running test from the standard make test (not working)
cd %WORKSPACE%\workspace\haptix-comm\build
ctest -C "Release" --verbose --extra-verbose || echo "test failed"
echo # END SECTION

echo # BEGIN SECTION: export testing results
set TEST_RESULT_PATH=%WORKSPACE%\test_results
if exist %TEST_RESULT_PATH% ( rmdir /q /s %TEST_RESULT_PATH% )
move test_results %TEST_RESULT_PATH% || goto :error
echo # END SECTION

if NOT DEFINED KEEP_WORKSPACE (
   echo # BEGIN SECTION: clean up workspace
   cd %WORKSPACE%
   rmdir /s /q %WORKSPACE%\workspace || goto :error
   echo # END SECTION
)
goto :EOF

:error - error routine
echo Failed with error #%errorlevel%.
exit /b %errorlevel%
