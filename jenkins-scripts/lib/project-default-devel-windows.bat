:: Windows standard file to build Visual Studio projects
::
:: Parameters:
::   - VCS_DIRECTORY : WORKSPACE/VCS_DIRECTORY should contain the sources
::   - BUILD_TYPE    : (default Release) [ Release | Debug ] Build type to use
::   - DEPENDENCY_PKG: (optional) one package as dependency (only one is supported by now)
::   - KEEP_WORKSPACE: (optional) true | false. Clean workspace at the end
::
:: Actions
::   - Configure the compiler
::   - Clean and create the WORKSPACE/workspace
::   - Download and unzip the DEPENDENCY_PKG (if any)
::   - configure, compile and install
::   - run tests

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat
set TEST_RESULT_PATH=%WORKSPACE%\test_results
set TEST_RESULT_PATH_LEGACY=%WORKSPACE%\build\test_results

@if "%BUILD_TYPE%" == "" set BUILD_TYPE=Release

:: safety checks
if not defined VCS_DIRECTORY (
  echo # BEGIN SECTION: ERROR: VCS_DIRECTORY is not set
  echo VCS_DIRECTORY variable was not set. Please set it before calling this script
  echo # END SECTION
  exit 1
)

if not exist %WORKSPACE%\%VCS_DIRECTORY% (
  echo # BEGIN SECTION: ERROR: %VCS_DIRECTORY% does not exist
  echo VCS_DIRECTORY variable points to %WORKSPACE%\%VCS_DIRECTORY% but it does not exists
  echo # END SECTION
  exit 1
)

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
  call %win_lib% :unzip_7za %DEPENDENCY_PKG% %DEPENDENCY_PKG% > install.log || goto:error
)
echo # END SECTION

echo # BEGIN SECTION: move %VCS_DIRECTORY% source to workspace
xcopy %WORKSPACE%\%VCS_DIRECTORY% %VCS_DIRECTORY% /s /e /i > xcopy.log || goto :error
cd %VCS_DIRECTORY% || goto :error
mkdir build
cd build
echo # END SECTION

if exist ..\configure.bat (
  echo # BEGIN SECTION: configuring %VCS_DIRECTORY% in %BUILD_TYPE%
  call ..\configure.bat %BUILD_TYPE% || goto :error
) else (
  echo # BEGIN SECTION: configuring %VCS_DIRECTORY% using cmake 
  cmake .. %VS_CMAKE_GEN% %VS_DEFAULT_CMAKE_FLAGS% %ARG_CMAKE_FLAGS% || goto :error
)

echo "Workaround: to always enable the test compilation (configure.bat usually set it to false)
cmake .. -DENABLE_TESTS_COMPILATION:BOOL=True || echo "second run of cmake for enable tests failed"
echo # END SECTION

echo # BEGIN SECTION: compiling %VCS_DIRECTORY%
nmake || goto %win_lib% :error
echo # END SECTION
echo # BEGIN SECTION: installing %VCS_DIRECTORY%
nmake install || goto %win_lib% :error
echo # END SECTION

echo # BEGIN SECTION: running tests
cd %WORKSPACE%\workspace\haptix-comm\build
REM nmake test is not working test/ directory exists and nmake is not
REM able to handle it.
ctest -C "%BUILD_TYPE%" --force-new-ctest-process -VV  || echo "tests failed"
echo # END SECTION

echo # BEGIN SECTION: export testing results
if exist %TEST_RESULT_PATH% ( rmdir /q /s %TEST_RESULT_PATH% )
if exist %TEST_RESULT_PATH_LEGACY% ( rmdir /q /s %TEST_RESULT_PATH_LEGACY% )
mkdir %WORKSPACE%\build\
xcopy test_results %TEST_RESULT_PATH% /s /i /e || goto :error
xcopy %TEST_RESULT_PATH% %TEST_RESULT_PATH_LEGACY% /s /e /i
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
