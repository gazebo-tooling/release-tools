:: Windows standard file to build Visual Studio projects
::
:: Parameters:
::   - VCS_DIRECTORY : relative path to WORKSPACE containing the sources
::   - BUILD_TYPE    : (default Release) [ Release | Debug ] Build type to use
::   - DEPEN_PKGS    : (optional) list of dependencies (separted by spaces)
::   - KEEP_WORKSPACE: (optional) true | false. Clean workspace at the end
::   - ENABLE_TESTS  : (optional) true | false. Do not compile and run tests 
::
:: Actions
::   - Configure the compiler
::   - Clean and create the WORKSPACE/ws
::   - Download and unzip the DEPEN_PKGS
::   - configure, compile and install
::   - run tests

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat
set TEST_RESULT_PATH=%WORKSPACE%\test_results
set TEST_RESULT_PATH_LEGACY=%WORKSPACE%\build\test_results
set LOCAL_WS=%WORKSPACE%\ws
set LOCAL_WS_SOFTWARE_DIR=%LOCAL_WS%\%VCS_DIRECTORY%
set WORKSPACE_INSTALL_DIR=%LOCAL_WS%\install

:: default values
@if "%BUILD_TYPE%" == "" set BUILD_TYPE=Release
@if "%ENABLE_TESTS%" == "" set ENABLE_TESTS=TRUE

:: safety checks
@if not defined VCS_DIRECTORY (
  echo # BEGIN SECTION: ERROR: VCS_DIRECTORY is not set
  echo VCS_DIRECTORY variable was not set. Please set it before calling this script
  echo # END SECTION
  exit 1
)

@if not exist %WORKSPACE%\%VCS_DIRECTORY% (
  echo # BEGIN SECTION: ERROR: %VCS_DIRECTORY% does not exist
  echo VCS_DIRECTORY variable points to %WORKSPACE%\%VCS_DIRECTORY% but it does not exists
  echo # END SECTION
  exit 1
)

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc2017_compiler
echo # END SECTION


:: Set the PATH variable so that dependencies installed inside this workspace
:: are visible to the build system and the run time.
:: NOTE: This should be called after %win_lib% :configure_msvc####_compiler so
::       that we benefit from its fix to the quotes around the PATH variable
@set PATH=%WORKSPACE_INSTALL_DIR%;%WORKSPACE_INSTALL_DIR%\bin;%WORKSPACE_INSTALL_DIR%\lib;%WORKSPACE_INSTALL_DIR%\include;%WORKSPACE_INSTALL_DIR%\share;%PATH%


echo # BEGIN SECTION: Setup Workspace
if not DEFINED KEEP_WORKSPACE (
  echo # BEGIN SECTION: Preclean Workspace
  if exist %LOCAL_WS% ( rmdir /s /q %LOCAL_WS% ) || goto :error
  echo # END SECTION
)

mkdir %LOCAL_WS% || echo "Workspace already exists!"
cd /d %LOCAL_WS%
echo # END SECTION


echo # BEGIN SECTION: move %VCS_DIRECTORY% source to workspace
if exist %LOCAL_WS_SOFTWARE_DIR% ( rmdir /q /s %LOCAL_WS_SOFTWARE_DIR% )
xcopy %WORKSPACE%\%VCS_DIRECTORY% %LOCAL_WS_SOFTWARE_DIR% /s /e /i /q || goto :error
cd /d %LOCAL_WS_SOFTWARE_DIR% || goto :error
echo # END SECTION


if exist .\configure.bat (
  echo # BEGIN SECTION: Configuring %VCS_DIRECTORY% using its configure.bat script
  call .\configure.bat "%BUILD_TYPE%" || goto :error
) else (
  echo # BEGIN SECTION: Configuring %VCS_DIRECTORY% using cmake
  md build
  cd /d build
  cmake .. -G "NMake Makefiles" -DCMAKE_INSTALL_PREFIX="%WORKSPACE_INSTALL_DIR%" || goto :error
)

:: We want to make sure that tests are enabled for this project's build
cmake .. -DBUILD_TESTING:BOOL=%ENABLE_TESTS% || echo "Second run of cmake (for enabling the tests) failed"
echo # END SECTION

echo # BEGIN SECTION: Compiling %VCS_DIRECTORY%
nmake VERBOSE=1 || goto %win_lib% :error
echo # END SECTION
echo # BEGIN SECTION: Installing %VCS_DIRECTORY%
nmake install || goto %win_lib% :error
echo # END SECTION

if "%ENABLE_TESTS%" == "TRUE" (
    echo # BEGIN SECTION: running tests
    cd /d %LOCAL_WS%\build
    :: nmake test is not working test/ directory exists and nmake is not able to handle it.
    ctest -C "%BUILD_TYPE%" --force-new-ctest-process -VV  || echo "tests failed"
    echo # END SECTION

    echo # BEGIN SECTION: Export Testing Results
    if exist %TEST_RESULT_PATH% ( rmdir /q /s %TEST_RESULT_PATH% )
    if exist %TEST_RESULT_PATH_LEGACY% ( rmdir /q /s %TEST_RESULT_PATH_LEGACY% )
    xcopy test_results %TEST_RESULT_PATH% /s /i /e || goto :error
    xcopy test_results %TEST_RESULT_PATH_LEGACY% /s /i /e || goto :error
    echo # END SECTION
)

if NOT DEFINED KEEP_WORKSPACE (
   echo # BEGIN SECTION: Clean Up Workspace
   cd /d %WORKSPACE%
   rmdir /s /q %LOCAL_WS% || goto :error
   echo # END SECTION
)
goto :EOF

:error - error routine
echo Failed with error #%errorlevel%.
exit /b %errorlevel%
