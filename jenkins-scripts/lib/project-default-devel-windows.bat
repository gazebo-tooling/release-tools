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

:: default values
@if "%BUILD_TYPE%" == "" set BUILD_TYPE=Release
@if "%ENABLE_TESTS%" == "" set ENABLE_TESTS=TRUE

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
if NOT DEFINED KEEP_WORKSPACE (
  echo # BEGIN SECTION: preclean workspace
  IF exist %LOCAL_WS% ( rmdir /s /q %LOCAL_WS% ) || goto :error
  echo # END SECTION
)
mkdir %LOCAL_WS% || echo "Workspace already exists!"
cd %LOCAL_WS%
echo # END SECTION

for %%p in (%DEPEN_PKGS%) do (
  echo # BEGIN SECTION: downloading and unzip dependency %%p
  call %win_lib% :download_7za || goto :error
  call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/%%p %%p || goto :error
  call %win_lib% :unzip_7za %%p %%p > install.log || goto:error
)
echo # END SECTION

echo # BEGIN SECTION: move %VCS_DIRECTORY% source to workspace
if exist %LOCAL_WS_SOFTWARE_DIR% ( rmdir /q /s %LOCAL_WS_SOFTWARE_DIR% )
xcopy %WORKSPACE%\%VCS_DIRECTORY% %LOCAL_WS_SOFTWARE_DIR% /s /e /i || goto :error
cd %LOCAL_WS_SOFTWARE_DIR% || goto :error
mkdir build
cd build
echo # END SECTION

if exist ..\configure.bat (
  echo # BEGIN SECTION: configuring %VCS_DIRECTORY% in %BUILD_TYPE% / %BITNESS%
  call ..\configure.bat %BUILD_TYPE% %BITNESS% || goto :error
) else (
  echo # BEGIN SECTION: configuring %VCS_DIRECTORY% using cmake 
  cmake .. %VS_CMAKE_GEN% %VS_DEFAULT_CMAKE_FLAGS% %ARG_CMAKE_FLAGS% || goto :error
)

echo "Workaround: to always enable the test compilation (configure.bat usually set it to false)
cmake .. -DENABLE_TESTS_COMPILATION:BOOL=%ENABLE_TESTS% || echo "second run of cmake for enable tests failed"
echo # END SECTION

echo # BEGIN SECTION: compiling %VCS_DIRECTORY%
nmake VERBOSE=1 || goto %win_lib% :error
echo # END SECTION
echo # BEGIN SECTION: installing %VCS_DIRECTORY%
nmake install || goto %win_lib% :error
echo # END SECTION

if "%ENABLE_TESTS%" == "TRUE" (
    echo # BEGIN SECTION: running tests
    cd %LOCAL_WS%\build
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
)

if NOT DEFINED KEEP_WORKSPACE (
   echo # BEGIN SECTION: clean up workspace
   cd %WORKSPACE%
   rmdir /s /q %LOCAL_WS% || goto :error
   echo # END SECTION
)
goto :EOF

:error - error routine
echo Failed with error #%errorlevel%.
exit /b %errorlevel%
