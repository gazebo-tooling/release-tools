:: Windows standard file to build Visual Studio projects
::
:: Parameters:
::   - VCS_DIRECTORY : relative path to WORKSPACE containing the sources
::   - GAZEBODISTRO_FILE : vcs yaml file in the gazebodistro repository
::   - COLCON_PACKAGE : package name to test in colcon ws
::   - BUILD_TYPE     : (default Release) [ Release | Debug ] Build type to use
::   - DEPEN_PKGS     : (optional) list of dependencies (separted by spaces)
::   - KEEP_WORKSPACE : (optional) true | false. Clean workspace at the end
::   - ENABLE_TESTS   : (optional) true | false. Do not compile and run tests
::
:: Actions
::   - Configure the compiler
::   - Clean and create the WORKSPACE/ws
::   - Download and unzip the DEPEN_PKGS
::   - configure, compile and install
::   - run tests

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat
set TEST_RESULT_PATH=%WORKSPACE%\ws\build\%COLCON_PACKAGE%\test_results
set EXPORT_TEST_RESULT_PATH=%WORKSPACE%\test_results
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
call %win_lib% :configure_msvc2017_compiler
echo # END SECTION

echo # BEGIN SECTION: setup workspace
if NOT %KEEP_WORKSPACE% == 1 (
  echo # BEGIN SECTION: preclean workspace
  IF exist %LOCAL_WS% ( rmdir /s /q %LOCAL_WS% ) || goto :error
  echo # END SECTION
)
mkdir %LOCAL_WS% || echo "Workspace already exists!"
cd %LOCAL_WS%
echo # END SECTION


echo # BEGIN SECTION: move %VCS_DIRECTORY% source to workspace
if exist %LOCAL_WS_SOFTWARE_DIR% ( rmdir /q /s %LOCAL_WS_SOFTWARE_DIR% )
xcopy %WORKSPACE%\%VCS_DIRECTORY% %LOCAL_WS_SOFTWARE_DIR% /s /e /i > xcopy_vcs_directory.log || goto :error
echo # END SECTION

echo # BEGIN SECTION: get open robotics deps (%GAZEBODISTRO_FILE%) sources into the workspace
call %win_lib% get_source_from_gazebodistro %GAZEBODISTRO_FILE% %LOCAL_WS% || goto :error
echo # END SECTION

for %%p in (%DEPEN_PKGS%) do (
  echo # BEGIN SECTION: install external dependency %%p
  call %win_lib% :install_vcpkg_package %%p || goto:error
  echo # END SECTION
)

echo # BEGIN SECTION: packages in workspace
call %win_lib% list_workspace_pkgs || goto :error
echo # END SECTION

if exist ..\configure.bat (
  echo "DEPRECATED configure.bat file detected. It should be removed from upstream sources"
)

echo # BEGIN SECTION: compiling %VCS_DIRECTORY%
cd %LOCAL_WS%
call %win_lib% build_workspace || goto :error
echo # END SECTION

if "%ENABLE_TESTS%" == "TRUE" (
    echo # BEGIN SECTION: running tests
    call %win_lib% :tests_in_workspace %COLCON_PACKAGE%
    echo # END SECTION

    echo # BEGIN SECTION: export testing results
    if exist %EXPORT_TEST_RESULT_PATH% ( rmdir /q /s %EXPORT_TEST_RESULT_PATH% )
    mkdir %EXPORT_TEST_RESULT_PATH%
    xcopy %TEST_RESULT_PATH% %EXPORT_TEST_RESULT_PATH% /s /i /e || goto :error
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
