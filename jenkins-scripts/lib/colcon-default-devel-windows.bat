:: Windows standard file to build Visual Studio projects
::
:: Parameters:
::   - VCS_DIRECTORY : relative path to WORKSPACE containing the sources
::   - GAZEBODISTRO_FILE : (optional) vcs yaml file in the gazebodistro repository
::   - COLCON_PACKAGE : package name to test in colcon ws
::   - COLCON_AUTO_MAJOR_VERSION (default false): auto detect major version from CMakeLists
::   - COLCON_PACKAGE_EXTRA_CMAKE_ARGS : (optional) CMake arg to inject into colcon
::   - BUILD_TYPE     : (default Release) [ Release | Debug ] Build type to use
::   - KEEP_WORKSPACE : (optional) true | false. Clean workspace at the end
::   - ENABLE_TESTS   : (optional) true | false. Do not compile and run tests
::
:: Actions
::   - Configure the compiler
::   - Clean and create the WORKSPACE/ws
::   - Install the binary external dependencies
::   - configure, compile and install
::   - run tests

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat
set EXPORT_TEST_RESULT_PATH=%WORKSPACE%\build\test_results
set LOCAL_WS=%WORKSPACE%\ws
set LOCAL_WS_SOFTWARE_DIR=%LOCAL_WS%\%VCS_DIRECTORY%
set LOCAL_WS_BUILD=%WORKSPACE%\build

call %SCRIPT_DIR%\lib\windows_env_vars.bat

:: default values
@if "%BUILD_TYPE%" == "" set BUILD_TYPE=Release
@if "%ENABLE_TESTS%" == "" set ENABLE_TESTS=TRUE
@if "%COLCON_AUTO_MAJOR_VERSION%" == "" set COLCON_AUTO_MAJOR_VERSION=false

setlocal ENABLEDELAYEDEXPANSION
if "%COLCON_AUTO_MAJOR_VERSION%" == "true" (
   for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set PKG_MAJOR_VERSION=%%i
   set COLCON_PACKAGE=%COLCON_PACKAGE%!PKG_MAJOR_VERSION!
   echo "MAJOR_VERSION detected: !PKG_MAJOR_VERSION!"
)

set TEST_RESULT_PATH=%WORKSPACE%\ws\build\!COLCON_PACKAGE!\test_results

setlocal ENABLEDELAYEDEXPANSION
if not defined GAZEBODISTRO_FILE (
  for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set PKG_MAJOR_VERSION=%%i
  set GAZEBODISTRO_FILE=%VCS_DIRECTORY%!PKG_MAJOR_VERSION!.yaml
) else (
  echo Using user defined GAZEBODISTRO_FILE: %GAZEBODISTRO_FILE%
)

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

echo # BEGIN SECTION: pixi: installation
call %win_lib% :pixi_installation || goto :error
echo # END SECTION

echo # BEGIN SECTION: filtering PATH
set "PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v10.2\bin;C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v10.2\libnvvp;;c:\Program Files\AdoptOpenJDK\jdk-8.0.242.08-openj9\bin;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files\Amazon\cfn-bootstrap\;C:\cinc-project\cinc\bin\;C:\ProgramData\chocolatey\bin;C:\Program Files\Git\cmd;C:\Program Files\PowerShell\7\;C:\Program Files\NVIDIA Corporation\Nsight Compute 2019.5.0\;C:\Windows\system32\config\systemprofile\AppData\Local\Microsoft\WindowsApps"
echo # END SECTION

echo # BEGIN SECTION: pixi: create legacy environment
call %win_lib% :pixi_create_gz_environment_legacy || goto :error
echo # END SECTION

echo # BEGIN SECTION: pixi: info
call %win_lib% :pixi_cmd info || goto :error
echo # END SECTION

echo # BEGIN SECTION: pixi: list packages
call %win_lib% :pixi_cmd list || goto :error
echo # END SECTION

echo # BEGIN SECTION: pixi: enable shell
call %win_lib% :pixi_load_shell
echo %PATH%
where vcs
echo # END SECTION

echo # BEGIN SECTION: setup workspace
if not defined KEEP_WORKSPACE (
  IF exist %LOCAL_WS_BUILD% (
    echo # BEGIN SECTION: preclean workspace
    rmdir /s /q %LOCAL_WS_BUILD% || goto :error
    echo # END SECTION
  )
)
mkdir %LOCAL_WS% || echo "Workspace already exists!"
cd %LOCAL_WS%
echo # END SECTION

echo # BEGIN SECTION: get open robotics deps (%GAZEBODISTRO_FILE%) sources into the workspace
if exist %LOCAL_WS_SOFTWARE_DIR% ( rmdir /q /s %LOCAL_WS_SOFTWARE_DIR% )
call %win_lib% :get_source_from_gazebodistro %GAZEBODISTRO_FILE% %LOCAL_WS% || goto :error
echo # END SECTION

:: this step is important since overwrite the gazebodistro file
echo # BEGIN SECTION: move %VCS_DIRECTORY% source to workspace
if exist %LOCAL_WS_SOFTWARE_DIR% ( rmdir /q /s %LOCAL_WS_SOFTWARE_DIR% )
xcopy %WORKSPACE%\%VCS_DIRECTORY% %LOCAL_WS_SOFTWARE_DIR% /s /e /i > xcopy_vcs_directory.log || goto :error
echo # END SECTION

echo # BEGIN SECTION: packages in workspace
call %win_lib% :list_workspace_pkgs || goto :error
echo # END SECTION

echo # BEGIN SECTION: compiling %VCS_DIRECTORY%
cd %LOCAL_WS%
call %win_lib% :build_workspace !COLCON_PACKAGE! !COLCON_PACKAGE_EXTRA_CMAKE_ARGS! || goto :error
echo # END SECTION

if "%ENABLE_TESTS%" == "TRUE" (
    echo # BEGIN SECTION: running tests for !COLCON_PACKAGE!
    call %win_lib% :tests_in_workspace !COLCON_PACKAGE!
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
exit %errorlevel%
