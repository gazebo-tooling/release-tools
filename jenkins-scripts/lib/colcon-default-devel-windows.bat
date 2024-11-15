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


setlocal ENABLEDELAYEDEXPANSION
if not defined GAZEBODISTRO_FILE (
  for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set PKG_MAJOR_VERSION=%%i
  if errorlevel 1 exit 1
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

echo # BEGIN SECTION: remove vcpkg install directory
:: Prepare a clean vcpkg environment with external dependencies
call %win_lib% :remove_vcpkg_installation || goto :error
echo # END SECTION

if defined USE_PIXI (
  if not defined REUSE_PIXI_INSTALLATION (
    echo # BEGIN SECTION: pixi: installation
    call %win_lib% :pixi_installation || goto :error
    echo # END SECTION

    echo # BEGIN SECTION: pixi: create legacy environment
    call %win_lib% :pixi_create_gz_environment_legacy || goto :error
    echo # END SECTION
  )

  echo # BEGIN SECTION: pixi: info
  call %win_lib% :pixi_cmd info || goto :error
  echo # END SECTION

  echo # BEGIN SECTION: pixi: list packages
  call %win_lib% :pixi_cmd list || goto :error
  echo # END SECTION

  echo # BEGIN SECTION: pixi: enable shell
  call %win_lib% :pixi_load_shell
  echo # END SECTION
) else (
  :: Call vcvarsall and all the friends
  echo # BEGIN SECTION: configure the MSVC compiler
  call %win_lib% :configure_msvc2019_compiler
  echo # END SECTION

  echo # BEGIN SECTION: vcpkg: install all dependencies
  call %win_lib% :setup_vcpkg_all_dependencies || goto :error
  echo # END SECTION

  echo # BEGIN SECTION: vcpkg: list installed packages
  call %win_lib% :list_vcpkg_packages || goto :error
  echo # END SECTION
)

:: Check if package is in colcon workspace
echo # BEGIN SECTION: Update package !COLCON_PACKAGE! from gz to ignition
echo Packages in workspace:
colcon list --names-only

colcon list --names-only | find "!COLCON_PACKAGE!"
if errorlevel 1 (
  :: REQUIRED for Gazebo Fortress
  set COLCON_PACKAGE=!COLCON_PACKAGE:gz=ignition!
  set COLCON_PACKAGE=!COLCON_PACKAGE:sim=gazebo!
)
colcon list --names-only | find "!COLCON_PACKAGE!"
if errorlevel 1 (
  echo Failed to find package !COLCON_PACKAGE! in workspace.
  goto :error
)
echo Using package name !COLCON_PACKAGE!
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

if exist %LOCAL_WS_SOFTWARE_DIR%\configure.bat (
  echo "DEPRECATED configure.bat file detected. It should be removed from upstream sources"
)

echo # BEGIN SECTION: compiling %VCS_DIRECTORY%
cd %LOCAL_WS%
call %win_lib% :build_workspace !COLCON_PACKAGE! !COLCON_PACKAGE_EXTRA_CMAKE_ARGS! || goto :error
echo # END SECTION

if "%ENABLE_TESTS%" == "TRUE" (
    set TEST_RESULT_PATH=%WORKSPACE%\ws\build\!COLCON_PACKAGE!\test_results

    echo # BEGIN SECTION: running tests for !COLCON_PACKAGE!
    call %win_lib% :tests_in_workspace !COLCON_PACKAGE!
    echo # END SECTION

    echo # BEGIN SECTION: export testing results
    if exist %EXPORT_TEST_RESULT_PATH% ( rmdir /q /s %EXPORT_TEST_RESULT_PATH% )
    mkdir %EXPORT_TEST_RESULT_PATH%
    xcopy !TEST_RESULT_PATH! %EXPORT_TEST_RESULT_PATH% /s /i /e || goto :error
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
