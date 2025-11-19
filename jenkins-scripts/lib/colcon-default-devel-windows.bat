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
::   - CONDA_ENV_NAME : (optional) conda environment name to use. If not set, it will be auto-detected based on COLCON_PACKAGE and major version
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
set LOCAL_WS_BUILD=%LOCAL_WS%\build
set LOCAL_WS_SRC=%LOCAL_WS%\src
set LOCAL_WS_SOFTWARE_DIR=%LOCAL_WS_SRC%\%VCS_DIRECTORY%

:: default values
@if "%BUILD_TYPE%" == "" set BUILD_TYPE=Release
@if "%ENABLE_TESTS%" == "" set ENABLE_TESTS=TRUE
@if "%COLCON_AUTO_MAJOR_VERSION%" == "" set COLCON_AUTO_MAJOR_VERSION=false

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

set DXDIAG_FILE=%WORKSPACE%\dxdiag.txt
if "%GPU_SUPPORT_NEEDED%" == "true" (
  echo # BEGIN SECTION: dxdiag info
  dxdiag /t %DXDIAG_FILE%
  type %DXDIAG_FILE%
  echo Checking for correct NVIDIA GPU support %DXDIAG_FILE%
  findstr /C:"Manufacturer: NVIDIA" %DXDIAG_FILE%
  if errorlevel 1 (
    echo ERROR: NVIDIA GPU not found in dxdiag
    goto :error
  )
  echo # END SECTION
)

if not defined REUSE_PIXI_INSTALLATION (
  echo # BEGIN SECTION: pixi: installation
  call %win_lib% :pixi_installation || goto :error
  echo # END SECTION
  echo # BEGIN SECTION: pixi: create bootstrap environment
  call %win_lib% :pixi_create_bootstrap_environment || goto :error
  echo # END SECTION
)

echo # BEGIN SECTION: pixi: load bootstrap shell
call %win_lib% :pixi_load_bootstrap_shell || goto :error
echo # END SECTION

setlocal ENABLEDELAYEDEXPANSION
set COLCON_PACKAGE_ORIGINAL=%COLCON_PACKAGE%
if "%COLCON_AUTO_MAJOR_VERSION%" == "true" (
  :: detect major version from CMakeLists.txt
  for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set PKG_MAJOR_VERSION=%%i
  set COLCON_PACKAGE=%COLCON_PACKAGE%!PKG_MAJOR_VERSION!
  echo "MAJOR_VERSION detected: !PKG_MAJOR_VERSION!"
)

::Auto-detect conda environment if not set
if not defined CONDA_ENV_NAME (
  echo # BEGIN SECTION: auto-detect conda environment
  call %win_lib% :detect_conda_env %COLCON_PACKAGE_ORIGINAL% !PKG_MAJOR_VERSION!
  if not defined CONDA_ENV_NAME (
    echo ERROR: Could not detect conda environment.
    exit 1
  )
  echo # END SECTION
) else (
  echo Using user-specified conda environment: %CONDA_ENV_NAME%
)

:: TODO review the above commands to avoid them in REUSE_PIXI_INSTALLATION
if not defined REUSE_PIXI_INSTALLATION (
  echo # BEGIN SECTION: pixi: create %CONDA_ENV_NAME% environment
  call %win_lib% :pixi_create_gz_environment %CONDA_ENV_NAME% || goto :error
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

echo # BEGIN SECTION: pixi: custom environment variable for gz
if "!CONDA_PREFIX!"=="" (
  echo # BEGIN SECTION: ERROR: CONDA_PREFIX is not set
  echo CONDA_PREFIX variable was not set. Please set it before calling this script
  echo # END SECTION
  goto :error
)
set OGRE_RESOURCE_PATH=!CONDA_PREFIX!\Library\bin
set OGRE2_RESOURCE_PATH=!CONDA_PREFIX!\Library\bin\OGRE-Next
echo # END SECTION

echo # BEGIN SECTION: setup workspace
if not defined KEEP_WORKSPACE (
  IF exist %LOCAL_WS% (
    echo # BEGIN SECTION: preclean workspace
    rmdir /s /q %LOCAL_WS% || goto :error
    echo # END SECTION
  )
)
mkdir %LOCAL_WS%
mkdir %LOCAL_WS_SRC%
echo # END SECTION

setlocal ENABLEDELAYEDEXPANSION
if not defined GAZEBODISTRO_FILE (
  set GAZEBODISTRO_FILE=%VCS_DIRECTORY%!PKG_MAJOR_VERSION!.yaml
) else (
  echo Using user defined GAZEBODISTRO_FILE: %GAZEBODISTRO_FILE%
)

echo # BEGIN SECTION: get open robotics deps (%GAZEBODISTRO_FILE%) sources into the workspace
call %win_lib% :get_source_from_gazebodistro %GAZEBODISTRO_FILE% %LOCAL_WS_SRC% || goto :error
echo # END SECTION

:: this step is important since overwrite the gazebodistro file
echo # BEGIN SECTION: move %VCS_DIRECTORY% source to workspace
if exist %LOCAL_WS_SOFTWARE_DIR% ( rmdir /q /s %LOCAL_WS_SOFTWARE_DIR% )
xcopy %WORKSPACE%\%VCS_DIRECTORY% %LOCAL_WS_SOFTWARE_DIR% /s /e /i > xcopy_vcs_directory.log || goto :error
echo # END SECTION

echo # BEGIN SECTION: packages in workspace
cd %LOCAL_WS%
call %win_lib% :list_workspace_pkgs || goto :error
echo # END SECTION

:: Check if package is in colcon workspace
echo # BEGIN SECTION: Check if package !COLCON_PACKAGE! is in colcon workspace
echo Packages in workspace:
colcon list --names-only

colcon list --names-only | find "!COLCON_PACKAGE!" > nul 2>&1
if errorlevel 1 (
  echo # END SECTION
  echo # BEGIN SECTION: try package name !COLCON_PACKAGE_ORIGINAL! without major version
  set COLCON_PACKAGE=!COLCON_PACKAGE_ORIGINAL!
)
colcon list --names-only | find "!COLCON_PACKAGE!"
if errorlevel 1 (
  echo # END SECTION
  echo # BEGIN SECTION: Update package !COLCON_PACKAGE! from gz to ignition
  :: REQUIRED for Gazebo Fortress
  :: Special case for gz-tools version 1 to endup being ignition-tools
  if "!COLCON_PACKAGE!" == "gz-tools" (
    if "!PKG_MAJOR_VERSION!" == "1" (
      set COLCON_PACKAGE=ignition-tools
    )
  ) else (
  set COLCON_PACKAGE=%COLCON_PACKAGE%!PKG_MAJOR_VERSION!
  set COLCON_PACKAGE=!COLCON_PACKAGE:gz=ignition!
  set COLCON_PACKAGE=!COLCON_PACKAGE:sim=gazebo!
  )
)
colcon list --names-only | find "!COLCON_PACKAGE!" > nul 2>&1
if errorlevel 1 (
  echo Failed to find package !COLCON_PACKAGE! in workspace.
  goto :error
)
echo Using package name !COLCON_PACKAGE!
echo # END SECTION

echo # BEGIN SECTION: compiling %VCS_DIRECTORY%
cd %LOCAL_WS%
call %win_lib% :build_workspace !COLCON_PACKAGE! !COLCON_PACKAGE_EXTRA_CMAKE_ARGS! || goto :error
echo # END SECTION

if "%ENABLE_TESTS%" == "TRUE" (
    set TEST_RESULT_PATH=%LOCAL_WS_BUILD%\!COLCON_PACKAGE!\test_results

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
exit %EXTRA_EXIT_PARAM% %errorlevel%
