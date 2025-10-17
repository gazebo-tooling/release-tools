:: needed to import functions from other batch files
call :%*
exit /b

:: ##################################
:: Configure the build environment for MSVC 2017
:configure_msvc2019_compiler
::
::

if defined VSCMD_VER (
  @echo "VS compiler already configured"
  goto :EOF
)

:: See: https://issues.jenkins-ci.org/browse/JENKINS-11992
@set path=%path:"=%

:: By default should be the same
set MSVC_KEYWORD=%PLATFORM_TO_BUILD%

IF %PLATFORM_TO_BUILD% == x86 (
  echo "Using 32bits VS configuration"
  set BITNESS=32
) ELSE (
  REM Visual studio is accepting many keywords to compile for 64bits
  REM We need to set x86_amd64 to make express version to be able to
  REM Cross compile from x86 -> amd64
  echo "Using 64bits VS configuration"
  set BITNESS=64
  set MSVC_KEYWORD=x86_amd64
  set PLATFORM_TO_BUILD=amd64
  set PreferredToolArchitecture=x64
)

echo "Configure the VC++ compilation"
set MSVC22_ON_WIN32_C=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat
:: 2019 versions
set MSVC_ON_WIN64_E=C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\VC\Auxiliary\Build\vcvarsall.bat
set MSVC_ON_WIN32_E=C:\Program Files\Microsoft Visual Studio\2019\Enterprise\VC\Auxiliary\Build\vcvarsall.bat
set MSVC_ON_WIN64_C=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat
set MSVC_ON_WIN32_C=C:\Program Files\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat

set LIB_DIR="%~dp0"
call %LIB_DIR%\windows_env_vars.bat

IF exist "%MSVC22_ON_WIN32_C%" (
   call "%MSVC22_ON_WIN32_C%" %MSVC_KEYWORD% || goto %win_lib% :error
) ELSE IF exist "%MSVC_ON_WIN64_E%" (
   call "%MSVC_ON_WIN64_E%" %MSVC_KEYWORD% || goto %win_lib% :error
) ELSE IF exist "%MSVC_ON_WIN32_E%" (
   call "%MSVC_ON_WIN32_E%" %MSVC_KEYWORD% || goto %win_lib% :error
) ELSE IF exist "%MSVC_ON_WIN64_C%" (
   call "%MSVC_ON_WIN64_C%" %MSVC_KEYWORD% || goto %win_lib% :error
) ELSE IF exist "%MSVC_ON_WIN32_C%" (
   call "%MSVC_ON_WIN32_C%" %MSVC_KEYWORD% || goto %win_lib% :error
) ELSE (
   echo "Could not find the vcvarsall.bat file"
   exit %EXTRA_EXIT_PARAM% -1
)

goto :EOF

:: ##################################
:: Download an URL to the current directory
:wget
:: arg1 URL to download
:: arg2 filename (not including the path, just the filename)
set URL=%~1
set FILENAME=%~2
set RETRIES=3
set COUNT=0
echo Downloading %URL%
:retry
powershell -command "Invoke-WebRequest -Uri %URL% -OutFile %cd%\%FILENAME%"
if errorlevel 1 (
    set /a COUNT+=1
    echo Download failed. Retry attempt !COUNT! of %RETRIES%.
    if !COUNT! geq %RETRIES% (
        echo Maximum retry attempts reached. exit %EXTRA_EXIT_PARAM%ing...
        exit %EXTRA_EXIT_PARAM% 1
    )
    timeout /t 5 >nul
    goto retry
)
goto :EOF

:: ##################################
:get_source_from_gazebodistro
::
:: arg1: Name of the yaml file in the gazebodistro repro
:: arg2: directory destination (default .)
setlocal EnableDelayedExpansion
set gzdistro_dir=gazebodistro

if "%GAZEBODISTRO_BRANCH%" == "" (set GAZEBODISTRO_BRANCH=master)

if exist %gzdistro_dir% (rmdir /s /q %gzdistro_dir%)
git clone https://github.com/gazebo-tooling/gazebodistro %gzdistro_dir% -b %GAZEBODISTRO_BRANCH%
:: Check if ci_matching_branch name is used
if "%ghprbSourceBranch%" == "" (echo ghprbSourceBranch is unset) else (
  python "%SCRIPT_DIR%\tools\detect_ci_matching_branch.py" "%ghprbSourceBranch%"
  :: "if ERRORLEVEL N" tests for ">= N"
  :: To test for error code == 0, use !(error >= 1)
  if NOT ERRORLEVEL 1 (
    echo trying to checkout branch %ghprbSourceBranch% from gazebodistro
    git -C %gzdistro_dir% fetch origin %ghprbSourceBranch% || rem
    git -C %gzdistro_dir% checkout %ghprbSourceBranch% || rem
  ) else (
    echo branch name %ghprbSourceBranch% is not a match
  )
  :: print branch for informational purposes
  git -C %gzdistro_dir% branch
)
vcs import --retry 5 --force < "%gzdistro_dir%\%1" "%2" || goto :error
vcs pull || goto :error
goto :EOF

:: ##################################
:_colcon_build_cmd
:: arg1 colcon command line extra arguments
:: arg2 colcon cmake extra command line arguments
set LIB_DIR="%~dp0"
call %LIB_DIR%\windows_env_vars.bat

:: batch is failing to parse correctly two arguments (--package-select foo)
:: in just one variable. Workaround here passing EXTRA_ARGS + COLCON_PACKAGE
set COLCON_EXTRA_ARGS=%1
set COLCON_PACKAGE=%2
set COLCON_EXTRA_CMAKE_ARGS=%3
set COLCON_EXTRA_CMAKE_ARGS2=%4

:: TODO: be sure that this way of defining MAKEFLAGS is working
set MAKEFLAGS=-j%MAKE_JOBS%

echo "COLCON_EXTRA_ARGS: %COLCON_EXTRA_ARGS% %COLCON_PACKAGE%"

@echo on
colcon build --build-base "build"^
  --install-base "install"^
  --parallel-workers %MAKE_JOBS%^
  %COLCON_EXTRA_ARGS% %COLCON_PACKAGE%^
  --cmake-args " -DCMAKE_BUILD_TYPE=%BUILD_TYPE%"^
  %COLCON_EXTRA_CMAKE_ARGS% %COLCON_EXTRA_CMAKE_ARGS2%^
  --event-handler console_cohesion+ || goto :error
@echo off

goto :EOF

:: ##################################
::
:: Build all the workspaces packages except the package provided in arg1
::
:: arg1: name of the colcon package excluded from building
:: arg2: extra cmake parameter to pass to the target package COLCON_PACKAGE
:build_workspace

set COLCON_PACKAGE=%1
set _COLCON_EXTRA_CMAKE_ARGS=%2

:: Check if package is in colcon workspace
echo # BEGIN SECTION Packages in workspace:
colcon list --names-only
echo # END SECTION

:: two runs to get the dependencies built with testing and the package under
:: test build with tests
echo # BEGIN SECTION: colcon compilation without test for dependencies of !COLCON_PACKAGE!
call :_colcon_build_cmd --packages-skip !COLCON_PACKAGE! "-DBUILD_TESTING=0" "-DCMAKE_CXX_FLAGS=-w"
echo # END SECTION
echo # BEGIN SECTION: colcon compilation with tests for !COLCON_PACKAGE!
call :_colcon_build_cmd --packages-select !COLCON_PACKAGE! %_COLCON_EXTRA_CMAKE_ARGS% " -DBUILD_TESTING=1"
echo # END SECTION
goto :EOF

:: ##################################
:list_workspace_pkgs
colcon list -t || goto :error
vcs export --exact || goto :error
goto :EOF

:: ##################################
:tests_in_workspace
:: arg1: package whitelist to test
set COLCON_PACKAGE=%1

echo # BEGIN SECTION: colcon test for !COLCON_PACKAGE!
colcon test --install-base "install"^
            --packages-select !COLCON_PACKAGE!^
            --executor sequential^
            --event-handler console_direct+
echo # END SECTION
echo # BEGIN SECTION: colcon test-result
colcon test-result --all
echo # END SECTION
goto :EOF

:: ##################################
:: 
:: Download the pixi binary to the system
:pixi_installation
set LIB_DIR=%~dp0
call %LIB_DIR%\windows_env_vars.bat

echo Downloading pixi %PIXI_VERSION% in %PIXI_TMPDIR%
if not exist "%PIXI_TMPDIR%" mkdir "%PIXI_TMPDIR%"
pushd %PIXI_TMPDIR%
call :wget "%PIXI_URL%" pixi.exe
if errorlevel 1 exit %EXTRA_EXIT_PARAM% 1
popd 
goto :EOF

:: ##################################
:: 
:: Create the pixi bootstrap environment
:pixi_create_bootstrap_environment

set LIB_DIR="%~dp0"
call %LIB_DIR%\windows_env_vars.bat

if exist %PIXI_BOOTSTRAP_PROJECT_PATH% ( rmdir /s /q "%PIXI_BOOTSTRAP_PROJECT_PATH%")
mkdir "%PIXI_BOOTSTRAP_PROJECT_PATH%"
copy %CONDA_ROOT_DIR%\config-detector\pixi.* %PIXI_BOOTSTRAP_PROJECT_PATH%
pushd %PIXI_BOOTSTRAP_PROJECT_PATH%
call %win_lib% :pixi_bootstrap_cmd install
if errorlevel 1 exit %EXTRA_EXIT_PARAM% 1
popd
goto :EOF

:: ##################################
::
:: Create a pixi environment
:pixi_create_gz_environment
:: arg1: environment name inside conda/envs in this repo
set LIB_DIR="%~dp0"
call %LIB_DIR%\windows_env_vars.bat

set ENV_NAME=%1
set CONDA_ENV_PATH=%CONDA_ENVS_DIR%\%ENV_NAME%
if not exist %CONDA_ENV_PATH% (
  echo Can not find %CONDA_ENV_PATH% directory in the system
  exit %EXTRA_EXIT_PARAM% 1
)
if exist %PIXI_PROJECT_PATH% ( rmdir /s /q "%PIXI_PROJECT_PATH%")
if errorlevel 1 exit %EXTRA_EXIT_PARAM% 1
mkdir %PIXI_PROJECT_PATH%
copy %CONDA_ENV_PATH%\pixi.* %PIXI_PROJECT_PATH%
if errorlevel 1 exit %EXTRA_EXIT_PARAM% 1
pushd %PIXI_PROJECT_PATH%
if errorlevel 1 exit %EXTRA_EXIT_PARAM% 1
call %win_lib% :pixi_cmd install
if errorlevel 1 exit %EXTRA_EXIT_PARAM% 1
popd
goto :EOF

:: ##################################
:pixi_load_bootstrap_shell
:: pixi shell won't work since it spawns a blocking cmd inside Jenkins
:: instead use the hook and execute them in the current shell
set LIB_DIR="%~dp0"
call %LIB_DIR%\windows_env_vars.bat
set HOOK_FILE=%PIXI_BOOTSTRAP_PROJECT_PATH%\hooks.bat

pushd %PIXI_BOOTSTRAP_PROJECT_PATH%
echo Running pixi %~1 %~2
%PIXI_TMP% shell-hook --locked > %HOOK_FILE%
:: ERRORS in hooks will make the build to fail. Be permissive
type %HOOK_FILE%
call %HOOK_FILE%
popd
goto :EOF

:: ##################################
:pixi_load_shell
:: pixi shell won't work since it spawns a blocking cmd inside Jenkins
:: instead use the hook and execute them in the current shell
set LIB_DIR="%~dp0"
call %LIB_DIR%\windows_env_vars.bat
set HOOK_FILE=%PIXI_PROJECT_PATH%\hooks.bat

pushd %PIXI_PROJECT_PATH%
echo Running pixi %~1 %~2
%PIXI_TMP% shell-hook --locked > %HOOK_FILE%
:: ERRORS in hooks will make the build to fail. Be permissive
type %HOOK_FILE%
call %HOOK_FILE%
popd
goto :EOF

:pixi_bootstrap_cmd
:: arg1 pixi command to run on PIXI_BOOTSTRAP_PROJECT_PATH
:: arg2 pixi second argument
set LIB_DIR="%~dp0"
call %LIB_DIR%\windows_env_vars.bat
echo Running pixi %~1 %~2 %~3 %~4
:: If using --manifest-file Windows will complain about permissions
:: using error number 5 :?. Use pushd and popd to go into the
:: project directory.
pushd %PIXI_BOOTSTRAP_PROJECT_PATH%
call "%PIXI_TMP%" %1 %2 %3 %4
if errorlevel 1 exit %EXTRA_EXIT_PARAM% 1
popd
goto :EOF


:: ##################################
:pixi_cmd
:: arg1 pixi command to run on PIXI_PROJECT_PATH
:: arg2 pixi second argument
set LIB_DIR="%~dp0"
call %LIB_DIR%\windows_env_vars.bat
echo Running pixi %~1 %~2
:: If using --manifest-file Windows will complain about permissions
:: using error number 5 :?. Use pushd and popd to go into the 
:: project directory.
pushd %PIXI_PROJECT_PATH%
call "%PIXI_TMP%" %1 %2
if errorlevel 1 exit %EXTRA_EXIT_PARAM% 1
popd
goto :EOF

:: ##################################
:: Detect conda environment for a package and major version
:detect_conda_env
:: arg1: package name
:: arg2: major version (optional)

set "SCRIPT_DIR=%~dp0"
set "PACKAGE_NAME=%~1"
set "MAJOR_VERSION=%~2"

for /f %%i in ('python "%SCRIPT_DIR%\..\dsl\tools\get_ciconfigs_from_package_and_version.py" "--yaml-file" "%SCRIPT_DIR%\..\dsl\gz-collections.yaml" "%PACKAGE_NAME%" "%MAJOR_VERSION%"') do (
  set "CONDA_ENV_NAME=%%i"
)

echo Detected conda environment: %CONDA_ENV_NAME%
goto :EOF

:: ##################################
:error - error routine
::
echo Failed in windows_library with error #%errorlevel%.
exit %EXTRA_EXIT_PARAM% %errorlevel%
