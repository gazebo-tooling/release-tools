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
:: conda vars
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
   exit -1
)

goto :EOF

:: ##################################
:: Download an URL to the current directory
:wget
::
:: arg1 URL to download
:: arg2 filename (not including the path, just the filename)
echo Downloading %~1
:: Note that http://gazebosim.org/distributions/win32/deps/ redirects to an https
:: version of the website. However the jenkins machine fails to validate the secure
:: https version, so we use the --no-check-certificate option to prevent wget from
:: quitting prematurely.
wget %~1 --no-check-certificate -O %cd%\%~2 || goto :error
goto :EOF

:: ##################################
:: Download the unzip utility from osrfoundation.org
:download_7za
::
if not exist 7z.dll (call :wget http://osrf-distributions.s3.us-east-1.amazonaws.com/win32/deps/7z.dll 7z.dll || goto :error)
if not exist 7z.exe (call :wget http://osrf-distributions.s3.us-east-1.amazonaws.com/win32/deps/7z.exe 7z.exe || goto :error)
goto :EOF

:: ##################################
:: Unzip using 7za
:unzip_7za
::
:: arg1 - File to unzip
echo Uncompressing %~1
IF NOT exist %~1 ( echo "Zip file does not exist: %~1" && goto :error )
7z.exe x %~1 -aoa || goto :error
goto :EOF

:: ##################################
:: Unzip using 7za and then install
:unzip_install
::
echo Uncompressing %~1 to %~d0\install
IF NOT exist %~1 ( echo "Zip file does not exist: %~1" && goto :error )
call :download_7za || goto :error
7z.exe x %~1 -aoa -o%WORKSPACE_INSTALL_DIR% || goto :error
goto :EOF

:: ##################################
:: Download some prebuilt package from our repository, unzip it using 7za, and then install it
:download_unzip_install
::
echo # BEGIN SECTION: downloading, unzipping, and installing dependency %1
call :wget https://s3.amazonaws.com/osrf-distributions/win32/deps/%1 %1 || goto :error
call :unzip_install %1 || goto :error
goto :EOF

:: ##################################
:: Install an ignition project from source
:install_ign_project
::
:: arg1: Name of the ignition project (e.g. ign-cmake, ign-math)
:: arg2: [Optional] desired branch
::
set IGN_PROJECT_DEPENDENCY_DIR=%LOCAL_WS%\%1
if exist %IGN_PROJECT_DEPENDENCY_DIR% ( rmdir /s /q %IGN_PROJECT_DEPENDENCY_DIR% )
if "%2"=="" (
  echo Installing master branch of %1
  git clone https://github.com/gazebosim/%1 %IGN_PROJECT_DEPENDENCY_DIR% -b master
) else (
  echo Installing branch %2 of %1
  git clone https://github.com/gazebosim/%1 %IGN_PROJECT_DEPENDENCY_DIR% -b %2
)
cd /d %IGN_PROJECT_DEPENDENCY_DIR%
call .\configure.bat
nmake || goto :error
nmake install || goto :error
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
echo "COLCON_EXTRA_CMAKE_ARGS: %COLCON_EXTRA_CMAKE_ARGS%"
echo "COLCON_EXTRA_CMAKE_ARGS2: %COLCON_EXTRA_CMAKE_ARGS2%"

colcon build --build-base "build"^
	     --install-base "install"^
	     --parallel-workers %MAKE_JOBS%^
	     %COLCON_EXTRA_ARGS% %COLCON_PACKAGE%^
	     --cmake-args " -DCMAKE_BUILD_TYPE=%BUILD_TYPE%"^
             %COLCON_EXTRA_CMAKE_ARGS% %COLCON_EXTRA_CMAKE_ARGS2%^
             --event-handler console_cohesion+ || goto :error
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


:install_pixi
echo "Installing pixi"

set "PIXI_VERSION=0.30.0"
set "PIXI_URL=https://github.com/prefix-dev/pixi/releases/download/v%PIXI_VERSION%/pixi-x86_64-pc-windows-msvc.exe"
set "PIXI_TMPDIR=%TMP%\pixi-%RANDOM%"
set "PIXI_TMP=%PIXI_TMPDIR%\pixi.exe"
set "REPO_ROOT=C:\Conda"
echo "REPO_ROOT is %REPO_ROOT%"
set "MINIFORGE_ROOT=%REPO_ROOT%\.pixi\envs\default"

echo Downloading pixi %PIXI_VERSION% in %PIXI_TMPDIR%
if not exist "%PIXI_TMPDIR%" mkdir "%PIXI_TMPDIR%"
pushd %PIXI_TMPDIR%
call :wget "%PIXI_URL%" pixi.exe
if errorlevel 1 exit 1
popd 
REM echo "calling certuil"
REM certutil -urlcache -split -f "%PIXI_URL%" "%PIXI_TMP%"

echo Importing environment
pushd "%REPO_ROOT%"
call :wget "https://raw.githubusercontent.com/j-rivero/conda_testing/refs/heads/main/gz-environment.yaml" "requirements.yaml"
call "%PIXI_TMP%" init --import requirements.yaml --platform win-64
if errorlevel 1 exit 1
echo Creating environment
call "%PIXI_TMP%" install
if errorlevel 1 exit 1
echo Listing environment
call "%PIXI_TMP%" list
if errorlevel 1 exit 1
popd
goto :EOF

:: ##################################
:install_miniforge
if exist %CONDA_BASE_PATH% (
  echo "Miniforge installation exists. Removing"
  del /s /f /q %CONDA_BASE_PATH%
)
mkdir %CONDA_BASE_PATH%
cd %CONDA_BASE_PATH%
call :wget https://github.com/conda-forge/miniforge/releases/download/24.7.1-0/Miniforge3-Windows-x86_64.exe Miniforge3-Windows-x86_64.exe|| goto :error
dir /s
Miniforge3-Windows-x86_64 /help
echo "Installing miniforge 1"
Miniforge3-Windows-x86_64 /InstallationType=JustMe /RegisterPython=0 /S /D=%miniforge_install%" || goto :error
echo "Installing miniforge 2"
start /wait "" Miniforge3-Windows-x86_64.exe /InstallationType=JustMe /RegisterPython=0 /S /D=%miniforge_install%" || goto :error
echo "Miniforge installed!"
%CONDA_CMD% /help
goto :EOF

:: ##################################
:conda_info
echo "Running %CONDA_CMD%"
%CONDA_CMD% info
goto :EOF

:: ##################################
:conda_list
%CONDA_CMD% list
goto :EOF

:: ##################################
:conda_create_lock_environment
%CONDA_CMD% create -n conda-lock conda-lock
goto :EOF

:: ##################################
:conda_lock_create_gazebo_env
conda-lock install -n gz-locked-env gz-environment.conda-lock.yml	
goto :EOF

:: ##################################
:error - error routine
::
echo Failed in windows_library with error #%errorlevel%.
exit %errorlevel%
