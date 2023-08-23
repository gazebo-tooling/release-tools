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
  set VCPKG_DEFAULT_TRIPLET=x86-windows
) ELSE (
  REM Visual studio is accepting many keywords to compile for 64bits
  REM We need to set x86_amd64 to make express version to be able to
  REM Cross compile from x86 -> amd64
  echo "Using 64bits VS configuration"
  set BITNESS=64
  set MSVC_KEYWORD=x86_amd64
  set PLATFORM_TO_BUILD=amd64
  set VCPKG_DEFAULT_TRIPLET=x64-windows
  set PreferredToolArchitecture=x64
)

echo "Configure the VC++ compilation"
set MSVC22_ON_WIN32_C=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat
:: 2019 versions
set MSVC_ON_WIN64_E=C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\VC\Auxiliary\Build\vcvarsall.bat
set MSVC_ON_WIN32_E=C:\Program Files\Microsoft Visual Studio\2019\Enterprise\VC\Auxiliary\Build\vcvarsall.bat
set MSVC_ON_WIN64_C=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat
set MSVC_ON_WIN32_C=C:\Program Files\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat
:: libraries from vcpkg
set LIB_DIR="%~dp0"
call %LIB_DIR%\windows_env_vars.bat
set PATH=%PATH%;%VCPKG_DIR%\installed\%VCPKG_DEFAULT_TRIPLET%\bin

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
::
:: The CMAKE_BUILD_TYPE is needed to workaround on issue
:: https://github.com/Microsoft/vcpkg/issues/1626
::
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
		          " -DCMAKE_TOOLCHAIN_FILE=%VCPKG_CMAKE_TOOLCHAIN_FILE%"^
	                  " -DVCPKG_TARGET_TRIPLET=%VCPKG_DEFAULT_TRIPLET%"^
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

:: ##################################
:check_vcpkg_snapshot
setlocal EnableDelayedExpansion
:: look for same sha in repository HEAD and in the tag
for /f %%i in ('git -C %VCPKG_DIR% rev-parse HEAD') do set VCPKG_HEAD=%%i
for /f %%i in ('git -C %VCPKG_DIR% rev-list -n 1 %VCPKG_SNAPSHOT%') do set VCPKG_TAG=%%i
if NOT %VCPKG_HEAD% == %VCPKG_TAG% (
  echo The vpckg directory is not using the expected snapshot %VCPKG_SNAPSHOT%
  echo VCPKG_HEAD points to %VCPKG_HEAD% while VCPKG_TAG points to %VCPKG_TAG%
  echo They should point to the same commit hash
  goto :error
)
goto :EOF

:: ##################################
:list_vcpkg_packages
%VCPKG_CMD% list || goto :error
goto :EOF

:: ##################################
:remove_vcpkg_installation
:: remove the installed directory to simulate all packages removal
:: vcpkg cli does not support the operation
set LIB_DIR="%~dp0"
call %LIB_DIR%\windows_env_vars.bat || goto :error
if [%VCPKG_INSTALLED_FILES_DIR%]==[] (
  echo VCPKG_INSTALLED_FILES_DIR variable seems empty, this is a bug
  goto :error
)
del /s /f /q %VCPKG_INSTALLED_FILES_DIR%
goto :EOF

:: ##################################
:_prepare_vcpkg_to_install
set LIB_DIR=%~dp0
call %LIB_DIR%\windows_env_vars.bat || goto :error
call %win_lib% :check_vcpkg_snapshot || goto :error
:: update osrf vcpkg overlay
pushd .
cd %VCPKG_OSRF_DIR%
git pull origin master || goto :error
popd
goto :EOF

:: ##################################
:_install_and_upgrade_vcpkg_package
:: arg1: package to install
if [%1] == [] (
  echo "_install_and_upgrade_vcpkg_package called with no argument"
  goto :error
)
:: workaround on permissions problems for default VCPKG_DEFAULT_BINARY_CACHE
set VCPKG_DEFAULT_BINARY_CACHE=C:\Users\Administrator\AppData\Local\vcpkg\archives
if not exist %VCPKG_DEFAULT_BINARY_CACHE% mkdir %VCPKG_DEFAULT_BINARY_CACHE%
%VCPKG_CMD% install --recurse "%1" --overlay-ports="%VCPKG_OSRF_DIR%"
:: vcpkg does not upgrade installed packages using the install command
:: since most of the packages are coming from a frozen snapshot, it is
:: not a problem. However upgrading is needed for the osrf port overlay
%VCPKG_CMD% upgrade "%1" --no-dry-run --overlay-ports="%VCPKG_OSRF_DIR%"
goto :EOF

:: ##################################
:install_vcpkg_package
:: arg1: package to install
if [%1] == [] (
  echo "install_vcpkg_package called with no argument"
  goto :error
)
call %win_lib% :_prepare_vcpkg_to_install|| goto :error
call %win_lib% :_install_and_upgrade_vcpkg_package "%1" || goto :error
goto :EOF

:: ##################################
:remove_vcpkg_package
:: arg1: package to install
set LIB_DIR=%~dp0
call %LIB_DIR%\windows_env_vars.bat || goto :error

%VCPKG_CMD% remove --recurse "%1"
goto :EOF

:: ##################################
:enable_vcpkg_integration
%VCPKG_CMD% integrate install || goto :error
goto :EOF

:: ##################################
:disable_vcpkg_integration
%VCPKG_CMD% integrate remove || goto :error
goto :EOF

:: ##################################
:setup_vcpkg_all_dependencies
set LIB_DIR=%~dp0
call %LIB_DIR%\windows_env_vars.bat || goto :error
call %win_lib% :enable_vcpkg_integration || goto :error
call %win_lib% :_prepare_vcpkg_to_install|| goto :error
for %%p in (%VCPKG_DEPENDENCIES_LEGACY%) do (
  call %win_lib% :_install_and_upgrade_vcpkg_package %%p || goto :error
)
goto :EOF

:: ##################################
:error - error routine
::
echo Failed in windows_library with error #%errorlevel%.
exit %errorlevel%
