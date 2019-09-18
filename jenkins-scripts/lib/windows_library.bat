:: needed to import functions from other batch files
call :%*
exit /b


:: ##################################
:: Configure the build environment for MSVC 2017
:configure_msvc2017_compiler
::
::

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
set MSVC_ON_WIN64_E=C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\VC\Auxiliary\Build\vcvarsall.bat
set MSVC_ON_WIN32_E=C:\Program Files\Microsoft Visual Studio\2017\Enterprise\VC\Auxiliary\Build\vcvarsall.bat
set MSVC_ON_WIN64_C=C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat
set MSVC_ON_WIN32_C=C:\Program Files\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat
:: libraries from vcpkg
set LIB_DIR="%~dp0"
call %LIB_DIR%\windows_env_vars.bat
set PATH=%PATH%;%VCPKG_DIR%\installed\%VCPKG_DEFAULT_TRIPLET%\bin

IF exist "%MSVC_ON_WIN64_E%" (
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
:: Configure the build environment for MSVC 2013
:: (This is for backwards compatibility)
:configure_msvc_compiler
::
::

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
)

echo "Configure the VC++ compilation"
set MSVC_ON_WIN64=C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat
set MSVC_ON_WIN32=C:\Program Files\Microsoft Visual Studio 12.0\VC\vcvarsall.bat

IF exist "%MSVC_ON_WIN64%" (
   call "%MSVC_ON_WIN64%" %MSVC_KEYWORD% || goto %win_lib% :error
) ELSE IF exist "%MSVC_ON_WIN32%" (
   call "%MSVC_ON_WIN32%" %MSVC_KEYWORD% || goto %win_lib% :error
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
  echo Installing default branch of %1
  hg clone https://bitbucket.org/ignitionrobotics/%1 %IGN_PROJECT_DEPENDENCY_DIR% -b default
) else (
  echo Installing branch %2 of %1
  hg clone https://bitbucket.org/ignitionrobotics/%1 %IGN_PROJECT_DEPENDENCY_DIR% -b %2
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
set gzdistro_dir=gazebodistro

if "%GAZEBODISTRO_BRANCH%" == "" (set GAZEBODISTRO_BRANCH=default)

if exist %gzdistro_dir% (rmdir /s /q %gzdistro_dir%)
hg clone https://bitbucket.org/osrf/gazebodistro %gzdistro_dir% -b %GAZEBODISTRO_BRANCH%
vcs import --retry 5  < "%gzdistro_dir%\%1" "%2" || goto :error
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

set COLCON_EXTRA_ARGS=%1
set COLCON_EXTRA_CMAKE_ARGS=%2

colcon build --build-base "build"^
             --install-base "install"^
             --parallel-workers %MAKE_JOBS%^
             %EXTRA_COLCON_ARGS%^
             --cmake-args " -DCMAKE_BUILD_TYPE=%BUILD_TYPE%"^
                          " -DCMAKE_TOOLCHAIN_FILE=%VCPKG_CMAKE_TOOLCHAIN_FILE%"^
                          " -DVCPKG_TARGET_TRIPLET=%VCPKG_DEFAULT_TRIPLET%"^
                          %COLCON_EXTRA_CMAKE_ARGS%^
             --event-handler console_cohesion+ || type %HOMEPATH%/.colcon/latest & goto :error

:: ##################################
::
:: Build all the workspaces packages except the package provided in arg1
::
:: arg1: name of the colcon package excluded from building
:build_workspace

set COLCON_PACKAGE=%1

:: two runs to get the dependencies built with testing and the package under
:: test build with tests
call :_colcon_build_cmd "--packages-skip %COLCON_PACKAGE%" " -DBUILD_TESTING=0"
call :_colcon_build_cmd "--packages-select %COLCON_PACKAGE%" " -DBUILD_TESTING=1"
goto :EOF

:: ##################################
:list_workspace_pkgs
colcon list -g || goto :error
goto :EOF

:: ##################################
:tests_in_workspace
:: arg1: package whitelist to test
set COLCON_PACKAGE=%1

colcon test --build-base "build" --event-handler console_direct+ --install-base "install" --packages-select %COLCON_PACKAGE% --executor sequential || goto :error
colcon test-result --build-base "build" --all
goto :EOF

:: ##################################
:install_vcpkg_package
:: arg1: package to install
set LIB_DIR=%~dp0
call %LIB_DIR%\windows_env_vars.bat || goto :error

%VCPKG_CMD% install "%1"
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
:install_osrf_vcpkg_package
:: arg1: package to install
set LIB_DIR=%~dp0
set PKG=%1
set PORT_DIR=%VCPKG_DIR%\ports\%PKG%
call %LIB_DIR%\windows_env_vars.bat || goto :error

if exist %PORT_DIR% (
  rmdir /s /q %PORT_DIR% || goto :error
)

if NOT exist %VCPKG_OSRF_DIR% (
  git clone https://github.com/osrf/vcpkg-ports %VCPKG_OSRF_DIR%
  cd %VCPKG_OSRF_DIR%
) else (
  cd %VCPKG_OSRF_DIR%
  git pull origin master || goto :error
)

:: copy port to the official tree
xcopy %VCPKG_OSRF_DIR%\%PKG% %PORT_DIR% /s /i /e || goto :error

call %win_lib% :install_vcpkg_package %1 || goto :error
goto :EOF

:: ##################################
:error - error routine
::
echo Failed in windows_library with error #%errorlevel%.
exit /B %errorlevel%
