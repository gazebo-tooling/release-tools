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
set MSVC_ON_WIN64=C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat
set MSVC_ON_WIN32=C:\Program Files\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat

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
if not exist 7za.exe (call :wget http://gazebosim.org/distributions/win32/deps/7za.exe 7za.exe || goto :error)
goto :EOF

:: ##################################
:: Unzip using 7za
:unzip_7za
::
:: arg1 - File to unzip
echo Uncompressing %~1 
IF NOT exist %~1 ( echo "Zip file does not exist: %~1" && goto :error )
7za.exe x %~1 -aoa || goto :error
goto :EOF

:: ##################################
:: Unzip using 7za and then install
:unzip_install
::
echo Uncompressing %~1 to %~d0\install
IF NOT exist %~1 ( echo "Zip file does not exist: %~1" && goto :error )
call :download_7za || goto :error
7za.exe x %~1 -aoa -o%WORKSPACE_INSTALL_DIR% || goto :error
goto :EOF

:: ##################################
:: Download some prebuilt package from our repository, unzip it using 7za, and then install it
:download_unzip_install
::
echo # BEGIN SECTION: downloading, unzipping, and installing dependency %1
call :wget http://gazebosim.org/distributions/win32/deps/%1 %1 || goto :error
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
:error - error routine
::
echo Failed with error #%errorlevel%.
exit /b %errorlevel%
goto :EOF
