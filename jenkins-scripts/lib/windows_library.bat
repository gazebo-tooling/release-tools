:: needed to import functions from other batch files
call :%*
exit /b

:: ##################################
:configure_msvc_compiler
::
::

:: See: https://issues.jenkins-ci.org/browse/JENKINS-11992
set path=%path:"=%

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
:wget - Download an URL to the current directory
:: 
:: arg1 URL to download
:: arg2 filename (not including the path, just the filename)
echo Downloading %~1
wget %~1 -O %cd%\%~2 || goto :error
goto :EOF

:: ##################################
:download_7za - Download the unzip utility from osrfoundation.org
::
if not exist 7za.exe (call :wget http://packages.osrfoundation.org/win32/deps/7za.exe 7za.exe || goto :error)
goto :EOF

:: ##################################
:unzip_7za - Unzip using 7za
::
:: arg1 - File to unzip
echo Uncompressing %~1 
IF NOT exist %~1 ( echo "Zip file does not exist: %~1" && goto :error )
7za.exe x %~1 -aoa || goto :error
goto :EOF

:: ##################################
:unzip_install - Unzip using 7za and then install
::
echo Uncompressing %~1 to %~d0\install
IF NOT exist %~1 ( echo "Zip file does not exist: %~1" && goto :error )
7za.exe x %~1 -aoa -o%WORKSPACE_INSTALL_DIR% || goto :error
goto :EOF

:: ##################################
:download_unzip_install - Download some prebuilt package, unzip it using 7za, and then install
::
echo # BEGIN SECTION: downloading, unzipping, and installing dependency %1
call :download_7za || goto :error
call :wget http://gazebosim.org/distributions/win32/deps/%1 %1 || goto :error
call :unzip_install %1 > install.log || goto :error
goto :EOF

:: ##################################
:install_ign_project
::
:: arg1: Name of the ignition project (e.g. ign-cmake, ign-math)
:: arg2: [Optional] desired branch
::
set IGN_PROJECT_DEPENDENCY_DIR=%LOCAL_WS%\%1
if exist %IGN_PROJECT_DEPENDENCY_DIR% ( rmdir /s /q %IGN_PROJECT_DEPENDENCY_DIR% )
if "%2"=="" (
  echo Installing default branch of %1
  hg clone https://bitbucket.org/ignitionrobotics/%1 -b default %IGN_PROJECT_DEPENDENCY_DIR%
) else (
  echo Installing branch %2 of %1
  hg clone https://bitbucket.org/ignitionrobotics/%1 -b %2 %IGN_PROJECT_DEPENDENCY_DIR%
)
cd %IGN_PROJECT_DEPENDENCY_DIR%
call configure.bat
nmake || goto :error
nmake install || goto :error
goto :EOF


:: ##################################
:error - error routine
::
echo Failed with error #%errorlevel%.
exit /b %errorlevel%
goto :EOF
