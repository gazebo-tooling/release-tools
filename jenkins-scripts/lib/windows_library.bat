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
set MSVC_ON_WIN64=c:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat
set MSVC_ON_WIN32=c:\Program Files\Microsoft Visual Studio 12.0\VC\vcvarsall.bat

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
bitsadmin /transfer mydownloadjob /download /priority high %~1 %cd%\%~2 || goto :error
goto :EOF

:: ##################################
:download_7za - Download the unzip utility from osrfoundation.org
::
call :wget http://packages.osrfoundation.org/win32/deps/7za.exe 7za.exe || goto :error

goto :EOF

:: ##################################
:unzip_7za - Unzip using 7za
::
:: arg1 - File to unzip
echo Uncompressing %~1 
IF NOT exist %~1 ( echo "Zip file does not exists: %~1" && goto :error )
7za.exe x %~1 -aoa || goto :error
goto :EOF

:: ##################################
:error - error routine
::
echo Failed with error #%errorlevel%.
exit /b %errorlevel%
goto :EOF
