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
:error - error routine
::
echo Failed in windows_library with error #%errorlevel%.
exit %EXTRA_EXIT_PARAM% %errorlevel%
