:: Trigger local builds for windows builds. Only support Pixi builds
::
:: arg1 script [ script to run ]
:: arg2 VCS_DIR [ local checkout of sources directory ]
:: arg3 build_mode
::         0. Build all the steps (default)
::         1. Only run the compilation, reuse build environment
::       
set SCRIPT_DIR=%~dp0

:: Customize the number of building threads. A value is needed
set "MAKE_JOBS=8"

if "%~2"=="" (
    echo "local_build.bat <script> <sources> [build_mode]"
    exit /b 1
)

set "build_mode=0"
if not "%~3"=="" (
    set "build_mode=%~3"
)

set "WORKSPACE=%TMP%\%RANDOM%"
set "VCS_DIRECTORY=%~2"
:: Only work with PIXI
set "USE_PIXI=1"
:: KEEP_WORKSPACE should help with debugging and re-run the compilation only
set "KEEP_WORKSPACE=1"
:: Avoid closing the whole terminal
set "EXTRA_EXIT_PARAM=/b"

set "DBG_LAST_BUILD_FILE=%SCRIPT_DIR%/.debug_last_build.bat"


mkdir "%WORKSPACE%"
xcopy "%VCS_DIRECTORY%" "%WORKSPACE%\%VCS_DIRECTORY%" /s /e /i  > log

echo "USING BUILD_MODE=%build_mode%"
if "%build_mode%"=="0" (
    set "REUSE_PIXI_INSTALLATION="   
) else if "%build_mode%"=="1" (
    set "REUSE_PIXI_INSTALLATION=1"
) else (
    echo "Value invalid for build_mode: %build_mode%"
    exit /b 1
)

call "%~1"

@echo off
if errorlevel 1 echo " BUILD FAILED with error: %errorlevel%"
echo Local build finished

echo call %LOCAL_WS%\install\setup.bash >> %DBG_LAST_BUILD_FILE%
echo cd %LOCAL_WS% >> %DBG_LAST_BUILD_FILE%
echo + Reproduce the conditions of the last build
echo + by running call %DBG_LAST_BUILD_FILE%
echo
echo "Local build finished:"
echo "Build root is %WORKSPACE%"
echo "Build workspace is %LOCAL_WS%"
