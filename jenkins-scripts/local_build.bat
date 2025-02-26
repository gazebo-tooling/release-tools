:: Trigger local builds for windows builds. Only support Pixi builds
set SCRIPT_DIR=%~dp0

:: Customize the number of building threads. A value is needed
set "MAKE_JOBS=8"

if "%~2"=="" (
    echo "Usage: local_build.bat <script> <sources> [build_mode]"
    echo.
    echo "Arguments:"
    echo "  script      The script to run"
    echo "  sources     Local checkout of sources directory"
    echo "  build_mode  Build mode (optional):"
    echo "              0 - Build all the steps (default)"
    echo "              1 - Only run the compilation, reuse build environment (useful for testing code changes)"
    exit /b 1
)

set "build_mode=0"
if not "%~3"=="" (
    set "build_mode=%~3"
)

set "WORKSPACE=%TMP%\%RANDOM%"
set "SRC_DIRECTORY=%~2"
:: Only work with PIXI
set "USE_PIXI=1"
:: KEEP_WORKSPACE should help with debugging and re-run the compilation only
set "KEEP_WORKSPACE=1"
:: Avoid closing the whole terminal
set "EXTRA_EXIT_PARAM=/b"

set "DBG_LAST_BUILD_FILE=%SCRIPT_DIR%\.debug_last_build.bat"

mkdir "%WORKSPACE%"
xcopy "%SRC_DIRECTORY%\*" "%WORKSPACE%\%~nx2" /s /e /i /Y >> log

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
if errorlevel 1 (
    echo "Local build failed"
    echo " ERROR: %errorlevel%"

    if "%LOCAL_WS%"=="" (
        echo "LOCAL_WS is not set. Internal script error."
        exit /b 1
    )
) else (
    echo "Local build finished successfully"
)
:: Construct the DBG_LAST_BUILD_FILE to reproduce the last build
:: source the setup.bat file and log into the directory
echo call %LOCAL_WS%\install\setup.bat >> %DBG_LAST_BUILD_FILE%
echo cd %LOCAL_WS% >> %DBG_LAST_BUILD_FILE%

echo   - Build root is %WORKSPACE%
echo   - Build workspace is %LOCAL_WS%
echo Reproduce the conditions of the last build
echo   - by running call %DBG_LAST_BUILD_FILE%