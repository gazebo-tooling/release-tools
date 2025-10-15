@echo off
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
set "PYTHON_SCRIPT=%SCRIPT_DIR%find_conda_config.py"

if not exist "%PYTHON_SCRIPT%" (
    echo WARNING: find_conda_config.py not found at %PYTHON_SCRIPT%. Falling back to legacy environment.
    set "DETECTED_CONDA_ENV_NAME=legacy"
    echo legacy
    goto :end
)

REM Run the Python script to detect conda configuration
echo Detecting conda environment for %PACKAGE_NAME% v%MAJOR_VERSION%...

for /f "tokens=2 delims=:" %%i in ('python "%PYTHON_SCRIPT%" "%PACKAGE_NAME%" "%MAJOR_VERSION%" 2^>nul ^| findstr "GZ_CONDA_CONFIGS="') do (
    set "CONDA_CONFIGS=%%i"
)

REM Check if we got a result
if not defined CONDA_CONFIGS (
    echo WARNING: Could not detect conda configuration for %PACKAGE_NAME% v%MAJOR_VERSION%. Falling back to legacy environment.
    set "DETECTED_CONDA_ENV_NAME=legacy"
    echo legacy
    goto :end
)

REM Parse the first conda configuration to get environment name
for /f "tokens=1 delims=;" %%j in ("%CONDA_CONFIGS%") do (
    for /f "tokens=2 delims=:" %%k in ("%%j") do (
        set "CONDA_VERSION=%%k"
    )
)

echo Detected conda environment: %CONDA_VERSION%
set "DETECTED_CONDA_ENV_NAME=%CONDA_VERSION%"
echo %CONDA_VERSION%

:end
REM Export the detected environment name for the calling script
endlocal & set "DETECTED_CONDA_ENV_NAME=%DETECTED_CONDA_ENV_NAME%"