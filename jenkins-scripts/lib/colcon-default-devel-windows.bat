:: Windows build of a Gazebo library with the pixi_ci driver
:: (jenkins-scripts/tools/pixi_ci/README.md lists the job variables)
::
:: Parameters:
::   - arg1       : gz-collections.yaml library name, e.g. gz-fuel-tools
::   - SCRIPT_DIR : the jenkins-scripts directory, set by the per-package .bat
::
:: Actions
::   - Download pixi and create the bootstrap environment, unless
::     REUSE_PIXI_INSTALLATION is defined
::   - Run the driver with the bootstrap environment python

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat
set LIB_DIR=%SCRIPT_DIR%\lib
call "%LIB_DIR%\windows_env_vars.bat"

if not defined REUSE_PIXI_INSTALLATION (
  echo # BEGIN SECTION: pixi: installation
  call %win_lib% :pixi_installation || goto :error
  echo # END SECTION
  echo # BEGIN SECTION: pixi: create bootstrap environment
  call %win_lib% :pixi_create_bootstrap_environment || goto :error
  echo # END SECTION
)
if defined REUSE_PIXI_INSTALLATION if not exist "%PIXI_BOOTSTRAP_PROJECT_PATH%\.pixi\envs\default\python.exe" goto :no_bootstrap_env

:: Run the interpreter directly: .bat scripts can not capture the output of
:: pixi run correctly (see conda/config-detector/pixi.toml)
"%PIXI_BOOTSTRAP_PROJECT_PATH%\.pixi\envs\default\python.exe" -u "%SCRIPT_DIR%\tools\run_pixi_ci.py" %1 || goto :error
goto :EOF

:no_bootstrap_env
echo ERROR: REUSE_PIXI_INSTALLATION is set but %PIXI_BOOTSTRAP_PROJECT_PATH% has no bootstrap environment: run once without it
exit %EXTRA_EXIT_PARAM% 1

:error - error routine
echo Failed with error #%errorlevel%.
exit %EXTRA_EXIT_PARAM% %errorlevel%
