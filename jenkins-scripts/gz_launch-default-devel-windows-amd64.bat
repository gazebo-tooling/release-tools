@echo on
set SCRIPT_DIR=%~dp0

if not defined VCS_DIRECTORY set VCS_DIRECTORY=gz-launch

call "%SCRIPT_DIR%\lib\colcon-default-devel-windows.bat" gz-launch
