set SCRIPT_DIR=%~dp0
set PLATFORM_TO_BUILD=amd64
set VCS_DIRECTORY=gazebo

for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set GAZEBO_MAJOR_VERSION=%%i

if %GAZEBO_MAJOR_VERSION% GEQ 9 (
  call "%SCRIPT_DIR%/lib/gazebo-base-windows.bat"
) else (
  call "%SCRIPT_DIR%/lib/gazebo8-base-windows.bat"
)
