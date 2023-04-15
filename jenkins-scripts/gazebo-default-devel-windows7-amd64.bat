set SCRIPT_DIR=%~dp0
set PLATFORM_TO_BUILD=amd64
set VCS_DIRECTORY=gazebo-classic

for /f %%i in ('python "%SCRIPT_DIR%\tools\detect_cmake_major_version.py" "%WORKSPACE%\%VCS_DIRECTORY%\CMakeLists.txt"') do set GAZEBO_MAJOR_VERSION=%%i

call "%SCRIPT_DIR%/lib/gazebo-base-windows.bat"
