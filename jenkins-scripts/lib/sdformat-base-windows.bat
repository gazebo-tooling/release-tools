:: sdformat base script
::
:: Parameters:
::  - USE_IGNITION_ZIP : (default true) [true | false]. Use zip to install ignition 
::                       instead of compile
::  - BUILD_TYPE       : (default Release) [ Release | Debug ] Build type to use
::  - IGNMATH_BRANCH   : (default default) [optional]. Ignition math branch to
::                       compile. If in use, USE_IGNITION_ZIP will be false
::                   

@if "%BUILD_TYPE%" == "" set BUILD_TYPE=Release

:: Get the requirement of ign-math from SearchForStuff
findstr /r "set(IGNITION-MATH_REQUIRED_MAJOR_VERSION" %WORKSPACE%\sdformat\cmake\SearchForStuff.cmake > version.txt
set /p IGN_MATH_REQUIRED_VERSION=<version.txt
set IGN_MATH_REQUIRED_VERSION=%IGN_MATH_REQUIRED_VERSION:~41,1%
set IGNMATH_BRANCH="ign-math%IGN_MATH_REQUIRED_VERSION%"
:: hard-code ign-math3 for now until we fix configure scripts
@if %IGN_MATH_REQUIRED_VERSION% EQU 4 set IGNMATH_BRANCH="ign-math3"
@if "%USE_IGNITION_ZIP%" == "" set USE_IGNITION_ZIP=FALSE
set IGNMATH_ZIP=%IGNMATH_BRANCH% :: should not be needed

:: When CI is run on the default branch use the .zip. Otherwise compile ign-math
@if "%SRC_BRANCH%" == "default" (
  set IGNMATH_BRANCH="default"
  set IGNMATH_ZIP="ign-math3"
  @if "%USE_IGNITION_ZIP%" == "" set USE_IGNITION_ZIP=TRUE
)

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat
set TEST_RESULT_PATH="%WORKSPACE%\test_results"
set TEST_RESULT_PATH_LEGACY=%WORKSPACE%\build\test_results

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc_compiler
echo # END SECTION

echo # BEGIN SECTION: preclean of workspace
IF exist %WORKSPACE%\workspace ( rmdir /s /q workspace ) || goto :error
mkdir %WORKSPACE%\workspace 
cd %WORKSPACE%\workspace
echo # END SECTION

IF %USE_IGNITION_ZIP% == FALSE (
  echo # BEGIN SECTION: compile and install ign-math
  IF exist %WORKSPACE%\ign-math ( rmdir /s /q %WORKSPACE%\ign-math ) || goto :error
  hg clone https://bitbucket.org/ignitionrobotics/ign-math %WORKSPACE%\ign-math -u %IGNMATH_BRANCH% || goto :error
  set VCS_DIRECTORY=ign-math
  set KEEP_WORKSPACE=TRUE
  call "%SCRIPT_DIR%\lib\project-default-devel-windows.bat"
  echo # END SECTION
)

echo # BEGIN SECTION: download and uncompress dependencies
cd %WORKSPACE%\workspace
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip boost_1_56_0.zip

call %win_lib% :download_7za
call %win_lib% :unzip_7za boost_1_56_0.zip > install_boost.log
IF %USE_IGNITION_ZIP% == TRUE (
  call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/%IGNMATH_ZIP%.zip %IGNMATH_ZIP%.zip
  call %win_lib% :unzip_7za %IGNMATH_ZIP%.zip
)
echo # END SECTION

echo # BEGIN SECTION: move sources so we agree with configure.bat layout
xcopy %WORKSPACE%\sdformat %WORKSPACE%\workspace\sdformat /s /i /e > xcopy.log || goto :error
echo # END SECTION

echo # BEGIN SECTION: configure in %BUILD_TYPE%
cd %WORKSPACE%\workspace\sdformat
mkdir build
cd build
call "..\configure.bat" %BUILD_TYPE% %BITNESS% || goto :error
echo # END SECTION

echo # BEGIN SECTION: compile
nmake || goto :error
echo # END SECTION

echo # BEGIN SECTION: install
nmake install || goto :error
echo # END SECTION

echo # BEGIN SECTION: run tests
REM Need to find a way of running test from the standard make test (not working)
ctest -C "%BUILD_TYPE%" --verbose --extra-verbose || echo "test failed"
echo # END SECTION

echo # BEGIN SECTION: export testing results
rmdir /q /s %TEST_RESULT_PATH%
if exist %TEST_RESULT_PATH_LEGACY% ( rmdir /q /s %TEST_RESULT_PATH_LEGACY% )
mkdir %WORKSPACE%\build\
xcopy test_results %TEST_RESULT_PATH% /s /i /e || goto :error
xcopy %TEST_RESULT_PATH% %TEST_RESULT_PATH_LEGACY% /s /e /i
echo # END SECTION


if NOT DEFINED KEEP_WORKSPACE (
   echo # BEGIN SECTION: clean up workspace
   cd %WORKSPACE%
   rmdir /s /q %WORKSPACE%\workspace || goto :error
   echo # END SECTION
)

goto :EOF

:error:error
echo "The program is stopping with errors! Check the log" 
exit /b %errorlevel%
