:: sdformat base script
::
:: Parameters:
::  - USE_GZ_ZIP : (default true) [true | false]. Use zip to install ignition
::                       instead of compile
::  - BUILD_TYPE       : (default Release) [ Release | Debug ] Build type to use
::

@if "%BUILD_TYPE%" == "" set BUILD_TYPE=Release

:: Get the requirement of ign-math from SeachForStuff
findstr /r /b "find_package(ignition-math" %WORKSPACE%\sdformat\cmake\SearchForStuff.cmake > version.txt
set /p IGN_MATH_REQUIRED_VERSION=<version.txt
set IGN_MATH_REQUIRED_VERSION=%IGN_MATH_REQUIRED_VERSION:~26,1%
set IGNMATH_BRANCH="ign-math%IGN_MATH_REQUIRED_VERSION%"
@if "%USE_GZ_ZIP%" == "" set USE_GZ_ZIP=FALSE
set IGNMATH_ZIP=%IGNMATH_BRANCH% :: should not be needed

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat
set TEST_RESULT_PATH="%WORKSPACE%\test_results"
set TEST_RESULT_PATH_LEGACY=%WORKSPACE%\build\test_results
set LOCAL_WS=%WORKSPACE%\ws

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc_2019_compiler
echo # END SECTION

echo # BEGIN SECTION: preclean of workspace
IF exist %LOCAL_WS% ( rmdir /s /q %LOCAL_WS% ) || goto :error
mkdir %LOCAL_WS%
cd %LOCAL_WS%
echo # END SECTION

IF %USE_GZ_ZIP% == FALSE (
  echo # BEGIN SECTION: compile and install ign-math
  IF exist %WORKSPACE%\ign-math ( rmdir /s /q %WORKSPACE%\ign-math ) || goto :error
  git clone https://github.com/gazebosim/gz-math %WORKSPACE%\ign-math -u %IGNMATH_BRANCH% || goto :error
  set VCS_DIRECTORY=ign-math
  set KEEP_WORKSPACE=TRUE
  call "%SCRIPT_DIR%\lib\project-default-devel-windows.bat"
  echo # END SECTION
)

echo # BEGIN SECTION: download and uncompress dependencies
:: avoid conflicts with vcpkg packages
call %win_lib% :disable_vcpkg_integration

cd %LOCAL_WS%
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip boost_1_56_0.zip

call %win_lib% :download_7za
call %win_lib% :unzip_7za boost_1_56_0.zip > install_boost.log
IF %USE_GZ_ZIP% == TRUE (
  call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/%IGNMATH_ZIP%.zip %IGNMATH_ZIP%.zip
  call %win_lib% :unzip_7za %IGNMATH_ZIP%.zip
)
echo # END SECTION

echo # BEGIN SECTION: move sources so we agree with configure.bat layout
xcopy %WORKSPACE%\sdformat %LOCAL_WS%\sdformat /s /i /e > xcopy.log || goto :error
echo # END SECTION

echo # BEGIN SECTION: configure in %BUILD_TYPE%
cd %LOCAL_WS%\sdformat
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
   rmdir /s /q %LOCAL_WS% || goto :error
   echo # END SECTION
)

goto :EOF

:error:error
echo "The program is stopping with errors! Check the log"
exit /b %errorlevel%
