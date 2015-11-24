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
@if "%USE_IGNITION_ZIP%" == "" set USE_IGNITION_ZIP=TRUE

@if "%IGNMATH_BRANCH%" == "" (
  set IGNMATH_BRANCH=default
) else (
  :: When passing a value, we always go for compilation not zip
  set USE_IGNITION_ZIP=FALSE
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
  call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/ign-math2.zip ign-math2.zip
  call %win_lib% :unzip_7za ign-math2.zip
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
