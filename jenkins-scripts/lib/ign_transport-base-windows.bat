@echo on

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
echo "# BEGIN SECTION: configure the MSVC compiler"
call %win_lib% :configure_msvc_compiler
echo "# END SECTION"

echo %IGN_CLEAN_WORKSPACE%
if "%IGN_CLEAN_WORKSPACE%" == FALSE (
  echo "# BEGIN SECTION: preclean of workspace"
  IF exist workspace ( rmdir /s /q workspace ) || goto :error
  echo "# END SECTION"
)

echo "# BEGIN SECTION: prepare the workspace"
mkdir workspace 
cd workspace || goto :error

REM Note that your jenkins job should put source in %WORKSPACE%/ign-transport
echo "Move sources so we agree with configure.bat layout"
move %WORKSPACE%\ign-transport .
echo "# END SECTION"

echo " # BEGIN SECTION: Download dependencies and unzip"
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip cppzmq-noarch.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-win%BITNESS%-vc12.zip protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip

call %win_lib% :download_7za
call %win_lib% :unzip_7za cppzmq-noarch.zip
call %win_lib% :unzip_7za protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :unzip_7za zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip
echo "# END SECTION"

echo " # BEGIN SECTION: add zeromq to PATH for dll load"
REM Add path for zeromq dynamic library .ddl
set PATH=%PATH%;%WORKSPACE%/workspace/ZeroMQ 3.2.4/bin/
echo "# END SECTION"

echo " # BEGIN SECTION: compilation"
cd %WORKSPACE%/workspace/ign-transport
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto :error
nmake || goto :error
echo "# END SECTION"

echo " # BEGIN SECTION: installation"
nmake install || goto :error
echo "# END SECTION"

if NOT "%IGN_TEST_DISABLE%" == "TRUE" (
  echo "# BEGIN SECTION: run tests"
  REM Need to find a way of running test from the standard make test (not working)
  ctest -C "Release" --verbose --extra-verbose || exit 0
  echo "# END SECTION"
  
  echo "# BEGIN SECTION: export testing results"
  move test_results %WORKSPACE%/test_results
  echo "# END SECTION"
)

echo "# BEGIN SECTION: clean up build artifacts"
cd %WORKSPACE%
rmdir /s /q workspace || goto :error
echo "# END SECTION"

:error - error routine
::
echo Failed with error #%errorlevel%.
exit /b %errorlevel%
goto :EOF
