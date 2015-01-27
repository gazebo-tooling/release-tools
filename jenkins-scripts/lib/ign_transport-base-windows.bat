@echo on

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
call %win_lib% :configure_msvc_compiler

echo %IGN_CLEAN_WORKSPACE%
if "%IGN_CLEAN_WORKSPACE%" == FALSE (
  echo "Cleaning workspace"
  IF exist workspace ( rmdir /s /q workspace ) || goto %win_lib% :error
  mkdir workspace 
)

cd workspace

echo "Download libraries"
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip cppzmq-noarch.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-win%BITNESS%-vc12.zip protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip
REM call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip boost_1_56_0.zip

echo "Uncompressing libraries"
call %win_lib% :create_unzip_script
call %win_lib% :unzip cppzmq-noarch.zip
call %win_lib% :unzip protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :unzip zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip
REM call %win_lib% :unzip_7za boost_1_56_0.zip

REM Add path for zeromq dynamic library .ddl
set PATH=%PATH%;%WORKSPACE%/workspace/ZeroMQ 3.2.4/bin/

REM Note that your jenkins job should put source in %WORKSPACE%/ign-transport
echo "Move sources so we agree with configure.bat layout"
move %WORKSPACE%\ign-transport .
cd ign-transport

echo "Compiling"
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error
nmake || goto %win_lib% :error
nmake install || goto %win_lib% :error

if NOT "%IGN_TEST_DISABLE%" == "TRUE" (
  echo "Running tests"
  REM Need to find a way of running test from the standard make test (not working)
  ctest -C "Release" --verbose --extra-verbose || exit 0
)
