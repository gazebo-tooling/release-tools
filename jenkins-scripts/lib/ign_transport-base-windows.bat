set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

IF %PLATFORM_TO_BUILD% == x86 (
  echo "Using 32bits VS configuration"
  set BITNESS=32
) ELSE (
  echo "Using 64bits VS configuration"
  set BITNESS=64
)

echo "Configure the VC++ compilation"
set MSVC_ON_WIN64=c:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat
set MSVC_ON_WIN32=c:\Program Files\Microsoft Visual Studio 12.0\VC\vcvarsall.bat

IF exist "%MSVC_ON_WIN64%" ( 
   call "%MSVC_ON_WIN64%" %PLATFORM_TO_BUILD%
) ELSE IF exist "%MSVC_ON_WIN32%" (
   call "%MSVC_ON_WIN32%" %PLATFORM_TO_BUILD%
) ELSE (
   echo "Could not find the vcvarsall.bat file"
   exit -1
)

IF exist workspace ( rmdir /s /q workspace ) 
mkdir workspace
cd workspace

echo "Download libraries"
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip cppzmq-noarch.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-win%BITNESS%-vc12.zip protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip

echo "Uncompressing libraries"
call %win_lib% :create_unzip_script || goto:error
call %win_lib% :unzip cppzmq-noarch.zip || goto:error
call %win_lib% :unzip protobuf-2.6.0-win%BITNESS%-vc12.zip || goto:error
call %win_lib% :unzip zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip || goto:error

REM Note that your jenkins job should put source in %WORKSPACE%/ign-transport
echo "Move sources so we agree with configure.bat layout"
mv %WORKSPACE%\ign-transport .
cd ign-transport

echo "Compiling"
mkdir build
cd build
call "..\configure.bat" || goto:error
nmake || goto:error
nmake install || goto:error

echo "Running tests"
REM Need to find a way of running test from the standard make test (not working)
ctest -C "Release" --verbose --extra-verbose || exit 0

mv test_results %WORKSPACE%
