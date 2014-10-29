@echo on

set SCRIPT_DIR="%~dp0"
call "%SCRIPT_DIR%/lib/windows_library.bat"
if %errorlevel% neq 0 exit /b %errorlevel%

REM i386 for the moment to ignition-transport
set PLATFORM_TO_BUILD=x86

IF %PLATFORM_TO_BUILD% == x86 (
  echo "Using 32bits VS configuration"
) ELSE (
  echo "Using 64bits VS configuration"
)

REM Configure the VC++ compilation
call "c:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" %PLATFORM_TO_BUILD%

cd "C:\Temp"
del workspace /s
mkdir workspace
cd workspace

call:wget http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip cppzmq-noarch.zip
call:wget http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-win32-vc12.zip protobuf-2.6.0-win32-vc12.zip
call:wget http://packages.osrfoundation.org/win32/deps/zeromq-3.2.4-x86.zip zeromq-3.2.4-x86.zip

REM get the unzip script for our library
call:create_unzip_script
call:unzip cppzmq-noarch.zip
call:unzip protobuf-2.6.0-win32-vc12.zip
call:unzip zeromq-3.2.4-x86.zip

REM TODO: mercurial autoinstalled in windows if not present? Is that even possible?
hg clone https://bitbucket.org/ignitionrobotics/ign-transport
cd ign-transport

mkdir build
cd build
..\configure
nmake
nmake install
