@echo on

set SCRIPT_DIR="%~dp0"

REM 1. Prepare the BatchLibrary for the start command
call "%SCRIPT_DIR%/BatchLib.bat"

REM 2. Start of the Batchlib, acquisition of the command line parameters, swtich to the temporary LibTest1.BLibTmp.bat
<:%BL.Start%

call :bl.import "%SCRIPT_DIR%/lib/windows_library.bat"

REM i386 for the moment to ignition-transport
set PLATFORM_TO_BUILD=x86

IF %PLATFORM_TO_BUILD% == x86 (
  echo "Using 32bits VS configuration"
) ELSE (
  echo "Using 64bits VS configuration"
)

REM Configure the VC++ compilation
call "c:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" %PLATFORM_TO_BUILD%

del workspace /q /s /f 
mkdir workspace
cd workspace

call:wget http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip cppzmq-noarch.zip
call:wget http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-win32-vc12.zip protobuf-2.6.0-win32-vc12.zip
call:wget http://packages.osrfoundation.org/win32/deps/zeromq-3.2.4-x86.zip zeromq-3.2.4-x86.zip

REM get the unzip script for our library
call:create_unzip_script || goto:error
call:unzip cppzmq-noarch.zip || goto:error
call:unzip protobuf-2.6.0-win32-vc12.zip || goto:error
call:unzip zeromq-3.2.4-x86.zip || goto:error

REM TODO: mercurial autoinstalled in windows if not present? Is that even possible?
hg clone https://bitbucket.org/ignitionrobotics/ign-transport
cd ign-transport

mkdir build
cd build
call "..\configure.bat" || goto:error
nmake || goto:error
nmake install || goto:error
