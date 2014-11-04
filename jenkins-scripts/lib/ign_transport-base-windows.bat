REM Configure the VC++ compilation
call "c:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" %PLATFORM_TO_BUILD%

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

echo "Cloning ignition transport"
REM TODO: mercurial autoinstalled in windows if not present? Is that even possible?
hg clone https://bitbucket.org/ignitionrobotics/ign-transport > clone_ign_transport.log
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
