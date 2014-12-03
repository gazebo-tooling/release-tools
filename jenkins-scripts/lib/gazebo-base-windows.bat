set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
call %win_lib% :configure_msvc_compiler

IF exist workspace ( rmdir /s /q workspace ) || goto %win_lib% :error
mkdir workspace 
cd workspace

echo "Download libraries"
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip boost_1_56_0.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-win%BITNESS%-vc12.zip protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/OgreSDK_vc11_x64_v1-9-0unstable.zip OgreSDK_vc11_x64_v1-9-0unstable.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/FreeImage-vc12-x64-release-debug.zip FreeImage-vc12-x64-release-debug.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/tbb43_20141023oss_win.zip tbb43_20141023oss_win.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/libcurl-vc12-x64-release-static-ipv6-sspi-winssl.zip libcurl-vc12-x64-release-static-ipv6-sspi-winssl.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/dlfcn-win32-vc12-x64-release-debug.zip dlfcn-win32-vc12-x64-release-debug.zip

echo "Uncompressing libraries"
call %win_lib% :download_7za
call %win_lib% :unzip_7za boost_1_56_0.zip 
call %win_lib% :unzip_7za protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :unzip_7za OgreSDK_vc11_x64_v1-9-0unstable.zip
call %win_lib% :unzip_7za FreeImage-vc12-x64-release-debug.zip
call %win_lib% :unzip_7za tbb43_20141023oss_win.zip
call %win_lib% :unzip_7za libcurl-vc12-x64-release-static-ipv6-sspi-winssl.zip
call %win_lib% :unzip_7za dlfcn-win32-vc12-x64-release-debug.zip

echo "Compile sdformat special branch"
hg clone https://bitbucket.org/osrf/sdformat
cd sdformat
hg up win_gerkey
mkdir build
cd build
call "..\configure.bat" || goto %win_lib% :error
echo "Compiling sdformat"
nmake
nmake install
cd ..\..

REM Note that your jenkins job should put source in %WORKSPACE%/ign-transport
echo "Move sources so we agree with configure.bat layout"
move %WORKSPACE%\gazebo .
cd gazebo

echo "Compiling"
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error

REM Next is gazebo_common
nmake gazebo_rendering || goto %win_lib% :error
