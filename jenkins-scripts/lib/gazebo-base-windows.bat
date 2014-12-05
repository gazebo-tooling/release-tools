set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
call %win_lib% :configure_msvc_compiler

:: IF exist workspace ( rmdir /s /q workspace ) || goto %win_lib% :error
:: reusing the workspace

IF NOT exist workspace ( 

mkdir workspace 
cd workspace

echo "Download libraries"
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip boost_1_56_0.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-win%BITNESS%-vc12.zip protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/ogre_src_v1-8-1-vc12-x64-release-debug.zip ogre_src_v1-8-1-vc12-x64-release-debug.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/FreeImage-vc12-x64-release-debug.zip FreeImage-vc12-x64-release-debug.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/tbb43_20141023oss_win.zip tbb43_20141023oss_win.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/libcurl-vc12-x64-release-debug-static-ipv6-sspi-winssl.zip libcurl-vc12-x64-release-debug-static-ipv6-sspi-winssl.zip	
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/dlfcn-win32-vc12-x64-release-debug.zip dlfcn-win32-vc12-x64-release-debug.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/zziplib-0.13.62-vc12-x64-release-debug.zip zziplib-0.13.62-vc12-x64-release-debug.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/zlib-1.2.8-vc12-x64-release-debug.zip zlib-1.2.8-vc12-x64-release-debug.zip

echo "Uncompressing libraries"
call %win_lib% :download_7za
call %win_lib% :unzip_7za boost_1_56_0.zip 
call %win_lib% :unzip_7za protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :unzip_7za FreeImage-vc12-x64-release-debug.zip
call %win_lib% :unzip_7za ogre_src_v1-8-1-vc12-x64-release-debug.zip
call %win_lib% :unzip_7za tbb43_20141023oss_win.zip
call %win_lib% :unzip_7za libcurl-vc12-x64-release-debug-static-ipv6-sspi-winssl.zip
call %win_lib% :unzip_7za dlfcn-win32-vc12-x64-release-debug.zip
call %win_lib% :unzip_7za zlib-1.2.8-vc12-x64-release-debug.zip
call %win_lib% :unzip_7za zziplib-0.13.62-vc12-x64-release-debug.zip


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
) ELSE (
  :: Remove gazebo copy
  rmdir /s /q workspace\gazebo || goto %win_lib% :error
)

REM Note that your jenkins job should put source in %WORKSPACE%/ign-transport
echo "Move sources so we agree with configure.bat layout"
move %WORKSPACE%\gazebo .
cd gazebo

echo "Compiling"
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error

call %win_lib% :download_7za
call %win_lib% :unzip_7za http://download.qt-project.org/official_releases/jom/jom.zip jom.zip
call %win_lib% :unzip_7za jom.zip

jom -j4 gzclient

nmake gzserver || goto %win_lib% :error
nmake gzclient || goto %win_lib% :error
nmake || goto %win_lib% :error
