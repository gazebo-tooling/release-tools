set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Let's keep the workspace by default
set KEEP_WORKSPACE=0

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc_compiler
echo # END SECTION

:: IF exist workspace ( rmdir /s /q workspace ) || goto %win_lib% :error
:: reusing the workspace

IF NOT exist %WORKSPACE%\workspace ( 

mkdir %WORKSPACE%\workspace 
cd %WORKSPACE%\workspace

echo # BEGIN SECTION: downloading gazebo dependencies and unip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/FreeImage-vc12-x64-release-debug.zip FreeImage-vc12-x64-release-debug.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip boost_1_56_0.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/bzip2-1.0.6-vc12-x64-release-debug.zip bzip2-1.0.6-vc12-x64-release-debug.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/dlfcn-win32-vc12-x64-release-debug.zip dlfcn-win32-vc12-x64-release-debug.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/freetype-2.4.0-vc12-x64-release-debug.zip freetype-2.4.0-vc12-x64-release-debug.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/libcurl-vc12-x64-release-debug-static-ipv6-sspi-winssl.zip libcurl-vc12-x64-release-debug-static-ipv6-sspi-winssl.zip	
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/ogre_src_v1-8-1-vc12-x64-release-debug.zip ogre_src_v1-8-1-vc12-x64-release-debug.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-win%BITNESS%-vc12.zip protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/tbb43_20141023oss_win.zip tbb43_20141023oss_win.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/zziplib-0.13.62-vc12-x64-release-debug.zip zziplib-0.13.62-vc12-x64-release-debug.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/zlib-1.2.8-vc12-x64-release-debug.zip zlib-1.2.8-vc12-x64-release-debug.zip
call %win_lib% :wget http://download.qt-project.org/official_releases/jom/jom.zip jom.zip

call %win_lib% :download_7za
call %win_lib% :unzip_7za FreeImage-vc12-x64-release-debug.zip
call %win_lib% :unzip_7za boost_1_56_0.zip 
call %win_lib% :unzip_7za bzip2-1.0.6-vc12-x64-release-debug.zip
call %win_lib% :unzip_7za dlfcn-win32-vc12-x64-release-debug.zip
call %win_lib% :unzip_7za freetype-2.4.0-vc12-x64-release-debug.zip
call %win_lib% :unzip_7za libcurl-vc12-x64-release-debug-static-ipv6-sspi-winssl.zip
call %win_lib% :unzip_7za ogre_src_v1-8-1-vc12-x64-release-debug.zip
call %win_lib% :unzip_7za protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :unzip_7za tbb43_20141023oss_win.zip
call %win_lib% :unzip_7za zlib-1.2.8-vc12-x64-release-debug.zip
call %win_lib% :unzip_7za zziplib-0.13.62-vc12-x64-release-debug.zip
call %win_lib% :unzip_7za jom.zip
echo # END SECTION
) ELSE (
  echo # BEGIN SECTION: reusing workspace 
  :: Remove gazebo copy
  IF EXIST %WORKSPACE%\workspace\gazebo ( rmdir /s /q %WORKSPACE%\workspace\gazebo ) || goto :error
  echo # END SECTION
)

echo # BEGIN SECTION: compile and install sdformat
if EXIST sdformat ( rmdir /s /q %WORKSPACE%\workspace\sdformat )
hg clone https://bitbucket.org/osrf/sdformat %WORKSPACE%\workspace\sdformat
cd %WORKSPACE%\workspace\sdformat
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error
copy %WORKSPACE%\workspace\jom.exe .
jom
nmake install
echo # END SECTION

echo # BEGIN SECTION: copy gazebo sources to workspace
:: Note that your jenkins job should put source in %WORKSPACE%/ign-transport
xcopy %WORKSPACE%\gazebo %WORKSPACE%\workspace\gazebo /s /i /e > xcopy.log
cd %WORKSPACE%\workspace\gazebo
echo # END SECTION

echo # BEGIN SECTION: configure gazebo
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error
echo # END SECTION

echo # BEGIN SECTION: compile deps
copy %WORKSPACE%\workspace\jom.exe .
jom -j%MAKE_JOBS% gazebo_ode gazebo_opende_ou gazebo_ccd || goto :error
echo # END SECTION

echo # BEGIN SECTION: compile gazebo_common
jom -j%MAKE_JOBS% gazebo_common || goto :error
echo # END SECTION

echo # BEGIN SECTION: compile gzclient
jom -j%MAKE_JOBS% gzclient || goto :error
echo # END SECTION

echo # BEGIN SECTION: compile gzserver
jom -j%MAKE_JOBS% gzserver || goto :error
echo # END SECTION

if NOT DEFINED KEEP_WORKSPACE (
   echo # BEGIN SECTION: clean up workspace
   cd %WORKSPACE%
   rmdir /s /q %WORKSPACE%\workspace || goto :error
   echo # END SECTION
)

goto :EOF

:error
echo "The program is stopping with errors! Check the log" 
exit /b %errorlevel%
