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
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/OGRE-SDK-1.9.0-vc120-x64-12.03.2016.zip OGRE-SDK-1.9.0-vc120-x64-12.03.2016.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-cmake3.5-win%BITNESS%-vc12.zip protobuf-2.6.0-cmake3.5-win%BITNESS%-vc12.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/qt5-x64-static-release.zip qt5-x64-static-release.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/qwt_6.1.2~osrf_qt5.zip qwt_6.1.2~osrf_qt5.zip
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
call %win_lib% :unzip_7za OGRE-SDK-1.9.0-vc120-x64-12.03.2016.zip
call %win_lib% :unzip_7za protobuf-2.6.0-cmake3.5-win%BITNESS%-vc12.zip
call %win_lib% :unzip_7za qt5-x64-static-release.zip
call %win_lib% :unzip_7za qwt_6.1.2~osrf_qt5.zip
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

echo # BEGIN SECTION: compile and install ign-transport
set KEEP_WORKSPACE=TRUE
set IGN_TEST_DISABLE=TRUE
set IGN_TRANSPORT_DIR=%WORKSPACE%\ign-transport
if EXIST %IGN_TRANSPORT_DIR% ( rmdir /s /q %IGN_TRANSPORT_DIR% )
hg clone https://bitbucket.org/ignitionrobotics/ign-transport %IGN_TRANSPORT_DIR%
call "%SCRIPT_DIR%/lib/ign_transport-base-windows.bat" || goto :error
echo # END SECTION

:: compile ign-math if needed. ign-transport will probably do it first
set IGN_MATH_WS_DIR=%WORKSPACE%\workspace\ign-math
if EXIST %IGN_MATH_WS_DIR% (
  echo # BEGIN SECTION: ign-math already found
  echo # END SECTION
) ELSE (
  echo # BEGIN SECTION: compile and install ign-math
  set VCS_DIRECTORY=ign-math
  set KEEP_WORKSPACE=TRUE
  set ENABLE_TESTS=FALSE
  call "%SCRIPT_DIR%/lib/ign_transport-base-windows.bat" || goto :error
  echo # END SECTION
)

echo # BEGIN SECTION: compile and install sdformat
set SDFORMAT_DIR=%WORKSPACE%\workspace\sdformat
if EXIST %SDFORMAT_DIR% ( rmdir /s /q %SDFORMAT_DIR% )
hg clone https://bitbucket.org/osrf/sdformat %SDFORMAT_DIR% -b sdf5
cd %SDFORMAT_DIR%
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error
copy %WORKSPACE%\workspace\jom.exe .
jom -j %MAKE_JOBS% || goto :error
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

echo # BEGIN SECTION: compiling gazebo
copy %WORKSPACE%\workspace\jom.exe .
jom -j%MAKE_JOBS% || goto :error
echo # END SECTION

:: echo # BEGIN SECTION: compiling test suite
:: jom -j%MAKE_JOBS% tests || goto :error
:: echo # END SECTION

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
