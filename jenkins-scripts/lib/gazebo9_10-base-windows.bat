set win_lib=%SCRIPT_DIR%\lib\windows_library.bat
set LOCAL_WS=%WORKSPACE%\ws

:: Let's keep the workspace by default
set KEEP_WORKSPACE=0

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc2017_compiler
echo # END SECTION

:: avoid conflicts with vcpkg packages
call %win_lib% :disable_vcpkg_integration
call %win_lib% :remove_vcpkg_package protobuf 
call %win_lib% :remove_vcpkg_package qt5
call %win_lib% :remove_vcpkg_package qwt

:: IF exist %LOCAL_WS% ( rmdir /s /q %LOCAL_WS% ) || goto %win_lib% :error
:: reusing the workspace

IF NOT exist %LOCAL_WS% (

mkdir %LOCAL_WS%
cd %LOCAL_WS%
set WORKSPACE_INSTALL_DIR=%LOCAL_WS%\install
mkdir %WORKSPACE_INSTALL_DIR%

echo # BEGIN SECTION: downloading gazebo dependencies and unip
call %win_lib% :download_7za
call %win_lib% :download_unzip_install dlfcn-win32-vc15-x64-dll-MD.zip
call %win_lib% :download_unzip_install curl-7.57.0-vc15-x64-dll-MD.zip
call %win_lib% :download_unzip_install protobuf-3.4.1-vc15-x64-dll-MD.zip
call %win_lib% :download_unzip_install libzip-1.4.0_zlip-1.2.11_vc15-x64-dll-MD.zip
call %win_lib% :download_unzip_install FreeImage3180Win32Win64.zip
call %win_lib% :download_unzip_install boost_1_67_0.zip
call %win_lib% :download_unzip_install ogre-sdk-1.10.12-vc15-x64.zip
call %win_lib% :download_unzip_install qt-opensource-windows-x86-msvc2015_64-5.7.0.zip
call %win_lib% :download_unzip_install bzip2-1.0.6-vc12-x64-release-debug.zip
call %win_lib% :download_unzip_install freetype-2.4.0-vc12-x64-release-debug.zip
call %win_lib% :download_unzip_install qwt_6.1.2~osrf_qt5.zip
call %win_lib% :download_unzip_install tbb43_20141023oss_win.zip
call %win_lib% :download_unzip_install zziplib-0.13.62-vc12-x64-release-debug.zip

call %win_lib% :wget http://download.qt-project.org/official_releases/jom/jom.zip jom.zip
call %win_lib% :unzip_7za jom.zip

echo # END SECTION
) ELSE (
  echo # BEGIN SECTION: reusing workspace
  :: Remove gazebo copy
  IF EXIST %LOCAL_WS%\gazebo ( rmdir /s /q %LOCAL_WS%\gazebo ) || goto :error
  echo # END SECTION
)

echo # BEGIN SECTION: compile and install ign-transport
set IGN_TRANSPORT_DIR=%WORKSPACE%\ign-transport
if EXIST %IGN_TRANSPORT_DIR% ( rmdir /s /q %IGN_TRANSPORT_DIR% )
hg clone https://bitbucket.org/ignitionrobotics/ign-transport %IGN_TRANSPORT_DIR% -b ign-transport4
set VCS_DIRECTORY=ign-transport
set KEEP_WORKSPACE=TRUE
set ENABLE_TESTS=FALSE
set BUILD_TYPE=Release
call "%SCRIPT_DIR%/lib/generic-default-devel-windows.bat"
echo # END SECTION

echo # BEGIN SECTION: compile and install sdformat
set SDFORMAT_DIR=%WORKSPACE%\sdformat
if EXIST %SDFORMAT_DIR% ( rmdir /s /q %SDFORMAT_DIR% )
hg clone https://bitbucket.org/osrf/sdformat %SDFORMAT_DIR% -b sdf6
set VCS_DIRECTORY=sdformat
set KEEP_WORKSPACE=TRUE
set ENABLE_TESTS=FALSE
set BUILD_TYPE=Release
call "%SCRIPT_DIR%/lib/generic-default-devel-windows.bat"
echo # END SECTION

echo # BEGIN SECTION: overwrite, since sdformat downloads boost_1.67.0_any_variant_version.zip
xcopy %WORKSPACE_INSTALL_DIR%\boost_1_67_0\boost %WORKSPACE_INSTALL_DIR%\include\boost /s /i /e /y > xcopy.log
xcopy %WORKSPACE_INSTALL_DIR%\boost_1_67_0\lib64-msvc-14.1 %WORKSPACE_INSTALL_DIR%\lib /s /i /e /y > xcopy.log
echo # END SECTION

echo # BEGIN SECTION: copy gazebo sources to workspace
:: Note that your jenkins job should put source in %WORKSPACE%/gazebo
xcopy %WORKSPACE%\gazebo %LOCAL_WS%\gazebo /s /i /e > xcopy.log
cd %LOCAL_WS%\gazebo
echo # END SECTION

echo # BEGIN SECTION: configure gazebo
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error
echo # END SECTION

echo # BEGIN SECTION: compiling gazebo
copy %LOCAL_WS%\jom.exe .
jom -j%MAKE_JOBS% || goto :error
echo # END SECTION

echo # BEGIN SECTION: compiling test suite
jom -j%MAKE_JOBS% tests || goto :error
echo # END SECTION

echo # BEGIN SECTION: run tests
:: nmake test is not working test/ directory exists and nmake is not able to handle it.
ctest -C "%BUILD_TYPE%" --force-new-ctest-process -VV
echo # END SECTION

echo # BEGIN SECTION: export testing results
rmdir /q /s %TEST_RESULT_PATH%
if exist %TEST_RESULT_PATH_LEGACY% ( rmdir /q /s %TEST_RESULT_PATH_LEGACY% )
mkdir %WORKSPACE%\build\
xcopy test_results %TEST_RESULT_PATH% /s /i /e || goto :error
xcopy %TEST_RESULT_PATH% %TEST_RESULT_PATH_LEGACY% /s /e /i
echo # END SECTION

if NOT DEFINED KEEP_WORKSPACE (
   echo # BEGIN SECTION: clean up workspace
   cd %WORKSPACE%
   rmdir /s /q %LOCAL_WS% || goto :error
   echo # END SECTION
)

goto :EOF

:error
echo "The program is stopping with errors! Check the log"
exit /b %errorlevel%
