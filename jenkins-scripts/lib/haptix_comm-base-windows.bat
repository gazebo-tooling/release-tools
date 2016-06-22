:: Supposed to has been called ignition-base before
:: asumming a haptix-comm source at WORKSPACE/haptix-comm
::

@echo on

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc_compiler
echo # END SECTION

set IGN_TEST_DISABLE=TRUE
@REM Need to keep workspace when calling ign-transport
set KEEP_WORKSPACE=TRUE
set IGN_CLEAN_WORKSPACE=FALSE

set zeromq_zip_name=zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip
set protobuf_zip_name=protobuf-2.6.0-cmake3.5-win%BITNESS%-vc12.zip

cd %WORKSPACE%
IF exist workspace ( rmdir /s /q workspace ) || goto %win_lib% :error
mkdir workspace
cd workspace
xcopy %WORKSPACE%\haptix-comm %WORKSPACE%\workspace\haptix-comm /s /i /e > xcopy.log || goto %win_lib% :error
echo # END SECTION

:: We need ignition first
echo # BEGIN SECTION: cloning ign-transport (default branch)
:: Need close directly on WORKSPACE to call ign_transport as it is called from jenkins
set IGN_TRANSPORT_PATH=%WORKSPACE%\ign-transport

if exist %IGN_TRANSPORT_PATH% ( rmdir /s /q %IGN_TRANSPORT_PATH% ) || goto :error
hg clone https://bitbucket.org/ignitionrobotics/ign-transport %IGN_TRANSPORT_PATH% || goto :error
call "%SCRIPT_DIR%/lib/ign_transport-base-windows.bat"  || goto :error
:: configure.bat in haptix is using ../ign-transport to locate ignition
:: note that ign-transport directory was copied and used to build in
:: WORKSPACE/workspace/ign-transport by the ign-transport script
move %WORKSPACE%/workspace/ign-transport %WORKSPACE%/workspace/haptix-comm
:: Do not keep the workspace anymore
set KEEP_WORKSPACE=
echo # END SECTION

echo # BEGIN SECTION: downloading haptix-comm dependencies and unzip
cd %WORKSPACE%/workspace
call %win_lib% :download_7za
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/%zeromq_zip_name% %zeromq_zip_name% || goto :error
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip cppzmq-noarch.zip  || goto :error
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/%protobuf_zip_name% %protobuf_zip_name%  || goto :error
call %win_lib% :unzip_7za %zeromq_zip_name% > zeromq_7z.log
call %win_lib% :unzip_7za cppzmq-noarch.zip > cppzmq_7z.log
call %win_lib% :unzip_7za %protobuf_zip_name% > protobuf_7z.log
echo # END SECTION

echo # BEGIN SECTION: configuring haptix-comm
cd %WORKSPACE%/workspace/haptix-comm || goto :error
mkdir build || goto :error
cd build || goto :error
call "..\configure.bat" Release %BITNESS% || goto :error
echo # END SECTION
echo # BEGIN SECTION: compiling haptix-comm
nmake || goto %win_lib% :error
echo # END SECTION
echo # BEGIN SECTION: installing haptix-comm
nmake install || goto %win_lib% :error
echo # END SECTION

echo # BEGIN SECTION: compiling haptix example
cd ..\example
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error
nmake || goto %win_lib% :error
echo # END SECTION

echo # BEGIN SECTION: running tests
REM Need to find a way of running test from the standard make test (not working)
cd %WORKSPACE%/workspace/haptix-comm/build
ctest -C "Release" --verbose --extra-verbose || echo "test failed"
echo # END SECTION

echo # BEGIN SECTION: export testing results
set TEST_RESULT_PATH=%WORKSPACE%\test_results
REM LEGACY to be used with dsl scripts
set TEST_RESULT_PATH_LEGACY=%WORKSPACE%\build\test_results
if exist %TEST_RESULT_PATH% ( rmdir /s /q %TEST_RESULT_PATH% )
if exist %TEST_RESULT_PATH_LEGACY% ( rmdir /q /s %TEST_RESULT_PATH_LEGACY% )
mkdir %WORKSPACE%\build\
move test_results %TEST_RESULT_PATH% || goto :error
xcopy %TEST_RESULT_PATH% %TEST_RESULT_PATH_LEGACY% /s /e /i
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
