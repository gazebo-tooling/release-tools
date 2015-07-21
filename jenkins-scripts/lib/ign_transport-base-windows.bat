:echo on

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc_compiler
echo # END SECTION

echo %IGN_CLEAN_WORKSPACE%
if %IGN_CLEAN_WORKSPACE% == true (
  echo # BEGIN SECTION: preclean of workspace
  IF exist workspace ( rmdir /s /q workspace ) || goto :error
  echo # END SECTION
) else (
  echo # BEGIN SECTION: delete old sources
  IF exist workspace\ign-transport ( rmdir /s /q workspace\ign-transport ) || goto :error
  echo # END SECTION
)

IF NOT exist %WORKSPACE%\workspace ( 

mkdir %WORKSPACE%\workspace || echo "The workspace already exists. Fine"
cd %WORKSPACE%\workspace || goto :error

echo # BEGIN SECTION: downloading ign-transport dependencies and unzip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip cppzmq-noarch.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-win%BITNESS%-vc12.zip protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip

call %win_lib% :download_7za
call %win_lib% :unzip_7za cppzmq-noarch.zip || goto :error
call %win_lib% :unzip_7za protobuf-2.6.0-win%BITNESS%-vc12.zip || goto :error
call %win_lib% :unzip_7za zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip || goto :error
echo # END SECTION
) ELSE (
  echo # BEGIN SECTION: reusing workspace 
  :: Remove code copy
  IF EXIST %WORKSPACE%\workspace\ign-transport ( rmdir /s /q %WORKSPACE%\workspace\ign-transport ) || goto :error
  echo # END SECTION
)

echo # BEGIN SECTION: move sources so we agree with configure.bat layout
xcopy %WORKSPACE%\ign-transport %WORKSPACE%\workspace\ign-transport /s /i /e > xcopy.log || goto :error
echo # END SECTION

echo # BEGIN SECTION: add zeromq to PATH for dll load
REM Add path for zeromq dynamic library .ddl
set PATH=%PATH%;%WORKSPACE%\workspace\ZeroMQ 3.2.4\bin\
echo # END SECTION

echo # BEGIN SECTION: ign-transport compilation
cd %WORKSPACE%\workspace\ign-transport || goto :error
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto :error
nmake || goto :error
echo # END SECTION

echo # BEGIN SECTION: ign-transport installation
nmake install || goto :error
echo # END SECTION

set TEST_RESULT_PATH=%WORKSPACE%\test_results

if NOT "%IGN_TEST_DISABLE%" == "TRUE" (
  echo # BEGIN SECTION: run tests
  REM Need to find a way of running test from the standard make test (not working)
  ctest -C "Release" --verbose --extra-verbose || echo "tests failed"
  echo # END SECTION
  
  echo # BEGIN SECTION: export testing results
  echo "Path to remove: %TEST_RESULT_PATH%"
  rmdir /q /s %TEST_RESULT_PATH% || echo "TEST_RESULT_PATH did not exists, that's fine"
  xcopy test_results %TEST_RESULT_PATH% /s /i /e || goto :error
  echo # END SECTION
)

if NOT DEFINED KEEP_WORKSPACE (
   echo # BEGIN SECTION: clean up workspace
   cd %WORKSPACE%
   rmdir /s /q %WORKSPACE%\workspace || goto :error
   echo # END SECTION
)

goto :EOF

:error - error routine
::
echo Failed with error #%errorlevel%.
exit /b %errorlevel%
