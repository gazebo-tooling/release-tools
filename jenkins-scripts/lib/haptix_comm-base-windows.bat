:: Supposed to has been called ignition-base before
:: asumming a haptix-comm source at WORKSPACE/haptix-comm
::

@echo on

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

echo # BEGIN SECTION: setup all needed variables and workspace
set IGN_TEST_DISABLE=TRUE
@REM Need to keep workspace when calling ign-transport
set KEEP_WORKSPACE=TRUE
set IGN_CLEAN_WORKSPACE=FALSE

cd %WORKSPACE%
IF exist workspace ( rmdir /s /q workspace ) || goto %win_lib% :error
mkdir workspace
cd workspace
xcopy %WORKSPACE%\haptix-comm %WORKSPACE%\workspace\haptix-comm /s /i /e > xcopy.log || goto %win_lib% :error
echo # END SECTION

:: We need ignition first
echo # BEGIN SECTION: clonning ign-transport (default branch)
@REM Need close directly on WORKSPACE to call ign_transport as it is called from jenkins
hg clone https://bitbucket.org/ignitionrobotics/ign-transport %WORKSPACE%\ign-transport
call %SCRIPT_DIR%/lib/ign_transport-base-windows.bat
echo # END SECTION

echo # BEGIN SECTION: downloading haptix-comm dependencies and unzip
cd %WORKSPACE%/workspace
call %win_lib% :download_7za
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip boost_1_56_0.zip
call %win_lib% :unzip_7za boost_1_56_0.zip > install_boost.log
echo # END SECTION

echo # BEGIN SECTION: configuring haptix-comm
cd %WORKSPACE%/workspace/haptix-comm || goto %win_lib% :error
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error
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
move test_results %WORKSPACE%/test_results
echo # END SECTION

if NOT DEFINED KEEP_WORKSPACE (
   echo # BEGIN SECTION: clean up workspace
   rmdir /s /q %WORKSPACE%\workspace || goto :error
   echo # END SECTION
)

goto :EOF

:error
echo "The program is stopping with errors! Check the log" 
exit /b %errorlevel%
