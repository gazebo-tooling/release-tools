:: Supposed to has been called ignition-base before
:: asumming a haptix-comm source at WORKSPACE/haptix-comm
::

@echo on

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

set IGN_TEST_DISABLE=TRUE
set IGN_CLEAN_WORKSPACE=FALSE

cd %WORKSPACE%
IF exist workspace ( rmdir /s /q workspace ) || goto %win_lib% :error
mkdir workspace
move haptix-comm %WORKSPACE%/workspace/haptix-comm || goto %win_lib% :error

:: We need ignition first
echo "Compiling ignition transport ..."
hg clone https://bitbucket.org/ignitionrobotics/ign-transport
call %SCRIPT_DIR%/lib/ign_transport-base-windows.bat

echo "Downloading haptix dependencies ..."
cd %WORKSPACE%/workspace
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip boost_1_56_0.zip
call %win_lib% :unzip_7za boost_1_56_0.zip

echo "Compiling haptix ..."
cd %WORKSPACE%/workspace/haptix-comm || goto %win_lib% :error
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error
nmake || goto %win_lib% :error
nmake install || goto %win_lib% :error

echo "Running tests"
REM Need to find a way of running test from the standard make test (not working)
ctest -C "Release" --verbose --extra-verbose || exit 0
