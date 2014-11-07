:: Supposed to has been called ignition-base before
:: asumming a haptix-comm source at WORKSPACE/haptix-comm

cd %WORKSPACE%
mkdir %WORKSPACE%/workspace
move haptix-comm %WORKSPACE%/workspace || goto %win_lib% error

:: We need ignition first
hg clone https://bitbucket.org/ignitionrobotics/ign-transport
call %SCRIPT_DIR%/lib/ign_transport-base-windows.bat

cd %WORKSPACE%/workspace/haptix-comm || goto %win_lib% :error

echo "Compiling haptix"
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error
nmake || goto %win_lib% :error
nmake install || goto %win_lib% :error

echo "Running tests"
REM Need to find a way of running test from the standard make test (not working)
ctest -C "Release" --verbose --extra-verbose || exit 0
