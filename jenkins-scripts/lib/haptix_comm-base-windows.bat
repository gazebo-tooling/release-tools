:: Supposed to has been called ignition-base before

cd %WORKSPACE%/workspace/haptix-comv || goto %win_lib% :error

echo "Compiling haptix"
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error
nmake || goto %win_lib% :error
nmake install || goto %win_lib% :error

echo "Running tests"
REM Need to find a way of running test from the standard make test (not working)
ctest -C "Release" --verbose --extra-verbose || exit 0
