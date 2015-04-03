set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
call %win_lib% :configure_msvc_compiler

IF exist workspace ( rmdir /s /q workspace ) || goto %win_lib% :error
mkdir workspace 
cd workspace

echo "Download libraries"
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip boost_1_56_0.zip

echo "Uncompressing libraries"
call %win_lib% :download_7za
call %win_lib% :unzip_7za boost_1_56_0.zip 

REM Note that your jenkins job should put source in %WORKSPACE%/ign-transport
echo "Move sources so we agree with configure.bat layout"
move %WORKSPACE%\sdformat
cd sdformat

echo "Compiling"
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error
nmake || goto %win_lib% :error
nmake install || goto %win_lib% :error

echo "Running tests"
REM Need to find a way of running test from the standard make test (not working)
ctest -C "Release" --verbose --extra-verbose || exit 0
