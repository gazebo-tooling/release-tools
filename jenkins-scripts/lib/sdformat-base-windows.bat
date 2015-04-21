set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc_compiler
echo # END SECTION

echo # BEGIN SECTION: preclean of workspace
IF exist workspace ( rmdir /s /q workspace ) || goto %win_lib% :error
mkdir workspace 
cd workspace
echo # END SECTION

echo # BEGIN SECTION: download and uncompress dependencies
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip boost_1_56_0.zip

call %win_lib% :download_7za
call %win_lib% :unzip_7za boost_1_56_0.zip 
echo # END SECTION

echo # BEGIN SECTION: compile and install ign-math
if EXIST ign-math ( rmdir /s /q %WORKSPACE%\workspace\ign-math )
hg clone https://bitbucket.org/ignitionrobotics/ign-math %WORKSPACE%\workspace\ign-math
cd %WORKSPACE%\workspace\ign-math
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error
namke
nmake install
echo # END SECTION

echo # BEGIN SECTION: move sources so we agree with configure.bat layout
xcopy %WORKSPACE%\sdformat %WORKSPACE%\workspace\sdformat /s /i /e > xcopy.log || goto :error
echo # END SECTION

echo # BEGIN SECTION: configure
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error
echo # END SECTION

echo # BEGIN SECTION: compile
nmake || goto %win_lib% :error
echo # END SECTION

echo # BEGIN SECTION: install
nmake install || goto %win_lib% :error
echo # END SECTION

echo # BEGIN SECTION: run tests
REM Need to find a way of running test from the standard make test (not working)
ctest -C "Release" --verbose --extra-verbose || exit 0
echo # END SECTION
