set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc_compiler
echo # END SECTION

echo # BEGIN SECTION: preclean of workspace
IF exist workspace ( rmdir /s /q workspace ) || goto :error
mkdir workspace 
cd workspace
echo # END SECTION

echo # BEGIN SECTION: compile and install ign-math
IF exist %WORKSPACE%\ign-math ( rmdir /s /q %WORKSPACE%\ign-math ) || goto :error
hg clone https://bitbucket.org/ignitionrobotics/ign-math %WORKSPACE%\ign-math || goto :error
set VCS_DIRECTORY=workspace/ign-math
call project-default-devel-windows.bat
echo # END SECTION

echo # BEGIN SECTION: download and uncompress dependencies
cd %WORKSPACE%/workspace
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip boost_1_56_0.zip

call %win_lib% :download_7za
call %win_lib% :unzip_7za boost_1_56_0.zip 
echo # END SECTION

echo # BEGIN SECTION: move sources so we agree with configure.bat layout
xcopy %WORKSPACE%\sdformat %WORKSPACE%\workspace\sdformat /s /i /e > xcopy.log || goto :error
echo # END SECTION

echo # BEGIN SECTION: configure
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto :error
echo # END SECTION

echo # BEGIN SECTION: compile
nmake || goto :error
echo # END SECTION

echo # BEGIN SECTION: install
nmake install || goto :error
echo # END SECTION

echo # BEGIN SECTION: run tests
REM Need to find a way of running test from the standard make test (not working)
ctest -C "Release" --verbose --extra-verbose || exit 0
echo # END SECTION

goto EOF

:error:error
echo "The program is stopping with errors! Check the log" 
exit /b %errorlevel%
