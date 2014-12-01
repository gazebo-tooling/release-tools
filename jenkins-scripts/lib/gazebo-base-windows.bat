set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: Call vcvarsall and all the friends
call %win_lib% :configure_msvc_compiler

IF exist workspace ( rmdir /s /q workspace ) || goto %win_lib% :error
mkdir workspace 
cd workspace

echo "Download libraries"
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip boost_1_56_0.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-win%BITNESS%-vc12.zip protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/OgreSDK_vc10_v1-8-1.zip OgreSDK_vc10_v1-8-1.zip
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/FreeImage.git.zip FreeImage.git.zip

echo "Uncompressing libraries"
call %win_lib% :download_7za
call %win_lib% :unzip_7za boost_1_56_0.zip
call %win_lib% :unzip_7za protobuf-2.6.0-win%BITNESS%-vc12.zip
call %win_lib% :unzip_7za OgreSDK_vc10_v1-8-1.zip
call %win_lib% :unzip_7za FreeImage.git.zip

REM Note that your jenkins job should put source in %WORKSPACE%/ign-transport
echo "Move sources so we agree with configure.bat layout"
move %WORKSPACE%\gazebo .
cd gazebo

echo "Compiling"
mkdir build
cd build
call "..\configure.bat" Release %BITNESS% || goto %win_lib% :error

REM Start by compiling gazebo_math
nmake gazebo_math || goto %win_lib% :error

REM Next is gazebo_common
nmake gazebo_common || goto %win_lib% :error
