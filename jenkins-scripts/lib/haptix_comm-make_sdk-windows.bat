:: SDK Creation script
:: arg1 bitness [ x86 | amd64 ]
:: TODO: Needs migration to use the windows_library.bat functions and
:: TODO: use haptix-comm-base and ignition-comm-base

@echo on

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: remove previous packages
del %WORKSPACE%\pkgs\*.zip
:: Default branches
@if "%IGN_TRANSPORT_BRANCH%" == "" set IGN_TRANSPORT_BRANCH=default
@if "%HAPTIX_COMM_BRANCH%" == "" set HAPTIX_COMM_BRANCH=default

@set PLATFORM_TO_BUILD=x86
@if not "%1"=="" set PLATFORM_TO_BUILD=%1

:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc_compiler
echo # END SECTION

@echo ""
@echo "======================="
@echo "%bitness%bits SDK Generation  "
@echo ""
@echo " ign-transport branch: %IGN_TRANSPORT_BRANCH%"
@echo " haptix-comm   branch: %HAPTIX_COMM_BRANCH%"
@echo "======================="
@echo ""

echo # BEGIN SECTION: setup all needed variables and workspace
mkdir workspace 
cd workspace || goto :error

set cwd=%cd%
set tmpdir=%cwd%\hx_gz_sdk_tmp
rmdir "%tmpdir%" /S /Q
mkdir "%tmpdir%"
cd "%tmpdir%"

set zeromq_zip_name=zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip
set protobuf_zip_name=protobuf-2.6.0-cmake3.5-win%BITNESS%-vc12.zip
echo # END SECTION

echo # BEGIN SECTION: Download dependencies and unzip
@rem Download stuff.  Note that bitsadmin requires an absolute path.
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/%zeromq_zip_name% %zeromq_zip_name% || goto :error
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip cppzmq-noarch.zip  || goto :error
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/%protobuf_zip_name% %protobuf_zip_name%  || goto :error

@rem Unzip stuff
@rem
echo "Uncompressing libraries"
call %win_lib% :download_7za
call %win_lib% :unzip_7za %zeromq_zip_name% > zeromq_7z.log
call %win_lib% :unzip_7za cppzmq-noarch.zip > cppzmq_7z.log
call %win_lib% :unzip_7za %protobuf_zip_name% > protobuf_7z.lob
echo # END SECTION

echo # BEGIN SECTION: Cloning ignition-transport [%IGN_TRANSPORT_BRANCH% branch]
hg clone https://bitbucket.org/ignitionrobotics/ign-transport -b %IGN_TRANSPORT_BRANCH%
cd ign-transport
hg tip > ignition-transport.info
cd ..
echo # END SECTION

echo # BEGIN SECTION: Cloning haptix-comm [%HAPTIX_COMM_BRANCH% branch]
hg clone https://bitbucket.org/osrf/haptix-comm haptix-comm -b %HAPTIX_COMM_BRANCH%
cd haptix-comm
hg tip > haptix-comm.info
cd ..
echo # END SECTION

set srcdir=%cd%

setlocal Enabledelayedexpansion
for %%b in (Debug, Release) do (
    echo # BEGIN SECTION: SDK generation %%b for %BITNESS% bits

    cd %srcdir%

    echo # BEGIN SECTION: build ign-transport in %%b
    cd ign-transport
    mkdir build
    cd build
    del CMakeCache.txt
    call ..\configure %%b %BITNESS%
    nmake VERBOSE=1 > ign-transport.log || goto :error
    nmake install
    set /p IGNTRANSPORT_VERSION=<VERSION || goto :error
    cd ..\..
    echo # END SECTION

    echo # BEGIN SECTION: build haptix-comm in %%b
    cd haptix-comm
    mkdir build
    cd build
    del CMakeCache.txt
    call ..\configure %%b %BITNESS%
    nmake VERBOSE=1 > haptix.log || goto :error
    nmake install
    set /p HAPTIX_VERSION=<VERSION || goto:error
    echo # END SECTION
   
    echo # BEGIN SECTION: build haptix-comm examples in %%b
    cd ..
    cd example
    mkdir build
    cd build
    del CMakeCache.txt
    call ..\configure %%b %BITNESS%
    nmake VERBOSE=1 > haptix_example.log || goto :error
    echo # END SECTION
   

    cd ..\..\..

    echo # BEGIN SECTION: generate zip [version !HAPTIX_VERSION!] in %%b
    :: Package it all up
    :: Our goal here is to create an "install" layout for all the stuff
    :: needed to use haptix-comm.  That layout can be then be zipped and
    :: distributed.  Lots of assumptions are being made here.
    :: We need to use expansion at runtime values for variables inside the loop this is
    :: why the ! var ! is being used. For more information, please read:
    :: http://ss64.com/nt/delayedexpansion.html
    set "build_type=%%b"
    set "installdir=%cwd%\hx_gz_sdk-!HAPTIX_VERSION!-!build_type!"
    :: WORKSPACE\pkgs to agree with repository_uploader script layout
    set "sdk_zip_file=%WORKSPACE%\pkgs\hx_gz_sdk-!build_type!-!HAPTIX_VERSION!-win%BITNESS%.zip"
    set "sdk_latest_zip_file=%WORKSPACE%\pkgs\hx_gz_sdk-!build_type!-latest-win%BITNESS%.zip"

    echo " * Build type             : !build_type!"
    echo " * Installation directory : !installdir!"
    echo " * SDK file               : !sdk_zip_file!"
      
    rmdir !installdir! /S /Q
    mkdir "!installdir!" || goto :error

    mkdir "!installdir!\deps\protobuf-2.6.0-win%BITNESS%-vc12\vsprojects\!build_type!" || goto :error
    :: Protobuf
    xcopy "protobuf-2.6.0-win%BITNESS%-vc12\vsprojects\!build_type!\*.lib" "!installdir!\deps\protobuf-2.6.0-win%BITNESS%-vc12\vsprojects\!build_type!" /s /e /i || goto :error
    xcopy "protobuf-2.6.0-win%BITNESS%-vc12\vsprojects\google" "!installdir!\deps\protobuf-2.6.0-win%BITNESS%-vc12\vsprojects\google" /s /e /i
    :: ZeroMQ
    xcopy "ZeroMQ 3.2.4\COPYING*" "!installdir!\deps\ZeroMQ 3.2.4" /s /e /i
    xcopy "ZeroMQ 3.2.4\bin\libzmq-v120*" "!installdir!\deps\ZeroMQ 3.2.4\bin" /s /e /i
    ::xcopy "ZeroMQ 3.2.4\bin\msvc*" "!installdir!\deps\ZeroMQ 3.2.4\bin" /s /e /i
    xcopy "ZeroMQ 3.2.4\include" "!installdir!\deps\ZeroMQ 3.2.4\include" /s /e /i
    xcopy "ZeroMQ 3.2.4\lib\libzmq-v120*" "!installdir!\deps\ZeroMQ 3.2.4\lib" /s /e /i
    :: Ignition transport
    mkdir "!installdir!\deps\ign-transport"
    xcopy "ign-transport\build\install\!build_type!\include" "!installdir!\deps\ign-transport\!build_type!\include" /s /e /i
    xcopy "ign-transport\build\install\!build_type!\lib" "!installdir!\deps\ign-transport\!build_type!\lib" /s /e /i
    xcopy "ign-transport\ignition-transport.info" "!installdir!"
    :: haptix-comm
    mkdir "!installdir!\haptix-comm"
    xcopy "haptix-comm\build\install\!build_type!\include" "!installdir!\haptix-comm\!build_type!\include" /s /e /i
    xcopy "haptix-comm\build\install\!build_type!\lib" "!installdir!\haptix-comm\!build_type!\lib" /s /e /i
    xcopy "haptix-comm\haptix-comm.props" "!installdir!"
    xcopy "haptix-comm\haptix-comm.info" "!installdir!"
    :: haptix-example
    :: example .exe files
    mkdir "!installdir!\example\" 
    xcopy "haptix-comm\example\build\*.exe" "!installdir!\example\" /i

    :: ------ MATLAB -------------------------------
    :: - zeromq matlab stuff
    :: - haptix-comm MATLAB stuff (.m files + .mex)
    mkdir "!installdir!\matlab\" 
    xcopy "haptix-comm\build\install\!build_type!\lib\haptix-comm\matlab\*" "!installdir!\matlab\" /s /e /i

    cd ..
    echo # END SECTION

    echo "Generating SDK zip files: !sdk_zip_file!" > sdk_zip_file.log
    "%tmpdir%\7za.exe" a -tzip "!sdk_zip_file!" "!installdir!" || goto :error
    copy "!sdk_zip_file!" "!sdk_latest_zip_file!"
    echo # END SECTION
)
setlocal disabledelayedexpansion

if NOT DEFINED KEEP_WORKSPACE (
   echo # BEGIN SECTION: clean up workspace
   for /D %%p IN ("%WORKSPACE%\workspace\*") DO rmdir "%%p" /s /q
   REM for some reason the rmdir line below seems to do what we intented to do
   REM but fails with the following message:
   REM "The process cannot access the file because it is being used by another process"
   REM rmdir /s /q %WORKSPACE%\workspace || goto :error
   echo # END SECTION
)

goto :EOF

:error
echo "The program is stopping with errors! Check the log" 
