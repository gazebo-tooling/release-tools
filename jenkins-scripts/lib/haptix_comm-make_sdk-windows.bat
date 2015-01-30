:: SDK Creation script
:: arg1 bitness [ x86 | amd64 ]
:: TODO: Needs migration to use the windows_library.bat functions and
:: TODO: use haptix-comm-base and ignition-comm-base

@echo on

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

:: remove previous packages
del %WORKSPACE%\*.zip

@set PLATFORM_TO_BUILD=x86
@if not "%1"=="" set PLATFORM_TO_BUILD=%1

:: Default branches
@if "%IGN_TRANSPORT_BRANCH%" == "" set IGN_TRANSPORT_BRANCH=default
@if "%HAPTIX_COMM_BRANCH%" == "" set HAPTIX_COMM_BRANCH=default

IF %PLATFORM_TO_BUILD% == x86 (
  set BITNESS=32
) ELSE (
  REM Visual studio is accepting many keywords to compile for 64bits
  REM We need to set x86_amd64 to make express version to be able to
  REM Cross compile from x86 -> amd64
  echo "Using 64bits VS configuration"
  set BITNESS=64
  set MSVC_KEYWORD=x86_amd64
  set PLATFORM_TO_BUILD=amd64
)

@echo ""
@echo "======================="
@echo "%bitness%bits SDK Generation  "
@echo ""
@echo " ign-transport branch: %IGN_TRANSPORT_BRANCH%"
@echo " haptix-comm   branch: %HAPTIX_COMM_BRANCH%"
@echo "======================="
@echo ""

@echo " - Configure the VC++ compilation"

set MSVC_ON_WIN64=c:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat
set MSVC_ON_WIN32=c:\Program Files\Microsoft Visual Studio 12.0\VC\vcvarsall.bat

IF exist "%MSVC_ON_WIN64%" ( 
   call "%MSVC_ON_WIN64%" %MSVC_KEYWORD% || goto :error
) ELSE IF exist "%MSVC_ON_WIN32%" (
   call "%MSVC_ON_WIN32%" %MSVC_KEYWORD% || goto :error
) ELSE (
   echo "Could not find the vcvarsall.bat file"
   exit -1
)

@rem Setup directories
set cwd=%cd%
set tmpdir=%cwd%\hx_gz_sdk_tmp
rmdir "%tmpdir%" /S /Q
mkdir "%tmpdir%"
cd "%tmpdir%"

set zeromq_zip_name=zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip
set protobuf_zip_name=protobuf-2.6.0-win%BITNESS%-vc12.zip

@rem Download stuff.  Note that bitsadmin requires an absolute path.
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/%zeromq_zip_name% %zeromq_zip_name% || goto :error
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip cppzmq-noarch.zip  || goto :error
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/%protobuf_zip_name% %protobuf_zip_name%  || goto :error
call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip boost_1_56_0.zip || goto :error

@rem Unzip stuff
@rem
echo "Uncompressing libraries"
call %win_lib% :download_7za
call %win_lib% :unzip_7za %zeromq_zip_name% > zeromq_7z.log
call %win_lib% :unzip_7za cppzmq-noarch.zip > cppzmq_7z.log
call %win_lib% :unzip_7za %protobuf_zip_name% > protobuf_7z.lob
call %win_lib% :unzip_7za boost_1_56_0.zip > boost_7z.lob

@rem Clone stuff
hg clone https://bitbucket.org/ignitionrobotics/ign-transport -b %IGN_TRANSPORT_BRANCH%
cd ign-transport
hg tip > ignition-transport.info
cd ..

hg clone https://bitbucket.org/osrf/haptix-comm haptix-comm -b %HAPTIX_COMM_BRANCH%
cd haptix-comm
REM set haptix_hash variable. Yes, we need need to do this for structure
for /f "delims=" %%a in ('hg id -i') do @set haptix_hash=%%a
hg tip > haptix-comm.info
cd ..

set srcdir=%cd%

setLocal Enabledelayedexpansion

for %%b in (Debug, Release) do (

    cd %srcdir%

    echo "Build ign-transport in %%b"
    cd ign-transport
    mkdir build
    cd build
    del CMakeCache.txt
    call ..\configure %%b %BITNESS%
    nmake VERBOSE=1 > ign-transport.log || goto :error
    nmake install
    cd ..\..

    echo "Build haptix-comm in %%b"
    cd haptix-comm
    mkdir build
    cd build
    del CMakeCache.txt
    call ..\configure %%b %BITNESS%
    nmake VERBOSE=1 > haptix.log || goto :error
    nmake install
    cd ..\..

    :: Package it all up
    :: Our goal here is to create an "install" layout for all the stuff
    :: needed to use haptix-comm.  That layout can be then be zipped and
    :: distributed.  Lots of assumptions are being made here.

    :: We need to use expansion at runtime values for variables inside the loop this is
    :: why the !var! is being used. For more information, please read:
    :: http://ss64.com/nt/delayedexpansion.html
    set "build_type=%%b"
    set "installdir=%cwd%\hx_gz_sdk_!build_type!"
    
    echo "Installation directory installdir = %installdir%"
      
    rmdir !installdir! /S /Q
    mkdir !installdir! || goto :error

    mkdir "!installdir!\deps\protobuf-2.6.0-win%BITNESS%-vc12\vsprojects\!build_type!" || goto :error
    :: Protobuf
    echo "Current directory is: %cd%"
    echo "Try to run: protobuf-2.6.0-win%BITNESS%-vc12\vsprojects\!build_type!\*.lib !installdir!\deps\protobuf-2.6.0-win%BITNESS%-vc12\vsprojects\!build_type!"
    xcopy "protobuf-2.6.0-win%BITNESS%-vc12\vsprojects\!build_type!\*.lib" "!installdir!\deps\protobuf-2.6.0-win%BITNESS%-vc12\vsprojects\!build_type!" /s /e /i || goto :error
    xcopy "protobuf-2.6.0-win%BITNESS%-vc12\vsprojects\google" "!installdir!\deps\protobuf-2.6.0-win%BITNESS%-vc12\vsprojects\google" /s /e /i
    :: ZeroMQ
    xcopy "ZeroMQ 3.2.4\COPYING*" "!installdir!\deps\ZeroMQ 3.2.4" /s /e /i
    xcopy "ZeroMQ 3.2.4\bin\libzmq-v120-mt-3*" "!installdir!\deps\ZeroMQ 3.2.4\bin" /s /e /i
    ::xcopy "ZeroMQ 3.2.4\bin\msvc*" "!installdir!\deps\ZeroMQ 3.2.4\bin" /s /e /i
    xcopy "ZeroMQ 3.2.4\include" "!installdir!\deps\ZeroMQ 3.2.4\include" /s /e /i
    xcopy "ZeroMQ 3.2.4\lib\libzmq-v120-mt-3*" "!installdir!\deps\ZeroMQ 3.2.4\lib" /s /e /i
    :: - zeromq matlab stuff
    mkdir "!installdir!\matlab\"
    xcopy "ZeroMQ 3.2.4\bin\libzmq-v120-mt-3*.dll" "!installdir!\matlab" /s /e /i
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
    :: - haptix-comm MATLAB stuff
    xcopy "haptix-comm\matlab\*" "!installdir!\matlab"
    :: MEX generation
    :: TODO: check for the absolute path if really exists
    :: @echo on

    :: if %%b == 'Debug' (
    ::   set zmq_lib=libzmq-v120-mt-gd-3_2_4.lib
    :: ) else (
     ::  set zmq_lib=libzmq-v120-mt-3_2_4.lib
   ::  )

    :: "C:\Program files\MATLAB\R2014b\bin\mex" "!installdir!\matlab\hx_getdeviceinfo.c" -I"!installdir!\haptix-comm\!build_type!\include" -L"!installdir!\haptix-comm\!build_type!\lib" -lhaptix-comm -lhaptix_msgs -L"!installdir!\deps\protobuf-2.6.0-win%BITNESS%-vc12\vsprojects\!build_type!" -lprotobuf  -L"!installdir!\deps\ZeroMQ 3.2.4\lib" -l%zmq_lib% -I"!installdir!\deps\ign-transport\!build_type!\include" -L"!installdir!\deps\ign-transport\!build_type!\lib" -lignition-transport -lws2_32 -lIphlpapi -v || goto :error
    :: TODO: need hx_update.c
    :: copy "hx_*.mex*" "!installdir!\matlab" || goto :error

    set sdk_zip_file=hx_gz_sdk-!build_type!-%haptix_hash%-win%BITNESS%.zip

    cd ..
    echo "Current directory is %cd%"
    echo "Generating SDK zip file: %sdk_zip_file%"
    "%tmpdir%\7za.exe" a -tzip ../%sdk_zip_file% "hx_gz_sdk_!build_type!\"
)
setlocal disabledelayedexpansion

goto :EOF

:error
echo "The program is stopping with errors! Check the log" 
