@echo on

set SCRIPT_DIR="%~dp0"
call "%SCRIPT_DIR%/lib/windows_library.bat"

REM i386 for the moment to ignition-transport
set PLATFORM_TO_BUILD=x86

IF %PLATFORM_TO_BUILD% == x86 (
  echo "Using 32bits VS configuration"
) ELSE (
  echo "Using 64bits VS configuration"
)

REM Configure the VC++ compilation
call "c:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" %PLATFORM_TO_BUILD%

cd "C:\Temp"
del workspace /q /s /f 
mkdir workspace
cd workspace

call:wget http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip cppzmq-noarch.zip
call:wget http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-win32-vc12.zip protobuf-2.6.0-win32-vc12.zip
call:wget http://packages.osrfoundation.org/win32/deps/zeromq-3.2.4-x86.zip zeromq-3.2.4-x86.zip

REM get the unzip script for our library
call:create_unzip_script
call:unzip cppzmq-noarch.zip
call:unzip protobuf-2.6.0-win32-vc12.zip
call:unzip zeromq-3.2.4-x86.zip

REM TODO: mercurial autoinstalled in windows if not present? Is that even possible?
hg clone https://bitbucket.org/ignitionrobotics/ign-transport
cd ign-transport

mkdir build
cd build
call "..\configure.bat" || goto:error
nmake || goto:error
nmake install || goto:error

REM ##################################################################################
REM
REM TODO: remove this copy/paste when loading function from a file is working
REM TODO: http://www.dostips.com/forum/viewtopic.php?f=3&t=1626&hilit=%20library 

:create_unzip_script - Create a script to unzip files
REM ##################################
REM Unzip script from http://jagaroth.livejournal.com/69147.html 
REM
REM Changing working folder back to current directory for Vista & 7 compatibility
%~d0
CD %~dp0
REM Folder changed

REM This script upzip's files...
> j_unzip.vbs ECHO '
>> j_unzip.vbs ECHO ' Dim ArgObj, var1, var2
>> j_unzip.vbs ECHO Set ArgObj = WScript.Arguments
>> j_unzip.vbs ECHO.
>> j_unzip.vbs ECHO If (Wscript.Arguments.Count ^> 0) Then
>> j_unzip.vbs ECHO. var1 = ArgObj(0)
>> j_unzip.vbs ECHO Else
>> j_unzip.vbs ECHO. var1 = ""
>> j_unzip.vbs ECHO End if
>> j_unzip.vbs ECHO.
>> j_unzip.vbs ECHO If var1 = "" then
>> j_unzip.vbs ECHO. strFileZIP = "example.zip"
>> j_unzip.vbs ECHO Else
>> j_unzip.vbs ECHO. strFileZIP = var1
>> j_unzip.vbs ECHO End if
>> j_unzip.vbs ECHO.
>> j_unzip.vbs ECHO 'The location of the zip file.
>> j_unzip.vbs ECHO REM Set WshShell = CreateObject("Wscript.Shell")
>> j_unzip.vbs ECHO REM CurDir = WshShell.ExpandEnvironmentStrings("%%cd%%")
>> j_unzip.vbs ECHO Dim sCurPath
>> j_unzip.vbs ECHO sCurPath = CreateObject("Scripting.FileSystemObject").GetAbsolutePathName(".")
>> j_unzip.vbs ECHO strZipFile = sCurPath ^& "\" ^& strFileZIP
>> j_unzip.vbs ECHO 'The folder the contents should be extracted to.
>> j_unzip.vbs ECHO outFolder = sCurPath ^& "\"
>> j_unzip.vbs ECHO.
>> j_unzip.vbs ECHO. WScript.Echo ( "Extracting file " ^& strFileZIP)
>> j_unzip.vbs ECHO.
>> j_unzip.vbs ECHO Set objShell = CreateObject( "Shell.Application" )
>> j_unzip.vbs ECHO Set objSource = objShell.NameSpace(strZipFile).Items()
>> j_unzip.vbs ECHO Set objTarget = objShell.NameSpace(outFolder)
>> j_unzip.vbs ECHO intOptions = 256
>> j_unzip.vbs ECHO objTarget.CopyHere objSource, intOptions
>> j_unzip.vbs ECHO.
>> j_unzip.vbs ECHO. WScrip.Echo ( "Extracted." )
>> j_unzip.vbs ECHO.
REM ##################################
GOTO:EOF

:wget - Download internet file in your current directory. [URL] [filename]
REM Downloading %~1
bitsadmin /transfer mydownloadjob /download /priority normal %~1 %cd%\%~2 || goto :error
GOTO:EOF

REM TODO: automatically create the download script if it does not exists
:unzip - Unzip file [zip_file]
cscript //B j_unzip.vbs %~1 || goto:error
GOTO:EOF

:error
echo Failed with error #%errorlevel%.
exit /b %errorlevel%
GOTO:EOF
