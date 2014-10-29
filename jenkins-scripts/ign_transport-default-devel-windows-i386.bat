set SCRIPT_DIR="%~dp0"

REM i386 for the moment to ignition-transport
set PLATFORM_TO_BUILD=x86

IF %PLATFORM_TO_BUILD% == x86 (
  echo "Using 32bits VS configuration"
  set VS_CMAKE_GEN=%VS32bits_CMAKE_GEN%
) ELSE (
  echo "Using 64bits VS configuration"
  set VS_CMAKE_GEN=%VS64bits_CMAKE_GEN%
)

REM Configure the VC++ compilation
call "c:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" %PLATFORM_TO_BUILD%

cd "C:\Temp"
del workspace /s
mkdir workspace
cd workspace

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

bitsadmin /transfer mydownloadjob /download /priority normal http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip %cd%\cppzmq-noarch.zip
bitsadmin /transfer mydownloadjob /download /priority normal http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-win32-vc12.zip %cd%\protobuf-2.6.0-win32-vc12.zip
bitsadmin /transfer mydownloadjob /download /priority normal http://packages.osrfoundation.org/win32/deps/zeromq-3.2.4-x86.zip %cd%\zeromq-3.2.4-x86.zip

cscript //B j_unzip.vbs cppzmq-noarch.zip
cscript //B j_unzip.vbs protobuf-2.6.0-win32-vc12.zip
cscript //B j_unzip.vbs zeromq-3.2.4-x86.zip

REM TODO: mercurial autoinstalled in windows if not present? Is that even possible?
hg clone https://bitbucket.org/ignitionrobotics/ign-transport
cd ign-transport

mkdir build
cd build
..\configure
nmake
nmake install
