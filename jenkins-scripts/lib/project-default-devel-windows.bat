REM Windows standard file to build Visual Studio projects

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat

IF %PLATFORM_TO_BUILD% == x86 (
  echo "Using 32bits VS configuration"
  set VS_CMAKE_GEN=%VS32bits_CMAKE_GEN%
) ELSE (
  echo "Using 64bits VS configuration"
  set VS_CMAKE_GEN=%VS64bits_CMAKE_GEN%
)

REM Configure the VC++ compilation
call "c:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" %PLATFORM_TO_BUILD%

echo %WORKSPACE%
cd %WORKSPACE%
REM Reset the workspace directory if exists
if exist workspace ( rmdir workspace /s /q )
mkdir workspace
cd workspace || goto :error

echo "Download dependency if needed"
REM Todo: support multiple dependencies
if defined DEPENDENCY_PKG (
  call %win_lib% :download_7za
  call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/%DEPENDENCY_PKG% %DEPENDENCY_PKG% || goto :error
  call %win_lib% :unzip_7za %DEPENDENCY_PKG% %DEPENDENCY_PKG% > install_boost.log || goto:error
)

echo "Copy sources inside workspace"
xcopy %WORKSPACE%\%VCS_DIRECTORY% %VCS_DIRECTORY% /s /e /i > xcopy.log || goto :error
cd %VCS_DIRECTORY% || goto :error
mkdir build
cd build

if exist ../configure.bat (
  echo "Found configure.bat running it"
  ../configure.bat
)

echo "cmake .. %VS_CMAKE_GEN% %VS_DEFAULT_CMAKE_FLAGS% %ARG_CMAKE_FLAGS%"
cmake .. %VS_CMAKE_GEN% %VS_DEFAULT_CMAKE_FLAGS% %ARG_CMAKE_FLAGS% || goto :error

REM Running the compilation
msbuild %VS_DEFAULT_MSBUILD_FLAGS% ALL_BUILD.vcxproj || goto :error

REM Need to find a way of running test from msbuild passing ARGS=-VV
ctest -C "Release" --verbose --extra-verbose || echo "tests failed"

if NOT DEFINED %KEEP_WORKSPACE% (
   echo # BEGIN SECTION: clean up workspace
   rmdir /s /q workspace || goto :error
   echo # END SECTION
)

:error - error routine
::
echo Failed with error #%errorlevel%.
if exist workspace ( rmdir workspace /s /q )
exit /b %errorlevel%
goto :EOF
