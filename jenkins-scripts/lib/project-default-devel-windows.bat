REM Windows standard file to build Visual Studio projects

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
  call %win_lib% :wget http://packages.osrfoundation.org/win32/deps/%DEPENDENCY_PKG% %DEPENDENCY_PKG%
  call %win_lib% :unzip_7za %DEPENDENCY_PKG% %DEPENDENCY_PKG% > install_boost.log
)

echo "Copy sources inside workspace"
xcopy %WORKSPACE%\%VCS_DIRECTORY% . /s /e /i || goto :error
cd %VCS_DIRECTORY%
mkdir build
cd build

echo "Run configure.bat if it exists"
if exist ../configure.bat (
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
exit /b %errorlevel%
goto :EOF
