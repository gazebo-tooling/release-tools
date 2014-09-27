REM Windows standard file to build Visual Studio projects

IF "%PLATFORM_TO_BUILD%" == "32" (
  echo "Using 32bits VS configuration"
  set VCVARSALL=%VS32bits_VCVARSALL%
) ELSE (
  echo "Using 64bits VS configuration"
  set VCVARSALL=%VS64bits_VCVARSALL%
)

REM Configure the VC++ compilation
call %VCVARSALL%

echo %WORKSPACE%
cd %WORKSPACE%
mkdir build
cd build

cmake .. %VS_DEFAULT_FLAGS% %ARG_CMAKE_FLAGS%

msbuild ALL_BUILD.vcxproj
