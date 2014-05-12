REM Windows standard file

call "c:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat"
echo %WORKSPACE%
cd %WORKSPACE%
mkdir build
cd build
cmake ..
dir
msbuild ALL_BUILD.vcxproj  
