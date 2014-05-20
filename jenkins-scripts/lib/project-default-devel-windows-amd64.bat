REM Windows standard file

call "c:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat"
echo %WORKSPACE%
cd %WORKSPACE%
mkdir build
cd build
cmake .. -DBOOST_ROOT:STRING="C:\local\boost_1_55_0"
msbuild ALL_BUILD.vcxproj  
