set VS32bits_CMAKE_GEN='-G"Visual Studio 12"'
set VS64bits_CMAKE_GEN='-G"Visual Studio 12 Win64"'

set VS_DEFAULT_FLAGS='/p:Configuration=Release -DCMAKE_BUILD_TYPE:STRING="Release"'

REM Software avalibale on windows node
set WINNODE_BOOST_ROOT='-DBOOST_ROOT:STRING="C:\local\boost_1_55_0" -DBOOST_LIBRARYDIR:STRING="C:\local\boost_1_55_0\lib64-msvc-12.0"'
