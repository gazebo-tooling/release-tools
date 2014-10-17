set VS32bits_CMAKE_GEN=-G"Visual Studio 12"
set VS64bits_CMAKE_GEN=-G"Visual Studio 12 Win64"

set VS_DEFAULT_CMAKE_FLAGS=-DCMAKE_BUILD_TYPE:STRING=Release
set VS_DEFAULT_MSBUILD_FLAGS=/p:Configuration=Release

REM Software avalibale on windows node
set CMAKE_BOOST_FLAGS=-DBOOST_ROOT:STRING="C:\local\boost_1_55_0" -DBOOST_LIBRARYDIR:STRING="C:\local\boost_1_55_0\lib64-msvc-12.0"

REM Protobuf variables
set CMAKE_PROTOBUF_FLAGS=-DPROTOBUF_SRC_ROOT_FOLDER="C:\local\protobuf-2.6.0"
set PROTOBUF_DLL_PATH=C:/local/protobuf-2.6.0/vsprojects/Release/

REM ZeroMQ variables
set CMAKE_ZEROMQ_FLAGS=-DZeroMQ_ROOT_DIR="C:\Program Files (x86)\ZeroMQ 4.0.4" 
set ZEROMQ_DLL_PATH=C:\Program Files (x86)\ZeroMQ 4.0.4\bin

REM cppzmq header 
set CMAKE_CPPZMQ_FLAGS=-DCPPZMQ_HEADER_PATH="C:\local\cppzmq"
