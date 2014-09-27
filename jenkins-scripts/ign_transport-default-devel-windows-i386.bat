set SCRIPT_DIR="%~dp0"

call "%SCRIPT_DIR%/lib/windows_configuration.bat"
REM x64 or x86
set PLATFORM_TO_BUILD=x86
set PATH=%PATH%;C:\local\protobuf_2.6.0
set ARG_CMAKE_FLAGS=%CMAKE_ZEROMQ_FLAGS% %CMAKE_PROTOBUF_FLAGS% %CMAKE_CPPZMQ_FLAGS%

call "%SCRIPT_DIR%/lib/project-default-devel-windows.bat"
