:: Windows standard file to build ignition collections
::
:: Parameters:
::   - IGN_COLLECTION: name of the ignition collection
::

:: safety checks
if not defined IGN_COLLECTION (
  echo # BEGIN SECTION: ERROR: IGN_COLLECTION is not set
  echo IGN_COLLECTION variable was not set. Please set it before calling this script
  echo # END SECTION
  exit 1
)

set win_lib=%SCRIPT_DIR%\lib\windows_library.bat
set LOCAL_WS=%WORKSPACE%\ws
set LOCAL_WS_BUILD=%WORKSPACE%\build
set GAZEBODISTRO_FILE=collection-%IGN_COLLECTION%

:: default values
@if "%BUILD_TYPE%" == "" set BUILD_TYPE=Release
@if "%ENABLE_TESTS%" == "" set ENABLE_TESTS=TRUE


:: Call vcvarsall and all the friends
echo # BEGIN SECTION: configure the MSVC compiler
call %win_lib% :configure_msvc2017_compiler
echo # END SECTION

echo # BEGIN SECTION: setup workspace
if defined KEEP_WORKSPACE (
  IF exist %LOCAL_WS_BUILD% (
    echo # BEGIN SECTION: preclean workspace
    rmdir /s /q %LOCAL_WS_BUILD% || goto :error
    echo # END SECTION
  )
)
mkdir %LOCAL_WS% || echo "Workspace already exists!"
cd %LOCAL_WS%
echo # END SECTION

echo # BEGIN SECTION: get open robotics deps (%GAZEBODISTRO_FILE%) sources into the workspace
if exist %LOCAL_WS_SOFTWARE_DIR% ( rmdir /q /s %LOCAL_WS_SOFTWARE_DIR% )
call %win_lib% :get_source_from_gazebodistro %GAZEBODISTRO_FILE% %LOCAL_WS% || goto :error
echo # END SECTION

for %%p in (%DEPEN_PKGS%) do (
  call %win_lib% :enable_vcpkg_integration || goto :error
  echo # BEGIN SECTION: install external dependency %%p
  call %win_lib% :install_vcpkg_package %%p || goto :error
  echo # END SECTION
)

echo # BEGIN SECTION: packages in workspace
call %win_lib% :list_workspace_pkgs || goto :error
echo # END SECTION

echo # BEGIN SECTION: compiling collection
cd %LOCAL_WS%
call %win_lib% :build_workspace || goto :error
echo # END SECTION

if NOT DEFINED KEEP_WORKSPACE (
   echo # BEGIN SECTION: clean up workspace
   cd %WORKSPACE%
   rmdir /s /q %LOCAL_WS% || goto :error
   echo # END SECTION
)
goto :EOF

:error - error routine
echo Failed with error #%errorlevel%.
exit /b %errorlevel%
