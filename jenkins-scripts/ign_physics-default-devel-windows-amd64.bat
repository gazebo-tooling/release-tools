set SCRIPT_DIR=%~dp0

set VCS_DIRECTORY=ign-physics
set PLATFORM_TO_BUILD=x86_amd64
set IGN_CLEAN_WORKSPACE=true

set DART_BOOST_PKGS="boost-algorithm boost-filesystem boost-lexical-cast boost-math boost-system"
set DART_DEPEN_PKGS="%DART_BOOST_PKGS% assimp bullet3 ccd fcl tinyxml2 urdfdom"
set DEPEN_PKGS="eigen3 tinyxml2 %DART_DEPEN_PKGS%"
:: This needs to be migrated to DSL to get multi-major versions correctly
set COLCON_PACKAGE=ignition-physics
set COLCON_AUTO_MAJOR_VERSION=true

call "%SCRIPT_DIR%/lib/colcon-default-devel-windows.bat"
