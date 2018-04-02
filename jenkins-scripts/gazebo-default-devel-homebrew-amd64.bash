#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Identify GAZEBO_MAJOR_VERSION to help with dependency resolution
GAZEBO_MAJOR_VERSION=$(\
  python ${SCRIPT_DIR}/tools/detect_cmake_major_version.py \
  ${WORKSPACE}/gazebo/CMakeLists.txt)

if [ $GAZEBO_MAJOR_VERSION -ge 7 ]; then
  RERUN_FAILED_TESTS=1
fi

if [[ $GAZEBO_MAJOR_VERSION -ge 8 ]]; then
PRE_TESTS_EXECUTION_HOOK="""
echo '#BEGIN SECTION: compile the tests suite'
make tests -j\${MAKE_JOBS} \${MAKE_VERBOSE_STR}
echo '# END SECTION'
"""
fi

# clear the heightmap paging cache
rm -rf $HOME/.gazebo/paging

. ${SCRIPT_DIR}/lib/project-default-devel-homebrew-amd64.bash gazebo
