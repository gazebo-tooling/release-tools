#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

RERUN_FAILED_TESTS=1

PRE_TESTS_EXECUTION_HOOK="""
echo '#BEGIN SECTION: compile the tests suite'
make tests -j\${MAKE_JOBS} \${MAKE_VERBOSE_STR}
echo '# END SECTION'
"""

# clear the heightmap paging cache
rm -rf $HOME/.gazebo/paging

. ${SCRIPT_DIR}/lib/project-default-devel-homebrew-amd64.bash gazebo
