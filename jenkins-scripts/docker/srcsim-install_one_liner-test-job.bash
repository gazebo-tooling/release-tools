#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true

# Import library
. ${SCRIPT_DIR}/lib/_srcsim_lib.bash

# Both empty, the one line script should handle all the stuff
export INSTALL_JOB_PKG=""
export INSTALL_JOB_REPOS=""

INSTALL_JOB_POSTINSTALL_HOOK="""
echo '# BEGIN SECTION: run the one-liner installation'
export USER=root
# curl -ssL http://get.srcsim.gazebosim.org | sh
# TODO: temporary testing method
sh ${WORKSPACE}/scripts/one-line-installations/srcsim.sh
echo '# END SECTION'

echo '# BEGIN SECTION: testing by running qual2 launch file'
${SRCSIM_ENV_SETUP}

TEST_TIMEOUT=400

TEST_START=\`date +%s\`
timeout --preserve-status \$TEST_TIMEOUT roslaunch srcsim qual2.launch extra_gazebo_args:=\"-r\" init:=\"true\" walk_test:=true || true
TEST_END=\`date +%s\`
DIFF=\`echo \"\$TEST_END - \$TEST_START\" | bc\`

if [ \$DIFF -lt \$TEST_TIMEOUT ]; then
   echo 'The test took less than \$TEST_TIMEOUT. Something bad happened'
   exit 1
fi
echo '# END SECTION'
"""

# Need bc to proper testing and parsing the time
export DEPENDENCY_PKGS DEPENDENCY_PKGS="bc"

. ${SCRIPT_DIR}/lib/generic-install-base.bash
