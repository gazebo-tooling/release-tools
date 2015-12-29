#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true

# Both empty, the one line script should handle all the stuff
export INSTALL_JOB_PKG=""
export INSTALL_JOB_REPOS=""

INSTALL_JOB_POSTINSTALL_HOOK="""
echo '# BEGIN SECTION: run the one-liner installation'
curl -ssL https://bitbucket.org/osrf/release-tools/raw/default/one-line-installations/gazebo.sh | sh
echo '# END SECTION'

echo '# BEGIN SECTION: test the script'
TEST_START=\`date +%s\`
timeout --preserve-status 180 gazebo --verbose || true
TEST_END=\`date +%s\`
DIFF=\`echo \"\$TEST_END - \$TEST_START\" | bc\`

if [ \$DIFF -lt 180 ]; then
   echo 'The test took less than 180s. Something bad happened'
   exit 1
fi
echo '# END SECTION'
"""

# Need bc to proper testing and parsing the time
export DEPENDENCY_PKGS DEPENDENCY_PKGS="bc"

. ${SCRIPT_DIR}/lib/generic-install-base.bash
