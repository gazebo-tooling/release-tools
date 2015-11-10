#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true

# Both empty, the one line script should handle all the stuff
export INSTALL_JOB_PKG="handsim"
export INSTALL_JOB_REPOS="stable"

# To simulate an offline update first install handsim, after it 
# run the unbundler
INSTALL_JOB_POSTINSTALL_HOOK="""
echo '# BEGIN SECTION: run the unbundler update'
wget http://packages.osrfoundation.org/haptix/handsim-debs-latest.zip
echo '# END SECTION'

echo '# BEGIN SECTION: run the unbundler'
cd \$WORKSPACE
unzip handsim-debs-latest.zip
cd handsim-*
bash -x ./handsim-unbundler.bash
echo '# END SECTION'
"""

# Need bc to proper testing and parsing the time
export DEPENDENCY_PKGS="wget unzip"

. ${SCRIPT_DIR}/lib/generic-install-base.bash
