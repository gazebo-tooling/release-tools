#!/bin/bash
#
# Please, see README for documentation
# 


if [[ ${#} -lt 2 ]]; then
    echo "Usage: ${0} <package-name> <script-name>"
    exit -1
fi

export PACKAGE=${1}
export WORKSPACE=/tmp/workspace

. prepare_env.sh

set_up_workspace

# get release-tools
set_up_release_tools

# get source code
set_up_software_code

SCRIPT_TO_CALL="${SCRIPT_DIR}/docker/${2}"

echo " + running script ${SCRIPT_TO_CALL}"
chmod +x ${SCRIPT_TO_CALL}
HOME=${FAKE_HOME} ${SCRIPT_TO_CALL}
