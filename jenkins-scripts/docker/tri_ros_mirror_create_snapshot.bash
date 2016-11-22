#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# TRI ROS Mirror common library
. ${SCRIPT_DIR}/lib/_tri_ros_mirror_lib.bash

mkdir -p ${WORKSPACE}/info

# Update the current aptly mirror
${APLTY_MIRROR_NAME} mirror update ${APLTY_MIRROR_NAME} 

# Create a snapshot
timestamp=$(date '+%Y_%m_%d')
snapshot_name="${APLTY_MIRROR_NAME}_${timestamp}"
# User can provide a tag to easily identify the snapshot by name
if [[ -n ${SNAPSHOT_TAG} ]]; then
  snapshot_name="${snapshot_name}+${SNAPSHOT_TAG}"
fi
${APLTY_CMD} snapshot create ${snapshot_name} from mirror ${APLTY_MIRROR_NAME}

echo ${snapshot_name} > $WORKSPACE/info/new_snapshot_name
generate_snapshot_info ${snapshot_name}
