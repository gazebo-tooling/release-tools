#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export USE_APTLY_MANAGEMENT=true

# TRI ROS Mirror common library
. ${SCRIPT_DIR}/lib/_tri_ros_mirror_lib.bash

if [[ -z ${SNAPSHOT_NAME} ]]; then
  echo "SNAPSHOT_NAME variable is empty"
  exit 1
fi

# Switch the snapshot published
${APLTY_CMD} publish switch ${DISTRO} ${APLTY_PUBLISH_REPO_PREFIX} ${SNAPSHOT_NAME}

mkdir -p ${WORKSPACE}/info
generate_snapshot_info ${SNAPSHOT_NAME}
