#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# TRI ROS Mirror common library
. ${SCRIPT_DIR}/_common_library.bash

if [[ -z ${SNAPSHOT_NAME} ]]; then
  echo "SNAPSHOT_NAME variable is empty"
  exit 1
fi

# Switch the snapshot published
${APLTY_MIRROR_NAME} publish switch ${DISTRO} ${APLTY_PUBLISH_REPO_PREFIX} ${SNAPSHOT_NAME}
generate_snapshot_info ${SNAPSHOT_NAME}
