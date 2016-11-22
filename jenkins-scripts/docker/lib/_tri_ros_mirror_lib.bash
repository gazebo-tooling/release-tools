#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

DISTRO=${DISTRO:-trusty}
ARCH=${ARCH:-amd64}
APLTY_CMD="sudo aptly"
APLTY_MIRROR_NAME="ros_stable"
APLTY_PUBLISH_REPO_PREFIX="ros-stable-trusty-pkgs"

generate_snapshot_info()
{
  local snapshots_name=${1}

  # export list of availabe snapshots and other information
  ${APLTY_CMD} snapshot list > ${WORKSPACE}/info/snapshot_list
  # get the package list from snapshot
  ${APLTY_CMD} snapshot show ${snapshot_name} > ${WORKSPACE}/info/snapshot_pkgs
  # run verify exploring missing ros- packages
  ${APLTY_CMD} snapshot verify ${snapshot_name} > ${WORKSPACE}/info/verify_snapshot || true
}
