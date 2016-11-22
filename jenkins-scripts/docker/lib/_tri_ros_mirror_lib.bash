#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

DISTRO=${DISTRO:-trusty}
ARCH=${ARCH:-amd64}
APLTY_CMD="sudo aptly -config=/etc/aptly.conf"
APLTY_MIRROR_NAME="ros_stable"
APLTY_PUBLISH_REPO_PREFIX="ros-stable-trusty-pkgs"
USE_APTLY_MANAGEMENT=${USE_APTLY_MANAGEMENT:-false}

if $USE_APTLY_MANAGEMENT; then
  # Be sure that we have ROS keys
  GPG_CMD="sudo gpg --no-default-keyring --keyring trustedkeys.gpg --keyserver keys.gnupg.net" 
  if [[ -z $(${GPG_CMD} --list-keys | grep "B01FA116") ]]; then
    ${GPG_CMD} --recv-keys "5523BAEEB01FA116"
  fi
fi

generate_snapshot_info()
{
  local snapshot_name=${1}

  # export list of availabe snapshots and other information
  ${APLTY_CMD} snapshot list > ${WORKSPACE}/info/snapshot_list
  # get the package list from snapshot
  ${APLTY_CMD} snapshot show --with-packages ${snapshot_name} > ${WORKSPACE}/info/snapshot_pkgs
  # run verify exploring missing ros- packages
  ${APLTY_CMD} snapshot verify ${snapshot_name} | grep "ros-" > ${WORKSPACE}/info/missing_ros_dependencies_in_snapshot || true
}
