#!/bin/bash

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Helper script to check if there are unreleased changes for supported versions.
# The script prints a list of links that can be clicked to see if there are new
# changes since the latest release.
#
# Arguments
#
# 1. <collection>: Required: A supported collection, i.e. "citadel", "fortesss", etc.
#
# Usage
#
#   bash unreleased.bash <collection>
#
# For example
#
#  bash unreleased.bash edifice

# shellcheck source=./_dashboard_lib.sh
. "${SCRIPT_DIR}"/_dashboard_lib.sh

COLLECTION=$1
PACKAGE_REPO=${2:-stable}

for LIB in $(get_libraries_by_collection "${COLLECTION}" ); do
  LIB_SHORT="${LIB/ignition-/ign-}"
  LIB_NAME=${LIB_SHORT//[[:digit:]]/}
  LIB_SHORT="${LIB_SHORT/sdformat/sdf}"

  PKG_VERSION=$(wget -qO- http://packages.osrfoundation.org/gazebo/ubuntu-${PACKAGE_REPO}/dists/focal/main/binary-amd64/Packages | \
    grep -1 -m 1 -e "Source: ${LIB}" -e "Package: ${LIB}" | \
    sed -n 's/^Version: \(.*\)/\1/p' | uniq)

  PKG_VERSION=${PKG_VERSION%%~*}

  COMPACT_VERSION=${PKG_VERSION#*  }
  COMPACT_VERSION=${COMPACT_VERSION%%-*}

  echo https://github.com/gazebosim/${LIB_NAME}/compare/${LIB}_${COMPACT_VERSION}...${LIB_SHORT}

done

