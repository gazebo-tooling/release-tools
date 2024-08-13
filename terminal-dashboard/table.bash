#!/bin/bash

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Arguments
#
# 1. <collection>: Required: A supported collection, i.e. "citadel", "fortesss", etc.
# 2. <package_repo>: Optional: stable / prerelease / nightly (defaults to stable)
#
# Usage
#
#   bash table.bash <collection> <package_repo>
#
# For example
#
#  bash table.bash edifice prerelease

# shellcheck source=./_dashboard_lib.sh
. ${SCRIPT_DIR}/_dashboard_lib.sh

COLLECTION=$1
PACKAGE_REPO=${2:-stable}

COLUMN="        "
GREEN="\e[42m"
YELLOW="\e[43m"
RED="\e[101m"

ARCHS=( "amd64")
DISTROS=( "ubuntu" )
# No nightlies or pre-releases for arm
if [[ $PACKAGE_REPO == "stable" ]]; then
  if [[ $COLLECTION == "citadel" || $COLLECTION == "fortress" ]]; then
    ARCHS+=( "i386" )
  fi

  ARCHS+=( "arm64" "armhf")
  # No debian version supported across the stack right now
  # DISTROS+=( "debian" )
fi

# Search heuristics used and context:
# We are assuming that all the gz- libraries have a package named libgz${LIB} or
# libgz${LIB}-dev except for the collection packages starting with Garden.
# Note that relying on the fact of having a packages available in a given
# arch does not imply that build is successful since arch=all debs are built
# once for amd64 and appear in all the arches.
# The Source field (for source packages) is not mandatory and it is probably
# not present when the binary package has the same name than the source
# package.
for LIB in $(get_libraries_by_collection "${COLLECTION}" ); do
  if [[ "${LIB}" != "gz-${COLLECTION}" && "${LIB}" != "ignition-${COLLECTION}" ]]; then
    LIB=lib${LIB}
  fi
  echo -e "\e[107m\e[90m${LIB}\e[49m\e[39m"

  LIB_VER=""

  for DISTRO in "${DISTROS[@]}"
  do
    if [[ $DISTRO == "ubuntu" ]]; then
      if [[ $COLLECTION == "citadel" ]]; then
        VERS=( "bionic" "focal" )
      elif [[ $COLLECTION == "fortress" ]]; then
        VERS=( "bionic" "focal" "jammy" )
      elif [[ $COLLECTION == "garden" ]]; then
        VERS=( "focal" "jammy" )
      elif [[ $COLLECTION == "harmonic" ]]; then
        VERS=( "jammy" )
      fi
    fi

    for VER in "${VERS[@]}"
    do
      PADDED_VER=$VER
      PADDED_VER="${PADDED_VER:0:8}${COLUMN:0:$((8 - ${#PADDED_VER}))}"

      echo -n "$PADDED_VER"
      for ARCH in "${ARCHS[@]}"
      do
        PADDED_ARCH=$ARCH
        PADDED_ARCH="${PADDED_ARCH:0:8}${COLUMN:0:$((8 - ${#PADDED_ARCH}))}"

        echo -n " "

        if [[ $ARCH == "i386" && $VER != "bionic" && $VER != "buster" ]]; then
          PKG_VERSION="disabled"
        else
          PKG_VERSION=$(wget -qO- http://packages.osrfoundation.org/gazebo/${DISTRO}-${PACKAGE_REPO}/dists/${VER}/main/binary-${ARCH}/Packages | \
            grep -2 -m 1 -e "Package: ${LIB}$" -e "Package: ${LIB}-dev$"| \
            sed -n 's/^Version: \(.*\)/\1/p' | uniq)
        fi

        PKG_VERSION=${PKG_VERSION%%~*}

        COMPACT_VERSION=${PKG_VERSION#*  }
        COMPACT_VERSION=${COMPACT_VERSION%%-*}

        if [[ -z ${PKG_VERSION} ]]; then
          PKG_VERSION="not-found"
          COLOR=$RED
        elif [[ "$PKG_VERSION" == "disabled" ]]; then
          COLOR=$GREEN
        elif [ -z "$LIB_VER" ]; then
          LIB_VER=${COMPACT_VERSION}
          COLOR=$GREEN
        elif [[ "$LIB_VER" != "$COMPACT_VERSION" ]]; then
          COLOR=$YELLOW
        else
          COLOR=$GREEN
        fi

        PKG_VERSION_JUSTIFY=$(printf '%-10.10s' "${PKG_VERSION}")
        echo -n -e "$COLOR$PADDED_ARCH ${PKG_VERSION_JUSTIFY}\e[49m"
      done
      echo ""
    done
  done
done
