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
if [[ $PACKAGE_REPO != "nightly" ]]; then
  ARCHS+=( "i386" "arm64" "armhf")
  DISTROS+=( "debian" )
fi

for LIB in $(get_libraries_by_collection "${COLLECTION}" ); do
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
      fi
    else
      VERS=( "buster" ) # "sid"
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
          # The Source field is not mandatory and it is probably not present when
          # the binary package has the same name than the source package
          PKG_VERSION=$(wget -qO- http://packages.osrfoundation.org/gazebo/${DISTRO}-${PACKAGE_REPO}/dists/${VER}/main/binary-${ARCH}/Packages | \
            grep -2 -m 1 -e "Source: ${LIB}" -e "Package: ${LIB}" | \
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
