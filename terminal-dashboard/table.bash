#!/bin/bash

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

COLLECTION=$1

PACKAGE_REPO=${2:-stable}

if [ "$COLLECTION" = "citadel" ]; then
  LIBS=(
    "ignition-cmake2"
    "ignition-math6"
    "ignition-tools"
    "ignition-common3"
    "ignition-msgs5"
    "ignition-transport8"
    "ignition-fuel-tools4"
    "ignition-plugin"
    "ignition-rendering3"
    "sdformat9"
    "ignition-physics2"
    "ignition-sensors3"
    "ignition-gui3"
    "ignition-gazebo3"
    "ignition-launch2"
    "ignition-citadel"
  )
elif [ "$COLLECTION" = "dome" ]; then
  LIBS=(
    "ignition-cmake2"
    "ignition-math6"
    "ignition-tools"
    "ignition-common3"
    "ignition-msgs6"
    "ignition-transport9"
    "ignition-fuel-tools5"
    "ignition-plugin"
    "ignition-rendering4"
    "sdformat10"
    "ignition-physics3"
    "ignition-sensors4"
    "ignition-gui4"
    "ignition-gazebo4"
    "ignition-launch3"
    "ignition-dome"
  )
elif [ "$COLLECTION" = "edifice" ]; then
  LIBS=(
    "ignition-cmake2"
    "ignition-math6"
    "ignition-utils1"
    "ignition-tools"
    "ignition-common4"
    "ignition-msgs7"
    "ignition-transport10"
    "ignition-fuel-tools6"
    "ignition-plugin"
    "ignition-rendering5"
    "sdformat11"
    "ignition-physics4"
    "ignition-sensors5"
    "ignition-gui5"
    "ignition-gazebo5"
    "ignition-launch4"
    "ignition-edifice"
  )
elif [ "$COLLECTION" = "fortress" ]; then
  LIBS=(
    "ignition-cmake2"
    "ignition-math6"
    "ignition-utils1"
    "ignition-tools"
    "ignition-common4"
    "ignition-msgs8"
    "ignition-transport11"
    "ignition-fuel-tools7"
    "ignition-plugin"
    "ignition-rendering6"
    "sdformat12"
    "ignition-physics5"
    "ignition-sensors6"
    "ignition-gui6"
    "ignition-gazebo6"
    "ignition-launch5"
    "ignition-fortress"
  )
else
  echo "Missing collection"
  exit
fi

ARCHS=( "amd64" "i386" "arm64" "armhf")

DISTROS=( "ubuntu" "debian" )

COLUMN="        "

GREEN="\e[42m"
YELLOW="\e[43m"
RED="\e[101m"

for LIB in "${LIBS[@]}"
do
  echo -e "\e[107m\e[90m${LIB}\e[49m\e[39m"

  LIB_VER=""

  for DISTRO in "${DISTROS[@]}"
  do
    if [[ $DISTRO == "ubuntu" ]]; then
      VERS=( "bionic" "focal" )
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

        if [[ $ARCH == "i386" && $VER == "focal" ]]; then
          PKG_VERSION="disabled"
        else
          PKG_VERSION=$(wget -qO- http://packages.osrfoundation.org/gazebo/${DISTRO}-${PACKAGE_REPO}/dists/${VER}/main/binary-${ARCH}/Packages | grep -1 "Source: ${LIB}" | sed -n 's/^Version: \(.*\)/\1/p' | uniq)
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
