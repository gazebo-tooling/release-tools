#!/bin/bash

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
  "ignition-physics4"
  "ignition-sensors5"
  "ignition-gui5"
  "ignition-gazebo5"
  "ignition-launch4"
  "ignition-edifice"
)

ARCHS=( "amd64" "i386" "arm64" "armhf")

DISTROS=( "ubuntu" "debian" )

COLUMN="        "

for LIB in "${LIBS[@]}"
do
  echo -e "\e[107m\e[90m${LIB}\e[49m\e[39m"

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
	  PKG_VERSION=$(wget -qO- http://packages.osrfoundation.org/gazebo/${DISTRO}-stable/dists/${VER}/main/binary-${ARCH}/Packages | grep -1 "Source: ${LIB}" | sed -n 's/^Version: \(.*\)/\1/p' | uniq)
	fi

	if [[ -z ${PKG_VERSION} ]]; then
	    PKG_VERSION="not-found"
	fi

	PKG_VERSION_JUSTIFY=$(printf '%-15.15s' "${PKG_VERSION}")
        if [[ ${PKG_VERSION} != "not-found" ]]; then
          echo -n -e "\e[42m$PADDED_ARCH ${PKG_VERSION_JUSTIFY}\e[49m"
         else
          echo -n -e "\e[101m$PADDED_ARCH ${PKG_VERSION_JUSTIFY}\e[49m"
        fi

      done
      echo ""
    done
  done
done
