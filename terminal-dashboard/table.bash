#!/bin/bash

LIBS=(
  "libignition-cmake2-dev_2.7.0-1"
  "libignition-math6_6.8.0-1"
  "libignition-utils1_1.0.0-1"
  "libignition-tools-dev_1.1.0-1"
  "libignition-common4_4.0.0-1"
  "libignition-msgs7_7.0.0-1"
  "libignition-transport10_10.0.0-1"
  "libignition-fuel-tools5_5.0.0-1"
  "libignition-plugin_1.2.0-1"
  "libignition-rendering5_5.0.0-1"
  "libignition-physics4_4.0.0-1"
  "libignition-sensors5_5.0.0-1"
  "libignition-gui5_5.0.0-1"
  "libignition-gazebo5_5.0.0-1"
  "libignition-launch4_4.0.0-1"
  "ignition-edifice_1.0.0-1"
)

ARCHS=( "amd64" "i386" "arm64" "armhf")

DISTROS=( "ubuntu" "debian" )

COLUMN="        "

for LIB in "${LIBS[@]}"
do
  echo -e "\e[107m\e[90m${LIB}\e[49m\e[39m"

  LIB_NO_VERSION=${LIB%%_*}
  LIB_NO_VERSION=${LIB_NO_VERSION#"lib"}
  LIB_NO_VERSION=${LIB_NO_VERSION%%-dev*}

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
          echo -n -e "\e[100m$PADDED_ARCH\e[49m"
          continue
        fi

        URL=http://packages.osrfoundation.org/gazebo/${DISTRO}-stable/pool/main/i/${LIB_NO_VERSION}/${LIB}~${VER}_${ARCH}.deb

        #echo "Checking [$URL]"

        if curl --head --silent --fail $URL > /dev/null;
         then
          echo -n -e "\e[42m$PADDED_ARCH\e[49m"
         else
          echo -n -e "\e[101m$PADDED_ARCH\e[49m"
        fi

      done
      echo ""
    done
  done
done
