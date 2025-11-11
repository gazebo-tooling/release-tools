#!/bin/bash

# **** WARNING ***** : when modifying this file
# **** WARNING ***** : any trailing whitespaces will break dependencies scapes

pythonv="python3"
if [[ ${DISTRO} == 'trusty' ]]; then
  pythonv="python"
fi

# need to override the above distro check since
# ROS1 pkgs depend on python2, except for noetic
if ${ENABLE_ROS} && ! ${ROS2}; then
  if [[ ${ROS_DISTRO} != 'noetic' ]]; then
    pythonv="python"
  fi
fi

# mesa-utils, x11-utils for dri checks, xsltproc for qtest->junit conversion and
# python-psutil for memory testing
# netcat-openbsd (nc command) for squid-deb-proxy checking
# net-tools (route command) for squid-deb-proxy checking
# gnupg apt-key requires gnupg, gnupg2 or gnupg1
BASE_DEPENDENCIES="build-essential \\
                   cmake           \\
                   debhelper       \\
                   mesa-utils      \\
                   x11-utils       \\
                   cppcheck        \\
                   xsltproc        \\
                   ${pythonv}-lxml \\
                   ${pythonv}-psutil \\
                   ${pythonv}      \\
                   bc              \\
                   netcat-openbsd  \\
                   gnupg2          \\
                   net-tools       \\
                   locales         \\
                   sudo"

BREW_BASE_DEPENDCIES="git cmake"

# 1. SDFORMAT
# ruby for xml_schemas generation and libxml2-utils for xmllint used in tests
SDFORMAT_NO_GZ_DEPENDENCIES="${pythonv}     \\
                              libxml2-utils  \\
                              libtinyxml-dev"

# SDFORMAT 10 and above use tinyxml2
if [[ ${SDFORMAT_MAJOR_VERSION} -ge 10 ]]; then
  SDFORMAT_NO_GZ_DEPENDENCIES="${SDFORMAT_NO_GZ_DEPENDENCIES} \\
                                libtinyxml2-dev"
fi

if [[ ${SDFORMAT_MAJOR_VERSION} -lt 8 ]]; then
SDFORMAT_NO_GZ_DEPENDENCIES="${SDFORMAT_NO_GZ_DEPENDENCIES} \\
                              libboost-system-dev          \\
                              libboost-filesystem-dev      \\
                              libboost-program-options-dev \\
                              libboost-regex-dev           \\
                              libboost-iostreams-dev"
fi

SDFORMAT_NO_GZ_DEPENDENCIES="${SDFORMAT_NO_GZ_DEPENDENCIES} \\
                            ruby-dev                          \\
                            ruby"

# SDFORMAT related dependencies
if [[ -z ${SDFORMAT_MAJOR_VERSION} ]]; then
    SDFORMAT_MAJOR_VERSION=6
fi

if [[ ${SDFORMAT_MAJOR_VERSION} -ge 9 && ${SDFORMAT_MAJOR_VERSION} -lt 13 ]]; then
    # sdformat9 requires ignition-math6 and
    # uses ignition-tools for a test
    SDFORMAT_BASE_DEPENDENCIES="${SDFORMAT_NO_GZ_DEPENDENCIES} \\
                                libignition-math6-dev           \\
                                libignition-tools-dev"
elif [[ ${SDFORMAT_MAJOR_VERSION} -eq 6 ]]; then
    # sdformat6 requires ignition-math4 and
    # uses ignition-tools for a test
    SDFORMAT_BASE_DEPENDENCIES="${SDFORMAT_NO_GZ_DEPENDENCIES} \\
                                libignition-math4-dev           \\
                                libignition-tools-dev"
else
    SDFORMAT_BASE_DEPENDENCIES="${SDFORMAT_NO_GZ_DEPENDENCIES}"
fi

ogre_pkg="libogre-1.9-dev"
if [[ ${DISTRO} != 'xenial' ]]; then
  ogre_pkg="libogre-1.9-dev libogre-2.1-dev"
fi

if [[ -z $ROS_DISTRO ]]; then
  echo "------------------------------------------------------------"
  echo "ROS_DISTRO was not set before using dependencies_archive.sh!"
  echo "skipping ROS related variables"
  echo "------------------------------------------------------------"
else
  # colcon has no python2 candidate
  if ${ROS2}; then
    ROS_CATKIN_BASE="${pythonv}-dev                      \\
                    python3-colcon-common-extensions     \\
                    ${pythonv}-rosdep                    \\
                    ${pythonv}-wstool                    \\
                    ${pythonv}-rosinstall                \\
                    ${pythonv}-rospkg                    \\
                    ${pythonv}-vcstools"
  else
    ROS_CATKIN_BASE="${pythonv}-dev              \\
                    ${pythonv}-catkin-pkg        \\
                    python3-colcon-common-extensions \\
                    ${pythonv}-rosdep            \\
                    ${pythonv}-wstool            \\
                    ros-${ROS_DISTRO}-catkin     \\
                    ros-${ROS_DISTRO}-ros        \\
                    ${pythonv}-rosinstall        \\
                    ${pythonv}-catkin-pkg        \\
                    ${pythonv}-rospkg            \\
                    ${pythonv}-vcstools"
  fi
fi

#
# IGNITION
#

# Completely rely on packages.apt from Garden
if [[ ${GZ_MATH_MAJOR_VERSION} -lt 7 ]]; then
  if [[ ${GZ_MATH_MAJOR_VERSION} -eq 4 ]]; then
    GZ_MATH_DEPENDENCIES="libeigen3-dev \\
                           libpython3-dev \\
                           ruby-dev \\
                           swig \\
                           libignition-cmake-dev"
  else
    GZ_MATH_DEPENDENCIES="libeigen3-dev \\
                           libpython3-dev \\
                           ruby-dev \\
                           swig \\
                           libignition-cmake2-dev"
  fi
fi

# IGN_TRANSPORT related dependencies. Default value points to the development
# version
if [[ -z ${GZ_TRANSPORT_MAJOR_VERSION} ]]; then
    GZ_TRANSPORT_MAJOR_VERSION=5
fi

# Completely rely on packages.apt from Garden
if [[ ${GZ_TRANSPORT_MAJOR_VERSION} -lt 12 ]]; then
  GZ_TRANSPORT_NO_GZ_DEPENDENCIES="pkg-config           \\
                                     ${pythonv}           \\
                                     ruby-ronn            \\
                                     libprotoc-dev        \\
                                     libprotobuf-dev      \\
                                     protobuf-compiler    \\
                                     uuid-dev             \\
                                     libzmq3-dev          \\
                                     libczmq-dev"
  if [[ ${GZ_TRANSPORT_MAJOR_VERSION} -eq 4 ]]; then
      export GZ_TRANSPORT_DEPENDENCIES="${GZ_TRANSPORT_NO_GZ_DEPENDENCIES} \\
                                  libignition-cmake-dev \\
                                  libignition-msgs-dev"
  elif [[ ${GZ_TRANSPORT_MAJOR_VERSION} -ge 8 ]]; then
      export GZ_TRANSPORT_DEPENDENCIES="${GZ_TRANSPORT_NO_GZ_DEPENDENCIES} \\
                                    libignition-cmake2-dev \\
                                    libignition-msgs5-dev \\
                                    libsqlite3-dev \\
                                    ruby-ffi"
  fi
  export GZ_TRANSPORT_DEPENDENCIES="${GZ_TRANSPORT_DEPENDENCIES} libignition-tools-dev"
fi

# Completely rely on packages.apt from Garden
if [[ ${GZ_COMMON_MAJOR_VERSION} -lt 5 ]]; then
  GZ_COMMON_NO_GZ_DEPENDENCIES="pkg-config     \\
                           ${pythonv}            \\
                           ruby-ronn             \\
                           uuid-dev              \\
                           libfreeimage-dev      \\
                           libgts-dev            \\
                           libavformat-dev       \\
                           libavcodec-dev        \\
                           libswscale-dev        \\
                           libavutil-dev         \\
                           libavdevice-dev       \\
                           libtinyxml2-dev       \\
                           uuid-dev"

  GZ_COMMON_DEPENDENCIES="${GZ_COMMON_NO_GZ_DEPENDENCIES} \\
                       libignition-cmake-dev \\
                       libignition-cmake2-dev \\
                       libignition-math4-dev \\
                       libignition-math6-dev"
fi

# Completely rely on packages.apt from Garden
if [[ ${GZ_FUEL_TOOLS_MAJOR_VERSION} -lt 8 ]]; then
  GZ_FUEL_TOOLS_DEPENDENCIES="libignition-cmake2-dev \\
                               libignition-common3-dev \\
                               libignition-tools-dev  \\
                               libcurl4-openssl-dev   \\
                               libjsoncpp-dev         \\
                               libyaml-dev            \\
                               libzip-dev"

  if [[ ${GZ_FUEL_TOOLS_MAJOR_VERSION} -eq 1 ]]; then
    GZ_FUEL_TOOLS_DEPENDENCIES="${GZ_FUEL_TOOLS_DEPENDENCIES} \\
                                 libignition-cmake-dev  \\
                                 libignition-common-dev"
  else
    GZ_FUEL_TOOLS_DEPENDENCIES="${GZ_FUEL_TOOLS_DEPENDENCIES} \\
                                 libignition-cmake2-dev  \\
                                 libignition-common3-dev \\
                                 libtinyxml2-dev"
  fi

  if [[ ${GZ_FUEL_TOOLS_MAJOR_VERSION} -ge 4 ]]; then
    GZ_FUEL_TOOLS_DEPENDENCIES="${GZ_FUEL_TOOLS_DEPENDENCIES} \\
                                 libignition-msgs5-dev"
  fi
fi

# Completely rely on packages.apt from Garden
if [[ ${GZ_MSGS_MAJOR_VERSION} -lt 9 ]]; then
  GZ_MSGS_DEPENDENCIES="libignition-tools-dev \\
                         libprotobuf-dev       \\
                         libprotoc-dev         \\
                         protobuf-compiler     \\
                         libtinyxml2-dev       \\
                         ruby                  \\
                         ruby-dev"

  if [[ -n ${GZ_MSGS_MAJOR_VERSION} && ${GZ_MSGS_MAJOR_VERSION} -eq 1 ]]; then
      GZ_MSGS_DEPENDENCIES="${GZ_MSGS_DEPENDENCIES} \\
                             libignition-cmake-dev \\
                             libignition-math4-dev"
  elif [[ -n ${GZ_MSGS_MAJOR_VERSION} && ${GZ_MSGS_MAJOR_VERSION} -ge 5 ]]; then
      GZ_MSGS_DEPENDENCIES="${GZ_MSGS_DEPENDENCIES} \\
                             libignition-cmake2-dev \\
                             libignition-math6-dev"
  fi
fi

# Completely rely on packages.apt from Garden
if [[ ${GZ_GUI_MAJOR_VERSION} -lt 7 ]]; then
  GZ_GUI_NO_GZ_DEPENDENCIES="qtbase5-dev \\
                        qtdeclarative5-dev \\
                        libtinyxml2-dev \\
                        libqwt-qt5-dev \\
                        qml-module-qt-labs-folderlistmodel \\
                        qml-module-qt-labs-platform \\
                        qml-module-qt-labs-settings \\
                        qml-module-qtcharts \\
                        qml-module-qtgraphicaleffects \\
                        qml-module-qtqml-models2 \\
                        qml-module-qtquick-controls \\
                        qml-module-qtquick-controls2 \\
                        qml-module-qtquick-dialogs \\
                        qml-module-qtquick-layouts \\
                        qml-module-qtquick-templates2 \\
                        qml-module-qtquick-window2 \\
                        qml-module-qtquick2 \\
                        qtquickcontrols2-5-dev"
  GZ_GUI_DEPENDENCIES="${GZ_GUI_NO_GZ_DEPENDENCIES} \\
                        libignition-cmake2-dev \\
                        libignition-common3-dev \\
                        libignition-math6-dev \\
                        libignition-plugin-dev \\
                        libignition-tools-dev"

  if [[ -n "${GZ_GUI_MAJOR_VERSION}" && ${GZ_GUI_MAJOR_VERSION} -ge 4 ]]; then
    GZ_GUI_DEPENDENCIES="${GZ_GUI_DEPENDENCIES} \\
                          libignition-common3-dev"
  elif [[ -n "${GZ_GUI_MAJOR_VERSION}" && ${GZ_GUI_MAJOR_VERSION} -eq 3 ]]; then
    GZ_GUI_DEPENDENCIES="${GZ_GUI_DEPENDENCIES} \\
                          libignition-common3-dev \\
                          libignition-msgs5-dev \\
                          libignition-rendering3-dev \\
                          libignition-transport8-dev"
  fi
fi

GZ_PHYSICS_DART_FROM_PKGS="true"

# Completely rely on packages.apt from Garden
if [[ ${GZ_PLUGIN_MAJOR_VERSION} -lt 2 ]]; then
  GZ_PLUGIN_DEPENDENCIES="libignition-cmake2-dev"
fi

# Completely rely on packages.apt from Garden
if [[ ${GZ_LAUNCH_MAJOR_VERSION} -lt 6 ]]; then
  GZ_LAUNCH_COMMON_DEPENDENCIES="libignition-cmake2-dev \\
                           libignition-common3-dev \\
                           libignition-math6-dev \\
                           libignition-plugin-dev \\
                           libignition-tools-dev \\
                           libtinyxml2-dev  \\
                           qtquickcontrols2-5-dev \\
                           libqt5core5a"

  if [[ -n "${GZ_LAUNCH_MAJOR_VERSION}" && ${GZ_LAUNCH_MAJOR_VERSION} -ge 3 ]]; then
    GZ_LAUNCH_DEPENDENCIES="${GZ_LAUNCH_COMMON_DEPENDENCIES} \\
                             libignition-gui4-dev \\
                             libignition-fuel-tools5-dev \\
                             libignition-gazebo4-dev \\
                             libignition-msgs6-dev \\
                             libignition-rendering4-dev  \\
                             libignition-sensors4-dev  \\
                             libignition-transport9-dev \\
                             libsdformat10-dev"
  else
    GZ_LAUNCH_DEPENDENCIES="${GZ_LAUNCH_DEPENDENCIES} \\
                             libignition-gui3-dev \\
                             libignition-fuel-tools4-dev \\
                             libignition-gazebo3-dev \\
                             libignition-msgs5-dev \\
                             libignition-rendering3-dev  \\
                             libignition-sensors3-dev  \\
                             libignition-transport8-dev \\
                             libsdformat9-dev \\
                             libwebsockets-dev \\
                             binutils-dev"
  fi
fi

# Completely rely on packages.apt from Garden
if [[ ${GZ_RENDERING_MAJOR_VERSION} -lt 7 ]]; then
  GZ_RENDERING_NO_GZ_DEPENDENCIES="${ogre_pkg}\\
                              freeglut3-dev \\
                              libfreeimage-dev \\
                              libglew-dev \\
                              libogre-1.9-dev \\
                              libx11-dev \\
                              mesa-common-dev \\
                              mesa-utils"

  GZ_RENDERING_DEPENDENCIES="${GZ_RENDERING_NO_GZ_DEPENDENCIES} \\
                              libignition-cmake2-dev \\
                              libignition-common3-dev \\
                              libignition-plugin-dev \\
                              libignition-math6-dev"
fi

# Completely rely on packages.apt from Garden
if [[ ${GZ_SENSORS_MAJOR_VERSION} -lt 7 ]]; then
  GZ_SENSORS_DEPENDENCIES="libignition-common3-dev     \\
                            libignition-cmake2-dev \\
                            libignition-math6-dev      \\
                            libignition-plugin-dev  \\
                            libignition-tools-dev"

  if [[ -n "${GZ_SENSORS_MAJOR_VERSION}" && ${GZ_SENSORS_MAJOR_VERSION} -ge 4 ]]; then
    GZ_SENSORS_DEPENDENCIES="${GZ_SENSORS_DEPENDENCIES} \\
                          libignition-msgs6-dev \\
                          libignition-rendering4-dev \\
                          libignition-transport9-dev \\
                          libsdformat10-dev"
  elif [[ -n "${GZ_SENSORS_MAJOR_VERSION}" && ${GZ_SENSORS_MAJOR_VERSION} -ge 3 ]]; then
    GZ_SENSORS_DEPENDENCIES="${GZ_SENSORS_DEPENDENCIES} \\
                          libignition-msgs5-dev \\
                          libignition-rendering3-dev \\
                          libignition-transport8-dev \\
                          libsdformat9-dev"
  fi
fi

# Completely rely on packages.apt from Garden
if [[ ${GZ_SIM_MAJOR_VERSION} -lt 7 ]]; then
  GZ_SIM_DEPENDENCIES="libignition-common3-dev  \\
                           libignition-cmake2-dev   \\
                           libgflags-dev            \\
                           libignition-math6-dev    \\
                           libignition-math6-eigen3-dev \\
                           libbenchmark-dev"
  if [[ -n "${GZ_SIM_MAJOR_VERSION}" && ${GZ_SIM_MAJOR_VERSION} -ge 4 ]]; then
    GZ_SIM_DEPENDENCIES="${GZ_SIM_DEPENDENCIES} \\
                          libsdformat10-dev"
  elif [[ -n "${GZ_SIM_MAJOR_VERSION}" && ${GZ_SIM_MAJOR_VERSION} -eq 3 ]]; then
    GZ_SIM_DEPENDENCIES="${GZ_SIM_DEPENDENCIES} \\
                          libignition-fuel-tools4-dev \\
                          libignition-gui3-dev \\
                          libignition-msgs5-dev \\
                          libignition-rendering3-dev \\
                          libignition-sensors3-dev \\
                          libignition-physics2           \\
                          libignition-physics2-dartsim   \\
                          libignition-physics2-dartsim-dev \\
                          libignition-physics2-dev       \\
                          libignition-transport8-dev \\
                          libsdformat9-dev"
  fi
fi

# Completely rely on packages.apt from Garden
if [[ ${GZ_UTILS_MAJOR_VERSION} -lt 2 ]]; then
  GZ_UTILS_DEPENDENCIES="libignition-cmake2-dev"
fi
