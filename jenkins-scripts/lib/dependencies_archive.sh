#!/bin/bash

# **** WARNING ***** : when modifying this file
# **** WARNING ***** : any trailing whitespaces will break dependencies scapes

# Dart flags. Enable it by default unless compiled from source
if [ -z ${DART_COMPILE_FROM_SOURCE} ]; then
   DART_COMPILE_FROM_SOURCE=false
fi

if [ -z ${DART_FROM_PKGS} ]; then
  DART_FROM_PKGS=false
fi

if ${DART_COMPILE_FROM_SOURCE}; then
    DART_FROM_PKGS=false
fi

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
SDFORMAT_NO_IGN_DEPENDENCIES="${pythonv}     \\
                              libxml2-utils  \\
                              libtinyxml-dev"

# SDFORMAT 10 and above use tinyxml2
if [[ ${SDFORMAT_MAJOR_VERSION} -ge 10 ]]; then
  SDFORMAT_NO_IGN_DEPENDENCIES="${SDFORMAT_NO_IGN_DEPENDENCIES} \\
                                libtinyxml2-dev"
fi

if [[ ${SDFORMAT_MAJOR_VERSION} -lt 8 ]]; then
SDFORMAT_NO_IGN_DEPENDENCIES="${SDFORMAT_NO_IGN_DEPENDENCIES} \\
                              libboost-system-dev          \\
                              libboost-filesystem-dev      \\
                              libboost-program-options-dev \\
                              libboost-regex-dev           \\
                              libboost-iostreams-dev"
fi

SDFORMAT_NO_IGN_DEPENDENCIES="${SDFORMAT_NO_IGN_DEPENDENCIES} \\
                            ruby-dev                          \\
                            ruby"

# SDFORMAT related dependencies
if [[ -z ${SDFORMAT_MAJOR_VERSION} ]]; then
    SDFORMAT_MAJOR_VERSION=6
fi

if [[ ${SDFORMAT_MAJOR_VERSION} -ge 9 ]]; then
    # sdformat9 requires ignition-math6 and
    # uses ignition-tools for a test
    SDFORMAT_BASE_DEPENDENCIES="${SDFORMAT_NO_IGN_DEPENDENCIES} \\
                                libignition-math6-dev           \\
                                libignition-tools-dev"
elif [[ ${SDFORMAT_MAJOR_VERSION} -eq 6 ]]; then
    # sdformat6 requires ignition-math4 and
    # uses ignition-tools for a test
    SDFORMAT_BASE_DEPENDENCIES="${SDFORMAT_NO_IGN_DEPENDENCIES} \\
                                libignition-math4-dev           \\
                                libignition-tools-dev"
else
    SDFORMAT_BASE_DEPENDENCIES="${SDFORMAT_NO_IGN_DEPENDENCIES}"
fi

# GAZEBO related dependencies. Default value points to the development version
# of gazebo, it is being used by compile from source tutorial
if [[ -z ${GAZEBO_MAJOR_VERSION} ]]; then
    GAZEBO_MAJOR_VERSION=11
fi

# Need to explicit define to use old sdformat package
if [[ -z ${USE_OLD_SDFORMAT} ]]; then
    USE_OLD_SDFORMAT=false
fi

if ${USE_OLD_SDFORMAT}; then
    sdformat_pkg="sdformat"
elif [[ ${GAZEBO_MAJOR_VERSION} -ge 11 ]]; then
    sdformat_pkg="libsdformat9-dev"
elif [[ ${GAZEBO_MAJOR_VERSION} -ge 9 ]]; then
    sdformat_pkg="libsdformat6-dev"
fi

ogre_pkg="libogre-1.9-dev"
if [[ ${DISTRO} != 'xenial' ]]; then
  ogre_pkg="libogre-1.9-dev libogre-2.1-dev"
fi

# Starting from utopic, we are using the bullet provided by ubuntu
bullet_pkg="libbullet-dev"

# choose dart version
if $DART_FROM_PKGS; then
  dart_pkgs="libdart-utils-urdf-dev \
             libdart-external-odelcpsolver-dev \
             libdart-external-ikfast-dev"
  if [[ ${GAZEBO_MAJOR_VERSION} -ge 11 ]]; then
    dart_pkgs="${dart_pkgs} libdart-collision-bullet-dev \
               libdart-collision-ode-dev"
  fi
  # If on Bionic/Xenial use custom dart packages, these have a different name
  # dart -> dart6
  if [[ ${DISTRO} == 'bionic' ]] || [[ ${DISTRO} == 'xenial' ]]; then
    dart_pkgs="${dart_pkgs//dart/dart6}"
  fi
fi

# By default, the stable version of gazebo
[[ -z ${GAZEBO_EXPERIMENTAL_BUILD} ]] && GAZEBO_EXPERIMENTAL_BUILD=false
if ! ${GAZEBO_EXPERIMENTAL_BUILD}; then
  # --------------------------------------
  # GAZEBO - current version
  # --------------------------------------

  GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT="libfreeimage-dev     \\
                            libprotoc-dev                    \\
                            libprotobuf-dev                  \\
                            protobuf-compiler                \\
                            freeglut3-dev                    \\
                            libcurl4-openssl-dev             \\
                            libtinyxml-dev                   \\
                            libtinyxml2-dev                  \\
                            libtar-dev                       \\
                            libtbb-dev                       \\
                            ${ogre_pkg}                      \\
                            libxml2-dev                      \\
                            pkg-config                       \\
                            qtbase5-dev                      \\
                            libqwt-qt5-dev                   \\
                            libltdl-dev                      \\
                            libgts-dev                       \\
                            libboost-thread-dev              \\
                            libboost-system-dev              \\
                            libboost-filesystem-dev          \\
                            libboost-program-options-dev     \\
                            libboost-regex-dev               \\
                            libboost-iostreams-dev           \\
                            ${bullet_pkg}                    \\
                            libsimbody-dev                   \\
                            ${dart_pkgs}"

  if [[ ${GAZEBO_MAJOR_VERSION} -ge 11 ]]; then
      GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT="${GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT} \\
                                           libignition-common3-dev \\
                                           libignition-fuel-tools4-dev \\
                                           libignition-transport8-dev \\
                                           libignition-math6-dev \\
                                           libignition-msgs5-dev"
  elif [[ ${GAZEBO_MAJOR_VERSION} -ge 9 ]]; then
      GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT="${GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT} \\
                                           libignition-common-dev \\
                                           libignition-fuel-tools1-dev \\
                                           libignition-transport4-dev \\
                                           libignition-math4-dev \\
                                           libignition-msgs-dev"
  fi

  GAZEBO_BASE_DEPENDENCIES="${GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT} \\
                            ${sdformat_pkg}"

  GAZEBO_EXTRA_DEPENDENCIES="libavformat-dev  \\
                             libavcodec-dev   \\
                             libgdal-dev      \\
                             libgraphviz-dev  \\
                             libswscale-dev   \\
                             libavdevice-dev   \\
                             ruby-ronn"
else
  # --------------------------------------
  # GAZEBO - experimental version
  # --------------------------------------
  GAZEBO_BASE_DEPENDENCIES="libgflags-dev            \\
                            pkg-config               \\
                            libprotoc-dev            \\
                            libprotobuf-dev          \\
                            protobuf-compiler        \\
                            ${pythonv}-protobuf      \\
                            libignition-common-dev   \\
                            libignition-msgs-dev"
fi


if [[ -z $ROS_DISTRO ]]; then
  echo "------------------------------------------------------------"
  echo "ROS_DISTRO was not set before using dependencies_archive.sh!"
  echo "skipping ROS related variables"
  echo "------------------------------------------------------------"
else
  # default versions for every ROS distribution
  if [[ -z ${GAZEBO_VERSION_FOR_ROS} ]]; then
    case ${ROS_DISTRO} in
      melodic)
        GAZEBO_VERSION_FOR_ROS="9"
      ;;
      noetic)
        GAZEBO_VERSION_FOR_ROS="11"
      ;;
      # ROS 2
      foxy)
        GAZEBO_VERSION_FOR_ROS="11"
      ;;
      galactic)
        GAZEBO_VERSION_FOR_ROS="11"
      ;;
      rolling)
        GAZEBO_VERSION_FOR_ROS="11"
    esac
  fi

  case ${GAZEBO_VERSION_FOR_ROS} in
     *)
      # both packages see  http://answers.ros.org/question/217970
      _GZ_ROS_PACKAGES="libgazebo${GAZEBO_VERSION_FOR_ROS}-dev \\
                       gazebo${GAZEBO_VERSION_FOR_ROS}"
    ;;
  esac

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

  #
  # ROS_GAZEBO_PKGS DEPENDECIES
  #
  ROS_GAZEBO_PKGS_COMMON_DEPS="${ROS_CATKIN_BASE}                \\
                               libtinyxml-dev                    \\
                               ros-${ROS_DISTRO}-std-msgs        \\
                               ros-${ROS_DISTRO}-trajectory-msgs \\
                               ros-${ROS_DISTRO}-sensor-msgs     \\
                               ros-${ROS_DISTRO}-geometry-msgs"

  if ${ROS2}; then
    # Most of the base deps (ament, lint, rclcpp) are already included in
    # ROS_CATKIN_BASE. TODO: change var name
    ROS_GAZEBO_PKGS_DEPENDENCIES="${ROS_GAZEBO_PKGS_COMMON_DEPS}              \\
                                  ros-${ROS_DISTRO}-builtin-interfaces        \\
                                  ros-${ROS_DISTRO}-rosidl-default-runtime    \\
                                  ros-${ROS_DISTRO}-rosidl-default-generators"
  else
    #
    # ROS1
    #
    ROS_GAZEBO_PKGS_DEPENDENCIES="${ROS_GAZEBO_PKGS_COMMON_DEPS}            \\
                                  ros-${ROS_DISTRO}-ros                     \\
                                  ros-${ROS_DISTRO}-catkin                  \\
                                  ros-${ROS_DISTRO}-pluginlib               \\
                                  ros-${ROS_DISTRO}-roscpp                  \\
                                  ros-${ROS_DISTRO}-angles                  \\
                                  ros-${ROS_DISTRO}-camera-info-manager     \\
                                  ros-${ROS_DISTRO}-cmake-modules           \\
                                  ros-${ROS_DISTRO}-controller-manager      \\
                                  ros-${ROS_DISTRO}-control-toolbox         \\
                                  ros-${ROS_DISTRO}-tf                      \\
                                  ros-${ROS_DISTRO}-cv-bridge               \\
                                  ros-${ROS_DISTRO}-diagnostic-updater      \\
                                  ros-${ROS_DISTRO}-dynamic-reconfigure     \\
                                  ros-${ROS_DISTRO}-image-transport         \\
                                  ros-${ROS_DISTRO}-joint-limits-interface  \\
                                  ros-${ROS_DISTRO}-message-generation      \\
                                  ros-${ROS_DISTRO}-nav-msgs                \\
                                  ros-${ROS_DISTRO}-nodelet                 \\
                                  ros-${ROS_DISTRO}-pcl-conversions         \\
                                  ros-${ROS_DISTRO}-polled-camera           \\
                                  ros-${ROS_DISTRO}-rosconsole              \\
                                  ros-${ROS_DISTRO}-rosgraph-msgs           \\
                                  ros-${ROS_DISTRO}-std-srvs                \\
                                  ros-${ROS_DISTRO}-tf                      \\
                                  ros-${ROS_DISTRO}-tf2-ros                 \\
                                  ros-${ROS_DISTRO}-transmission-interface  \\
                                  ros-${ROS_DISTRO}-urdf                    \\
                                  ros-${ROS_DISTRO}-xacro"

    ROS_GAZEBO_PKGS_EXAMPLE_DEPS="ros-${ROS_DISTRO}-xacro \\
                                 ${ROS_GAZEBO_PKGS_EXAMPLE_DEPS}"
  fi

  ROS_GAZEBO_PKGS_DEPENDENCIES="${ROS_GAZEBO_PKGS_DEPENDENCIES} \\
                                ${_GZ_ROS_PACKAGES}"

  if [[ -n ${USE_DEFAULT_GAZEBO_VERSION_FOR_ROS} ]] && ${USE_DEFAULT_GAZEBO_VERSION_FOR_ROS}; then
    ROS_GAZEBO_PKGS="ros-${ROS_DISTRO}-gazebo-msgs \
                     ros-${ROS_DISTRO}-gazebo-plugins \
                     ros-${ROS_DISTRO}-gazebo-ros \
                     ros-${ROS_DISTRO}-gazebo-ros-pkgs"
    # no ros-control in ROS2 yet
    if ! ${ROS2}; then
        # TODO(sloretz) Install gazebo-ros-control when it's ported to ROS 2
        ROS_GAZEBO_PKGS="${ROS_GAZEBO_PKGS} \
                     ros-${ROS_DISTRO}-gazebo-ros-control"
    fi
  else
    ROS_GAZEBO_PKGS="ros-${ROS_DISTRO}-gazebo${GAZEBO_VERSION_FOR_ROS}-msgs \
                     ros-${ROS_DISTRO}-gazebo${GAZEBO_VERSION_FOR_ROS}-plugins \
                     ros-${ROS_DISTRO}-gazebo${GAZEBO_VERSION_FOR_ROS}-ros \
                     ros-${ROS_DISTRO}-gazebo${GAZEBO_VERSION_FOR_ROS}-ros-pkgs"
    # no ros-control in ROS2 yet
    if ! ${ROS2}; then
        # TODO(sloretz) Install gazebo-ros-control when it's ported to ROS 2
        ROS_GAZEBO_PKGS="${ROS_GAZEBO_PKGS} \
                     ros-${ROS_DISTRO}-gazebo${GAZEBO_VERSION_FOR_ROS}-ros-control"
    fi
  fi
fi

#
# DART dependencies
#
DART_DEPENDENCIES="libflann-dev            \\
                   libgtest-dev            \\
                   libeigen3-dev           \\
                   libassimp-dev           \\
                   freeglut3-dev           \\
                   libxi-dev               \\
                   libxmu-dev              \\
                   libtinyxml-dev          \\
                   libtinyxml2-dev         \\
                   libfcl-dev              \\
                   liburdfdom-dev          \\
                   libboost-system-dev     \\
                   libboost-filesystem-dev"

if ${DART_COMPILE_FROM_SOURCE}; then
    GAZEBO_EXTRA_DEPENDENCIES="$GAZEBO_EXTRA_DEPENDENCIES \\
                               $DART_DEPENDENCIES"
fi

#
# IGNITION
#

IGN_MATH_DEPENDENCIES="libeigen3-dev \\
                       libpython3-dev \\
                       ruby-dev \\
                       swig \\
                       libignition-cmake-dev \\
                       libignition-cmake2-dev"

# IGN_TRANSPORT related dependencies. Default value points to the development
# version
if [[ -z ${IGN_TRANSPORT_MAJOR_VERSION} ]]; then
    IGN_TRANSPORT_MAJOR_VERSION=5
fi

IGN_TRANSPORT_NO_IGN_DEPENDENCIES="pkg-config           \\
                                   ${pythonv}           \\
                                   ruby-ronn            \\
                                   libprotoc-dev        \\
                                   libprotobuf-dev      \\
                                   protobuf-compiler    \\
                                   uuid-dev             \\
                                   libzmq3-dev          \\
                                   libczmq-dev"

if [[ ${IGN_TRANSPORT_MAJOR_VERSION} -eq 4 ]]; then
    export IGN_TRANSPORT_DEPENDENCIES="${IGN_TRANSPORT_NO_IGN_DEPENDENCIES} \\
                                libignition-cmake-dev \\
                                libignition-msgs-dev"
elif [[ ${IGN_TRANSPORT_MAJOR_VERSION} -ge 8 ]]; then
    export IGN_TRANSPORT_DEPENDENCIES="${IGN_TRANSPORT_NO_IGN_DEPENDENCIES} \\
                                  libignition-cmake2-dev \\
                                  libignition-msgs5-dev \\
                                  libsqlite3-dev \\
                                  ruby-ffi"
fi

export IGN_TRANSPORT_DEPENDENCIES="${IGN_TRANSPORT_DEPENDENCIES} libignition-tools-dev"

IGN_COMMON_NO_IGN_DEPENDENCIES="pkg-config     \\
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

IGN_COMMON_DEPENDENCIES="${IGN_COMMON_NO_IGN_DEPENDENCIES} \\
                     libignition-cmake-dev \\
                     libignition-cmake2-dev \\
                     libignition-math4-dev \\
                     libignition-math6-dev"

IGN_FUEL_TOOLS_DEPENDENCIES="libignition-cmake2-dev \\
                             libignition-common3-dev \\
                             libignition-tools-dev  \\
                             libcurl4-openssl-dev   \\
                             libjsoncpp-dev         \\
                             libyaml-dev            \\
                             libzip-dev"

if [[ ${IGN_FUEL_TOOLS_MAJOR_VERSION} -eq 1 ]]; then
  IGN_FUEL_TOOLS_DEPENDENCIES="${IGN_FUEL_TOOLS_DEPENDENCIES} \\
                               libignition-cmake-dev  \\
                               libignition-common-dev"
else
  IGN_FUEL_TOOLS_DEPENDENCIES="${IGN_FUEL_TOOLS_DEPENDENCIES} \\
                               libignition-cmake2-dev  \\
                               libignition-common3-dev \\
                               libtinyxml2-dev"
fi

if [[ ${IGN_FUEL_TOOLS_MAJOR_VERSION} -ge 4 ]]; then
  IGN_FUEL_TOOLS_DEPENDENCIES="${IGN_FUEL_TOOLS_DEPENDENCIES} \\
                               libignition-msgs5-dev"
fi


IGN_MSGS_DEPENDENCIES="libignition-tools-dev \\
                       libprotobuf-dev       \\
                       libprotoc-dev         \\
                       protobuf-compiler     \\
                       libtinyxml2-dev       \\
                       ruby                  \\
                       ruby-dev"

if [[ -n ${IGN_MSGS_MAJOR_VERSION} && ${IGN_MSGS_MAJOR_VERSION} -eq 1 ]]; then
    IGN_MSGS_DEPENDENCIES="${IGN_MSGS_DEPENDENCIES} \\
                           libignition-cmake-dev \\
                           libignition-math4-dev"
elif [[ -n ${IGN_MSGS_MAJOR_VERSION} && ${IGN_MSGS_MAJOR_VERSION} -ge 5 ]]; then
    IGN_MSGS_DEPENDENCIES="${IGN_MSGS_DEPENDENCIES} \\
                           libignition-cmake2-dev \\
                           libignition-math6-dev"
fi

IGN_GUI_NO_IGN_DEPENDENCIES="qtbase5-dev \\
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

IGN_GUI_DEPENDENCIES="${IGN_GUI_NO_IGN_DEPENDENCIES} \\
                      libignition-cmake2-dev \\
                      libignition-common3-dev \\
                      libignition-math6-dev \\
                      libignition-plugin-dev \\
                      libignition-tools-dev"

if [[ -n "${IGN_GUI_MAJOR_VERSION}" && ${IGN_GUI_MAJOR_VERSION} -ge 4 ]]; then
  IGN_GUI_DEPENDENCIES="${IGN_GUI_DEPENDENCIES} \\
                        libignition-common3-dev"
elif [[ -n "${IGN_GUI_MAJOR_VERSION}" && ${IGN_GUI_MAJOR_VERSION} -eq 3 ]]; then
  IGN_GUI_DEPENDENCIES="${IGN_GUI_DEPENDENCIES} \\
                        libignition-common3-dev \\
                        libignition-msgs5-dev \\
                        libignition-rendering3-dev \\
                        libignition-transport8-dev"
fi

IGN_PHYSICS_DART_FROM_PKGS="true"

IGN_PLUGIN_DEPENDENCIES="libignition-cmake2-dev"

IGN_LAUNCH_COMMON_DEPENDENCIES="libignition-cmake2-dev \\
                         libignition-common3-dev \\
                         libignition-math6-dev \\
                         libignition-plugin-dev \\
                         libignition-tools-dev \\
                         libtinyxml2-dev  \\
                         qtquickcontrols2-5-dev \\
                         libqt5core5a"

if [[ -n "${IGN_LAUNCH_MAJOR_VERSION}" && ${IGN_LAUNCH_MAJOR_VERSION} -ge 3 ]]; then
  IGN_LAUNCH_DEPENDENCIES="${IGN_LAUNCH_COMMON_DEPENDENCIES} \\
                           libignition-gui4-dev \\
                           libignition-fuel-tools5-dev \\
                           libignition-gazebo4-dev \\
                           libignition-msgs6-dev \\
                           libignition-rendering4-dev  \\
                           libignition-sensors4-dev  \\
                           libignition-transport9-dev \\
                           libsdformat10-dev"
else
  IGN_LAUNCH_DEPENDENCIES="${IGN_LAUNCH_DEPENDENCIES} \\
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

IGN_RENDERING_NO_IGN_DEPENDENCIES="${ogre_pkg}\\
                            freeglut3-dev \\
                            libfreeimage-dev \\
                            libglew-dev \\
                            libogre-1.9-dev \\
                            libx11-dev \\
                            mesa-common-dev \\
                            mesa-utils"

IGN_RENDERING_DEPENDENCIES="${IGN_RENDERING_NO_IGN_DEPENDENCIES} \\
                            libignition-cmake2-dev \\
                            libignition-common3-dev \\
                            libignition-plugin-dev \\
                            libignition-math6-dev"

IGN_SENSORS_DEPENDENCIES="libignition-common3-dev     \\
                          libignition-cmake2-dev \\
                          libignition-math6-dev      \\
                          libignition-plugin-dev  \\
                          libignition-tools-dev"
if [[ -n "${IGN_SENSORS_MAJOR_VERSION}" && ${IGN_SENSORS_MAJOR_VERSION} -ge 4 ]]; then
  IGN_SENSORS_DEPENDENCIES="${IGN_SENSORS_DEPENDENCIES} \\
                        libignition-msgs6-dev \\
                        libignition-rendering4-dev \\
                        libignition-transport9-dev \\
                        libsdformat10-dev"
elif [[ -n "${IGN_SENSORS_MAJOR_VERSION}" && ${IGN_SENSORS_MAJOR_VERSION} -ge 3 ]]; then
  IGN_SENSORS_DEPENDENCIES="${IGN_SENSORS_DEPENDENCIES} \\
                        libignition-msgs5-dev \\
                        libignition-rendering3-dev \\
                        libignition-transport8-dev \\
                        libsdformat9-dev"
fi

IGN_GAZEBO_DEPENDENCIES="libignition-common3-dev  \\
                         libignition-cmake2-dev   \\
                         libgflags-dev            \\
                         libignition-math6-dev    \\
                         libignition-math6-eigen3-dev \\
                         libbenchmark-dev"

if [[ -n "${IGN_GAZEBO_MAJOR_VERSION}" && ${IGN_GAZEBO_MAJOR_VERSION} -ge 4 ]]; then
  IGN_GAZEBO_DEPENDENCIES="${IGN_GAZEBO_DEPENDENCIES} \\
                        libsdformat10-dev"
elif [[ -n "${IGN_GAZEBO_MAJOR_VERSION}" && ${IGN_GAZEBO_MAJOR_VERSION} -eq 3 ]]; then
  IGN_GAZEBO_DEPENDENCIES="${IGN_GAZEBO_DEPENDENCIES} \\
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

IGN_UTILS_DEPENDENCIES="libignition-cmake2-dev"
