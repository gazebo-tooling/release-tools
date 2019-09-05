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
                   python-lxml     \\
                   python-psutil   \\
                   python          \\
                   bc              \\
                   netcat-openbsd  \\
                   gnupg2          \\
                   net-tools       \\
                   locales"

BREW_BASE_DEPENDCIES="mercurial git cmake"

# 1. SDFORMAT
# ruby for xml_schemas generation and libxml2-utils for xmllint used in tests
SDFORMAT_NO_IGN_DEPENDENCIES="python         \\
                              libxml2-utils  \\
                              libtinyxml-dev"

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

if [[ ${SDFORMAT_MAJOR_VERSION} -ge 8 ]]; then
    # sdformat8 requires ignition-math6 and
    # uses ignition-tools for a test
    SDFORMAT_BASE_DEPENDENCIES="${SDFORMAT_NO_IGN_DEPENDENCIES} \\
                                libignition-math6-dev           \\
                                libignition-tools-dev"
elif [[ ${SDFORMAT_MAJOR_VERSION} -ge 7 ]]; then
    # sdformat6 requires ignition-math5 and
    # uses ignition-tools for a test
    SDFORMAT_BASE_DEPENDENCIES="${SDFORMAT_NO_IGN_DEPENDENCIES} \\
                                libignition-math5-dev           \\
                                libignition-tools-dev"
elif [[ ${SDFORMAT_MAJOR_VERSION} -ge 6 ]]; then
    # sdformat6 requires ignition-math4 and
    # uses ignition-tools for a test
    SDFORMAT_BASE_DEPENDENCIES="${SDFORMAT_NO_IGN_DEPENDENCIES} \\
                                libignition-math4-dev           \\
                                libignition-tools-dev"
elif [[ ${SDFORMAT_MAJOR_VERSION} -ge 5 ]]; then
    # sdformat5 requires ignition-math3
    SDFORMAT_BASE_DEPENDENCIES="${SDFORMAT_NO_IGN_DEPENDENCIES} \\
                                libignition-math3-dev"
elif [[ ${SDFORMAT_MAJOR_VERSION} -ge 3 ]]; then
    # sdformat3 requires ignition-math2
    SDFORMAT_BASE_DEPENDENCIES="${SDFORMAT_NO_IGN_DEPENDENCIES} \\
                                libignition-math2-dev"
else
    SDFORMAT_BASE_DEPENDENCIES="${SDFORMAT_NO_IGN_DEPENDENCIES}"
fi

# GAZEBO related dependencies. Default value points to the development version
# of gazebo, it is being used by compile from source tutorial
if [[ -z ${GAZEBO_MAJOR_VERSION} ]]; then
    GAZEBO_MAJOR_VERSION=8
fi

# Need to explicit define to use old sdformat package
if [[ -z ${USE_OLD_SDFORMAT} ]]; then
    USE_OLD_SDFORMAT=false
fi

if ${USE_OLD_SDFORMAT}; then
    sdformat_pkg="sdformat"
elif [[ ${GAZEBO_MAJOR_VERSION} -ge 9 ]]; then
    sdformat_pkg="libsdformat6-dev"
elif [[ ${GAZEBO_MAJOR_VERSION} -ge 8 ]]; then
    sdformat_pkg="libsdformat5-dev"
elif [[ ${GAZEBO_MAJOR_VERSION} -ge 7 ]]; then
    sdformat_pkg="libsdformat4-dev"
elif [[ ${GAZEBO_MAJOR_VERSION} -ge 6 ]]; then
    sdformat_pkg="libsdformat3-dev"
else
    sdformat_pkg="libsdformat2-dev"
fi

# Old versions used libogre-dev
ogre_pkg="libogre-1.9-dev"
if [[ ${GAZEBO_MAJOR_VERSION} -le 4 ]]; then
    # Before gazebo5, ogre 1.9 was not supported
    ogre_pkg="libogre-1.8-dev"
elif [[ ${IGN_RENDERING_MAJOR_VERSION} -ge 1 ]]; then
    # support for both ogre-1.9 and ogre-2.1 was added in ign-rendering1
    ogre_pkg="libogre-1.9-dev libogre-2.1-dev"
fi

# Starting from utopic, we are using the bullet provided by ubuntu
bullet_pkg="libbullet-dev"

# choose dart version
if $DART_FROM_PKGS; then
    if [[ ${GAZEBO_MAJOR_VERSION} -ge 9 ]]; then
       dart_pkg="libdart6-utils-urdf-dev"
    elif [[ ${GAZEBO_MAJOR_VERSION} -ge 8 ]]; then
       dart_pkg="libdart-core5-dev"
    fi
fi

# gazebo8 and above use qt5
if [[ ${GAZEBO_MAJOR_VERSION} -le 7 ]]; then
  gazebo_qt_dependencies="libqt4-dev \\
                          libqtwebkit-dev"
else
  # After gazebo8 is released, these two lines should be all that remain
  gazebo_qt_dependencies="qtbase5-dev \\
                          libqwt-qt5-dev"
  # Install qt4 as well for gazebo8 until its release
  # 20170125 release date of gazebo8
  if [[ $(date +%Y%m%d) -le 20170125 ]]; then
    gazebo_qt_dependencies="${gazebo_qt_dependencies} \\
                            libqt4-dev \\
                            libqwt-dev"
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
                            libtar-dev                       \\
                            libtbb-dev                       \\
                            ${ogre_pkg}                      \\
                            libxml2-dev                      \\
                            pkg-config                       \\
                            ${gazebo_qt_dependencies}        \\
                            libltdl-dev                      \\
                            libgts-dev                       \\
                            libboost-thread-dev              \\
                            libboost-signals-dev             \\
                            libboost-system-dev              \\
                            libboost-filesystem-dev          \\
                            libboost-program-options-dev     \\
                            libboost-regex-dev               \\
                            libboost-iostreams-dev           \\
                            ${bullet_pkg}                    \\
                            libsimbody-dev                   \\
                            ${dart_pkg}"

  if [[ ${GAZEBO_MAJOR_VERSION} -eq 6 ]]; then
      GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT="${GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT} \\
                                           libignition-math2-dev"
  fi

  if [[ ${GAZEBO_MAJOR_VERSION} -eq 7 ]]; then
      GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT="${GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT} \\
                                libignition-transport-dev"
  fi

  if [[ ${GAZEBO_MAJOR_VERSION} -eq 8 ]]; then
      GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT="${GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT} \\
                                           libignition-transport3-dev \\
                                           libignition-math3-dev \\
                                           libignition-msgs0-dev"
  fi

  if [[ ${GAZEBO_MAJOR_VERSION} -ge 9 ]]; then
      GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT="${GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT} \\
                                           libignition-common-dev \\
                                           libignition-fuel-tools-dev \\
                                           libignition-transport4-dev \\
                                           libignition-math4-dev \\
                                           libignition-msgs-dev"
  fi

  # libtinyxml2-dev is not on precise
  # it is needed by gazebo7, which isn't supported on precise
  if [[ ${DISTRO} != 'precise' ]]; then
      GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT="${GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT} \\
                                libtinyxml2-dev"
  fi

  GAZEBO_BASE_DEPENDENCIES="${GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT} \\
                            ${sdformat_pkg}"

  GAZEBO_EXTRA_DEPENDENCIES="libavformat-dev  \\
                             libavcodec-dev   \\
                             libgraphviz-dev  \\
                             libswscale-dev   \\
                             libavdevice-dev   \\
                             ruby-ronn"

  # cegui is deprecated in gazebo 6
  if [[ ${GAZEBO_MAJOR_VERSION} -le 6 ]]; then
      GAZEBO_EXTRA_DEPENDENCIES="${GAZEBO_EXTRA_DEPENDENCIES} \\
                                 libcegui-mk2-dev"
  fi

  # gdal is not working on precise
  # it was added in gazebo5, which does not support precise
  if [[ ${DISTRO} != 'precise' ]]; then
      GAZEBO_EXTRA_DEPENDENCIES="${GAZEBO_EXTRA_DEPENDENCIES} \\
                                 libgdal-dev"
  fi
else
  # --------------------------------------
  # GAZEBO - experimental version
  # --------------------------------------
  GAZEBO_BASE_DEPENDENCIES="libgflags-dev            \\
                            pkg-config               \\
                            libprotoc-dev            \\
                            libprotobuf-dev          \\
                            protobuf-compiler        \\
                            python-protobuf          \\
                            libignition-common-dev   \\
                            libignition-msgs-dev     \\
                            libignition-transport3-dev"
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
      kinetic)
        GAZEBO_VERSION_FOR_ROS="7"
      ;;
      melodic)
        GAZEBO_VERSION_FOR_ROS="9"
      ;;
      # ROS 2
      crystal)
        GAZEBO_VERSION_FOR_ROS="9"
      ;;
      dashing)
        GAZEBO_VERSION_FOR_ROS="9"
    esac
  fi

  case ${GAZEBO_VERSION_FOR_ROS} in
    "2")
      _GZ_ROS_PACKAGES="gazebo2"
    ;;
     *)
      # both packages see  http://answers.ros.org/question/217970
      _GZ_ROS_PACKAGES="libgazebo${GAZEBO_VERSION_FOR_ROS}-dev \\
                       gazebo${GAZEBO_VERSION_FOR_ROS}"
    ;;
  esac

  # TODO rename the variable
  if ${ROS2}; then
    ROS_CATKIN_BASE="python-dev                      \\
                    python3-colcon-common-extensions \\
                    python-rosdep                    \\
                    python-wstool                    \\
                    python-rosinstall                \\
                    python-rospkg                    \\
                    python-vcstools"
  else
    ROS_CATKIN_BASE="python-dev              \\
                    python-catkin-pkg        \\
                    python-rosdep            \\
                    python-wstool            \\
                    ros-${ROS_DISTRO}-catkin \\
                    ros-${ROS_DISTRO}-ros    \\
                    python-rosinstall        \\
                    python-catkin-tools      \\
                    python-catkin-pkg        \\
                    python-rospkg            \\
                    python-vcstools"
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

    if [[ ${ROS_DISTRO} == 'kinetic' ]]; then
       ROS_GAZEBO_PKGS_DEPENDENCIES="${ROS_GAZEBO_PKGS_DEPENDENCIES} \\
                                     ros-${ROS_DISTRO}-ros-base \\
                                     ros-${ROS_DISTRO}-pcl-ros"
    fi

    ROS_GAZEBO_PKGS_EXAMPLE_DEPS="ros-${ROS_DISTRO}-xacro \\
                                 ${ROS_GAZEBO_PKGS_EXAMPLE_DEPS}"
  fi

  ROS_GAZEBO_PKGS_DEPENDENCIES="${ROS_GAZEBO_PKGS_DEPENDENCIES} \\
                                ${_GZ_ROS_PACKAGES}"

  if [[ -n ${USE_DEFAULT_GAZEBO_VERSION_FOR_ROS} ]] && ${USE_DEFAULT_GAZEBO_VERSION_FOR_ROS}; then
    ROS_GAZEBO_PKGS="ros-${ROS_DISTRO}-gazebo-msgs \
                     ros-${ROS_DISTRO}-gazebo-plugins \
                     ros-${ROS_DISTRO}-gazebo-ros \
                     ros-${ROS_DISTRO}-gazebo-ros-pkgs \
                     ros-${ROS_DISTRO}-gazebo-ros-control"

  else
    ROS_GAZEBO_PKGS="ros-${ROS_DISTRO}-gazebo${GAZEBO_VERSION_FOR_ROS}-msgs \
                     ros-${ROS_DISTRO}-gazebo${GAZEBO_VERSION_FOR_ROS}-plugins \
                     ros-${ROS_DISTRO}-gazebo${GAZEBO_VERSION_FOR_ROS}-ros \
                     ros-${ROS_DISTRO}-gazebo${GAZEBO_VERSION_FOR_ROS}-ros-pkgs \
                     ros-${ROS_DISTRO}-gazebo${GAZEBO_VERSION_FOR_ROS}-ros-control"
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
                       libignition-cmake-dev \\
                       libignition-cmake1-dev"
if [[ ${DISTRO} != 'xenial' ]]; then
  IGN_MATH_DEPENDENCIES="${IGN_MATH_DEPENDENCIES} \\
                         libignition-cmake2-dev"
fi

# IGN_TRANSPORT related dependencies. Default value points to the development
# version
if [[ -z ${IGN_TRANSPORT_MAJOR_VERSION} ]]; then
    IGN_TRANSPORT_MAJOR_VERSION=5
fi

IGN_TRANSPORT_NO_IGN_DEPENDENCIES="pkg-config           \\
                                   python               \\
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
elif [[ ${IGN_TRANSPORT_MAJOR_VERSION} -eq 5 ]]; then
    export IGN_TRANSPORT_DEPENDENCIES="${IGN_TRANSPORT_NO_IGN_DEPENDENCIES} \\
                                libignition-cmake1-dev \\
                                libignition-msgs2-dev \\
                                libsqlite3-dev \\
                                ruby-ffi"
elif [[ ${IGN_TRANSPORT_MAJOR_VERSION} -eq 6 ]]; then
    export IGN_TRANSPORT_DEPENDENCIES="${IGN_TRANSPORT_NO_IGN_DEPENDENCIES} \\
                                  libignition-cmake2-dev \\
                                  libignition-msgs3-dev \\
                                  libsqlite3-dev \\
                                  ruby-ffi"
elif [[ ${IGN_TRANSPORT_MAJOR_VERSION} -ge 7 ]]; then
    export IGN_TRANSPORT_DEPENDENCIES="${IGN_TRANSPORT_NO_IGN_DEPENDENCIES} \\
                                  libignition-cmake2-dev \\
                                  libignition-msgs4-dev \\
                                  libsqlite3-dev \\
                                  ruby-ffi"
else
    export IGN_TRANSPORT_DEPENDENCIES="${IGN_TRANSPORT_NO_IGN_DEPENDENCIES} \\
                                libignition-msgs0-dev"
fi

export IGN_TRANSPORT_DEPENDENCIES="${IGN_TRANSPORT_DEPENDENCIES} libignition-tools-dev"

IGN_COMMON_NO_IGN_DEPENDENCIES="pkg-config            \\
                         python                \\
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
                     libignition-cmake1-dev \\
                     libignition-math4-dev \\
                     libignition-math5-dev"
if [[ ${DISTRO} != 'xenial' ]]; then
  IGN_COMMON_DEPENDENCIES="${IGN_COMMON_DEPENDENCIES} \\
                           libignition-cmake2-dev \\
                           libignition-math6-dev"
fi

IGN_FUEL_TOOLS_DEPENDENCIES=" libignition-tools-dev  \\
                             libcurl4-openssl-dev   \\
                             libjsoncpp-dev         \\
                             libyaml-dev            \\
                             libzip-dev"
if [[ ${DISTRO} != 'xenial' ]]; then
  IGN_FUEL_TOOLS_DEPENDENCIES="${IGN_FUEL_TOOLS_DEPENDENCIES} \\
                           libignition-cmake2-dev \\
                           libignition-common3-dev"
fi

if [[ ${IGN_FUEL_TOOLS_MAJOR_VERSION} -le 2 ]]; then
  IGN_FUEL_TOOLS_DEPENDENCIES="${IGN_FUEL_TOOLS_DEPENDENCIES} \\
                               libignition-cmake-dev  \\
                               libignition-common-dev"
else
  IGN_FUEL_TOOLS_DEPENDENCIES="${IGN_FUEL_TOOLS_DEPENDENCIES} \\
                               libignition-cmake2-dev  \\
                               libignition-common3-dev \\
                               libtinyxml2-dev"
fi

IGN_MSGS_DEPENDENCIES="libignition-tools-dev \\
                       libprotobuf-dev       \\
                       libprotoc-dev         \\
                       protobuf-compiler     \\
                       ruby                  \\
                       ruby-dev"

if [[ -n ${IGN_MSGS_MAJOR_VERSION} && ${IGN_MSGS_MAJOR_VERSION} -le 0 ]]; then
    IGN_MSGS_DEPENDENCIES="${IGN_MSGS_DEPENDENCIES} \\
                           libignition-math3-dev"
elif [[ -n ${IGN_MSGS_MAJOR_VERSION} && ${IGN_MSGS_MAJOR_VERSION} -eq 1 ]]; then
    IGN_MSGS_DEPENDENCIES="${IGN_MSGS_DEPENDENCIES} \\
                           libignition-cmake-dev \\
                           libignition-math4-dev"
elif [[ -n ${IGN_MSGS_MAJOR_VERSION} && ${IGN_MSGS_MAJOR_VERSION} -eq 2 ]]; then
    IGN_MSGS_DEPENDENCIES="${IGN_MSGS_DEPENDENCIES} \\
                           libignition-cmake1-dev \\
                           libignition-math6-dev"
elif [[ -n ${IGN_MSGS_MAJOR_VERSION} && ${IGN_MSGS_MAJOR_VERSION} -ge 3 ]]; then
    IGN_MSGS_DEPENDENCIES="${IGN_MSGS_DEPENDENCIES} \\
                           libignition-cmake2-dev \\
                           libignition-math6-dev"
fi

IGN_GUI_NO_IGN_DEPENDENCIES="qtbase5-dev \\
                      qtdeclarative5-dev \\
                      libtinyxml2-dev \\
                      libqwt-qt5-dev"

if [[ ${DISTRO} != 'xenial' ]]; then
  IGN_GUI_NO_IGN_DEPENDENCIES="${IGN_GUI_NO_IGN_DEPENDENCIES} \\
                      qml-module-qtquick2 \\
                      qml-module-qtquick-controls \\
                      qml-module-qtquick-controls2 \\
                      qml-module-qtquick-dialogs \\
                      qml-module-qtquick-layouts \\
                      qml-module-qt-labs-folderlistmodel \\
                      qml-module-qt-labs-settings \\
                      qtquickcontrols2-5-dev"
fi

IGN_GUI_DEPENDENCIES="${IGN_GUI_NO_IGN_DEPENDENCIES} \\
                      libignition-cmake1-dev \\
                      libignition-math5-dev \\
                      libignition-tools-dev \\
                      libignition-transport5-dev \\
                      libignition-msgs2-dev \\
                      libignition-common2-dev"

if [[ ${DISTRO} != 'xenial' ]]; then
  IGN_GUI_DEPENDENCIES="${IGN_GUI_DEPENDENCIES} \\
                        libignition-cmake2-dev \\
                        libignition-common3-dev \\
                        libignition-math6-dev \\
                        libignition-msgs3-dev \\
                        libignition-plugin-dev \\
                        libignition-rendering-dev \\
                        libignition-transport6-dev"
fi

if [[ -n "${IGN_GUI_MAJOR_VERSION}" && ${IGN_GUI_MAJOR_VERSION} -eq 0 ]]; then
  IGN_GUI_DEPENDENCIES="${IGN_GUI_DEPENDENCIES} \\
                        libignition-rendering2-dev"
fi

if [[ -n "${IGN_GUI_MAJOR_VERSION}" && ${IGN_GUI_MAJOR_VERSION} -ge 2 ]]; then
  IGN_GUI_DEPENDENCIES="${IGN_GUI_DEPENDENCIES} \\
                        libignition-msgs4-dev \\
                        libignition-rendering2-dev \\
                        libignition-transport7-dev"
fi

IGN_PHYSICS_DEPENDENCIES="libbenchmark-dev \\
                          dart6-data \\
                          libdart6-collision-ode-dev \\
                          libdart6-dev \\
                          libdart6-utils-urdf-dev \\
                          libignition-cmake2-dev \\
                          libignition-common3-dev \\
                          libignition-math6-dev \\
                          libignition-math6-eigen3-dev \\
                          libignition-plugin-dev \\
                          libsdformat8-dev"
IGN_PHYSICS_DART_FROM_PKGS="true"

IGN_PLUGIN_DEPENDENCIES="libignition-cmake1-dev"
if [[ ${DISTRO} != 'xenial' ]]; then
  IGN_PLUGIN_DEPENDENCIES="${IGN_PLUGIN_DEPENDENCIES} \\
                           libignition-cmake2-dev"
fi

IGN_LAUNCH_DEPENDENCIES="libignition-cmake2-dev \\
                         libignition-common3-dev \\
                         libignition-plugin-dev \\
                         libignition-tools-dev \\
                         libsdformat8-dev \\
                         libtinyxml2-dev  \\
                         qtquickcontrols2-5-dev \\
                         libqt5core5a"

if [[ -n "${IGN_LAUNCH_MAJOR_VERSION}" && ${IGN_LAUNCH_MAJOR_VERSION} -lt 1 ]]; then
  IGN_LAUNCH_DEPENDENCIES="${IGN_LAUNCH_DEPENDENCIES} \\
                          libignition-gazebo-dev \\
                          libignition-gui-dev \\
                          libignition-msgs3-dev \\
                          libignition-sensors-dev \\
                          libignition-transport6-dev"
else
  IGN_LAUNCH_DEPENDENCIES="${IGN_LAUNCH_DEPENDENCIES} \\
                           libignition-gazebo2-dev \\
                           libignition-gui2-dev \\
                           libignition-msgs4-dev \\
                           libignition-sensors2-dev  \\
                           libignition-fuel-tools3-dev \\
                           libignition-transport7-dev \\
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

if [[ -n ${IGN_RENDERING_MAJOR_VERSION} && ${IGN_RENDERING_MAJOR_VERSION} -le 0 ]]; then
  IGN_RENDERING_DEPENDENCIES="${IGN_RENDERING_NO_IGN_DEPENDENCIES} \\
                              libignition-cmake1-dev \\
                              libignition-common2-dev \\
                              libignition-math5-dev"
else
  IGN_RENDERING_DEPENDENCIES="${IGN_RENDERING_NO_IGN_DEPENDENCIES} \\
                              libignition-cmake2-dev \\
                              libignition-common3-dev \\
                              libignition-plugin-dev \\
                              libignition-math6-dev"
fi

IGN_SENSORS_DEPENDENCIES="libignition-common3-dev     \\
                          libignition-cmake2-dev \\
                          libignition-math6-dev      \\
                          libignition-msgs3-dev       \\
                          libignition-plugin-dev  \\
                          libignition-tools-dev \\
                          libignition-transport6-dev \\
                          libignition-rendering-dev \\
                          libsdformat8-dev"

if [[ -n "${IGN_SENSORS_MAJOR_VERSION}" && ${IGN_SENSORS_MAJOR_VERSION} -ge 2 ]]; then
  IGN_SENSORS_DEPENDENCIES="${IGN_SENSORS_DEPENDENCIES} \\
                        libignition-msgs4-dev \\
                        libignition-rendering2-dev \\
                        libignition-transport7-dev"
fi

IGN_GAZEBO_DEPENDENCIES="libignition-common3-dev     \\
                         libignition-cmake2-dev \\
                         libignition-fuel-tools3-dev \\
                         libignition-gui-dev \\
                         libgflags-dev \\
                         libignition-math6-dev      \\
                         libignition-math6-eigen3-dev      \\
                         libignition-msgs3-dev       \\
                         libignition-physics           \\
                         libignition-physics-dartsim   \\
                         libignition-physics-dartsim-dev \\
                         libignition-physics-dev       \\
                         libignition-plugin-dev       \\
                         libignition-rendering-dev \\
                         libignition-sensors-dev \\
                         libignition-tools-dev \\
                         libignition-transport6-dev \\
                         libsdformat8-dev"

if [[ -n "${IGN_GAZEBO_MAJOR_VERSION}" && ${IGN_GAZEBO_MAJOR_VERSION} -ge 2 ]]; then
  IGN_GAZEBO_DEPENDENCIES="${IGN_GAZEBO_DEPENDENCIES} \\
                        libignition-gui2-dev \\
                        libignition-msgs4-dev \\
                        libignition-rendering2-dev \\
                        libignition-sensors2-dev \\
                        libignition-transport7-dev \\
                        libbenchmark-dev"
fi

IGN_RNDF_DEPENDENCIES="libignition-cmake-dev \\
                       libignition-math4-dev"

#
# SUBT
#
SUBT_DEPENDENCIES="mercurial \\
                   wget \\
                   curl \\
                   git  \\
                   ${ROS_CATKIN_BASE} \\
                   ignition-blueprint  \\
                   ros-${ROS_DISTRO}-desktop \\
                   ros-${ROS_DISTRO}-tf2-sensor-msgs \\
                   ros-${ROS_DISTRO}-robot-localization \\
                   ros-${ROS_DISTRO}-rotors-control \\
                   ros-${ROS_DISTRO}-ros-control \\
                   ros-${ROS_DISTRO}-twist-mux \\
                   ros-${ROS_DISTRO}-ros1-ign-bridge"
