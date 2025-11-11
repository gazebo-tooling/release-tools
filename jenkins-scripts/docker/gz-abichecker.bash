#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

if [[ -z ${ARCH} ]]; then
  echo "ARCH variable not set!"
  exit 1
fi

if [[ -z ${DISTRO} ]]; then
  echo "DISTRO variable not set!"
  exit 1
fi

# In Gazebo this variable is passed directly from the DSL job creation
if [[ -z ${ABI_JOB_SOFTWARE_NAME} ]]; then
  echo "ABI_JOB_SOFTWARE_NAME not set!"
  exit 1
fi

# convert from gz-package to GZ_PACKAGE_DEPENDENCIES
GZ_NAME_PREFIX=$(\
  echo ${ABI_JOB_SOFTWARE_NAME} | tr '[:lower:]-' '[:upper:]_')
ABI_JOB_PKG_DEPENDENCIES_VAR_NAME=${GZ_NAME_PREFIX}_DEPENDENCIES

# Identify GZ_MSGS_MAJOR_VERSION to help with dependency resolution
export GZ_NAME_PREFIX_MAJOR_VERSION=$(\
  python3 ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/${ABI_JOB_SOFTWARE_NAME}/CMakeLists.txt)
export ${GZ_NAME_PREFIX}_MAJOR_VERSION=${GZ_NAME_PREFIX_MAJOR_VERSION}

# Set the ABI_JOB_HEADER_PREFIX variable to help find the header locations.
#
# Since gz-cmake include paths are like the following:
# <prefix>/include/ignition/gazebo6/*
# <prefix>/include/ignition/math6/*
# <prefix>/include/gz/math7/*
# <prefix>/include/gz/sim7/*
#
# Set ABI_JOB_HEADER_PREFIX to match the last subfolder:
#   gazebo6, math6, math7, sim7
if [[ "${ABI_JOB_SOFTWARE_NAME}" = "gz-sim" && ${GZ_NAME_PREFIX_MAJOR_VERSION} -ge 7 ]]
then
  # special case for gz-sim7+
  export ABI_JOB_HEADER_PREFIX=sim[0-9]*
else
  # otherwise, strip the ign- or gz- prefix and append "[0-9]*"
  export ABI_JOB_HEADER_PREFIX=${ABI_JOB_SOFTWARE_NAME/[ignz]*-/}[0-9]*
fi

# check if NEED_C17_COMPILER should be set. Only needed for Bionic so only care about Citadel
# versions
if [[ "${ABI_JOB_SOFTWARE_NAME}" = "ign-gazebo" ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "ign-physics" ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "ign-sensors" ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "ign-common" ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "ign-gui" ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "ign-math" ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "ign-msgs" ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "ign-plugin" ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "ign-rendering" ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "ign-transport" ]]
then
  export NEED_C17_COMPILER=true
fi

# check if OGRE-2.2 include paths are needed
if [[ "${ABI_JOB_SOFTWARE_NAME}" = "gz-sim"      && ${GZ_NAME_PREFIX_MAJOR_VERSION} -eq 6 ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "gz-gui"       && ${GZ_NAME_PREFIX_MAJOR_VERSION} -eq 6 ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "gz-launch"    && ${GZ_NAME_PREFIX_MAJOR_VERSION} -eq 5 ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "gz-sensors"   && ${GZ_NAME_PREFIX_MAJOR_VERSION} -eq 6 ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "gz-rendering" && ${GZ_NAME_PREFIX_MAJOR_VERSION} -eq 6 ]]
then
  # OGRE 2.2 is packaged as "Ogre-Next" on jammy
  if [[ "${DISTRO}" == "jammy" ]]
  then
    export EXTRA_INCLUDES="""
   <add_include_paths>
     /usr/include/OGRE-Next/Hlms/Common
   </add_include_paths>
"""
  elif [[ "${DISTRO}" == "focal" ]]
  then
    export EXTRA_INCLUDES="""
   <add_include_paths>
     /usr/include/OGRE-2.2/Hlms/Common
   </add_include_paths>
"""
  fi
fi

if [[ "${ABI_JOB_SOFTWARE_NAME}" = "gz-sim"      && ${GZ_NAME_PREFIX_MAJOR_VERSION} -ge 6 ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "gz-gui"       && ${GZ_NAME_PREFIX_MAJOR_VERSION} -ge 7 ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "gz-launch"    && ${GZ_NAME_PREFIX_MAJOR_VERSION} -ge 6 ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "gz-sensors"   && ${GZ_NAME_PREFIX_MAJOR_VERSION} -ge 7 ]] || \
  [[ "${ABI_JOB_SOFTWARE_NAME}" = "gz-rendering" && ${GZ_NAME_PREFIX_MAJOR_VERSION} -ge 7 ]]
then
  # -fPIC needed to compile Qt
  export ABI_JOB_EXTRA_GCC_OPTIONS="-fPIC"
fi

# default to use stable repos
export ABI_JOB_REPOS="stable"

# set GZDEV_PROJECT_NAME so it can override repos if necessary
export GZDEV_PROJECT_NAME=${ABI_JOB_SOFTWARE_NAME/ign-/ignition-}${GZ_NAME_PREFIX_MAJOR_VERSION}

. ${SCRIPT_DIR}/lib/generic-abi-base.bash
