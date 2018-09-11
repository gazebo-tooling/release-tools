# initialize NEEDS_GZ11_SUPPORT to false if unset
export NEEDS_GZ11_SUPPORT=${NEEDS_GZ11_SUPPORT:-false}

# calculate NEEDS_GZ11_SUPPORT
if [[ ${DEST_BRANCH} == 'gz11' ]] || [[ ${SRC_BRANCH} == 'gz11' ]]; then
  export NEEDS_GZ11_SUPPORT=true
fi

if ${NEEDS_GZ11_SUPPORT}; then
  # needs gcc8 for c++17 features
  export USE_GCC8=true

  # define gz11 branches for using when enabled by BUILD_IGN_* variables
  export IGN_CMAKE_BRANCH="gz11"
  export IGN_MATH_BRANCH="gz11"
  export IGN_MSGS_BRANCH="gz11"
  export IGN_COMMON_BRANCH="gz11"
  export IGN_TRANSPORT_BRANCH="gz11"
  export IGN_RENDERING_BRANCH="gz11"
  export SDFORMAT_BRANCH="gz11"

  # Need stable + prerelease repositories for gz11 packages
  export BUILDING_JOB_REPOSITORIES="stable prerelease"
fi
