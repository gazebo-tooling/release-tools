# calculate NEEDS_GZ11_SUPPORT 
if [[ ${DEST_BRANCH} == 'gz11' ]] || [[ ${SRC_BRANCH} == 'gz11' ]]; then
  export NEEDS_GZ11_SUPPORT=true
else
  export NEEDS_GZ11_SUPPORT=true
fi

# needs gcc8 for c++17 features
export USE_GCC8=true

# define gz11 branches for using when enabled by BUILD_IGN_* variables
export IGN_CMAKE_BRANCH="gz11"
export IGN_MATH_BRANCH="gz11"
export IGN_MSGS_BRANCH="gz11"
  
# Need stable + prerelease repositories for gz11 packages
export BUILDING_JOB_REPOSITORIES="stable prerelease"
