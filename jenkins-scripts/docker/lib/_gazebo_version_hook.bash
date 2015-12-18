# Identify GAZEBO_MAJOR_VERSION to help with dependency resolution
GAZEBO_MAJOR_VERSION=`\
  grep 'set.*GAZEBO_MAJOR_VERSION ' ${WORKSPACE}/gazebo/CMakeLists.txt | \
  tr -d 'a-zA-Z _()'`

# Check gazebo version is integer
if ! [[ ${GAZEBO_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! GAZEBO_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

NEED_PRERELEASE=false
if [[ $GAZEBO_MAJOR_VERSION -ge 7 ]]; then
  # Need prerelease repo to get sdformat4 during the development cycle
  # 20160125 release date of gazebo7
  if [[ $(date +%Y%m%d) -le 20160125 ]]; then
    NEED_PRERELEASE=true
  fi
fi
