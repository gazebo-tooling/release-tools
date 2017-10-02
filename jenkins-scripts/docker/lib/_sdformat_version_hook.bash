# Identify SDFORMAT_MAJOR_VERSION to help with dependency resolution
SDFORMAT_MAJOR_VERSION=`\
  grep 'set.*SDF_MAJOR_VERSION ' ${WORKSPACE}/sdformat/CMakeLists.txt | \
  tr -d 'a-zA-Z _()'`

# Check SDFORMAT version is integer
if ! [[ ${SDFORMAT_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! SDFORMAT_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi
