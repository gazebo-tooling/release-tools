# Identify SDFORMAT_MAJOR_VERSION to help with dependency resolution
SDFORMAT_MAJOR_VERSION=$(\
  python ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/sdformat/CMakeLists.txt)

# Check SDFORMAT version is integer
if ! [[ ${SDFORMAT_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! SDFORMAT_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi
