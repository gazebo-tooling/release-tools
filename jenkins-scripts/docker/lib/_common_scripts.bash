export APT_INSTALL="sudo DEBIAN_FRONTEND=noninteractive apt-get install -y"

export CALCULATE_DEFAULT_MAKE_TEST_ARGS="""
  DEFAULT_MAKE_TEST_ARGS=-VV
  # Check junit_output support (needs CMake 3.21, not in Ubuntu Focal)
  if ctest --help | grep -q -- --output-junit; then
    export DEFAULT_MAKE_TEST_ARGS+=' --output-junit cmake_junit_output.xml'
  fi
"""
