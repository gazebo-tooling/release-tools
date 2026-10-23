#!/bin/bash -xe
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
export DOCKER_JOB_NAME="source_generation_job"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
. ${SCRIPT_DIR}/lib/_common_scripts.bash
echo '# END SECTION'

cat > build.sh << DELIM
$(generate_buildsh_header)

PKG_DIR=\$WORKSPACE/pkgs
SOURCES_DIR=\$WORKSPACE/sources
BUILD_DIR=\$SOURCES_DIR/build

cd \${WORKSPACE}
rm -fr \$SOURCES_DIR && mkdir \$SOURCES_DIR
git clone --depth 1 --branch ${SOURCE_REPO_REF} ${SOURCE_REPO_URI} \${SOURCES_DIR}
export SOURCE_DATE_EPOCH=\$(git -C \$SOURCES_DIR log -1 --pretty=%ct)
# assert that SOURCE_DATE_EPOCH is in the right format
case "\$SOURCE_DATE_EPOCH" in
  ''|*[!0-9]*)
    echo "ERROR: SOURCE_DATE_EPOCH='\$SOURCE_DATE_EPOCH' is not a positive integer" >&2
    exit 1
    ;;
esac

# Up to gz-cmake5 (Jetty), gz-cmake packages are side-by-side installable.
# For newer releases, gz-cmake packages conflict with each other, so we install
# the version specified in CMakeLists.txt, or for unversioned find_package(gz-cmake),
# use libgz-cmake-dev (rotary) if available and otherwise the latest versioned package.
if grep -qE 'find_package\((gz|ignition)-cmake[[:space:]]*[1-5]\b' "\${SOURCES_DIR}/CMakeLists.txt"; then
  (sudo apt-get install -y 'libgz-cmake[1-5]-dev') || \
    (echo "Can not find any ign-cmake/gz-cmake package" && exit 1)
else
  # Matches e.g. "find_package(gz-cmake6 REQUIRED)" or "find_package(gz-cmake 6 REQUIRED)"
  # and outputs "libgz-cmake6-dev". For Rotary, CMakeLists.txt uses unversioned
  # "find_package(gz-cmake REQUIRED)", which does not match and leaves GZ_CMAKE_PKG
  # empty so the fallback below selects "libgz-cmake-dev" (or the latest versioned
  # package if building a prerelease without the nightly repo).
  GZ_CMAKE_PKG=\$(sed -nE 's/^[[:space:]]*find_package\(gz-cmake[[:space:]]*([0-9]+).*/libgz-cmake\1-dev/p' "\${SOURCES_DIR}/CMakeLists.txt" | head -n 1)
  if [ -z "\$GZ_CMAKE_PKG" ]; then
    if apt-cache show libgz-cmake-dev > /dev/null 2>&1; then
      GZ_CMAKE_PKG="libgz-cmake-dev"
    else
      GZ_CMAKE_PKG=\$(apt-cache search --names-only '^libgz-cmake[0-9]+-dev$' | awk '{print \$1}' | sort -V | tail -n 1)
    fi
  fi
  sudo apt-get install -y "\${GZ_CMAKE_PKG}" || \
    (echo "Failed to install \${GZ_CMAKE_PKG}" && exit 1)
fi
rm -fr \$BUILD_DIR && mkdir \$BUILD_DIR
cd \${BUILD_DIR}
cmake .. -DPACKAGE_SOURCE_ONLY:BOOL=ON
make package_source

rm -fr \$PKG_DIR && mkdir \$PKG_DIR
find \${BUILD_DIR} -maxdepth 1 -name '*-${VERSION}.tar.*' -exec mv {} \${PKG_DIR} \\;

if [ \$(ls 2>/dev/null -Ubad1 -- "\${PKG_DIR}" | wc -l) -gt 1 ]; then
  echo "Found more than one file inside pkgs directory:"
  ls \${PKG_DIR}
  exit 1
fi
DELIM

export DEPENDENCY_PKGS="cmake git"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
