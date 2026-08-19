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

GZ_CMAKE_PKG=""
if [ -f "\${SOURCES_DIR}/.github/ci/packages.apt" ]; then
  GZ_CMAKE_PKG=\$(grep -E '^lib(gz|ign|ignition)(-rotary)?-cmake[0-9]*-dev' "\${SOURCES_DIR}/.github/ci/packages.apt" | head -n 1)
fi

if [ "\$GZ_CMAKE_PKG" = "libgz-rotary-cmake-dev" ]; then
  # rotary package is only available in nightly repo. If it's not available
  # (e.g. during a prerelease), fallback to the latest versioned package
  if ! apt-cache show libgz-rotary-cmake-dev > /dev/null 2>&1; then
    echo "libgz-rotary-cmake-dev not found, falling back to latest versioned package"
    GZ_CMAKE_PKG=\$(apt-cache search --names-only '^libgz-cmake[0-9]+-dev' | awk '{print \$1}' | sort -V | tail -n 1)
  fi
fi

if [ -n "\$GZ_CMAKE_PKG" ]; then
  echo "Installing detected cmake dependency: \${GZ_CMAKE_PKG}"
  sudo apt-get install -y "\${GZ_CMAKE_PKG}" || \
    (echo "Failed to install \${GZ_CMAKE_PKG}" && exit 1)
else
  echo "No external gz-cmake package required."
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
