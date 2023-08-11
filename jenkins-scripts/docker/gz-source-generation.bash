#!/bin/bash -xe
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
export DOCKER_JOB_NAME="source_generation_job"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

cat > build.sh << DELIM
#!/bin/bash
set -ex

PKG_DIR=\$WORKSPACE/pkgs
SOURCES_DIR=\$WORKSPACE/sources
BUILD_DIR=\$SOURCES_DIR/build

cd \${WORKSPACE}
rm -fr \$SOURCES_DIR && mkdir \$SOURCES_DIR
git clone --depth 1 --branch ${PACKAGE_NAME}_${VERSION} ${SOURCE_REPO_URI} \${SOURCES_DIR}
rm -fr \$BUILD_DIR && mkdir \$BUILD_DIR
cd \${BUILD_DIR}
cmake .. -DPACKAGE_SOURCE_ONLY:BOOL=ON
make package_source

rm -fr \$PKG_DIR && mkdir \$PKG_DIR
find \${BUILD_DIR} -maxdepth 1 -name '*${VERSION}.tar.*' -exec mv {} \${PKG_DIR} \\;

if [ $(ls 2>/dev/null -Ubad1 -- "\${PKG_DIR}" | wc -l) -gt 1 ]; then
  echo "Found more than one file inside pkgs directory:"
  ls \${PKG_DIR}
  exit 1
fi
DELIM

export DEPENDENCY_PKGS="cmake git"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
