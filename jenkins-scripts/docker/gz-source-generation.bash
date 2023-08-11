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

git clone --depth 1 --branch ${PACKAGE_NAME}_${VERSION} ${SOURCE_REPO_URI} sources
mkdir sources/build
cd sources/build
cmake .. -DPACKAGE_SOURCE_ONLY
make package_source

mkdir -p \${PKG_DIR}
find build/ -maxdepth 1 -name *${VERSION}.tar.bz2 -exec mv {} \${PKG_DIR} \\;

if [ $(ls 2>/dev/null -Ubad1 -- "\${PKG_DIR}" | wc -l) -gt 1 ]; then
  echo "Found more than one file inside pkgs directory:"
  ls \${PKG_DIR}
  exit 1
fi
DELIM

export DEPENDENCY_PKGS="cmake git"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
