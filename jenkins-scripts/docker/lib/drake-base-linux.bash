#!/bin/bash -x

# Drake can not work with ccache
export ENABLE_CCACHE=false

echo '# BEGIN SECTION: setup the testing enviroment'
DOCKER_JOB_NAME="drake_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

. ${SCRIPT_DIR}/lib/_drake_lib.bash

cat > build.sh << DELIM
###################################################
#
set -ex

${DRAKE_BAZEL_INSTALL}

echo '# BEGIN SECTION: install Drake dependencies'
INSTALL_PREREQS_DIR="${WORKSPACE}/repo/setup/ubuntu/16.04"
INSTALL_PREREQS_FILE="\$INSTALL_PREREQS_DIR/install_prereqs.sh"
# Remove last cmake dependencies
sed -i -e '/# TODO\(jamiesnape\).*/,\$d' \$INSTALL_PREREQS_FILE
# Install automatically all apt commands
sed -i -e 's:no-install-recommends:no-install-recommends -y:g' \$INSTALL_PREREQS_FILE
# Remove question to user
sed -i -e 's:.* read .*:yn=Y:g' \$INSTALL_PREREQS_FILE
chmod +x \$INSTALL_PREREQS_FILE
bash \$INSTALL_PREREQS_FILE
echo '# END SECTION'

echo '# BEGIN SECTION: compilation'
cd ${WORKSPACE}/repo
bazel build --jobs=${MAKE_JOBS} //...
echo '# END SECTION'

echo '# BEGIN SECTION: tests'
bazel test //... || true
echo '# END SECTION'

echo '# BEGIN SECTION: install'
bazel run :install --jobs=${MAKE_JOBS} -- /opt/drake
echo '# END SECTION'

echo '# BEGIN SECTION: particle test'
cd ${WORKSPACE}
git clone https://github.com/RobotLocomotion/drake-shambhala
cd drake-shambhala/drake_cmake_installed
mkdir build
cd build
cmake ..
make -j${MAKE_JOBS}
cd src/particles
timeout --preserve-status 5 ./uniformly_accelerated_particle_demo
echo '# END SECTION'
DELIM

SOFTWARE_DIR="repo"
OSRF_REPOS_TO_USE="stable"
USE_ROS_REPO="true" # Needed for libfcl-0.5-dev package
DEPENDENCY_PKGS="git \
                 ${BASE_DEPENDENCIES} \
                 ${DRAKE_DEPENDENCIES}"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
