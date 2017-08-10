#!/bin/bash -x

# Drake can not work with ccache
export ENABLE_CCACHE=false

echo '# BEGIN SECTION: setup the testing enviroment'
DOCKER_JOB_NAME="drake_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

cat > build.sh << DELIM
###################################################
#
set -ex

echo '# BEGIN SECTION: install bazel'
echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list
curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
apt-get update
apt-get install -y bazel
echo '# END SECTION'

echo '# BEGIN SECTION: install Drake dependencies'
export INSTALL_PREREQS_FILE=${WORKSPACE}/repo/setup/ubuntu/16.04/install_prereqs.sh
sed -i -e \'/# TODO(jamiesnape).*/,\$d' \$INSTALL_PREREQS_FILE
chmod +x \$INSTALL_PREREQS_FILE
./\$INSTALL_PREREQS_FILE
echo '# END SECTION'
DELIM

SOFTWARE_DIR="repo"
OSRF_REPOS_TO_USE="stable"
USE_ROS_REPO=true
DEPENDENCY_PKGS="${BASE_DEPENDENCIES}"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
