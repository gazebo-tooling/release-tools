#!/bin/bash -x

# Drake can not work with ccache
export ENABLE_CCACHE=false

echo '# BEGIN SECTION: setup the testing enviroment'
DOCKER_JOB_NAME="drake_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

# Import drake lib
. ${SCRIPT_DIR}/lib/_drake_lib.bash

cat > build.sh << DELIM
###################################################
#
set -ex

${DRAKE_BAZEL_INSTALL}

echo '# BEGIN SECTION: install Drake dependencies'
export INSTALL_PREREQS_FILE="${WORKSPACE}/repo/setup/ubuntu/16.04/install_prereqs.sh"
# Remove last cmake dependencies
sed -i -e '/# TODO\(jamiesnape\).*/,\$d' \$INSTALL_PREREQS_FILE
# Install automatically all apt commands
sed -i -e 's:no-install-recommends:no-install-recommends -y:g' \$INSTALL_PREREQS_FILE
# Remove question to user
sed -i -e 's:.* read .*:yn=Y:g' \$INSTALL_PREREQS_FILE
chmod +x \$INSTALL_PREREQS_FILE
bash \$INSTALL_PREREQS_FILE
echo '# END SECTION'
DELIM

SOFTWARE_DIR="repo"
OSRF_REPOS_TO_USE="stable"
USE_ROS_REPO=true
DEPENDENCY_PKGS="${BASE_DEPENDENCIES}"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
