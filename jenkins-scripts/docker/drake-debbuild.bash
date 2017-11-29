#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

. ${SCRIPT_DIR}/lib/_drake_lib.bash

# Needs it own script to be able to install basel as prehook
# Overwite for Bug: https://askubuntu.com/questions/769467/can-not-install-openjdk-9-jdk-because-it-tries-to-overwrite-file-aready-includ 
DOCKER_POSTINSTALL_HOOK="""\
echo '# BEGIN SECTION: install bazel' && \\
wget -O /tmp/bazel_0.6.1-linux-x86_64.deb https://github.com/bazelbuild/bazel/releases/download/0.6.1/bazel_0.6.1-linux-x86_64.deb && \\
dpkg -i /tmp/bazel_0.6.1-linux-x86_64.deb && \\
echo '# END SECTION'
"""

. ${SCRIPT_DIR}/lib/debian-git-repo-base.bash
