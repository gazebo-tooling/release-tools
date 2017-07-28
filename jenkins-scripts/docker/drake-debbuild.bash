#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Needs it own script to be able to install basel as prehook
DOCKER_POSTINSTALL_HOOK="""\
echo '# BEGIN SECTION: install bazel' && \\
echo \"deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable jdk1.8\" | tee /etc/apt/sources.list.d/bazel.list && \\
curl https://bazel.build/bazel-release.pub.gpg | apt-key add - && \\
apt-get update && \\
apt-get install -y bazel && \\
echo '# END SECTION'
"""

. ${SCRIPT_DIR}/lib/debian-git-repo-base.bash
