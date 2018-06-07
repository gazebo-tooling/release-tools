#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
#!/usr/bin/env bash
set -ex

RESULT_DIR=${WORKSPACE}/shellcheck_results
COMMON_OPTS="--exclude SC2034 --format checkstyle"

[[ -d \${RESULT_DIR} ]] && rm -fr \${RESULT_DIR}
mkdir \${RESULT_DIR}

BASH_FILES=\$(find ${WORKSPACE}/release-tools -name '*.bash')
SH_FILES=\$(find ${WORKSPACE}/release-tools -name '*.sh')

echo '# BEGIN SECTION: run shellcheck'
shellcheck --shell=bash \${COMMON_OPTS} -- \${BASH_FILES} > \${RESULT_DIR}/shellcheck_bash.xml
shellcheck --shell=sh \${COMMON_OPTS} -- \${SH_FILES} > \${RESULT_DIR}/shellcheck_sh.xml
echo '# END SECTION'
DELIM

OSRF_REPOS_TO_USE="stable"
DEPENDENCY_PKGS="devscripts \
		 ubuntu-dev-tools \
		 debhelper \
		 wget \
                 shellcheck"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
