#!/bin/bash -x

# Useful scripting variables:

# ABI_JOB_SOFTWARE_NAME: (mandatory) directory name to access the software
# ABI_JOB_PRECHECKER_HOOK (optional) code to run before checking
# ABI_JOB_POSTCHECKER_HOOK (optional) code to run after checking
# ABI_JOB_REPOS: OSRF repositories to use
# ABI_JOB_PKG_DEPENDENCIES: (optional) list (space separated) of pkg dependencies
# ABI_JOB_PKG_DEPENDENCIES_VAR_NAME: (option) variable in archive to get dependencies from
# ABI_JOB_CMAKE_PARAMS: (option) cmake parameters to be pased to cmake configuration
# ABI_JOB_IGNORE_HEADERS: (optional) relative (to root project path) list (space separated)
#                         of path headers to ignore

# Jenkins variables:
# SRC_BRANCH
# DEST_BRANCH (really used to get the package names)

OSRF_PKG_NAME=${DEST_BRANCH/ign-/ignition-}

set -e

echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
DOCKER_JOB_NAME="abi_job"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

# Could be empty, just fine
if [[ "${ABI_JOB_PKG_DEPENDENCIES_VAR_NAME}" != "" ]]; then
  eval ABI_JOB_PKG_DEPENDENCIES="\$${ABI_JOB_PKG_DEPENDENCIES_VAR_NAME} ${ABI_JOB_PKG_DEPENDENCIES}"
fi
if [[ -n "${DART_FROM_PKGS_VAR_NAME}" ]]; then
  eval DART_FROM_PKGS="\$${DART_FROM_PKGS_VAR_NAME}"
fi

ABI_CXX_STANDARD=c++11
if [[ "${USE_GCC8}" == "true" ]]; then
  ABI_CXX_STANDARD=c++17
fi

cat > build.sh << DELIM
#!/bin/bash

###################################################
# Make project-specific changes here
#
set -ex

# Bug in gcc5 with eigen see: https://bitbucket.org/osrf/release-tools/issues/147
if [[ "${USE_GCC6}" -gt 0 || -z "${USE_GCC6}" && "${DISTRO}" == xenial ]]; then
  apt-get update
  apt-get install -y software-properties-common
  add-apt-repository -y ppa:ubuntu-toolchain-r/test
  apt-get update
  apt-get install -y gcc-6 g++-6
  update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 10
  update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-6 10
  update-alternatives --config gcc
  update-alternatives --config g++
fi

if [ `expr length "${ABI_JOB_PRECHECKER_HOOK} "` -gt 1 ]; then
echo '# BEGIN SECTION: running pre ABI hook'
${ABI_JOB_PREABI_HOOK}
echo '# END SECTION'
fi

echo '# BEGIN SECTION: compile and install branch: ${SRC_BRANCH}'
cp -a $WORKSPACE/${ABI_JOB_SOFTWARE_NAME} /tmp/${ABI_JOB_SOFTWARE_NAME}
chown -R root:root /tmp/${ABI_JOB_SOFTWARE_NAME}
cd /tmp/${ABI_JOB_SOFTWARE_NAME}
hg pull
hg up ${SRC_BRANCH}
# Normal cmake routine for ${ABI_JOB_SOFTWARE_NAME}
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake ${ABI_JOB_CMAKE_PARAMS} \\
  -DBUILD_TESTING=OFF \\
  -DCMAKE_INSTALL_PREFIX=/usr/local/destination_branch \\
  /tmp/${ABI_JOB_SOFTWARE_NAME}
make -j${MAKE_JOBS}
make install
DEST_DIR=\$(find /usr/local/destination_branch/include -name ${ABI_JOB_SOFTWARE_NAME}-* -type d | sed -e 's:.*/::')
echo '# END SECTION'

echo '# BEGIN SECTION: install the ABI checker'
# Install abi-compliance-checker.git
cd $WORKSPACE
rm -fr $WORKSPACE/abi-compliance-checker
git clone git://github.com/lvc/abi-compliance-checker.git
cd abi-compliance-checker
perl Makefile.pl -install --prefix=/usr
# Install abi-compliance-checker.git
cd $WORKSPACE
rm -fr $WORKSPACE/auto-abi-checker
git clone https://github.com/osrf/auto-abi-checker
echo '# END SECTION'

echo '# BEGIN SECTION: run the ABI checker'
# clean previous reports
REPORTS_DIR=$WORKSPACE/reports/
rm -fr \${REPORTS_DIR} && mkdir -p \${REPORTS_DIR}
rm -fr compat_reports/
# run report tool
${WORKSPACE}/auto-abi-checker/auto-abi.py \
   --orig-type osrf-pkg --orig ${OSRF_PKG_NAME} \
   --new-type local-dir --new /usr/local/destination_branch

# TODO: implement a best method to locate results
find /tmp/ . -name compat_reports

# copy method version independant ( cp ... /*/ ... was not working)
find tmp/ -name compat_report.html -exec cp {} \${REPORTS_DIR} \;
echo '# END SECTION'

if [ `expr length "${ABI_JOB_POSTCHECKER_HOOK} "` -gt 1 ]; then
echo '# BEGIN SECTION: running post ABI hook'
${ABI_JOB_POSTABI_HOOK}
echo '# END SECTION'
fi
DELIM

OSRF_REPOS_TO_USE=${ABI_JOB_REPOS}
USE_ROS_REPO=true
DEPENDENCY_PKGS="${ABI_JOB_PKG_DEPENDENCIES} \
                  git \
                  exuberant-ctags \
                  mercurial \
                  ca-certificates \
                  python3-rosdistro"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
