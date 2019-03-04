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
# DEST_BRANCH
# SRC_BRANCH

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

cat > build.sh << DELIM
#!/bin/bash

###################################################
# Make project-specific changes here
#
set -ex

# Bug in gcc5 with eigen see: https://bitbucket.org/osrf/release-tools/issues/147
if [[ $DISTRO == xenial ]]; then
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

echo '# BEGIN SECTION: compile and install branch: ${DEST_BRANCH}'
cp -a $WORKSPACE/${ABI_JOB_SOFTWARE_NAME} /tmp/${ABI_JOB_SOFTWARE_NAME}
chown -R root:root /tmp/${ABI_JOB_SOFTWARE_NAME}
cd /tmp/${ABI_JOB_SOFTWARE_NAME}
hg pull
hg up ${DEST_BRANCH}
# Normal cmake routine for ${ABI_JOB_SOFTWARE_NAME}
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake ${ABI_JOB_CMAKE_PARAMS} \\
  -DCMAKE_INSTALL_PREFIX=/usr/local/destination_branch \\
  /tmp/${ABI_JOB_SOFTWARE_NAME}
make -j${MAKE_JOBS}
make install
DEST_DIR=\$(find /usr/local/destination_branch/include -name ${ABI_JOB_SOFTWARE_NAME}-* -type d | sed -e 's:.*/::')
echo '# END SECTION'

echo '# BEGIN SECTION: compile and install branch: ${SRC_BRANCH}'
# 2.2 Target branch
# Reusing the same building and source directory to save bandwith and
# compilation time.
cd /tmp/${ABI_JOB_SOFTWARE_NAME}
hg pull ${SRC_REPO} -b ${SRC_BRANCH}
hg up ${SRC_BRANCH}
# Normal cmake routine for ${ABI_JOB_SOFTWARE_NAME}
cd $WORKSPACE/build
cmake ${ABI_JOB_CMAKE_PARAMS} \\
  -DCMAKE_INSTALL_PREFIX=/usr/local/source_branch \\
  /tmp/${ABI_JOB_SOFTWARE_NAME}
make -j${MAKE_JOBS}
make install
SRC_DIR=\$(find /usr/local/source_branch/include -name ${ABI_JOB_SOFTWARE_NAME}-* -type d | sed -e 's:.*/::')
echo '# END SECTION'

echo '# BEGIN SECTION: install the ABI checker'
# Install abi-compliance-checker.git
cd $WORKSPACE
rm -fr $WORKSPACE/abi-compliance-checker
git clone git://github.com/lvc/abi-compliance-checker.git
cd abi-compliance-checker
perl Makefile.pl -install --prefix=/usr

mkdir -p $WORKSPACE/abi_checker
cd $WORKSPACE/abi_checker
cat > pkg.xml << CURRENT_DELIM
 <version>
     branch: $DEST_BRANCH
 </version>

 <headers>
   /usr/local/destination_branch/include/\$DEST_DIR
 </headers>

 <skip_headers>
CURRENT_DELIM

for header in $ABI_JOB_IGNORE_HEADERS; do
  echo "  /usr/local/destination_branch/include/\$DEST_DIR/\$header" >> pkg.xml
done

cat >> pkg.xml << CURRENT_DELIM_LIBS
 </skip_headers>
 <libs>
   /usr/local/destination_branch/lib/
 </libs>

 <gcc_options>
     -std=c++11
 </gcc_options>
CURRENT_DELIM_LIBS

cat > devel.xml << DEVEL_DELIM
 <version>
     branch: $SRC_BRANCH
 </version>

 <headers>
   /usr/local/source_branch/include/\$SRC_DIR
 </headers>

 <skip_headers>
DEVEL_DELIM

for header in $ABI_JOB_IGNORE_HEADERS; do
  echo "  /usr/local/source_branch/include/\$SRC_DIR/\$header" >> devel.xml
done

cat >> devel.xml << DEVEL_DELIM_LIBS
 </skip_headers>

 <libs>
   /usr/local/source_branch/lib/
 </libs>

 <gcc_options>
     -std=c++11
 </gcc_options>
DEVEL_DELIM_LIBS
echo '# END SECTION'

echo '# BEGIN SECTION: display the xml configuration'
cat devel.xml
echo
echo
cat pkg.xml
echo '# END SECTION'

echo '# BEGIN SECTION: run the ABI checker'
# clean previous reports
REPORTS_DIR=$WORKSPACE/reports/
rm -fr \${REPORTS_DIR} && mkdir -p \${REPORTS_DIR}
rm -fr compat_reports/
# run report tool
abi-compliance-checker -lang C++ -lib ${ABI_JOB_SOFTWARE_NAME} -old pkg.xml -new devel.xml || true

# copy method version independant ( cp ... /*/ ... was not working)
find compat_reports/ -name compat_report.html -exec cp {} \${REPORTS_DIR} \;
echo '# END SECTION'

if [ `expr length "${ABI_JOB_POSTCHECKER_HOOK} "` -gt 1 ]; then
echo '# BEGIN SECTION: running post ABI hook'
${ABI_JOB_POSTABI_HOOK}
echo '# END SECTION'
fi
DELIM

OSRF_REPOS_TO_USE=${ABI_JOB_REPOS}
DEPENDENCY_PKGS="${ABI_JOB_PKG_DEPENDENCIES} \
                  git \
                  exuberant-ctags \
                  mercurial \
                  ca-certificates"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
