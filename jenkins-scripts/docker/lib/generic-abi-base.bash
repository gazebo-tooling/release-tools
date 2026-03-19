#!/bin/bash -x

# Useful scripting variables:

# ABI_JOB_SOFTWARE_NAME: (mandatory) directory name to access the software
# ABI_JOB_PRECHECKER_HOOK (optional) code to run before checking
# ABI_JOB_POSTCHECKER_HOOK (optional) code to run after checking
# ABI_JOB_REPOS: OSRF repositories to use
# ABI_JOB_PKG_DEPENDENCIES: (optional) list (space separated) of pkg dependencies
# ABI_JOB_PKG_DEPENDENCIES_VAR_NAME: (option) variable in archive to get dependencies from
# ABI_JOB_CMAKE_PARAMS: (option) cmake parameters to be pased to cmake configuration
# ABI_JOB_HEADER_PREFIX: (optional) hint for identifying header install prefix
# ABI_JOB_IGNORE_HEADERS: (optional) relative (to root project path) list (space separated)
#                         of path headers to ignore
# ABI_JOB_EXTRA_GCC_OPTIONS: (optional) inject gcc_options in the descriptor file
#                            one per line

# Jenkins variables:
# DEST_BRANCH
# SRC_BRANCH

set -e

echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
DOCKER_JOB_NAME="abi_job"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
. ${SCRIPT_DIR}/lib/_common_scripts.bash
echo '# END SECTION'

# Could be empty, just fine
if [[ "${ABI_JOB_PKG_DEPENDENCIES_VAR_NAME}" != "" ]]; then
  eval ABI_JOB_PKG_DEPENDENCIES="\$${ABI_JOB_PKG_DEPENDENCIES_VAR_NAME} ${ABI_JOB_PKG_DEPENDENCIES}"
fi
if [[ -z "${ABI_JOB_HEADER_PREFIX}" ]]; then
  eval ABI_JOB_HEADER_PREFIX="\$${ABI_JOB_SOFTWARE_NAME}-*"
fi

ABI_CXX_STANDARD=c++11
if [[ "${NEED_C17_COMPILER}" == "true" ]]; then
  ABI_CXX_STANDARD=c++17
fi

cat > build.sh << DELIM
$(generate_buildsh_header)

if [ `expr length "${ABI_JOB_PRECHECKER_HOOK} "` -gt 1 ]; then
echo '# BEGIN SECTION: running pre ABI hook'
${ABI_JOB_PREABI_HOOK}
echo '# END SECTION'
fi

echo '# BEGIN SECTION: compile and install branch: ${DEST_BRANCH}'
cp -a $WORKSPACE/${ABI_JOB_SOFTWARE_NAME} /tmp/${ABI_JOB_SOFTWARE_NAME}
cd /tmp/${ABI_JOB_SOFTWARE_NAME}
git fetch origin
git checkout origin/${DEST_BRANCH}
# Normal cmake routine for ${ABI_JOB_SOFTWARE_NAME}
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake ${ABI_JOB_CMAKE_PARAMS} \\
  -DBUILD_TESTING=OFF \\
  -DCMAKE_INSTALL_PREFIX=/usr/local/destination_branch \\
  /tmp/${ABI_JOB_SOFTWARE_NAME}
make -j${MAKE_JOBS}
sudo make install
DEST_DIR=\$(find /usr/local/destination_branch/include -name ${ABI_JOB_HEADER_PREFIX} -type d | sed -e 's:.*/include/::')
echo '# END SECTION'

echo '# BEGIN SECTION: compile and install branch: ${SRC_BRANCH}'
# 2.2 Target branch
# Reusing the same building and source directory to save bandwith and
# compilation time.
cd /tmp/${ABI_JOB_SOFTWARE_NAME}
git remote add source_repo ${SRC_REPO}
git fetch source_repo
git checkout source_repo/${SRC_BRANCH}
# Normal cmake routine for ${ABI_JOB_SOFTWARE_NAME}
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake ${ABI_JOB_CMAKE_PARAMS} \\
  -DBUILD_TESTING=OFF \\
  -DCMAKE_INSTALL_PREFIX=/usr/local/source_branch \\
  /tmp/${ABI_JOB_SOFTWARE_NAME}
make -j${MAKE_JOBS}
sudo make install
SRC_DIR=\$(find /usr/local/source_branch/include -name ${ABI_JOB_HEADER_PREFIX} -type d | sed -e 's:.*/include/::')
echo '# END SECTION'

echo '# BEGIN SECTION: install the ABI checker'
# Install abi-compliance-checker.git
cd $WORKSPACE
rm -fr $WORKSPACE/abi-compliance-checker
git clone https://github.com/lvc/abi-compliance-checker
cd abi-compliance-checker
sudo perl Makefile.pl -install --prefix=/usr

mkdir -p $WORKSPACE/abi_checker
cd $WORKSPACE/abi_checker

cat > pkg.xml << CURRENT_DELIM
 <version>
     branch: $DEST_BRANCH
 </version>

 <headers>
   /usr/local/destination_branch/include/\$DEST_DIR
 </headers>

 ${EXTRA_INCLUDES}

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
     -std=${ABI_CXX_STANDARD}
     ${ABI_JOB_EXTRA_GCC_OPTIONS}
 </gcc_options>
CURRENT_DELIM_LIBS

cat > devel.xml << DEVEL_DELIM
 <version>
     branch: $SRC_BRANCH
 </version>

 <headers>
   /usr/local/source_branch/include/\$SRC_DIR
 </headers>

 ${EXTRA_INCLUDES}

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
     -std=${ABI_CXX_STANDARD}
     ${ABI_JOB_EXTRA_GCC_OPTIONS}
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

# if report was not generated, run again with -quick
if ! ls "compat_reports/${ABI_JOB_SOFTWARE_NAME}"
then
  abi-compliance-checker -lang C++ -lib ${ABI_JOB_SOFTWARE_NAME} -old pkg.xml -new devel.xml -quick || true
fi

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
                  ca-certificates"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
