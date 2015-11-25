#!/bin/bash -x
set -e

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
echo '# BEGIN SECTION: compile and install branch: ${ORIGIN_BRANCH}'
cp -a $WORKSPACE/sdformat /tmp/sdformat
chown -R root:root /tmp/sdformat
cd /tmp/sdformat
hg pull
hg up $ORIGIN_BRANCH
# Normal cmake routine for sdformat
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local/origin_branch \\
  /tmp/sdformat
make -j${MAKE_JOBS}
make install
ORIGIN_DIR=\$(find /usr/local/origin_branch/include -name sdformat-* -type d | sed -e 's:.*/::')
echo '# END SECTION'

echo '# BEGIN SECTION: compile and install branch: ${TARGET_BRANCH}'
# 2.2 Target branch
# Reusing the same building and source directory to save bandwith and
# compilation time.
cd /tmp/sdformat
hg up $TARGET_BRANCH
# Normal cmake routine for sdformat
cd $WORKSPACE/build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local/target_branch \\
  /tmp/sdformat
make -j${MAKE_JOBS}
make install
TARGET_DIR=\$(find /usr/local/target_branch/include -name sdformat-* -type d | sed -e 's:.*/::')
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
     branch: $ORIGIN_BRANCH
 </version>

 <headers>
   /usr/local/origin_branch/include/\$ORIGIN_DIR/sdf
 </headers>

 <libs>
   /usr/local/origin_branch/lib/
 </libs>
CURRENT_DELIM

cat > devel.xml << DEVEL_DELIM
 <version>
     branch: $TARGET_BRANCH
 </version>

 <headers>
   /usr/local/target_branch/include/\$TARGET_DIR/sdf
 </headers>

 <libs>
   /usr/local/target_branch/lib/
 </libs>

 <gcc_options>
     -std=c++11
 </gcc_options>
DEVEL_DELIM
echo '# END SECTION'

echo '# BEGIN SECTION: run the ABI checker'
# clean previous reports
rm -fr $WORKSPACE/compat_report.html
rm -fr compat_reports/
# run report tool
abi-compliance-checker -lib sdformat -old pkg.xml -new devel.xml || true
# copy method version independant ( cp ... /*/ ... was not working)
find compat_reports/ -name compat_report.html -exec cp {} $WORKSPACE/ \;
echo '# END SECTION'
DELIM

OSRF_REPOS_TO_USE="stable"
DEPENDENCY_PKGS="${SDFORMAT_BASE_DEPENDENCIES} \
                  git \
                  exuberant-ctags \
                  mercurial \
                  ca-certificates"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
