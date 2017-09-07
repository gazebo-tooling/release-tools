#!/bin/bash -x

# Useful scripting variables:

set -e

echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
DOCKER_JOB_NAME="eigen_abi_job"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

cat > build.sh << DELIM
#!/bin/bash

###################################################
# Make project-specific changes here
#
set -ex

echo '# BEGIN SECTION: compile and install pcl with eigen 3.3~beta1'
apt-get remove -y .*pcl.*
mkdir /tmp/xenial_pcl_eigen
cd /tmp/xenial_pcl_eigen
apt-get source pcl
cd pcl-*
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local/origin_branch ..
make -j${MAKE_JOBS}
make install
ORIGIN_DIR=\$(find /usr/local/origin_branch/include -name eigen-* -type d | sed -e 's:.*/::')
echo '# END SECTION'

echo '# BEGIN SECTION: replace system eigen for 3.3.3' 
apt-get remove -y .*eigen.*
mkdir  /tmp/eigen333
cd /tmp/eigen333
hg clone https://bitbucket.org/eigen/eigen/
cd eigen
hg up 3.3.3
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make -j${MAKE_JOBS}
make install
echo '# END SECTION'

echo '# BEGIN SECTION: compile and install pcl with eigen 3.3.3'
mkdir /tmp/xenial_pcl_eigen33
cd /tmp/xenial_pcl_eigen33
apt-get source pcl
cd pcl-*
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local/target_branch ..
make -j${MAKE_JOBS}
make install
TARGET_DIR=\$(find /usr/local/target_branch/include -name eigen-* -type d | sed -e 's:.*/::')
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
     branch: 3.3~beta1
 </version>

 <headers>
   /usr/local/origin_branch/include/\$ORIGIN_DIR
 </headers>

 <gcc_options>
     -std=c++11
 </gcc_options>

 <libs>
   /usr/local/origin_branch/lib/
 </libs>
CURRENT_DELIM

cat > devel.xml << DEVEL_DELIM
 <version>
     branch: 3.3.3
 </version>

 <headers>
   /usr/local/target_branch/include/\$TARGET_DIR
 </headers>

 <gcc_options>
     -std=c++11
 </gcc_options>

 <libs>
   /usr/local/target_branch/lib/
 </libs>
DEVEL_DELIM
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
abi-compliance-checker -l eigen3 -old pkg.xml -new devel.xml || true

# copy method version independant ( cp ... /*/ ... was not working)
find compat_reports/ -name compat_report.html -exec cp {} \${REPORTS_DIR} \;
echo '# END SECTION'
DELIM

OSRF_REPOS_TO_USE="stable"
DEPENDENCY_PKGS=" git \
                  exuberant-ctags \
                  mercurial \
                  ca-certificates \
		  mesa-common-dev \
		  libpcl-dev"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
