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

# gcc-5.4 segfaults on abi-checker
add-apt-repository -y ppa:ubuntu-toolchain-r/test
apt-get update
apt-get install -y gcc-6 g++-6
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-6
g++ --version

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

 <skip_headers>
    /usr/local/origin_branch/include/pcl-1.7/pcl/common/impl/
    /usr/local/origin_branch/include/pcl-1.7/pcl/features/impl/
    /usr/local/origin_branch/include/pcl-1.7/pcl/filters/impl/
    /usr/local/origin_branch/include/pcl-1.7/pcl/filters/box_clipper3D.h
    box_clipper3D.h
    /usr/local/origin_branch/include/pcl-1.7/pcl/impl/
    /usr/local/origin_branch/include/pcl-1.7/pcl/io/impl/
    /usr/local/origin_branch/include/pcl-1.7/pcl/range_image/impl/
    /usr/local/origin_branch/include/pcl-1.7/pcl/recognition/impl/
    /usr/local/origin_branch/include/pcl-1.7/pcl/registration/impl/
    pairwise_graph_registration.hpp
    /usr/local/origin_branch/include/pcl-1.7/pcl/tracking/impl/
    /usr/local/origin_branch/include/pcl-1.7/pcl/keypoints/impl/
    /usr/local/origin_branch/include/pcl-1.7/pcl/pcl_tests.h
    /usr/local/origin_branch/include/pcl-1.7/pcl/surface/impl/
    /usr/local/origin_branch/include/pcl-1.7/pcl/surface/3rdparty/
    /usr/local/origin_branch/include/pcl-1.7/pcl/geometry/get_boundary.h
    /usr/local/origin_branch/include/pcl-1.7/pcl/recognition/color_gradient_dot_modality.h
    /usr/local/origin_branch/include/pcl-1.7/pcl/recognition/dotmod.h
 </skip_headers>

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

 <skip_headers>
    /usr/local/target_branch/include/pcl-1.7/pcl/common/impl/
    /usr/local/target_branch/include/pcl-1.7/pcl/features/impl/
    /usr/local/target_branch/include/pcl-1.7/pcl/filters/impl/
    /usr/local/target_branch/include/pcl-1.7/pcl/filters/box_clipper3D.h
    box_clipper3D.h
    /usr/local/target_branch/include/pcl-1.7/pcl/impl/
    /usr/local/target_branch/include/pcl-1.7/pcl/io/impl/
    /usr/local/target_branch/include/pcl-1.7/pcl/range_image/impl/
    /usr/local/target_branch/include/pcl-1.7/pcl/recognition/impl/
    /usr/local/target_branch/include/pcl-1.7/pcl/registration/impl/
    pairwise_graph_registration.hpp
    /usr/local/target_branch/include/pcl-1.7/pcl/tracking/impl/
    /usr/local/target_branch/include/pcl-1.7/pcl/keypoints/impl/
    /usr/local/target_branch/include/pcl-1.7/pcl/pcl_tests.h
    /usr/local/target_branch/include/pcl-1.7/pcl/surface/impl/
    /usr/local/target_branch/include/pcl-1.7/pcl/surface/3rdparty/
    /usr/local/target_branch/include/pcl-1.7/pcl/geometry/get_boundary.h
    /usr/local/target_branch/include/pcl-1.7/pcl/recognition/color_gradient_dot_modality.h
    /usr/local/target_branch/include/pcl-1.7/pcl/recognition/dotmod.h
 </skip_headers>

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
# cleanup
find  /usr/local -name hv_go.* -exec rm {} \;
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
		  libpcl-dev \
		  software-properties-common"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
