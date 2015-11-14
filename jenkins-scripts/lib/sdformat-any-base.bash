#!/bin/bash -x
set -e

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

if [[ -z $REPO_TO_USE ]]; then
    export REPO_TO_USE=OSRF
fi

case $REPO_TO_USE in
    "OSRF" )
       export REPO_URL="http://packages.osrfoundation.org/drc/ubuntu"
       export REPO_KEY="http://packages.osrfoundation.org/drc.key"
       ;;
    "ROS" )
       export REPO_URL="http://packages.ros.org/ros/ubuntu"
       export REPO_KEY="http://packages.ros.org/ros.key"
       ;;
    * )
	echo "Unknown REPO_TO_USE value"
	exit 1
	;;
esac

# No SDFORMAT_PKG declared, check latest
# Another alternative is try to get the latest package available, but it 
# becomes no sensible to which code branch we are testing:
# apt-cache search sdformat | grep 'libsdformat.*-dev ' | tail  -1 |  awk '{ print $1 }' 
if [[ -z ${SDFORMAT_PKG} ]]; then
    # Identify SDFORMAT_MAJOR_VERSION to help with dependency resolution
    SDFORMAT_MAJOR_VERSION=`\
      grep 'set.*SDF_MAJOR_VERSION ' ${WORKSPACE}/sdformat/CMakeLists.txt | \
      tr -d 'a-zA-Z _()'`

    # Check sdformat version is integer
    if ! [[ ${SDFORMAT_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
       echo "Error! SDFORMAT_MAJOR_VERSION is not an integer, check the detection"
       exit -1
    fi

  SDFORMAT_PKG=libsdformat${SDFORMAT_MAJOR_VERSION}-dev
fi


cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# Step 1: install everything you need ans sdformat (OSRF repo) from binaries
apt-get install -y wget 
sh -c 'echo "deb ${REPO_URL} ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget ${REPO_KEY} -O - | apt-key add -
apt-get update

# Checkout latest libsdformatX-dev package
apt-get install -y ${BASE_DEPENDENCIES} ${SDFORMAT_BASE_DEPENDENCIES} ${SDFORMAT_PKG} git exuberant-ctags

# Step 2: configure and build
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake $WORKSPACE/sdformat -DCMAKE_INSTALL_PREFIX=/usr/local
make -j${MAKE_JOBS}
make install
sdformat_ORIGIN_DIR=\$(find /usr/include -name sdformat-* -type d | sed -e 's:.*/::')
sdformat_TARGET_DIR=\$(find /usr/local/include -name sdformat-* -type d | sed -e 's:.*/::')

# Install abi-compliance-checker.git
cd $WORKSPACE
rm -fr $WORKSPACE/abi-compliance-checker
git clone git://github.com/lvc/abi-compliance-checker.git  
cd abi-compliance-checker
perl Makefile.pl -install --prefix=/usr

BIN_VERSION=\$(dpkg -l ${SDFORMAT_PKG} | tail -n 1 | awk '{ print  \$3 }')

mkdir -p $WORKSPACE/abi_checker
cd $WORKSPACE/abi_checker
cat > pkg.xml << CURRENT_DELIM
<gcc_options>
     /* Additional GCC options, one per line */
 </gcc_options>
 <version>
     .deb pkg version: \$BIN_VERSION
 </version>
   
 <headers>
    /usr/include/\$sdformat_ORIGIN_DIR/
 </headers>
   
 <libs>
    /usr/lib/
 </libs>

 <gcc_options>
     -std=c++11
 </gcc_options>

CURRENT_DELIM

cat > devel.xml << DEVEL_DELIM
 <version>
     branch: $BRANCH
 </version>
   
 <headers>
    /usr/local/include/\$sdformat_TARGET_DIR
 </headers>
   
 <libs>
    /usr/local/lib/
 </libs>

 <gcc_options>
     -std=c++11
 </gcc_options>
DEVEL_DELIM

rm -fr compat_reports/ 
rm -fr $WORKSPACE/compat_report.html
abi-compliance-checker -lib sdformat -old pkg.xml -new devel.xml || true
# copy method version independant ( cp ... /*/ ... was not working)
find compat_reports/ -name compat_report.html -exec cp {} $WORKSPACE/ \;
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
