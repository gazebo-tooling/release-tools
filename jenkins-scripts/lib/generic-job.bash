#!/bin/bash -x
set -e

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

eval PROJECT_DEPENDECIES=\$${PKG_DEPENDENCIES_VAR_NAME}

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# OSRF repository to get zmq
apt-get install -y wget
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -

# Step 1: install everything you need
apt-get update
apt-get install -y ${BASE_DEPENDENCIES} ${PROJECT_DEPENDECIES}

# Step 2: configure and build
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake $WORKSPACE/${REPO_DIRECTORY}
make -j${MAKE_JOBS}
make install
make test ARGS="-VV" || true

# Step 3: code check
cd $WORKSPACE/${REPO_DIRECTORY}
sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
cat $WORKSPACE/build/cppcheck_results/*.xml
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
