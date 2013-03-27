#!/bin/bash -x
set -e

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# Step 1: install everything you need
apt-get install -s cmake build-essential libboost-system-dev libboost-filesystem-dev libboost-program-options-dev libboost-regex-dev libboost-iostreams-dev libtinyxml-dev

# Step 2: configure and build
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
ls -las /usr/bin/cmake
cmake $WORKSPACE/sdf
make -j3
make install
make test ARGS="-VV" || true
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
