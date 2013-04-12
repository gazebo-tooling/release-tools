#!/bin/bash -x

[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# Install NoseXUnit
apt-get install -y fping redis-server python-redis python-nose

# Testing
rm -fr $WORKSPACE/test_results
mkdir -p $WORKSPACE/test_results
cd $WORKSPACE/test_results
nosetests --with-xunit $WORKSPACE/cloudsim-client-tools/vrc_shaping/vrc_sniffer.py

DELIM

# Copy in my boto config file, to allow launching of AWS machines.
cp $HOME/boto.test-osrfoundation.org $WORKSPACE/boto.ini

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
