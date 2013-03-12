#!/bin/bash -x

[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

set +x # keep password secret
MAIL_PASS=`cat $HOME/mail_pass`
set -x # back to debug

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# Install dependencies
apt-get install -y python-pip python-yaml
pip install --upgrade boto

# Testing
cd $WORKSPACE/cloudsim
set +x # keep password secret
./bin/cloud-check.py $WORKSPACE/aws_accounts.yaml caguero@osrfoundation.org 24 osrfbuild@osrfoundation.org $MAIL_PASS
set -x # back to debug
DELIM

# Copy in my boto config file, to allow launching of AWS machines.
cp $HOME/boto.test-osrfoundation.org $WORKSPACE/boto.ini

# Make project-specific changes here
###################################################

rm -fr $WORKSPACE/aws_accounts.yaml
cp -a $HOME/aws_accounts.yaml $WORKSPACE/aws_accounts.yaml

sudo $WORKSPACE/pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
