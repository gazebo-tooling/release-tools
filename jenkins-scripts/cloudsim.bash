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


sudo apt-get install -y python openssh-client unzip zip mercurial apache2 redis-server python-pip python-redis
sudo pip install --upgrade boto

sudo pip install unittest-xml-reporting 
sudo apt-get install cmake 
sudo apt-get install python-software-properties


# run redis-server as it does not seem to start on its own
/etc/init.d/redis-server start

rm -rf $WORKSPACE/build $WORKSPACE/cloudsim/test-reports
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake $WORKSPACE/cloudsim
PYTHONPATH=$WORKSPACE/cloudsim/inside/cgi-bin make test ARGS="-VV" || true

DELIM

# Copy in my boto config file, to allow launching of AWS machines.
cp $HOME/boto.test-osrfoundation.org $WORKSPACE/boto.ini

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
