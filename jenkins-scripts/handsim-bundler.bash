#!/bin/bash

set -x
set -e

[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

sudo apt-get -y install zip

# We're going to cherry-pick certain debs out of the repo and produce a .zip
# file for people to download.
PKGS="libogre3d-1.9-dev \
    libogre3d-1.9.0 \
    libczmq1 \
    libczmq-dev \
    libsdformat3-dev \
    libsdformat3 \
    sdformat-sdf \
    libignition-transport0-dev \
    libignition-transport0 \
    libignition-math2-dev \
    libignition-math2 \
    libhaptix-comm-dev \
    libhaptix-comm0 \
    libsimbody-dev \
    libsimbody3.5 \
    handsim \
    gazebo7-haptix-common \
    gazebo7-haptix-plugin-base \
    gazebo7-haptix \
    libgazebo7-haptix-dev \
    libgazebo7-haptix"

# Try to work inside jenkins. If not possible, get a temporal directory.
dir=${WORKSPACE}

if [[ -z ${dir} ]]; then
  dir=`mktemp -d`
fi

subdir=handsim-debs-`date +%F-%H-%M-%S`
mkdir -p $dir/$subdir
cp $SCRIPT_DIR/handsim-unbundler.bash $dir/$subdir

root="/var/packages/gazebo/ubuntu"
cd $root
for p in $PKGS; do
  filename=`sudo GNUPGHOME=/var/lib/jenkins/.gnupg/ reprepro -A amd64 --list-format '${filename}' list trusty $p`
  cp $filename $dir/$subdir
done

cd $dir
zip -r $subdir.zip $subdir
sudo mkdir -p /var/packages/haptix
sudo cp $subdir.zip /var/packages/haptix
sudo ln -sf $subdir.zip /var/packages/haptix/handsim-debs-latest.zip
