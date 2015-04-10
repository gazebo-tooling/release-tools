#!/bin/bash

set -x
set -e

[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

apt-get install zip

# We're going to cherry-pick certain debs out of the repo and produce a .zip file for people to download.
PKGS="libogre3d-1.9-dev libogre3d-1.9.0 libsdformat3-dev-prerelease libsdformat3-prerelease sdformat-sdf-prerelease libignition-transport-dev libignition-transport0 libhaptix-comm-dev libhaptix-comm0 handsim gazebo6-common-prerelease gazebo6-plugin-base-prerelease gazebo6-prerelease libgazebo6-dev-prerelease libgazebo6-prerelease"


dir=`mktemp -d`
subdir=handsim-debs-`date +%F-%H-%M-%S`
mkdir -p $dir/$subdir
cp $SCRIPT_DIR/handsim-unbundler.bash $dir/$subdir

root="/var/packages/gazebo/ubuntu"
cd $root
for p in $PKGS; do
  filename=`reprepro -A amd64 --list-format '${filename}' list trusty $p`
  cp $filename $dir/$subdir
done

cd $dir
zip -r $subdir.zip $subdir
mkdir -p /var/packages/haptix
cp $subdir.zip /var/packages/haptix
ln -sf $subdir.zip /var/packages/haptix/handsim-debs-latest.zip
rm -rf $dir
