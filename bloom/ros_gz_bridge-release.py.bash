#!/bin/bash
# Launch all the suite of ros_gazebo_pkgs:
# Usage: ros_gz_bridge-release.py.bash <version>
#

if [[ $# -lt 2 ]]; then
    echo "$0 <version> <release_repo> <ros_distro> <token> 'other arguments used in release.py'"
    exit 1
fi

if [[ ${1%-*} != "${1}" ]]; then
  echo "Version should not contain a revision number. Use -r argument"
  exit 1
fi

for p in ros-ign-image ros-ign-bridge ros-ign-gazebo-demos ros-ign-point-cloud ros-ign ros-ign-gazebo; do
  ./release-bloom.py "${p}" $(for i in $@; do echo -n "$i "; done)
done
