#!/bin/bash
# Launch all the suite of gz_ros:
# Usage: ros_gz_bridge-release.py.bash <version>

if [[ $# -lt 2 ]]; then
    echo "$0 <version> <release_repo> <ros_distro> <token> 'other arguments used in release.py'"
    exit 1
fi

if [[ ${1%-*} != "${1}" ]]; then
  echo "Version should not contain a revision number. Use -r argument"
  exit 1
fi

# topological order to go into Jenkins in the best starting position to make blocks to work
for p in ros-gz-interfaces ros-gz-sim ros-gz-bridge ros-gz-image ros-gz-sim-demos ros-gz; do
  ./_release-bloom.py "${p}" $(for i in ""$@; do echo -n "$i "; done)
done
