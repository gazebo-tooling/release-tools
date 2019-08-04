#!/bin/bash
# Launch all the suite of ros_gazebo_pkgs:
# Usage: ros1_ign_bridge-release.py.bash <version>
#

if [[ $# -lt 2 ]]; then
    echo "$0 <version <release_repo> <token> 'other arguments used in release.py'"
    exit 1
fi

for p in ros1-ign-image ros1-ign-bridge ros1-ign-gazebo-demos ros1-ign-point-cloud ros1-ign; do
  ./release-bloom.py "${p}" $(for i in $@; do echo -n "$i "; done)
done
