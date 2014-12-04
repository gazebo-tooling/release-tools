#!/bin/bash
# Launch all the suite of ros_gazebo_pkgs:
# Usage: ros_gazebo_pkgs <version>
# 

if [[ $# -lt 2 ]]; then
    echo "robocup-release 'same arguments than release.py (except no package names)'"
    exit 1
fi

for p in robocup-msgs robocup-model-resources robocup-utils robocup-agent-plugin robocup-gamecontroller-plugin robocup-3d; do
    echo " ${p} $(for i in $@; do echo -n "$i "; done)"
    ./release-bloom.py ${p} $(for i in $@; do echo -n "$i "; done)
done
