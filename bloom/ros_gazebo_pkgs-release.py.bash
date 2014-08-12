#!/bin/bash
# Launch all the suite of ros_gazebo_pkgs:
# Usage: ros_gazebo_pkgs <version>
# 

if [[ $# -lt 2 ]]; then
    echo "ros_gazebo_pkgs-release 'same arguments than release.py (except no package names)'"
    exit 1
fi

for p in gazebo-msgs gazebo-plugins gazebo-ros gazebo-ros-control gazebo-ros-pkgs; do
    ./release-bloom.py ${p} $(for i in $@; do echo -n "$i "; done)
done
