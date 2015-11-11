#!/bin/bash
# Assume the folder with all .debs is on the desktop

[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
FOLDER="${SCRIPT_DIR%/*}"

if [[ $# -ge 1 ]]; then
  FOLDER=$1
fi

sudo apt-get remove -y '.*sdformat.*' '.*ignition.*' '.*gazebo.*' '.*libogre.*dev.*'

sudo dpkg -R -i $FOLDER || true
# If any new dependency is in ubuntu repositories, the dpkg command won't
# get if. apt-get install -f will fix the missing packages that are
# available from known repositories
sudo apt-get install -f -y -q
