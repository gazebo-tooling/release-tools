#!/bin/bash
# Assume the folder with all .debs is on the desktop

[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
FOLDER="${SCRIPT_DIR%/*}"

if [[ $# -ge 1 ]]; then
  FOLDER=$1
fi

sudo apt-get remove 'libsdformat*' 'sdformat*' 'libgazebo*' 'gazebo*'

echo $FOLDER

sudo dpkg -R -i $FOLDER
