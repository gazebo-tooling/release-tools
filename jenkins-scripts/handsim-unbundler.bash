#!/bin/sh
# Assume the folder with all .debs is on the desktop

FOLDER='.'

if [[ $# -ge 1 ]]; then
  FOLDER=$1
fi

sudo dpkg --purge *sdformat* *gazebo*

echo $FOLDER

sudo dpkg -R -i $FOLDER
