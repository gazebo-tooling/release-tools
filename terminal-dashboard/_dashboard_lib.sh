#!/bin/bash

get_libraries_by_collection()
{
    local COLLECTION=${1}

    if [ "$COLLECTION" = "citadel" ]; then
      LIBS="
	ignition-cmake2
	ignition-math6
	ignition-tools
	ignition-common3
	ignition-msgs5
	ignition-transport8
	ignition-fuel-tools4
	ignition-plugin
	ignition-rendering3
	sdformat9
	ignition-physics2
	ignition-sensors3
	ignition-gui3
	ignition-gazebo3
	ignition-launch2"
    elif [ "$COLLECTION" = "dome" ]; then
      LIBS="
	ignition-cmake2
	ignition-math6
	ignition-tools
	ignition-common3
	ignition-msgs6
	ignition-transport9
	ignition-fuel-tools5
	ignition-plugin
	ignition-rendering4
	sdformat10
	ignition-physics3
	ignition-sensors4
	ignition-gui4
	ignition-gazebo4
	ignition-launch3"
    elif [ "$COLLECTION" = "edifice" ]; then
      LIBS="
	ignition-cmake2
	ignition-math6
	ignition-utils1
	ignition-tools
	ignition-common4
	ignition-msgs7
	ignition-transport10
	ignition-fuel-tools6
	ignition-plugin
	ignition-rendering5
	sdformat11
	ignition-physics4
	ignition-sensors5
	ignition-gui5
	ignition-gazebo5
	ignition-launch4"
    elif [ "$COLLECTION" = "fortress" ]; then
      LIBS="
	ignition-cmake2
	ignition-math6
	ignition-utils1
	ignition-tools
	ignition-common4
	ignition-msgs8
	ignition-transport11
	ignition-fuel-tools7
	ignition-plugin
	ignition-rendering6
	sdformat12
	ignition-physics5
	ignition-sensors6
	ignition-gui6
	ignition-gazebo6
	ignition-launch5"
    else
      return 1
    fi

    echo "$LIBS"
}
