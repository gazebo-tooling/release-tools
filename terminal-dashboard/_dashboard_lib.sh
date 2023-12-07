#!/bin/bash

# TODO:jrivero https://github.com/gazebo-tooling/release-tools/issues/551

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
	ignition-launch2
	ignition-citadel"
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
	ignition-launch5
	ignition-fortress"
    elif [ "$COLLECTION" = "garden" ]; then
      LIBS="
	gz-cmake3
	gz-math7
	gz-utils2
	gz-tools2
	gz-common5
	gz-msgs9
	gz-transport12
	gz-fuel-tools8
	gz-plugin2
	gz-rendering7
	sdformat13
	gz-physics6
	gz-sensors7
	gz-gui7
	gz-sim7
	gz-launch6
	gz-garden"
   elif [ "$COLLECTION" = "harmonic" ]; then
      LIBS="
	gz-cmake3
	gz-math7
	gz-utils2
	gz-tools2
	gz-common5
	gz-msgs10
	gz-transport13
	gz-fuel-tools9
	gz-plugin2
	gz-rendering8
	sdformat14
	gz-physics7
	gz-sensors8
	gz-gui8
	gz-sim8
	gz-launch7
	gz-harmonic"
    else
      return 1
    fi
    echo "$LIBS"
}
