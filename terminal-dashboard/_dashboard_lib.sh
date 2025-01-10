#!/bin/bash

# TODO:jrivero https://github.com/gazebo-tooling/release-tools/issues/551

get_libraries_by_collection()
{
    local COLLECTION=${1}

    if [ "$COLLECTION" = "fortress" ]; then
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
   elif [ "$COLLECTION" = "ionic" ]; then
      LIBS="
  gz-cmake4
  gz-math8
  gz-utils3
  gz-tools2
  gz-common6
  gz-msgs11
  gz-transport14
  gz-fuel-tools10
  gz-plugin3
  gz-rendering9
  sdformat15
  gz-physics8
  gz-sensors9
  gz-gui9
  gz-sim9
  gz-launch8
  gz-ionic"
    else
      return 1
    fi
    echo "$LIBS"
}
