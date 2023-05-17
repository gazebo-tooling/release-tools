# Release new version of Gazebo unofficial wrappers

1. Background
  * Upstream versions released using this tutorial
2. Initial setup
  * Create the alternative -release repository
  * Create a custom track in tracks.yaml

## 1. Background

Each ROS release defines one version of Gazebo supported officially through
all the ROS packages. The different combinations of ROS <-> Gazebo can be
found in the [REP-3](http://www.ros.org/reps/rep-2000.html). Some examples:

 * ROS 2 Foxy: Citadel
 * ROS 2 Humble: Fortress

Some use cases require the use of alternative combinations of ROS and Gazebo
versions. The `ros_gz` code is usually prepared to be compatible with
different versions of Gazebo, especially the latest ones.

Although using the officially supported version is the recommended way
specially for non experienced users, some use cases might need to use a
newer version of Gazebo than the one selected in REP-2000.

### Upstream versions released using this tutorial

The `gbp -release repository` hosts the latest version released by the
maintainers of `ros_gz`. When using these instructions to release a new custom
version the version of `ros_gz` released will be the latest one existing in the
official `gbp -release repository`. The version would be the same but the
release number will start on 1000.

## 2. Initial setup 

To release a modified version of `ros_gz` which supports a different major
version of gazebo, before running bloom some actions need to be taken:

## 2. Initial setup

### 2.1 Create the alternative -release repository

For a new official wrappers the notation used below correspond to `ros_ign-release`:

 1. Fork (manually or using gh) current gbp repository:
    https://github.com/ros2-gbp/ros_ign-release

 1. Clone the new repo, go to the directory and run rename-gazebo-ros-pkgs.bash
    - Usage: *$ rename-ros_gz-pkgs.bash <desired_gz_version> <space separted list of rosdistros to release>*


### 2.2 Create a custom track in tracks.yml

Copy the existing ROS 2 yaml track information and rename it to `${ros2}_gz${version}`.
For example, the `humble` track to be used as base for Garden would be `humble_gzgarden`.

New versioning requires bumping to large numbers. Set:

```
    release_inc: '1000'
```

All non ubuntu generators can be removed.
