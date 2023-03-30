# Release new version of Gazebo unofficial wrappers

  1. Background
    * Upstream versions released using this tutorial
  2. Initial fork and repository setup (one-time)
    * 2.1 Create the alternative -release repository
    * 2.2 Create a custom track in tracks.yaml
  3. Release a new version
    * 3.1 Use the Dockefile with rocker to join the releasing environment
    * 3.2 Use Dockerfile with rocker
    * 3.3 Calling build.osrfoundation.org

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
newer version of Gazebo than the one selected in REP-2000. This document
explains how to create these unofficial variants of the ros_gz``

## 2. Initial fork and repository setup (one-time)

To support the release of these unofficial packages, a fork of the metadata
repository needs to be done and some changes done to the bloom templates
inside it:

### 2.1 Create the alternative -release repository

 1. Fork (manually or using gh) current gbp repository:
    https://github.com/ros2-gbp/ros_ign-release

 1. Make changes to bloom templates to get new package names and conflicts.
    Clone the new repo, go to the directory and run `rename-ros_gz-pkgs.bash`:
    ```
    $ cd ros_ign-release
    $ rename-ros_gz-pkgs.bash <desired_gz_version> <space separted list of rosdistros to release>*
    ```

### 2.2 Create a custom track in tracks.yml

Copy the existing ROS 2 yaml track information and rename it to `${ros2}_gz${version}`.
For example, the `humble` track to be used as base for Garden would be `humble_gzgarden`.

New versioning requires bumping to large numbers. Set:

```
    release_inc: '1000'
```

All non ubuntu generators can be removed.

### Upstream versions released using this tutorial

The `gbp -release repository` hosts the latest version released by the
maintainers of `ros_gz`. When using these instructions to release a new custom
version the version of `ros_gz` released will be the latest one existing in the
official `gbp -release repository`. The version would be the same but the
release number will start on 1000.

## 3. Release a new version

A custom environment is needed to release the new fork since some hacky changes
for rosdep need to be in place when running bloom.

### 3.1 Use the Dockefile with rocker to join the releasing environment

```bash
docker build . -t ros_gzgarden
# The final rosdep update restore your user cache
rocker --home --user ros_gzgarden /bin/bash && rosdep update
```

Caveat: This is going to later the rosdep cache of your user. Be sure of
running rosdep update after you leave the docker enviroment

### 3.2 Run Bloom on the custom -release repository

Inside the docker container:

```bash
   bloom-release --no-pull-request --rosdistro humble --track humble_gzgarden \
     --override-release-repository-url https://github.com/j-rivero/ros_ign-release \
   ros_gz
```

All the previous steps are designed to generate the appropriate Ubuntu metadata
inside the `forked -prerelease repository`. The metadata is being hosted in git
tags following the schema:


```
    release/<ros_distro>/<gazebo_ros_pkg_name>/<version>
    debian/ros-<ros_distro>-<gazebo_ros_pkg_name>_<version>_<ubuntu_distro>
```

The `debian/...` tag contains the upstream code together with the `debian/`
metadata repository ready to be build using `debbuild` or any other debian
generation tool.

### 3.3 Calling build.osrfoundation.org

From outside the docker environment, `build.osrfoundation.org` buildfarm
can be called:

1. Trigger Jenkins jobs with ros_gazebo_pkgs-release.py.bash
```bash
   $ ros_gz-multi-release.py.bash <version <release_repo> <ros_distro> <token> 'other arguments used in release.py'*

   Example:
   $ ros_gz-multi-release.py.bash 0.203 https://github.com/osrf/ros_ign-release humble xxx -r 1001 --dry-run
```
