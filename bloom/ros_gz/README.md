# Release new version of Gazebo unofficial wrappers

0. Quick commands for existing users
1. Background
  * Upstream versions released using this tutorial
2. Initial setup
  * Create the alternative -release repository
  * Create a custom track in tracks.yaml
3. Release a new version
  * Use Dockerfile with rocker
  * 

## 0. Quick commands

1. bloom in the osrf/ repository
```bash

   bloom-release --no-pull-request --rosdistro humble --track humble_gzgarden \
     --override-release-repository-url https://github.com/j-rivero/gbp-ros_ign-release \
     --override-release-repository-push-url https://github.com/j-rivero/ros_ign-release \
   ros_gz
```

1. Trigger Jenkins jobs with ros_gazebo_pkgs-release.py.bash
```bash
   $ ros_gazebo_pkgs-release. <version <release_repo> <ros_distro> <token> 'other arguments used in release.py'*

   Example:
   $ ros_gazebo_pkgs-release.py.bash 3.4.4 https://github.com/osrf/gazebo11_ros2_pkgs-release foxy xxx -r 1 --dry-run
```

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


Initial setup:
 1. (optional now?) Make rosdep to resolve the system packages changed
 1. Fork and modify the official [gbp -release repo](https://github.com/ros-gbp/gazebo_ros_pkgs-release).

Doing a release:

 1. (skip if Intial setup was made) Sync fork for branches and tags (TODO)
 1. Run bloom on the modified release repository.
 4. Custom infrastructure to create .deb packages.

We'll go over these steps in more detail below.

## 2. Initial setup

### 2.1 Create the alternative -release repository

For a new official wrappers the notation used below correspond to:
`ros_ign-release

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

## Release a new version

### Use the Dockefile with rocker

```bash
docker build . -t ros_gzgarden
# The final rosdep update restore your user cache
rocker --home --user ros_gzgarden /bin/bash && rosdep update
```

Caveat: This is going to later the rosdep cache of your user. Be sure of
running rosdep update after you leave the docker enviroment

### Run Bloom on the custom -release repository

Inside the docker container:

```bash
   bloom-release --no-pull-request --rosdistro humble --track humble_gzgarden \
     --override-release-repository-url https://github.com/j-rivero/gbp-ros_ign-release \
   ros_gz
```

### Custom infrastructure to create .deb packages

All the previous steps are designed to generate the appropriate Ubuntu metadata
inside the `forked -prerelease repository`. The metadata is being hosted in
git tags following the schema:


    release/<ros_distro>/<gazebo_ros_pkg_name>/<version>
    debian/ros-<ros_distro>-<gazebo_ros_pkg_name>_<version>_<ubuntu_distro>

The `debian/...` tag contains the upstream code together with the `debian/` metadata repository ready to be
build using `debbuild` or any other debian generation tool.

For reference, the OSRF buildfarm is using [this script](https://github.com/gazebo-tooling/release-tools/blob/master/jenkins-scripts/docker/lib/debbuild-bloom-base.bash)

