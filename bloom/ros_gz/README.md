# Release new version of Gazebo unofficial wrappers

1. Background
   * Upstream versions released using this tutorial
2. Initial setup
   * Create the alternative -release repository
   * Create a custom track in tracks.yaml
3. Run a new release
   * Prerequisites
   * Bloom a new release

## 1. Background

Each ROS release defines one version of Gazebo supported officially through
all the ROS packages. The different combinations of ROS <-> Gazebo can be
found in the [REP-3](http://www.ros.org/reps/rep-2000.html). Some examples:

 * ROS 2 Foxy: Citadel
 * ROS 2 Humble: Fortress
 * ROS 2 Iron: Fortress

Some use cases require the use of alternative combinations of ROS and Gazebo
versions. The `ros_gz` code is usually prepared to be compatible with
different versions of Gazebo, especially the latest ones.

Although using the officially supported version is the recommended way
specially for non experienced users, some use cases might need to use a
newer version of Gazebo than the one selected in REP-2000.

### List of active relases

| Gazebo Release | ROS / ROS 2 Release | status     | ros_gz branch | -release repository | CI | release cmd |
| ---------------|---------------------|------------|---------------|---------------------|----|-------------|
| Harmonic       | Humble              | stable     | humble        | https://github.com/gazebo-release/ros_gz-gzharmonic-release | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ros_gzharmonic_bridge-install-pkg_humble-ci-jammy-amd64)](https://build.osrfoundation.org/job/ros_gzharmonic_bridge-install-pkg_humble-ci-jammy-amd64/) | <details>```RELEASE_REPO_URL=https://github.com/gazebo-release/ros_gz-gzharmonic-release ./bloom_from_special_env.bash hubmle harmonic https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/replace_fortress_with_harmonic/00-replace-gz-fortress-with-harmonic.list```</details> |
| Harmonic       | Iron                | stable     | ros2          | https://github.com/gazebo-release/ros_gz-gzharmonic-release | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ros_gzharmonic_bridge-install-pkg_iron-ci-jammy-amd64)](https://build.osrfoundation.org/job/ros_gzharmonic_bridge-install-pkg_iron-ci-jammy-amd64/) | <details>```RELEASE_REPO_URL=https://github.com/gazebo-release/ros_gz-gzharmonic-release ./bloom_from_special_env.bash iron harmonic https://raw.githubusercontent.com/osrf/osrf-rosdep```</details> |

### List of EOL releases

| Gazebo Release | ROS / ROS 2 Release | status     | ros_gz branch | -release repository | CI | release cmd |
| ---------------|---------------------|------------|---------------|---------------------|----|-------------|
| Garden         | Humble              | stable     | humble        | https://github.com/gazebo-release/ros_ign-release | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ros_gzgarden_bridge-install-pkg_humble-ci-jammy-amd64)](https://build.osrfoundation.org/job/ros_gzgarden_bridge-install-pkg_humble-ci-jammy-amd64/) | <details>```RELEASE_REPO_URL=https://github.com/gazebo-release/ros_ign-release ./bloom_from_special_env.bash humble garden https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/replace_fortress_with_garden/00-replace-gz-fortress-with-garden.list```</details> |
| Garden         | Iron                | stable     | iron          | https://github.com/gazebo-release/ros_ign-release | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ros_gzgarden_bridge-install-pkg_iron-ci-jammy-amd64)](https://build.osrfoundation.org/job/ros_gzgarden_bridge-install-pkg_iron-ci-jammy-amd64/) | <details>```RELEASE_REPO_URL=https://github.com/gazebo-release/ros_ign-release ./bloom_from_special_env.bash iron garden https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/replace_fortress_with_garden/00-replace-gz-fortress-with-garden.list```</details> |

### Upstream versions released using this tutorial

The `gbp -release repository` hosts the latest version released by the
maintainers of `ros_gz`. When using these instructions to release a new custom
version the version of `ros_gz` released will be the latest one existing in the
official `gbp -release repository`. The version would be the same but the
release number will start on 1000.

## 2. Initial setup

To release a modified version of `ros_gz` which supports a different major
version of gazebo, before running bloom some actions need to be taken:

### 2.1 Create the alternative -release repository

For a new official wrappers the notation used below correspond to `ros_ign-release`:

 1. Fork (or create a new repo with all the tags and branches) the current gbp repository:
    https://github.com/ros2-gbp/ros_ign-release

Note: the same -release repository can obviously hosts multiple ROS 2 releases but
can not have multipe Gazebo releases since the bloom templates are set to just one
package name.

On the fork, the `rename-ros_gz-pkgs.bash` script will change the bloom templates to modify package names and inject the `GZ_VERSION` needed:

 1. Clone the new repo, go to the directory and run rename-gazebo-ros-pkgs.bash
    - Usage: `$ rename-ros_gz-pkgs.bash <desired_gz_version> <space separted list of rosdistros to release>`
    - Example: `$ rename-ros_gz-pkgs.bash harmonic humble`

If several unofficial wrappers using different Gazebo releases are going to exist for the same ROS 2 distribution (i.e: Harmonic for ROS 2 Iron), the rename script supports to declare a conflict using the `GZ_RELEASE_TO_CONFLICT` variable:

```
GZ_RELEASE_TO_CONFLICT=garden ./rename-ros_gz-pkgs.bash harmonic humble
```

### 2.2 Create a custom track in tracks.yml

Copy the existing ROS 2 yaml track information and rename it to `${ros2}_gz${version}`.
For example, the `humble` track to be used as base for Harmonic would be `humble_gzharmonic`.

New versioning requires bumping to large numbers. Set:

```
    release_inc: '1000'
```

Debian, rhel and fedora generators can be removed.

## 3. Run a new release

To execute a new release of the ros_gz unofficial wrappers there are mainly two
steps to do: use bloom to generate new metadata in the -release repo fork and
use ros_gz-release.py script to trigger the builds builds.osrfoundation.org.

The new version will be the latest one released in the rosdistro index of the
official ros_gz packages.

### Prerequisites

The host system to release should have `docker` and `rocker` installed.

### 3.1 Bloom a new release

Blooming a new release of the ros_gz unofficial wrappers requires some changes
to be done in the releasing enviroment affecting the rosdep rules. To facilitate
this, there is a `Dockerfile` that provides the needed modifications and a script
that encapsulates the bloom arguments to be passed and the use of this enviroment.

```
./bloom_from_special_env.bash <ROS_DISTRO> <TARGET_NEW_GAZEBO_COLLECTION> <URL_OSRF_ROSDEP>

i.e: release ros_gz on ROS 2 Humble replacing Gazebo Fortress by Gazebo Harmonic
./bloom_from_special_env.bash \
    humble \
    harmonic \
    https://raw.githubusercontent.com/osrf/osrf-rosdep/refs/heads/master/gz/replace_fortress_with_harmonic/00-replace-gz-fortress-with-harmonic.list
```

The script will create the docker enviroment with the rosdep modifications needed
and invoke rocker with `--home` and `--user` flags to pass the credentials and
customatizations needed for the bloom call. It will run the `bloom-release` command
with the arguments required for the ros_gz wrappers.

The script supports to inject a custom RELEASE_REPO_URL that points to a bloom gbp
repository different than https://github.com/gazebo-release/ros_ign-release.

```
i.e use a https://github.com/gazebo-testing/ros_ign-gzharmonic-release as gbp testing repository

RELEASE_REPO_URL=https://github.com/gazebo-testing/ros_ign-gzharmonic-release \
  ./bloom_from_special_env.bash \
    humble \
    harmonic \
    https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/replace_fortress_with_harmonic/00-replace-gz-fortress-with-harmonic.list
```

### 3.2 Launching jobs in the osrf buildfarm

The previous step generates the metadata needed to build the debians but there is
a final step to call the jobs inside the buildfarm that will create the debians.

To do so, for simulating the calls to be done to the buildfarm:

```
./ros_gz-release.py.bash <version-without-revision> <release-repo> <ros_distro> <token> --dry-run -r <revision>
# Example for 0.244.11-1001 version for ROS Humble in ros_ign-release
./ros_gz-release.py.bash 0.244.11 https://github.com/gazebo-release/ros_ign-release humble foo --dry-run -r 1001
```

For releasing directly, just remove the `--dry-run` argument.
