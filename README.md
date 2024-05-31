# Release Tools

This repository holds scripts and tools that are used for testing and releasing
Gazebo software.

## Scripts

  * [release.py](release.py): Triggers new debian and homebrew releases (major, minor, patch, pre-release...).
  * For scripts working on -release repositories, please see [release-repo-scripts/README.md](release-repo-scripts/README.md)
  * For scripts working on library source repository, please see [source-repo-scripts/README.md](source-repo-scripts/README.md)
  * For scripts related to unofficial ROS packages, please see [bloom scripts](bloom/ros_gazebo_pkgs/README.md)
  * For scripts working on DSL Jenkins code and/or gz-collections.yaml, please see [dsl/tools](jenkins-scripts/dsl/tools/README.md)

### Making releases

For the developers officially maintaining Gazebo software that need to run a new release of the software:

 * The [Gazebo Release Process](https://gazebosim.org/docs/garden/release)
   document provides an overview of how releasing works.
 * The [Gazebo Release Instructions](https://gazebosim.org/docs/garden/releases-instructions)
   document provides step by step instructions to run new releases.
